/*
 * charge_statemachine.c
 *
 *  Created on: 07.10.2020
 *      Author: alexander.schroeder
 */

#include "mar_charge_statemachine.h"

#include <math.h>

#include "hw.h"
#include "hal.h"
#include "comm_can.h"

#include "mar_vars.h"
#include "LTC6804_handler.h"
#include "mar_safety.h"
#include "mar_terminal.h"

#include <string.h>

#include "commands.h"  // debug

#define ID_CHG_NMT			0x70A
#define ID_MC_NMT			0x701
#define ID_NMT_BOADCAST		0x000
#define ID_CHG_TPDO1		0x18A
#define ID_CHG_RPDO1		0x20A

#define CMD_BAT_DISCOVER	0x40
#define CMD_BAT_U			0x41
#define CMD_BAT_I			0x42
#define CMD_BAT_SoC			0x43
#define CMD_BAT_dis_lim		0x44
#define CMD_BAT_chg_lim		0x45
#define CMD_BAT_cell_min	0x46
#define CMD_BAT_cell_max	0x47
#define CMD_BAT_int_temp	0x48
#define CMD_BAT_temp_1		0x49
#define CMD_BAT_temp_2		0x4A
#define CMD_BAT_temp_3		0x4B
#define CMD_BAT_temp_x		0c4C
#define CMD_BAT_temp_y		0x4D
#define CMD_BAT_cell_1_4	0x60
#define CMD_BAT_cell_5_8	0x61
#define CMD_BAT_cell_9_12	0x62
#define CMD_BAT_balance		0x63
#define CMD_BAT_set_state	0x70

#define I2chg(x)			((uint16_t) (x*16))
#define U2chg(x)			((uint16_t) (x*256))

volatile float I_CHG_max = 25;

bool charger_present, charger_detected;
uint32_t charger_present_timeout;
uint32_t charge_cycles;


bool battery_present, standby_present;
uint32_t battery_present_timeout, standby_present_timeout;
bool tx_BAT_SET_STATE;
uint32_t tx_BAT_SET_STATE_timer;

enum
{
	btmd_sleep,
	btmd_discover,
	btmd_standby,
	btmd_precharge,
	btmd_run,
	btmd_error,
	btmd_reset
} bat_mode_rx, bat_mode_tx;
float U_bat_ext, dis_limit_ext_bat;
float external_battery_discharge_limit;


float remote_chg_U, remote_chg_I;
bool tx_NMT, tx_RPDO1;
uint32_t NMT_timer, RPDO1_timer;

union
{
	uint8_t B[8];

	struct
	{
		uint8_t filling;
		uint8_t SoC_display;
		uint8_t charge_cycle_type;
		uint16_t U_max;
		uint16_t I_max;
		uint8_t battery_status;
	} __attribute__ ((aligned (8), packed));
} charger_RPDO1;

uint32_t chg_timer, chg_counter;
volatile bool charge_enable = true;
CHG_state_t cp_state;

void CAN_CHG_callback(CANRxFrame *frame)
{
	if (frame->IDE)
	{
		if (((frame->EID & 0xFF) >= 10) && ((frame->EID >> 8) >= 0x40))
		{
			switch (frame->EID >> 8)
			{
			case CMD_BAT_DISCOVER:
				battery_present_timeout = chVTGetSystemTimeX();
				battery_present = true;

				bat_mode_rx = frame->data8[0];
				break;

			case CMD_BAT_U:
				standby_present = true;
				standby_present_timeout = chVTGetSystemTimeX();

				memcpy(&U_bat_ext, frame->data32, 4);
				break;

			case CMD_BAT_dis_lim:
				standby_present = true;
				standby_present_timeout = chVTGetSystemTimeX();

				memcpy(&dis_limit_ext_bat, frame->data32, 4);
				break;
			}
		}
	}
	else
	{
	switch (frame->SID)
	{
	case ID_CHG_NMT:
//		commands_printf("NMT msg rx");
		charger_present_timeout = chVTGetSystemTimeX();
		charger_present = true;
		break;

	case ID_CHG_TPDO1:
//		commands_printf("TPDO1 msg rx");
		remote_chg_I = frame->data16[0]/256.;
		remote_chg_U = frame->data16[1]/256.;
		break;

	default:
//		commands_printf("other msg rx 0x%03X", frame->SID);
		break;
	}
}
}

void chg_init(void)
{
	CHRG_OFF;

	cp_state = cpst_init;
	remote_chg_U = 0;
	remote_chg_I = 0;

	for (uint8_t i = 0; i < 8; i++)
		charger_RPDO1.B[i] = 0;

	charger_present = false;
	charger_detected = false;
	charge_cycles = 0;
	charger_present_timeout = chVTGetSystemTimeX();

	battery_present = false;
	battery_present_timeout = chVTGetSystemTimeX();
	standby_present = false;
	standby_present_timeout = chVTGetSystemTimeX();

	bat_mode_rx = btmd_sleep;
	bat_mode_tx = btmd_sleep;
	tx_BAT_SET_STATE = false;
	tx_BAT_SET_STATE_timer = chVTGetSystemTimeX();
	U_bat_ext = 0;
	dis_limit_ext_bat = 0;
	external_battery_discharge_limit = 1;

	tx_NMT = false;
	tx_RPDO1 = false;
	NMT_timer = chVTGetSystemTimeX();
	RPDO1_timer = chVTGetSystemTimeX();

	comm_can_set_rx_callback2(&CAN_CHG_callback);
}

void charge_statemachine()
{
	if (!charge_enable)
		cp_state = cpst_wait_for_enable;

	if (chVTTimeElapsedSinceX(charger_present_timeout) > S2ST(2))
		charger_present = false;

	if (chVTTimeElapsedSinceX(standby_present_timeout) > S2ST(2))
		standby_present = false;

	if (chVTTimeElapsedSinceX(battery_present_timeout) > S2ST(2))
		battery_present = false;

	if ((chVTTimeElapsedSinceX(NMT_timer) > S2ST(1)) && tx_NMT)
	{
		NMT_timer = chVTGetSystemTimeX();
		uint8_t cmd = 0x05;
		comm_can_transmit_sid2(ID_MC_NMT, (uint8_t *) &cmd, sizeof(cmd));  // tx hearthbeat of MC
	}

	if ((chVTTimeElapsedSinceX(RPDO1_timer) > MS2ST(250)) && tx_RPDO1)
	{
		RPDO1_timer = chVTGetSystemTimeX();
		comm_can_transmit_sid2(ID_CHG_RPDO1, (uint8_t *) &charger_RPDO1, sizeof(charger_RPDO1));  // tx RPDO1
	}

	if ((chVTTimeElapsedSinceX(tx_BAT_SET_STATE_timer) > MS2ST(100)) && tx_BAT_SET_STATE)
	{
		tx_BAT_SET_STATE_timer = chVTGetSystemTimeX();
		comm_can_transmit_eid2(0xFF01 | (CMD_BAT_set_state << 16), (uint8_t *) &bat_mode_tx, 1);  // set state of external battery
	}

	charger_detected = charger_present || (U_CHG > 20);
	if (charger_present)
		safety_reset_sleep_counter();

	switch (cp_state)
	{
	case cpst_init:
		commands_printf("Charge statemachine init!");
		chg_init();

		chg_timer = chVTGetSystemTimeX();
		cp_state = cpst_wait_for_init;
		break;

	case cpst_wait_for_init:
		I_CHG_offset = I_CHG_filt;

		if (chVTTimeElapsedSinceX(chg_timer) > S2ST(10))
		{
			if (charge_enable)
			{
				commands_printf("Waiting for charge permission from BMS!");

				cp_state = cpst_wait_charge_allowed;
			}
			else
			{
				commands_printf("Waiting for charge enable!");

				cp_state = cpst_wait_for_enable;
			}
		}
		break;

	case cpst_wait_charge_allowed:
		if (BMS_Charge_permitted)
		{
			commands_printf("Waiting for charger!");

			cp_state = cpst_discover;
		}
		break;

	case cpst_wait_for_enable:
		if (charge_enable)
		{
			commands_printf("Charging enabled! Reinitializing!");

			cp_state = cpst_init;
		}
		break;

	case cpst_discover:
		if (charger_present)
		{
			commands_printf("Charger found!");
			commands_printf("Waiting for voltages to equalize!");
			I_CHG_offset = I_CHG_filt;

			safety_lock_motor();

			uint16_t cmd = 0x0A81;
			comm_can_transmit_sid2(ID_NMT_BOADCAST, (uint8_t *) &cmd, sizeof(cmd));  // NMT reset of charger

			cmd = 0x0A01;
			comm_can_transmit_sid2(ID_NMT_BOADCAST, (uint8_t *) &cmd, sizeof(cmd));  // NMT start of charger

			tx_NMT = true;

			charger_RPDO1.I_max = I2chg(1);
			charger_RPDO1.U_max = U2chg(U_DC);
			charger_RPDO1.battery_status = 1;
			tx_RPDO1 = true;

			chg_timer = chVTGetSystemTimeX();
			cp_state = chgst_wait_equalize;
		}
		if (battery_present)
		{
			commands_printf("External battery found!");
			I_CHG_offset = I_CHG_filt;

			commands_printf("Activating battery!");
			tx_BAT_SET_STATE = true;
			bat_mode_tx = btmd_standby;

			chg_timer = chVTGetSystemTimeX();
			cp_state = batst_connect;
		}
		break;

	case batst_connect:
		if (standby_present)
		{
			if (U_DC > (U_bat_ext + 1))
			{
				commands_printf("Internal battery higher: Use internal and switch when equal.");

				cp_state = batst_run_internal;
			}
			else if (U_DC < (U_bat_ext - 1))
			{
				commands_printf("External battery higher: Use external until equal.");
				commands_printf("Internal battery deactivated. Waiting for external battery to switch on!");

				external_battery_discharge_limit = 0;
				BAT_OFF;
				bat_mode_tx = btmd_run;

				chg_timer = chVTGetSystemTimeX();
				cp_state = batst_switch_external;
			}
			else
			{
				commands_printf("Batteries equal(ish): Simply switch on.");

				CHRG_ON;
				bat_mode_tx = btmd_run;

				cp_state = batst_run_parallel;
			}
		}

		if ((chVTTimeElapsedSinceX(chg_timer) >= S2ST(1)) || !battery_present || !standby_present)
		{
			commands_printf("Failed to obtain status messages!");
			cp_state = cpst_error;
		}
		break;

	case batst_run_parallel:
		external_battery_discharge_limit = dis_limit_ext_bat;

		if (!standby_present || !battery_present)
		{
			commands_printf("Lost external Battery!");

			cp_state = batst_bat_lost;
		}
		break;

	case batst_switch_external:
		if (bat_mode_rx == btmd_run)
		{
			cp_state = batst_run_external;
		}

		if ((chVTTimeElapsedSinceX(chg_timer) >= S2ST(1)) || !battery_present || !standby_present)
		{
			commands_printf("Failed to obtain status messages!");
			cp_state = cpst_error;
		}
		break;

	case batst_bat_lost:
		CHRG_OFF;
		BAT_ON;
		tx_BAT_SET_STATE = false;

		cp_state = cpst_wait_for_reset;
		break;

	case chgst_wait_equalize:
		if ((float) fabs(U_DC - U_CHG) < 2.)
		{
			commands_printf("Waiting for current to rise!");

			CHRG_ON;
			charger_RPDO1.U_max = U2chg(45);

			chg_timer = chVTGetSystemTimeX();
			cp_state = chgst_wait_settle;
			break;
		}

		if ((chVTTimeElapsedSinceX(chg_timer) >=	 S2ST(30)) || (!charger_present))
			cp_state = cpst_error;

		break;

	case chgst_wait_settle:
		if (I_CHG >= 0.75)
		{
			commands_printf("Charging started!");

			chg_timer = chVTGetSystemTimeX();
			cp_state = chgst_charging;
		}

		if (!BMS_Charge_permitted)
			cp_state = chgst_charge_finished;

		if ((chVTTimeElapsedSinceX(chg_timer) >=	 S2ST(30)) || (!charger_present))
			cp_state = cpst_error;

		break;

	case chgst_charging:
		if ((I_CHG > 30) || (I_CHG < -1) || (!charger_present) || (BMS.Temp_sensors[4] > AUX_temp_cutoff))
		{
			CHRG_OFF;

			cp_state = cpst_error;
			break;
		}

		if (chVTTimeElapsedSinceX(chg_timer) >=	 MS2ST(100))
		{
			chg_timer = chVTGetSystemTimeX();

			static float chg_I_filt = 1;
			chg_I_filt -= chg_I_filt/10;
			chg_I_filt += I_CHG_max*BMS_Charge_Limit;

			charger_RPDO1.I_max = I2chg(chg_I_filt/10);
		}

		if ((I_CHG < 0.5) || !BMS_Charge_permitted)
		{
			commands_printf("Charge_Limit %.0f%% Max_U %.3fV", (double) BMS_Charge_Limit*100, (double) Global_Max_U);
			cp_state = chgst_charge_finished;
		}
		break;

	case chgst_charge_finished:
		commands_printf("Charging finished!");
		commands_printf("Waiting for restart!");

		CHRG_OFF;
		tx_NMT = false;
		tx_RPDO1 = false;

		charge_cycles++;
		mar_write_conf();

		cp_state = cpst_wait_for_restart;
		break;

	case cpst_error:
		commands_printf("Charge port error!");
		commands_printf("U_DC %.1fV U_CHG %.1fV", (double) U_DC, (double) U_CHG);
		commands_printf("I_CHG %.3fA", (double) I_CHG);
		commands_printf("Chargeport temperature %.1f°C", (double) BMS.Temp_sensors[4]);
		commands_printf("charger present %d", charger_present);
		commands_printf("battery present %d", battery_present);
		commands_printf("status present %d", standby_present);

		CHRG_OFF;
		tx_NMT = false;
		tx_RPDO1 = false;
		tx_BAT_SET_STATE = false;

		cp_state = cpst_wait_for_reset;
		break;

	case cpst_wait_for_restart:
		if (!charger_present || (BMS_Charge_Limit >= 0.9))
			cp_state = cpst_wait_charge_allowed;
		break;

	case cpst_wait_for_reset:
		if (!charger_present && !battery_present)
			cp_state = cpst_wait_charge_allowed;
		break;

	default:
		cp_state = cpst_init;
	}
}
