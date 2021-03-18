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

#include "commands.h"  // debug

#define ID_CHG_NMT			0x70A
#define ID_MC_NMT			0x701
#define ID_NMT_BOADCAST		0x000
#define ID_CHG_TPDO1		0x18A
#define ID_CHG_RPDO1		0x20A

#define I2chg(x)			((uint16_t) (x*16))
#define U2chg(x)			((uint16_t) (x*256))

volatile float I_CHG_max = 25;

bool charger_present, charger_detected;
uint32_t charger_present_timeout;
uint32_t charge_cycles;

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
CHG_state_t chg_state;

void CAN_CHG_callback(CANRxFrame *frame)
{
	if (frame->IDE)
		return;

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

void chg_init()
{
	CHRG_OFF;

	chg_state = chgst_init;
	remote_chg_U = 0;
	remote_chg_I = 0;

	for (uint8_t i = 0; i < 8; i++)
		charger_RPDO1.B[i] = 0;

	charger_present = false;
	charger_detected = false;
	charge_cycles = 0;

	charger_present_timeout = chVTGetSystemTimeX();
	tx_NMT = false;
	tx_RPDO1 = false;
	NMT_timer = chVTGetSystemTimeX();
	RPDO1_timer = chVTGetSystemTimeX();

	comm_can_set_rx_callback2(&CAN_CHG_callback);
}

void charge_statemachine()
{
	if (!charge_enable)
		chg_state = chgst_wait_for_enable;

	if (chVTTimeElapsedSinceX(charger_present_timeout) > S2ST(2))
		charger_present = false;

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

	charger_detected = charger_present || (U_CHG > 20);
	if (charger_present)
		safety_reset_sleep_counter();

//	static CHG_state_t chg_state_back;
//	uint32_t state_timer;
//	if ((chVTTimeElapsedSinceX(state_timer) > MS2ST(500)) || (chg_state != chg_state_back))
//	{
//		chg_state_back = chg_state;
//		state_timer = chVTGetSystemTimeX();
//		comm_can_transmit_sid2(0x42, (uint8_t *) &chg_state, sizeof(chg_state));
//	}

	switch (chg_state)
	{
	case chgst_init:
		commands_printf("Charge statemachine init!");
		chg_init();

		chg_timer = chVTGetSystemTimeX();
		chg_state = chgst_wait_for_init;
		break;

	case chgst_wait_for_init:
		I_CHG_offset = I_CHG_filt;

		if (chVTTimeElapsedSinceX(chg_timer) > S2ST(10))
		{
			if (charge_enable)
			{
				commands_printf("Waiting for charge permission from BMS!");

				chg_state = chgst_wait_charge_allowed;
			}
			else
			{
				commands_printf("Waiting for charge enable!");

				chg_state = chgst_wait_for_enable;
			}
		}
		break;

	case chgst_wait_charge_allowed:
		if (BMS_Charge_permitted)
		{
			commands_printf("Waiting for charger!");

			chg_state = chgst_wait_for_charger;
		}
		break;

	case chgst_wait_for_enable:
		if (charge_enable)
		{
			commands_printf("Charging enabled! Reinitializing!");

			chg_state = chgst_init;
		}
		break;

	case chgst_wait_for_charger:
		if (charger_present || stand_alone)
		{
			commands_printf("Charger Found!");
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
			chg_state = chgst_wait_equalize;
		}
		break;

	case chgst_wait_equalize:
		if ((float) fabs(U_DC - U_CHG) < 2.)
		{
			commands_printf("Waiting for current to rise!");

			CHRG_ON;
			charger_RPDO1.U_max = U2chg(45);

			chg_timer = chVTGetSystemTimeX();
			chg_state = chgst_wait_settle;
			break;
		}

		if ((chVTTimeElapsedSinceX(chg_timer) >=	 S2ST(30)) || (!charger_present))
			chg_state = chgst_error;

		break;

	case chgst_wait_settle:
		if (I_CHG >= 0.75)
		{
			commands_printf("Charging started!");

			chg_timer = chVTGetSystemTimeX();
			chg_state = chgst_charging;
		}

		if (!BMS_Charge_permitted && !stand_alone)
			chg_state = chgst_charge_finished;

		if ((chVTTimeElapsedSinceX(chg_timer) >=	 S2ST(30)) || (!charger_present))
			chg_state = chgst_error;

		break;

	case chgst_charging:
		if ((I_CHG > 30) || (I_CHG < -1) || (!charger_present) || (BMS.Temp_sensors[2] > AUX_temp_cutoff))
		{
			CHRG_OFF;

			chg_state = chgst_error;
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

		if ((I_CHG < 0.5) || (!BMS_Charge_permitted && !stand_alone))
		{
			commands_printf("Charge_Limit %.0f%% Max_U %.3fV", (double) BMS_Charge_Limit*100, (double) Global_Max_U);
			chg_state = chgst_charge_finished;
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

		chg_state = chgst_wait_for_restart;
		break;

	case chgst_error:
		commands_printf("Charging error!");
		commands_printf("U_DC %.1fV U_CHG %.1fV", (double) U_DC, (double) U_CHG);
		commands_printf("I_CHG %.3fA", (double) I_CHG);
		commands_printf("charger present %d", charger_present);
		commands_printf("I_CHG %.1f�C", (double) BMS.Temp_sensors[2]);

		CHRG_OFF;
		tx_NMT = false;
		tx_RPDO1 = false;

		chg_state = chgst_wait_for_reset;
		break;

	case chgst_wait_for_restart:
		if (!charger_present || (BMS_Charge_Limit >= 0.9))
			chg_state = chgst_wait_charge_allowed;
		break;

	case chgst_wait_for_reset:
		if (!charger_present)
			chg_state = chgst_wait_charge_allowed;
		break;

	default:
		chg_state = chgst_init;
	}
}
