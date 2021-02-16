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

#include "maraneo_vars.h"
#include "LTC6804_handler.h"

#include "commands.h"  // debug

#define ID_CHG_NMT			0x70A
#define ID_MC_NMT			0x701
#define ID_NMT_BOADCAST		0x000
#define ID_CHG_TPDO1		0x18A
#define ID_CHG_RPDO1		0x20A

#define I2chg(x)			((uint16_t) (x*16))
#define U2chg(x)			((uint16_t) (x*256))

bool charger_present;
uint32_t charger_present_timeout;
float remote_chg_U, remote_chg_I;

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
	};
} charger_RPDO1;

uint32_t chg_timer, chg_counter;
volatile bool charge_en;
CHG_state_t chg_state;

void CAN_CHG_callback(uint32_t id, uint8_t *data, uint8_t len)
{
	(void) len;

	switch (id)
	{
	case ID_CHG_NMT:
		charger_present_timeout = chVTGetSystemTimeX();
		charger_present = true;
		break;

	case ID_CHG_TPDO1:
		remote_chg_I = ((uint16_t) data[0] | (uint16_t) data[1] << 8)/256.;
		remote_chg_U = ((uint16_t) data[2] | (uint16_t) data[3] << 8)/256.;
		break;
	}
}

void chg_init()
{
	CHRG_OFF;

	chg_state = chgst_init;
	charge_en = true;
	remote_chg_U = 0;
	remote_chg_I = 0;

	for (uint8_t i = 0; i < 8; i++)
		charger_RPDO1.B[i] = 0;

	charger_present = false;
	charger_present_timeout = chVTGetSystemTimeX();

	comm_can_set_rx_callback2(&CAN_CHG_callback);
}

void chg_increment_counter(void)
{
	eeprom_var chg_cy;
	conf_general_read_eeprom_var_custom(&chg_cy, 63);
	chg_cy.as_u32++;
//	conf_general_store_eeprom_var_custom(&chg_cy, 63);
}


void charge_statemachine()
{
	if (!charge_en)
		chg_state = chgst_wait_for_enable;

	if (chVTTimeElapsedSinceX(charger_present_timeout) > S2ST(2))
		charger_present = false;

	switch (chg_state)
	{
	case chgst_init:
		chg_init();

		chg_timer = chVTGetSystemTimeX();
		chg_state = chgst_wait_for_init;
		break;

	case chgst_wait_for_init:
		I_CHG_offset = I_CHG_filt;

		if (chVTTimeElapsedSinceX(chg_timer) > S2ST(10))
		{
			if (charge_en)
				chg_state = chgst_wait_for_charger;
			else
				chg_state = chgst_wait_for_enable;
		}
		break;

	case chgst_wait_for_enable:
		if (charge_en)
			chg_state = chgst_init;
		break;

	case chgst_wait_for_charger:
		if (charger_present)
		{
			I_CHG_offset = I_CHG_filt;

			Motor_lock = true;
			Motor_lock_timer = chVTGetSystemTimeX();

			uint16_t cmd = 0x0A01;
			comm_can_transmit_eid(ID_NMT_BOADCAST, (uint8_t *) &cmd, sizeof(cmd));  // NMT reset of charger

			cmd = 0x0A81;
			comm_can_transmit_eid(ID_NMT_BOADCAST, (uint8_t *) &cmd, sizeof(cmd));  // NMT start of charger

			cmd = 0x05;
			comm_can_transmit_eid(ID_MC_NMT, (uint8_t *) &cmd, 1);  // first hearthbeat of MC

			charger_RPDO1.I_max = I2chg(1);
			charger_RPDO1.U_max = U2chg(U_DC);
			comm_can_transmit_eid(ID_CHG_RPDO1, (uint8_t *) &charger_RPDO1, sizeof(charger_RPDO1));

			chg_counter = 0;
			chg_timer = chVTGetSystemTimeX();
			chg_state = chgst_wait_equalize;
		}
		break;

	case chgst_wait_equalize:
		if ((float) fabs(U_DC - U_CHG) < 2.)
		{
			CHRG_ON;

			chg_timer = chVTGetSystemTimeX();
			chg_state = chgst_charging;
			break;
		}

		if (chg_counter >= 4)
		{
			CHRG_OFF;
			chg_state = chgst_error;
			break;
		}

		if (chVTTimeElapsedSinceX(chg_timer) > MS2ST(250))
		{
			chg_counter++;
			comm_can_transmit_eid(ID_CHG_RPDO1, (uint8_t *) &charger_RPDO1, sizeof(charger_RPDO1));
		}
		break;


	case chgst_charging:
		if (U_DC >= 43.0)
		{
			CHRG_OFF;

			chg_state = chgst_error;
			break;
		}

		if ((I_CHG < 0.5) || (!BMS_Charge_permitted && !Stand_Alone))
		{
			CHRG_OFF;

			chg_state = chgst_charge_finished;
		}
		break;

	case chgst_charge_finished:
		chg_increment_counter();
		commands_printf("Charging finished!");

		chg_state = chgst_wait_for_reset;
		break;

	case chgst_error:
		commands_printf("Charging error!");
		CHRG_OFF;

		chg_state = chgst_wait_for_reset;
		break;

	case chgst_wait_for_reset:
		if (!charger_present)
			chg_state = chgst_wait_for_charger;
		break;

	default:
		chg_state = chgst_init;
	}
}
