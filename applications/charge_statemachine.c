/*
 * charge_statemachine.c
 *
 *  Created on: 07.10.2020
 *      Author: alexander.schroeder
 */

#include "hw.h"
#include "hal.h"

#include "maraneo_vars.h"
#include "charge_statemachine.h"
#include "LTC6804_handler.h"

#include "commands.h"  // debug


uint32_t chg_timer;
volatile bool charge_en;
CHG_state_t chg_state;

void chg_init()
{
	CHRG_OFF;

	chg_state = chgst_init;
	charge_en = true;
}


void charge_statemachine()
{
	if (!charge_en)
		chg_state = chgst_wait_for_enable;

	switch (chg_state)
	{
	case chgst_init:
		chg_init();

		chg_timer = chVTGetSystemTimeX();
		chg_state = chgst_wait_for_init;
	break;

	case chgst_wait_for_init:
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
		I_CHG_offset = I_CHG_filt;

		if (U_CHG > 20.0)
		{
			Motor_lock = true;
			Motor_lock_timer = chVTGetSystemTimeX();
		}

		if ((U_CHG > U_DC) && (U_DC < 40.0))
		{
			CHRG_ON;

			chg_timer = chVTGetSystemTimeX();
			chg_state = chgst_wait_1s;
		}
	break;

	case chgst_wait_1s:
		if (U_DC >= 41.0)
		{
			CHRG_OFF;

			chg_state = chgst_charge_finished;
		}

		if (chVTTimeElapsedSinceX(chg_timer) > S2ST(1))
			chg_state = chgst_charging;
	break;

	case chgst_charging:
		if ((U_DC >= 41.0) || (I_CHG < 0.5) || (!BMS_Charge_permitted))
		{
			CHRG_OFF;

			chg_state = chgst_charge_finished;
		}
	break;

	case chgst_charge_finished:
		// count chagre cycle

	case chgst_error:
		if (U_CHG < 20.0)
			chg_state = chgst_wait_for_charger;
	break;

	default:
		chg_state = chgst_init;
	}
}
