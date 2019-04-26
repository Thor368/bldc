/*
 * SCEN2_charge.c
 *
 *  Created on: 26.04.2019
 *      Author: alexander.schroeder
 */

#include "SCEN2_types.h"
#include "SCEN2_settings.h"
#include "SCEN2_battery.h"

#include "hal.h"
#include "mc_interface.h"


enum
{
	init,
	callibrating,
	wait_for_charger,
	charging,
	stop_charging,
	emergency_stop,
	wait_for_disconnect
} charge_state;

float chg_offset = 0;


void SCEN2_Charge_init(void)
{
	SCEN2_Battery_init();

	charge_state = init;
}

void SCEN2_Carge_handler(void)
{
	static systime_t chg_timer;
	static uint32_t chg_cc;
	float Charge_I = analog_IO.I_charge_raw - chg_offset;

	SCEN2_Battery_handler();

	switch (charge_state)
	{
	case init:
		CHG_DISABLE();
		chg_timer = chVTGetSystemTime();
		chg_offset = 0;
		chg_cc = 0;

		charge_state = callibrating;
	break;

	case callibrating:
		chg_offset += analog_IO.I_charge_raw;
		chg_cc++;

		if (chVTTimeElapsedSinceX(chg_timer) > MS2ST(500))
		{
			chg_offset /= chg_cc;
			chg_cc = 0;

			chg_timer = chVTGetSystemTime();
			charge_state = wait_for_charger;
		}
	break;

	case wait_for_charger:
		if (chVTTimeElapsedSinceX(chg_timer) > MS2ST(1))
		{
			chg_timer = chVTGetSystemTime();

			if (((analog_IO.U_charge > batteries[0].U) ||
				 (analog_IO.U_charge > batteries[1].U)) &&
				(analog_IO.U_charge < CHARGE_U_MAX))
				chg_cc++;
			else
				chg_cc = 0;

			if ((chg_cc >= 500) &&
				(batteries[0].SOC <= CHARGE_SOC_START) &&
				(batteries[1].SOC <= CHARGE_SOC_START))
			{
				// ToDo: switch batteries into sleep mode

				CHG_ENABLE();

				charge_state = charging;
			}
		}
	break;

	case charging:
		if ((batteries[0].OK_to_charge) &&
			((batteries[1].U - batteries[0].U) >= CHARGE_U_DELTA))
		{
			// ToDo: switch bat 0 into charge mode
		}
		else
		{
			// ToDo: switch bat 0 into sleep mode
		}

		if ((batteries[1].OK_to_charge) &&
			((batteries[0].U - batteries[1].U) >= CHARGE_U_DELTA))
		{
			// ToDo: switch bat 1 into charge mode
		}
		else
		{
			// ToDo: switch bat 1 into sleep mode
		}

		if ((analog_IO.U_charge > CHARGE_U_MAX) ||
			(analog_IO.U_charge < CHARGE_U_MIN) ||
			(Charge_I > CHARGE_I_MAX) ||
			(Charge_I > CHARGE_I_MIN))
		{
			charge_state = emergency_stop;
		}

		if ((batteries[0].SOC >= 1) &&
			(batteries[1].SOC >= 1) &&
			(Charge_I <= CHARGE_I_TRICKLE))
		{
			charge_state = stop_charging;
		}

	break;

	case stop_charging:
		// ToDo: switch batteries into sleep mode

		CHG_DISABLE();

		charge_state = wait_for_charger;
	break;

	case emergency_stop:
		CHG_DISABLE();

		// ToDo: switch batteries into sleep mode

		chg_timer = chVTGetSystemTime();
		chg_cc = 0;

		charge_state = wait_for_disconnect;
	break;

	case wait_for_disconnect:
		if (chVTTimeElapsedSinceX(chg_timer) > MS2ST(1))
		{
			chg_timer = chVTGetSystemTime();

			if (analog_IO.U_charge < CHARGE_U_MIN)
				chg_cc++;
			else
				chg_cc = 0;

			if (chg_cc >= 5000)
				charge_state = wait_for_charger;
		}
	break;
	}
}
