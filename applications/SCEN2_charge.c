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
#include "SCEN2_charge.h"


enum
{
	init,
	wait_for_charger,
	wait_for_HMI_acknowledgement,
	charging,
	wait_for_reset
} charge_state;

Charge_mode_t charge_mode;

void SCEN2_Charge_init(void)
{
	SCEN2_Battery_init();

	charge_state = init;
	charge_mode = no_charger;
}

uint8_t SCEN2_charge_state(void)
{
	return charge_state;
}


void SCEN2_Charge_handler(void)
{
	static systime_t timer;
	static uint32_t cc;
	static float offset_filter = 0;

	SCEN2_Battery_handler();

	if (charge_mode != charger_detected_with_ACK)
	{
		offset_filter += analog_IO.I_charge_raw;
		offset_filter -= offset_filter/10;
		analog_IO.I_charge_offset = offset_filter/9;
	}

	switch (charge_state)
	{
	case init:
		CHG_DISABLE();
		timer = chVTGetSystemTime();
		cc = 0;

		charge_state = wait_for_charger;
	break;

	case wait_for_charger:
		if (chVTTimeElapsedSinceX(timer) > MS2ST(1))
		{
			timer = chVTGetSystemTime();

			if (analog_IO.U_charge >= CHARGE_U_DETECT)
				cc++;
			else
				cc = 0;

			if (cc >= 500)
			{
				charge_mode = charger_detected_no_ACK;
				timer = chVTGetSystemTime();
				cc = 0;

				charge_state = wait_for_HMI_acknowledgement;
			}
		}
	break;

	case wait_for_HMI_acknowledgement:
		if (chVTTimeElapsedSinceX(timer) > MS2ST(1))
		{
			timer = chVTGetSystemTime();

			if (analog_IO.U_charge < CHARGE_U_DETECT)
				cc++;
			else
				cc = 0;

			if (cc >= 500)
				charge_mode = no_charger;
		}

		if (charge_mode == charger_detected_with_ACK)
		{
			CHG_ENABLE();

			timer = chVTGetSystemTime();

			charge_state = charging;
		}
		else if (charge_mode == no_charger)
		{
			cc = 0;
			timer = chVTGetSystemTime();

			charge_state = wait_for_charger;
		}
	break;

	case charging:
//		if ((chVTTimeElapsedSinceX(timer) > MS2ST(100)) &&
//			((analog_IO.U_charge > CHARGE_U_MAX) ||
//			 (Charge_I > CHARGE_I_MAX) ||
//			 (Charge_I < CHARGE_I_MIN)))
//		{
//			CHG_DISABLE();
//			errors.charger_error = true;
//
//			charge_state = wait_for_reset;
//		}

		if (charge_mode != charger_detected_with_ACK)
		{
			CHG_DISABLE();

			cc = 0;
			timer = chVTGetSystemTime();

			charge_state = wait_for_charger;
		}
	break;

	case wait_for_reset:
		if (charge_mode == no_charger)
		{
			errors.charger_error = false;

			cc = 0;
			timer = chVTGetSystemTime();

			charge_state = wait_for_charger;
		}
	break;
	}
}
