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
	callibrating,
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
	float Charge_I = analog_IO.I_charge_raw - analog_IO.I_charge_offset;

	SCEN2_Battery_handler();

	switch (charge_state)
	{
	case init:
		CHG_DISABLE();
		timer = chVTGetSystemTime();
		analog_IO.I_charge_offset = 0;
		cc = 0;

		charge_state = callibrating;
	break;

	case callibrating:
		analog_IO.I_charge_offset += analog_IO.I_charge_raw;
		cc++;

		if (chVTTimeElapsedSinceX(timer) > MS2ST(500))
		{
			analog_IO.I_charge_offset /= cc;
			cc = 0;

			timer = chVTGetSystemTime();
			charge_state = wait_for_charger;
		}
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
		if ((analog_IO.U_charge > CHARGE_U_MAX) ||
			(Charge_I > CHARGE_I_MAX) ||
			(Charge_I > CHARGE_I_MIN))
		{
			CHG_DISABLE();
			errors.charger_error = true;

			charge_state = wait_for_reset;
		}

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
			errors.charger_error = true;

			cc = 0;
			timer = chVTGetSystemTime();

			charge_state = wait_for_charger;
		}
	break;
	}
}
