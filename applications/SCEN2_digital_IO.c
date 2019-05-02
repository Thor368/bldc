/*
 * digital_IO.c
 *
 *  Created on: 25.04.2019
 *      Author: alexander.schroeder
 */

#include "SCEN2_types.h"
#include "SCEN2_settings.h"
#include "SCEN2_CAN_IDs.h"

#include "hal.h"
#include "mc_interface.h"

Digital_IO_t digital_IO;

void SCEN2_DIO_init(void)
{
	CHG_ENABLE();
	CAN_CP_ON();

	digital_IO.buttons.silver = 0;
	digital_IO.buttons.green = 0;
	digital_IO.buttons.blue = 0;
	digital_IO.buttons.red = 0;

	digital_IO.trigger.T1A = false;
	digital_IO.trigger.T1B = false;
	digital_IO.trigger.T2A = false;
	digital_IO.trigger.T2B = false;
}

void SCEN2_DIO_handler(void)
{
	if ((!digital_IO.buttons.silver) && (BUTTON_SILVER()))
	{
		digital_IO.buttons.silver = 1;
		can_IO.buttons.silver = 3;
	}

	if ((!digital_IO.buttons.green) && (BUTTON_GREEN()))
	{
		digital_IO.buttons.green = 1;
		can_IO.buttons.green = 3;
	}

	if ((!digital_IO.buttons.blue) && (BUTTON_BLUE()))
	{
		digital_IO.buttons.blue = 1;
		can_IO.buttons.blue = 3;
	}

	if ((!digital_IO.buttons.red) && (BUTTON_RED()))
	{
		digital_IO.buttons.red = 1;
		can_IO.buttons.red = 3;
	}
}

void SCEN2_button_handler(void)
{
	if (digital_IO.buttons.silver)
	{
		digital_IO.buttons.silver = 0;

		// do something
	}

	if (digital_IO.buttons.green)
	{
		digital_IO.buttons.green= 0;

		// do something
	}

	if (digital_IO.buttons.blue)
	{
		digital_IO.buttons.blue= 0;

		// do something
	}

	if (digital_IO.buttons.red)
	{
		digital_IO.buttons.red = 0;

		// do something
	}
}
