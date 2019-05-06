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
	CAN_STB_LO();
	CAN_CP_ON();

	digital_IO.buttons.all = 0;
	digital_IO.trigger.all = 0;
	errors.trigger_error = false;
}

void SCEN2_DIO_handler(void)
{
	digital_IO.buttons.silver = !BUTTON_SILVER();
	digital_IO.buttons.green = !BUTTON_GREEN();
	digital_IO.buttons.blue = !BUTTON_BLUE();
	digital_IO.buttons.red = !BUTTON_RED();

	static uint32_t T1_fault = 0;
	if (TRIGGER_1A() && !TRIGGER_1B())
	{
		T1_fault = 0;

		digital_IO.trigger.T1 = false;
	}
	else if (!TRIGGER_1A() && TRIGGER_1B())
	{
		T1_fault = 0;

		digital_IO.trigger.T1 = true;
	}
	else if (T1_fault >= 1000)
	{
		errors.trigger_error = true;
		digital_IO.trigger.T1 = false;
	}
	else if ((TRIGGER_1A() && TRIGGER_1B()) ||
			 (!TRIGGER_1A() && !TRIGGER_1B()))
		T1_fault++;

	// T2 disabled
}
