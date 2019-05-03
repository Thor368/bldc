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
	static bool s = false, g = false, b = false, r = false, T1 = false, T2 = false;

	if (!s && BUTTON_SILVER())
	{
		s = true;
		digital_IO.buttons.silver = 1;
		can_IO.buttons.silver = 3;
	}
	else if (s && !BUTTON_SILVER())
		s = false;

	if (!g && BUTTON_GREEN())
	{
		g = true;
		digital_IO.buttons.green = 1;
		can_IO.buttons.green = 3;
	}
	else if (s && !BUTTON_GREEN())
		g = false;

	if (!b && BUTTON_BLUE())
	{
		b = true;
		digital_IO.buttons.blue = 1;
		can_IO.buttons.blue = 3;
	}
	else if (s && !BUTTON_BLUE())
		b = false;

	if (!r && BUTTON_RED())
	{
		r = true;
		digital_IO.buttons.red = 1;
		can_IO.buttons.red = 3;
	}
	else if (s && !BUTTON_RED())
		r = false;

	if (TRIGGER_1A() && !TRIGGER_1B() && !T1)
	{
		T1 = true;

		digital_IO.trigger.T1 = false;
		can_IO.trigger.T1 = false;
	}
	else if (!TRIGGER_1A() && TRIGGER_1B() && T1)
	{
		T1 = false;

		digital_IO.trigger.T1 = false;
		can_IO.trigger.T1 = false;
	}
	else if ((TRIGGER_1A() && TRIGGER_1B()) ||
			(!TRIGGER_1A() && !TRIGGER_1B()))
	{
		errors.trigger_error = true;
		T1 = false;
		digital_IO.trigger.T1 = false;
		can_IO.trigger.T1 = false;
	}

//	if (TRIGGER_2A() && !TRIGGER_2B() && !T2)
//	{
//		T2 = true;
//
//		digital_IO.trigger.T2 = false;
//		can_IO.trigger.T2 = false;
//	}
//	else if (!TRIGGER_2A() && TRIGGER_2B() && T2)
//	{
//		T2 = false;
//
//		digital_IO.trigger.T2 = false;
//		can_IO.trigger.T2 = false;
//	}
//	else if ((TRIGGER_2A() && TRIGGER_2B()) ||
//			(!TRIGGER_2A() && !TRIGGER_2B()))
//	{
//		errors.trigger_error = true;
//		T2 = false;
//		digital_IO.trigger.T2 = false;
//		can_IO.trigger.T2 = false;
//	}
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
