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
	TRIG_SPLY_ON();
	DISP_SPLY_ON();

	digital_IO.buttons.all = 0;
	digital_IO.trigger.all = 0;
	errors.trigger_error = false;
}

void SCEN2_DIO_handler(void)
{
	static uint8_t silver_cc = 0;
	static uint8_t green_cc = 0;
	static uint8_t blue_cc = 0;
	static uint8_t red_cc = 0;

	if (digital_IO.buttons.silver == BUTTON_SILVER())
		silver_cc++;
	else
		silver_cc = 0;
	if (silver_cc >= BUTTON_DELAY)
	{
		digital_IO.buttons.silver = !BUTTON_SILVER();
		silver_cc = 0;
	}

	if (digital_IO.buttons.green == BUTTON_GREEN())
		green_cc++;
	else
		green_cc = 0;
	if (green_cc >= BUTTON_DELAY)
	{
		digital_IO.buttons.green= !BUTTON_GREEN();
		green_cc = 0;
	}

	if (digital_IO.buttons.blue == BUTTON_BLUE())
		blue_cc++;
	else
		blue_cc = 0;
	if (blue_cc >= BUTTON_DELAY)
	{
		digital_IO.buttons.blue= !BUTTON_BLUE();
		blue_cc = 0;
	}

	if (digital_IO.buttons.red == BUTTON_RED())
		red_cc++;
	else
		red_cc = 0;
	if (red_cc >= BUTTON_DELAY)
	{
		digital_IO.buttons.red= !BUTTON_RED();
		red_cc = 0;
	}


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
	else
		T1_fault++;

	static uint32_t T2_fault = 0;
	if (TRIGGER_2A() && !TRIGGER_2B())
	{
		T2_fault = 0;

		digital_IO.trigger.T2 = false;
	}
	else if (!TRIGGER_2A() && TRIGGER_2B())
	{
		T2_fault = 0;

		digital_IO.trigger.T2 = true;
	}
	else
		T2_fault++;

	if ((T1_fault > 1000) || (T2_fault > 1000))
		errors.trigger_error = true;
	else if ((T1_fault == 0) && (T2_fault == 0))
		errors.trigger_error = false;
}
