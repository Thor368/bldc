/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "LTC6804_handler.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private functions
static void BMS_cb_status(int argc, const char **argv);
static void BMS_charg_en(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

static volatile bool charge_en = false, charging = false;
static volatile float U_DC = 0;
static volatile float U_CHG = 0;
static volatile float I_CHG = 0, I_CHG_offset = 0;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
	CHRG_OFF;

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"bms_status",
			"Show BMS status",
			"",
			BMS_cb_status);

	terminal_register_command_callback(
			"bms_charge_enable",
			"Enable/disable charging (\"\" = report, 0 = off, 1 = on)",
			"[d]",
			BMS_charg_en);

	charge_en = true;
	charging = false;

	LTC_handler_Init();
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{
	CHRG_OFF;

	terminal_unregister_callback(BMS_cb_status);
	terminal_unregister_callback(BMS_charg_en);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf)
{
	(void)conf;
}

static THD_FUNCTION(my_thread, arg)
{
	(void)arg;
	float U_DC_filt = 0;
	float U_CHG_filt = 0;
	float I_CHG_filt = 0;

	chRegSetThreadName("App Custom");

	is_running = true;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.
		LTC_handler();

		U_DC_filt -= U_DC_filt/10;
		U_DC_filt += GET_INPUT_VOLTAGE();
		U_DC = U_DC_filt/10;

		U_CHG_filt -= U_CHG_filt/10;
		U_CHG_filt += GET_VOLTAGE(6);
		U_CHG = U_CHG_filt/10;

		I_CHG_filt -= I_CHG_filt/10;
		I_CHG_filt += GET_VOLTAGE(7);
		I_CHG = (I_CHG_filt - I_CHG_offset)/0.088;


		if (!charging)
			I_CHG_offset = I_CHG_filt;


		if (charge_en)
		{
			if ((U_CHG > 30.0) && (U_DC < 40.0))
			{
				charging = true;
				CHRG_ON;
			}
			else if (U_DC > 41.0)
			{
				charging = false;
				CHRG_OFF;
			}
		}
		else
		{
			charging = false;
			CHRG_OFF;
		}

		chThdSleepMilliseconds(10);
	}
}

static void BMS_cb_status(int argc, const char **argv)
{
	(void) argc;
	(void) argv;

	commands_printf("---CHARGEPORT---");
	commands_printf("Port voltage: %.1fV", (double) U_CHG);
	commands_printf("Port current: %.2fV", (double) I_CHG);
	commands_printf("Port current offset: %.2fV", (double) I_CHG_offset);

	if (charging)
		commands_printf("Chargeport: Charging");
	else if (charge_en)
		commands_printf("Chargeport: Enabled");
	else
		commands_printf("Chargeport: Disabled");

	commands_printf("\n---BMS---");
	commands_printf("State: %d", BMS.Status);
	commands_printf("Present: %d", BMS.BMS_present);
	commands_printf("Balance permission: %d", BMS.Balance_Permission);
	commands_printf("Balance derating: %d", BMS.Balance_derating);

	commands_printf("\nSELFCHECK");
	commands_printf("Cell test passed: %d", BMS.Cell_Test_Passed);
	commands_printf("GPIO test passed: %d", BMS.GPIO_Test_Passed);
	commands_printf("Status test passed: %d", BMS.Status_Test_Passed);
	commands_printf("MUX test passed: %d", BMS.MUX_Test_Passed);
	commands_printf("Secondary reference OK: %d", BMS.Secondary_Reference_OK);
	commands_printf("Internal temperature OK: %d", BMS.Int_Temp_OK);
	commands_printf("VA OK: %d", BMS.VA_OK);
	commands_printf("VD OK: %d", BMS.VD_OK);
	commands_printf("Wrong cell count: %d", BMS.Wrong_Cell_Count);
	commands_printf("Health: %d", BMS.Health);
	for (uint8_t i = 0; i < 12; i++)
		commands_printf("Open test #%d %.3fV %.3fV", i+1, (double) BMS.Cell_Source_U[i], (double) BMS.Cell_Sink_U[i]);

	commands_printf("\nCELLS");
	commands_printf("Pack voltage: %.2fV", (double) BMS.Cell_All_U);
	commands_printf("Cell average voltage: %.3fV", (double) BMS.Cell_Avr_U);
	commands_printf("Cell count: %d", BMS.Cell_Count);
	for (uint8_t i = 0; i < 12; i++)
	{
		char flags[23] = "";

		if (i == BMS.Cell_Min_index)
			strcat(flags, "MIN ");

		if (i == BMS.Cell_Max_index)
			strcat(flags, "MAX ");

		if (BMS.Cell_OV[i])
			strcat(flags, "OV ");

		if (BMS.Cell_UV[i])
			strcat(flags, "UV ");

		if (BMS.Cell_Bleed[i])
			strcat(flags, "DIS ");

		if (BMS.Open_Cell_Connection[i])
			strcat(flags, "OPEN");

		commands_printf("Cell %2d: %.3fV %s", i+1, (double) BMS.Cell_U[i], flags);
	}

	commands_printf("\nTEMPERATURES");
	commands_printf("Internal temperature: %.1f�C", (double) BMS.Int_Temp);
	for (uint8_t i = 0; i < 5; i++)
		commands_printf("External temperature #%d: %.1f�C", i+1, (double) BMS.Temp_sensors[i]);
}

static void BMS_charg_en(int argc, const char **argv)
{
	if (argc == 1)
		commands_printf("charge enable = %d", charge_en);
	else if (argc == 2)
	{
		int ret;
		if (!sscanf(argv[1], "%d", &ret))
			commands_printf("Illegal argument!");
		if (ret)
			charge_en = true;
		else
			charge_en = false;
		commands_printf("charge enable = %d", charge_en);
	}
	else
		commands_printf("Wrong argument count!");
}


//static void terminal_test(int argc, const char **argv)
//{
//	if (argc == 2) {
//		int d = -1;
//		sscanf(argv[1], "%d", &d);
//
//		commands_printf("You have entered %d", d);
//
//		// For example, read the ADC inputs on the COMM header.
//		commands_printf("ADC1: %.2f V ADC2: %.2f V",
//				(double)ADC_VOLTS(ADC_IND_EXT), (double)ADC_VOLTS(ADC_IND_EXT2));
//	} else {
//		commands_printf("This command requires one argument.\n");
//	}
//}
