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

#include <math.h>
#include <string.h>
#include <stdio.h>

#define EXT_NTC_RES(volt)				(-volt*10000/(volt - 3.3))
#define EXT_TEMP(ch)					(1.0 / ((logf(EXT_NTC_RES(ADC_VOLTS(ch)) / 10000.0) / mc_cfg->m_ntc_motor_beta) + (1.0 / 298.15)) - 273.15)


// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private functions
static void temp_print_all(int argc, const char **argv);
static void temp_config(int argc, const char **argv);

// Private variables
const volatile mc_configuration *mc_cfg;

static volatile bool stop_now = true;
static volatile bool is_running = false;

enum {
	init
} tc_state;

float T_tank = 0, T_liqu = 0, T_ice = 0;

float T_target = T_TARGET_DEFAULT;
float T_evaporator = T_EVAPORATOR_DEFAULT;
float T_fan_start = T_CONDENSER_START;
float T_fan_stop = T_CONDENSER_STOP;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"temp_read",
			"Print all Temperature sensors",
			"",
			temp_print_all);

	terminal_register_command_callback(
			"temp_config",
			"Print/set configuration",
			"[s]",
			temp_config);

	mc_cfg = mc_interface_get_configuration();
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(temp_print_all);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

static THD_FUNCTION(my_thread, arg)
{
	(void)arg;

	chRegSetThreadName("App Custom");

	is_running = true;

	for(;;)
	{
		// Check if it is time to stop.
		if (stop_now)
		{
			is_running = false;
			return;
		}

		timeout_reset(); // Reset timeout if everything is OK.

		static float T_tank_filt = 0;
		T_tank_filt -= T_tank_filt/20;
		T_tank_filt += EXT_TEMP(ADC_IND_T1);
		T_tank = T_tank_filt/20;

		T_liqu = EXT_TEMP(ADC_IND_T2);
		T_ice = EXT_TEMP(ADC_IND_T3);

		if (T_tank > -20)  // Tank sensor present
		{
			static bool running = false;
			if (T_tank > (T_target + 0.5))
			{
				if (!running)
				{
					running = true;
					mc_interface_set_pid_speed(10000);
					commands_printf("Compressor start");
				}
			}
			else if (T_tank < (T_target - 0.5))
			{
				if (running)
				{
					running = false;
					mc_interface_release_motor();
					commands_printf("Compressor stop");
				}
			}
		}
		else
			mc_interface_release_motor();

		chThdSleepMilliseconds(100);
	}
}

// Callback function for the terminal command with arguments.
static void temp_print_all(int argc, const char **argv)
{
	(void) argc;
	(void) argv;

	commands_printf("T_tank: %.2fV -> %.1f°C", (double) ADC_VOLTS(ADC_IND_T1), (double) T_tank);
	commands_printf("T_liqu: %.2fV -> %.1f°C", (double) ADC_VOLTS(ADC_IND_T2), (double) T_liqu);
	commands_printf("T_ice: %.2fV -> %.1f°C", (double) ADC_VOLTS(ADC_IND_T3), (double) T_ice);
	commands_printf("TMOS: %.1f°C", (double) NTC_TEMP(ADC_IND_TEMP_MOS));
	commands_printf("TMOT: %.1f°C", (double) NTC_TEMP_MOTOR(mc_cfg->m_ntc_motor_beta));
}

static void temp_config(int argc, const char **argv)
{
	if (argc == 1)
	{
		commands_printf("T_target: %.1f°C", (double) T_target);
		commands_printf("T_liquifier_min: %.1f°C", (double) T_evaporator);
		commands_printf("T_fan_start: %.1fV -> %.2f°C", (double) T_fan_start);
		commands_printf("T_fan_stop: %.1f°C", (double) T_fan_stop);
	}
	else if (argc == 3)
	{
		float val;
		if (!sscanf(argv[3], "%f", &val))
		{
			commands_printf("Invalid value");
			return;
		}

		if (strcmp(argv[2], "T_target"))
			T_target = val;
		else if (strcmp(argv[2], "T_evaporator"))
			T_evaporator = val;
		else if (strcmp(argv[2], "T_fan_start"))
			T_fan_start = val;
		else if (strcmp(argv[2], "T_fan_stop"))
			T_fan_stop = val;

		commands_printf("Value set");
	}
	else
		commands_printf("Wrong argument count");

	commands_printf(" ");
}
