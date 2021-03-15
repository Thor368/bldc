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
#include "mcpwm_foc.h"
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

// Private variables
const volatile mc_configuration *mc_cfg;

volatile bool stop_now = true;
volatile bool is_running = false;
volatile bool debug_on = false;

volatile uint8_t fan_pwm = 0;
volatile uint8_t pump_pwm = 0;

enum
{
	cmp_init,
	cmp_wait_for_start,
	cmp_ramp_up,
	cmp_failed_start,
	cmp_running,
	cmp_equalize
} compressor_state;

bool compressor_call = false;
bool manual_mode = false;

float T_tank = 0, T_cond = 0;
float U_fan = 0;

float T_target = T_TARGET_DEFAULT;
float T_fan_ramp_start = T_FAN_RAMP_START;
float T_fan_ramp_end = T_FAN_RAMP_END;
float U_fan_min = U_FAN_MIN;
float U_fan_max = U_FAN_MAX;
float U_pump_std = U_PUMP_STD;
float T_hyst_pos = T_HYST_POS;
float T_hyst_neg = T_HYST_NEG;


void write_conf(void)
{
	eeprom_var eep_conf;
	int pp = 0;

	eep_conf.as_u32 = REVOLTER_CONF_VERSION;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = T_target;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = T_fan_ramp_start;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = T_fan_ramp_end;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = U_fan_min;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = U_fan_max;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = U_pump_std;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = T_hyst_pos;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = T_hyst_neg;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);
}

void read_conf(void)
{
	eeprom_var eep_conf;
	int pp = 0;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	if (eep_conf.as_u32 != REVOLTER_CONF_VERSION)
		write_conf();

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	T_target = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	T_fan_ramp_start = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	T_fan_ramp_end = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	U_fan_min = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	U_fan_max = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	U_pump_std = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	T_hyst_pos = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	T_hyst_neg = eep_conf.as_float;
}

// Callback function for the terminal command with arguments.
static void temp_print_all(int argc, const char **argv)
{
	(void) argc;
	(void) argv;

	commands_printf("T_tank: %.1f°C", (double) T_tank);
	commands_printf("T_cond: %.1f°C", (double) T_cond);
	commands_printf("TMOS: %.1f°C", (double) NTC_TEMP(ADC_IND_TEMP_MOS));
	commands_printf("TMOT: %.1f°C", (double) NTC_TEMP_MOTOR(mc_cfg->m_ntc_motor_beta));
	commands_printf("---------------------");
	commands_printf("Pump_PWM: %d", pump_pwm);
	commands_printf("Fan_PWM: %d\n", fan_pwm);
}

static void temp_config(int argc, const char **argv)
{
	if (argc == 1)
	{
		commands_printf("T_target: %.1f°C", (double) T_target);
		commands_printf("T_fan_ramp_start: %.1f°C", (double) T_fan_ramp_start);
		commands_printf("T_fan_ramp_end: %.1f°C", (double) T_fan_ramp_end);
		commands_printf("T_fan_min: %.1fV", (double) U_fan_min);
		commands_printf("T_fan_max: %.1fV", (double) U_fan_max);
		commands_printf("U_pump_std: %.1fV", (double) U_pump_std);
		commands_printf("T_hyst_pos: %.1f°C", (double) T_hyst_pos);
		commands_printf("T_hyst_neg: %.1f°C", (double) T_hyst_neg);
		commands_printf("Debugging print: %d", debug_on);
	}
	else if (argc == 3)
	{
		float val;
		if (sscanf(argv[2], "%f", &val) != 1)
		{
			commands_printf("Invalid value");
			return;
		}

		bool success = true;
		if (!strcmp(argv[1], "T_target"))
			T_target = val;
		else if (!strcmp(argv[1], "T_fan_ramp_start"))
			T_fan_ramp_start = val;
		else if (!strcmp(argv[1], "T_fan_ramp_end"))
			T_fan_ramp_end = val;
		else if (!strcmp(argv[1], "U_fan_min"))
			U_fan_min = val;
		else if (!strcmp(argv[1], "U_fan_max"))
			U_fan_max = val;
		else if (!strcmp(argv[1], "U_pump_std"))
			U_pump_std = val;
		else if (!strcmp(argv[1], "T_hyst_pos"))
			T_hyst_pos = val;
		else if (!strcmp(argv[1], "T_hyst_neg"))
			T_hyst_neg = val;
		else if (!strcmp(argv[1], "debug_on"))
			debug_on = true;
		else if (!strcmp(argv[1], "U_fan"))
			U_fan = val;
		else
			success = false;

		if (success)
		{
			commands_printf("OK.\n");
			write_conf();
		}
		else
			commands_printf("Unknown parameter\n");
	}
	else
		commands_printf("Wrong argument count\n");

	commands_printf(" ");
}

static void temp_manual(int argc, const char **argv)
{
	(void) argc;
	(void) argv;
	manual_mode = true;
	commands_printf("Manual override activated.\n");
}

static void pwm_callback(void)
{
	static uint8_t cc = 0;
	if (cc >= 255)
	{
		cc = 0;
		PUMP_ON();
		FAN1_ON();
		FAN2_ON();
	}
	else
		cc++;

	if (cc == pump_pwm)
		PUMP_OFF();
	if (cc == fan_pwm)
	{
		FAN1_OFF();
		FAN2_OFF();
	}
}

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
	PUMP_OFF();
	FAN1_OFF();
	FAN2_OFF();

	pump_pwm = 0;
	fan_pwm = 0;

	U_fan = 0;
	manual_mode = false;
	debug_on = false;
	compressor_state = cmp_init;

	read_conf();

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	// Register PWM callback for soft PWM
	mc_interface_set_pwm_callback(pwm_callback);

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

	terminal_register_command_callback(
			"manual_mode",
			"Activate manual override",
			"",
			temp_manual);

	mc_cfg = mc_interface_get_configuration();
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(temp_print_all);
	mc_interface_set_pwm_callback(0);

	pump_pwm = 0;
	fan_pwm = 0;


	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}

	PUMP_OFF();
	FAN1_OFF();
	FAN2_OFF();
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

void sm_compressor(void)
{
	static systime_t cmp_timer = 0, cmp_counter = 0;

	if (manual_mode)
		compressor_state = cmp_wait_for_start;

	switch(compressor_state)
	{
	case cmp_init:
		cmp_timer = chVTGetSystemTime();
		mc_interface_release_motor();
		compressor_state = cmp_wait_for_start;
		break;

	case cmp_wait_for_start:
		if (compressor_call && !manual_mode)
		{
			cmp_timer = chVTGetSystemTime();
			U_fan = U_fan_min;
			mc_interface_set_current(5.);
			compressor_state = cmp_ramp_up;
			cmp_counter = 0;
		}
		break;

	case cmp_ramp_up:
//		mcpwm_foc_set_openloop(mc_cfg->l_current_max, ST2MS(chVTTimeElapsedSinceX(cmp_timer))*10000/t_RAMP_TIME);

//		if (chVTTimeElapsedSinceX(cmp_timer) >= MS2ST(t_RAMP_TIME))
		if (mc_interface_get_rpm() >= 7500)
		{
			mc_interface_set_pid_speed(7500);
			compressor_state = cmp_running;
		}
		else if (chVTTimeElapsedSinceX(cmp_timer) >= MS2ST(500) || (mc_interface_get_fault() != FAULT_CODE_NONE))
		{
			mc_interface_release_motor();
			cmp_timer = chVTGetSystemTime();
			cmp_counter++;
			U_fan = U_fan_min;
			compressor_state = cmp_failed_start;
		}
		break;

	case cmp_failed_start:
		if (chVTTimeElapsedSinceX(cmp_timer) >= MS2ST(5000))
		{
			if (cmp_counter >= 3)
			{
				compressor_state = cmp_equalize;
				cmp_timer = chVTGetSystemTime();
			}
			else
			{
				cmp_timer = chVTGetSystemTime();
				mc_interface_set_current(5.);
				compressor_state = cmp_ramp_up;
			}
		}
		break;

	case cmp_running:
		if ((T_cond < -20) || (T_cond > 50))
			U_fan = U_fan_max;
		else if (T_cond < 35)
			U_fan = U_fan_min;

		if (!compressor_call)
		{
			mc_interface_release_motor();
			cmp_timer = chVTGetSystemTime();
			U_fan = U_fan_min;
			compressor_state = cmp_equalize;
		}
		break;

	case cmp_equalize:
		if (chVTTimeElapsedSinceX(cmp_timer) >= S2ST(120))
		{
			U_fan = 0;
			compressor_state = cmp_wait_for_start;
		}
		break;

	default:
		compressor_state = cmp_init;
		break;
	}
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

		static float T_cond_filt = 0;
		T_cond_filt -= T_cond_filt/20;
		T_cond_filt += EXT_TEMP(ADC_IND_T2);
		T_cond = T_cond_filt/20;

		static systime_t log_t;
		if (chVTTimeElapsedSinceX(log_t) >= S2ST(1) && debug_on)
		{
			log_t = chVTGetSystemTime();
			commands_printf("T_tank: %.1f°C", (double) T_tank);
		}

		if (T_tank > -20)  // Tank sensor present
		{
			if (T_tank > (T_target + T_hyst_pos))
				compressor_call = true;
			else if (T_tank < (T_target - T_hyst_neg))
				compressor_call = false;
		}
		else
		{
			compressor_call = false;
			mc_interface_release_motor();
		}

		sm_compressor();

#ifdef BRIDGED_12V
			fan_pwm = U_fan/GET_INPUT_VOLTAGE()*255;
			pump_pwm = U_pump_std/GET_INPUT_VOLTAGE()*255;
#else
			fan_pwm = U_fan/12*255;
			pump_pwm = U_pump_std/12*255;
#endif

		chThdSleepMilliseconds(100);
	}
}
