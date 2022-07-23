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

#include "temp_control.h"
#include "temp_compressor.h"

#include "app.h"
#include "ch.h"
#include "hal.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "temp_HMI.h"

#define EXT_NTC_RES(volt)				(-volt*10000/(volt - 3.3))
#define EXT_TEMP(ch)					(1.0 / ((logf(EXT_NTC_RES(ADC_VOLTS(ch)) / 10000.0) / mc_cfg->m_ntc_motor_beta) + (1.0 / 298.15)) - 273.15)

#define TIM_PERIOD						3359l

#define SET_PUMP(rel)					{if (rel >= 1) TIM4->CCR1 = TIM_PERIOD - 1; else TIM4->CCR1 = TIM_PERIOD*rel - 1;}
#define SET_FAN(rel)					{if (rel >= 1) TIM4->CCR2 = TIM_PERIOD - 1; else TIM4->CCR2 = TIM_PERIOD*rel - 1;}


// GLOBAL VARS
bool manual_mode = false;

float T_tank = 0, T_return = 0, I_Comp = 0;
float U_fan = 0, U_DC = 0;

float T_target = T_TARGET_DEFAULT;
float I_fan_ramp_start = I_FAN_RAMP_START;
float I_fan_ramp_end = I_FAN_RAMP_END;
float U_fan_min = U_FAN_MIN;
float U_fan_max = U_FAN_MAX;
float U_pump_std = U_PUMP_STD;
float T_hyst_pos = T_HYST_POS;
float T_hyst_neg = T_HYST_NEG;
float RPM_min = RPM_MIN, RPM_max = RPM_MAX;
float RPM_P = 150, RPM_I = 5, RPM_D = 0, RPM_D_t = 10;
float dt_plot = -1.0;
bool FAN_PWM_invert = false;


// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private variables
const volatile mc_configuration *mc_cfg;


volatile bool stop_now = true;
volatile bool is_running = false;

void write_conf(void)
{
	eeprom_var eep_conf;
	int pp = 0;

	eep_conf.as_u32 = REVOLTER_CONF_VERSION;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = T_target;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = I_fan_ramp_start;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = I_fan_ramp_end;
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

	eep_conf.as_float = RPM_min;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = RPM_max;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = RPM_P;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = RPM_I;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = RPM_D;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_float = RPM_D_t;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_u32 = FAN_PWM_invert;
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
	I_fan_ramp_start = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	I_fan_ramp_end = eep_conf.as_float;

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

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	RPM_min = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	RPM_max = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	RPM_P = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	RPM_I = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	RPM_D = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	RPM_D_t = eep_conf.as_float;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	FAN_PWM_invert = eep_conf.as_u32;
}

// start plotting
static void temp_plot(int argc, const char **argv)
{
	(void) argc;
	(void) argv;

	commands_init_plot("Time", "Temperature");
	commands_plot_add_graph("Tank Temperature");
	commands_plot_set_graph(0);

	dt_plot = 0.0;

	commands_printf("Plot init.");
}

// Callback function for the terminal command with arguments.
static void temp_print_all(int argc, const char **argv)
{
	(void) argc;
	(void) argv;

	commands_printf("T_tank: %.1f°C", (double) T_tank);
	commands_printf("T_return: %.1f°C", (double) T_return);
	commands_printf("TMOS: %.1f°C", (double) NTC_TEMP(ADC_IND_TEMP_MOS));
	commands_printf("TMOT: %.1f°C", (double) NTC_TEMP_MOTOR(mc_cfg->m_ntc_motor_beta));
	commands_printf("---------------------");
	commands_printf("U_DC: %.2fV", (double) U_DC);
	commands_printf("Pump_PWM: %d", TIM4->CCR1);
	commands_printf("Fan_PWM: %d\n", TIM4->CCR2);
}

static void temp_config(int argc, const char **argv)
{
	if (argc == 1)
	{
		commands_printf("T_target: %.1f°C", (double) T_target);
		commands_printf("I_fan_ramp_start: %.1fA", (double) I_fan_ramp_start);
		commands_printf("I_fan_ramp_end: %.1fA", (double) I_fan_ramp_end);
		commands_printf("U_fan_min: %.1fV", (double) U_fan_min);
		commands_printf("U_fan_max: %.1fV", (double) U_fan_max);
		commands_printf("U_pump_std: %.1fV", (double) U_pump_std);
		commands_printf("T_hyst_pos: %.1f°C", (double) T_hyst_pos);
		commands_printf("T_hyst_neg: %.1f°C", (double) T_hyst_neg);
		commands_printf("RPM_min: %.1fRPM", (double) RPM_min);
		commands_printf("RPM_max: %.1fRPM", (double) RPM_max);
		commands_printf("RPM_P: %.1f", (double) RPM_P);
		commands_printf("RPM_I: %.1f", (double) RPM_I);
		commands_printf("RPM_D: %.1f", (double) RPM_D);
		commands_printf("RPM_D_t: %.1f", (double) RPM_D_t);
		commands_printf("FAN_PWM_invert: %d", FAN_PWM_invert);
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
		else if (!strcmp(argv[1], "I_fan_ramp_start"))
			I_fan_ramp_start = val;
		else if (!strcmp(argv[1], "I_fan_ramp_end"))
			I_fan_ramp_end = val;
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
		else if (!strcmp(argv[1], "U_fan"))
			U_fan = val;
		else if (!strcmp(argv[1], "RPM_min"))
			RPM_min = val;
		else if (!strcmp(argv[1], "RPM_max"))
			RPM_max = val;
		else if (!strcmp(argv[1], "RPM_P"))
			RPM_P = val;
		else if (!strcmp(argv[1], "RPM_I"))
			RPM_I = val;
		else if (!strcmp(argv[1], "RPM_D"))
			RPM_D = val;
		else if (!strcmp(argv[1], "RPM_D_t"))
			RPM_D_t = val;
		else if (!strcmp(argv[1], "FAN_PWM_invert"))
		{
			TIM4->CCER &= (uint16_t)~TIM_CCER_CC2P;
			FAN_PWM_invert = (val > 0.5);

			if (FAN_PWM_invert)
				TIM4->CCER |= TIM_CCER_CC2P;
		}
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

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
	U_fan = 0;
	manual_mode = false;
	dt_plot = -1;

	read_conf();
	compressor_init();
	HMI_init();

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

	terminal_register_command_callback(
			"manual_mode",
			"Activate manual override",
			"",
			temp_manual);

	terminal_register_command_callback(
			"temp_plot",
			"Start ploting",
			"",
			temp_plot);

	mc_cfg = mc_interface_get_configuration();

	// Power output config
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	// Channel 1 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  // Pump PWM
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	if (FAN_PWM_invert)
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  // Fan PWM
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_Cmd(TIM4, ENABLE);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(temp_print_all);
	mc_interface_set_pwm_callback(0);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}

	TIM_Cmd(TIM4, DISABLE);

	sdStop(&SD6);
	palSetPadMode(GPIOC, 6, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(GPIOC, 7, PAL_MODE_INPUT_PULLUP);
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

		static float T_tank_filt = NAN;
		if (isnan(T_tank_filt))
			T_tank_filt = EXT_TEMP(ADC_IND_T1)*20;
		T_tank_filt -= T_tank_filt/20;
		T_tank_filt += EXT_TEMP(ADC_IND_T1);
		T_tank = T_tank_filt/20;

		static float T_return_filt = NAN;
		if (isnan(T_return_filt))
			T_return_filt = EXT_TEMP(ADC_IND_T2)*20;
		T_return_filt -= T_return_filt/20;
		T_return_filt += EXT_TEMP(ADC_IND_T2);
		T_return = T_return_filt/20;

		static float U_DC_filt = NAN;
		if (isnan(U_DC_filt))
			U_DC_filt = GET_INPUT_VOLTAGE()*10;
		U_DC_filt -= U_DC_filt/10;
		U_DC_filt += GET_INPUT_VOLTAGE();
		U_DC = U_DC_filt/10;

		static float I_Comp_filt = 0;
		I_Comp_filt -= I_Comp_filt/25;
		I_Comp_filt += mc_interface_get_tot_current_filtered();
		I_Comp = I_Comp_filt/25;

		static systime_t log_t;
		if (chVTTimeElapsedSinceX(log_t) >= MS2ST(250))
		{
			log_t = chVTGetSystemTime();

			if (dt_plot >= 0)
			{
				commands_send_plot_points(dt_plot, T_tank);
				dt_plot += 0.25;
			}
		}

		sm_HMI();
		sm_compressor();

		if (FAN_PWM_invert)
		{
			SET_FAN(U_fan/12);
		}
		else
		{
			SET_FAN(U_fan/U_DC);
		}

		SET_PUMP(U_pump_std/U_DC);

		chThdSleepMilliseconds(100);
	}
}
