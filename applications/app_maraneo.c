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

#include <math.h>

// Some useful includes
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "utils.h"
#include "hw.h"

#include "mar_vars.h"
#include "LTC6804_handler.h"
#include "mar_charge_statemachine.h"
#include "mar_CAN.h"
#include "mar_terminal.h"
#include "mar_safety.h"

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private functions
static void pwm_callback(void);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

volatile float discharge_SoC;
volatile bool discharge_enable;

volatile float U_DC = 0, U_DC_filt = 0;
volatile float U_CHG = 0, U_CHG_filt = 0;
volatile float I_CHG = 0, I_CHG_filt = 0, I_CHG_offset = 0;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
	SHDN_OFF;
	CAN_OFF;
	BAT_ON;
	CHRG_OFF;

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	mc_interface_set_pwm_callback(pwm_callback);

	// Terminal commands for the VESC Tool terminal can be registered.

	mar_Init();
	safety_Init();
	LTC_handler_Init();
	mar_read_config();
	CAN_Init();
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{
	CHRG_OFF;
	CAN_ON;
	BAT_OFF;

	mc_interface_set_pwm_callback(0);

	mar_Deinit();

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

	chRegSetThreadName("App Custom");

	is_running = true;

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		U_DC_filt -= U_DC_filt/10;
		U_DC_filt += GET_INPUT_VOLTAGE();
		U_DC = U_DC_filt/10;

		U_CHG_filt -= U_CHG_filt/10;
		U_CHG_filt += GET_VOLTAGE(6);
		U_CHG = U_CHG_filt/10;

		I_CHG_filt -= I_CHG_filt/100;
		I_CHG_filt += GET_VOLTAGE_RAW(7)/0.0088;
		I_CHG = (I_CHG_filt - I_CHG_offset)/100;

		if (discharge_enable)
		{
			if (SoC < discharge_SoC)
			{
				discharge_SoC = 2;
				mc_interface_release_motor();
				discharge_enable = false;
			}
			else
				mcpwm_foc_set_openloop(20, 10);
		}

		LTC_handler();
		charge_statemachine();
		safety_checks();
		CAN_Status();

		chThdSleepMilliseconds(10);
	}
}

static void pwm_callback(void)
{
	if ((ADC_Value[7] > 3686) || (ADC_Value[7] < 1993))  // fast CP disconnect if curent >150A or <-5A
	{
		CHRG_OFF;
		cp_state = cpst_error;
	}
}
