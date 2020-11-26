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
#include "utils.h"
#include "hw.h"
#include "timeout.h"

#include "maraneo_vars.h"
#include "LTC6804_handler.h"
#include "mar_charge_statemachine.h"
#include "mar_CAN.h"
#include "mar_terminal.h"

// Threads
static THD_FUNCTION(my_thread, arg);
static THD_WORKING_AREA(my_thread_wa, 2048);

// Private functions
static void pwm_callback(void);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;

volatile float U_DC = 0, U_DC_filt = 0;
volatile float U_CHG = 0, U_CHG_filt = 0;
volatile float I_CHG = 0, I_CHG_filt = 0, I_CHG_offset = 0;

volatile bool Motor_lock = true;
volatile uint32_t Motor_lock_timer;

volatile uint32_t Sleep_Time = Sleep_Time_default;
volatile uint32_t sleep_timer;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void)
{
	chg_init();
	SHDN_OFF;

	stop_now = false;
	chThdCreateStatic(my_thread_wa, sizeof(my_thread_wa),
			NORMALPRIO, my_thread, NULL);

	mc_interface_set_pwm_callback(pwm_callback);

	// Terminal commands for the VESC Tool terminal can be registered.

	sleep_timer = chVTGetSystemTimeX();
	Motor_lock_timer = chVTGetSystemTimeX();

	mar_Init();
	LTC_handler_Init();
	mar_read_config();
	CAN_Init();
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void)
{
	charge_en = false;
	CHRG_OFF;

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

void Safty_checks(void)
{
	if (!BMS_Discharge_permitted)
	{
		Motor_lock = true;
		Motor_lock_timer = chVTGetSystemTimeX();
	}

	if (mc_interface_get_rpm() > 20000)
	{
		Motor_lock = true;
		Motor_lock_timer = chVTGetSystemTimeX();
	}

	if (Motor_lock && (chVTTimeElapsedSinceX(BMS.last_CV) > S2ST(30)))
		Motor_lock = false;

	if (mc_interface_get_rpm() > 100)
		sleep_timer = chVTGetSystemTimeX();

	if (chVTTimeElapsedSinceX(sleep_timer) > S2ST(Sleep_Time))
		SHDN_ON;
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

		if (!Motor_lock)
			timeout_reset(); // Reset timeout if everything is OK.

		U_DC_filt -= U_DC_filt/10;
		U_DC_filt += GET_INPUT_VOLTAGE();
		U_DC = U_DC_filt/10;

		U_CHG_filt -= U_CHG_filt/10;
		U_CHG_filt += GET_VOLTAGE(6);
		U_CHG = U_CHG_filt/10;

		I_CHG_filt -= I_CHG_filt/100;
		I_CHG_filt += GET_VOLTAGE_RAW(7)/0.0088;
		I_CHG = (I_CHG_filt - I_CHG_offset)/100;


		LTC_handler();
		charge_statemachine();
		Safty_checks();
		CAN_Status();

		chThdSleepMilliseconds(10);
	}
}

static void pwm_callback(void)
{
	if ((ADC_Value[7] > 3686) || (ADC_Value[7] < 1993))  // fast CP disconnect if curent >150A or <-5A
	{
		CHRG_OFF;
		chg_state = chgst_error;
	}
}
