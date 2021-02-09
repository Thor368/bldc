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
#include "timeout.h"
#include "commands.h"

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

volatile float discharge_SoC;
volatile bool discharge_enable;

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
	CAN_OFF;
	BAT_ON;

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
	CAN_OFF;
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

void Safty_checks(void)
{
	if (!BMS_Discharge_permitted)
	{
		Motor_lock = true;
		Motor_lock_timer = chVTGetSystemTimeX();
	}

	// I-n check
	static uint32_t RPM_check_timer;
	float I = mc_interface_get_tot_current_filtered();
	float RPM = mc_interface_get_rpm();
	float RPM_check = 0.0859*I*I*I - 13.47*I*I + 883*I + 2691.4;
	if ((RPM < RPM_check*1.1) && (RPM > RPM_check*0.9))
		RPM_check_timer = chVTGetSystemTimeX();

	if ((chVTTimeElapsedSinceX(RPM_check_timer) > S2ST(2)) || (RPM > 30000))
	{
		Motor_lock = true;
		Motor_lock_timer = chVTGetSystemTimeX();
	}

	if (Motor_lock && (chVTTimeElapsedSinceX(BMS.last_CV) > S2ST(30)))
		Motor_lock = false;

	// Sleeptimer logic
	if (mc_interface_get_rpm() > 100)
		sleep_timer = chVTGetSystemTimeX();

	if (chVTTimeElapsedSinceX(sleep_timer) > S2ST(Sleep_Time))
		SHDN_ON;

	// HBT checks
	static bool HBT1_safe = false, HBT2_safe = false;
	static uint8_t CAN_sample_counter = 200;
	static uint32_t CAN_sample_timer = 0;
	if (chVTTimeElapsedSinceX(CAN_HBT1_timeout) > S2ST(1))  // HBT1 timeout triped?
	{
		if (HBT1_safe && (CAN_sample_counter == 200))  // HBT was present before?
		{
			commands_printf("lost HBT1, start sampling");
			CAN_OFF;  // deactivate CAN

			CAN_sample_counter = 0;  // and sample resistance
			CAN_sample_timer = chVTGetSystemTimeX();
		}
	}
	else if (!HBT1_safe)
	{
		commands_printf("found HBT1");
		HBT1_safe = true;  // remember HBT was present
	}

	if (chVTTimeElapsedSinceX(CAN_HBT2_timeout) > S2ST(1))  // HBT2 timeout triped?
	{
		if (HBT2_safe && (CAN_sample_counter == 200))  // HBT was present before?
		{
			commands_printf("lost HBT2, start sampling");
			CAN_OFF;  // deactivate CAN
			CAN_sample_counter = 0;  // and sample resistance
			CAN_sample_timer = chVTGetSystemTimeX();
		}
	}
	else if (!HBT2_safe)
	{
		commands_printf("found HBT2");
		HBT2_safe = true;  // remember HBT was present
	}

	static float UL, UH;
	if (CAN_sample_counter == 200) {}  // chatch deactivated state
	else if ((CAN_sample_counter == 101) && (chVTTimeElapsedSinceX(CAN_sample_timer) >= S2ST(1)))  // last sample was negativ and 1s passed?
	{
		commands_printf("restart sampling");
		CAN_sample_counter = 0;  // start new sample
		CAN_sample_timer = chVTGetSystemTimeX();
	}
	else if (CAN_sample_counter == 100)  // CAN sampling finished
	{
		commands_printf("UL %.3f\nUH %.3f", (double) UL, (double) UH);
		if ((UL > 1.314) && (UH < 1.558))  // sample inside allowed window?
		{
			commands_printf("HBT found, endable CAN");
			CAN_sample_counter = 200;  // deactivate sampling
			HBT1_safe = false;
			HBT2_safe = false;  // reset timeout memory

			CAN_ON;
		}
		else
		{
			commands_printf("no HBT found, waiting 1s");
			CAN_sample_counter = 101;  // deactivate sampling
			CAN_sample_timer = chVTGetSystemTimeX();
		}
	}
	else if ((chVTTimeElapsedSinceX(CAN_sample_timer) >= MS2ST(1)) && (CAN_sample_counter < 100))  // CAN sample
	{
		CAN_sample_counter++;
		CAN_sample_timer = chVTGetSystemTimeX();

		static float UL_filt = 0;
		UL_filt -= UL_filt/10;
		UL_filt += GET_VOLTAGE_RAW(9);
		UL = UL_filt/10;

		static float UH_filt = 0;
		UH_filt -= UH_filt/10;
		UH_filt += GET_VOLTAGE_RAW(10);
		UH = UH_filt/10;
	}
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

		if (discharge_enable)
		{
			if (SoC < discharge_SoC)
			{
				discharge_SoC = 2;
				mc_interface_release_motor();
				discharge_enable = false;
				return;
			}

			mcpwm_foc_set_openloop(20, 10);
		}

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
