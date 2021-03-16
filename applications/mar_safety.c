/*
 * mar_safety.c
 *
 *  Created on: 18.02.2021
 *      Author: AUKTORAAlexanderSchr
 */

#include "hw.h"
#include "comm_can.h"
#include "mc_interface.h"
#include "timeout.h"
#include "commands.h"

#include "mar_vars.h"
#include "LTC6804_handler.h"


volatile bool Motor_lock = true;
volatile uint32_t Motor_lock_timer;

volatile uint32_t Sleep_Time = Sleep_Time_default;
volatile uint32_t sleep_timer;

float AUX_temp_cutoff;

void safety_reset_sleep_counter(void)
{
	sleep_timer = chVTGetSystemTimeX();
}

void safety_Init(void)
{
	sleep_timer = chVTGetSystemTimeX();
	Motor_lock_timer = chVTGetSystemTimeX();
	AUX_temp_cutoff = 100;
}

void safety_lock_motor(void)
{
	Motor_lock = true;
	Motor_lock_timer = chVTGetSystemTimeX();
}

void safety_checks(void)
{
	if (!BMS_Discharge_permitted)
		safety_lock_motor();

	// I-n check
	static uint32_t RPM_check_timer;
	float I = mc_interface_get_tot_current_filtered();
	float RPM = mc_interface_get_rpm();
	float RPM_check = 0.0859*I*I*I - 13.47*I*I + 883*I + 2691.4;
	if ((RPM < RPM_check*1.1) && (RPM > RPM_check*0.9))
		RPM_check_timer = chVTGetSystemTimeX();

	if ((chVTTimeElapsedSinceX(RPM_check_timer) > S2ST(2)) || (RPM > 30000))
		safety_lock_motor();

	if (Motor_lock && (chVTTimeElapsedSinceX(Motor_lock_timer) > S2ST(30)))
		Motor_lock = false;

	if (!Motor_lock)
		timeout_reset(); // Reset timeout if everything is OK.

	// Sleeptimer logic
	if (mc_interface_get_rpm() > 100)
		sleep_timer = chVTGetSystemTimeX();

	if (chVTTimeElapsedSinceX(sleep_timer) > S2ST(Sleep_Time))
		SHDN_ON;

	// HBT checks
	if (!stand_alone)
	{
		static bool HBT_safe = true;
		static uint8_t CAN_sample_counter = 200;
		static uint32_t CAN_sample_timer = 0;
		if (chVTTimeElapsedSinceX(CAN_HBT_timeout) > S2ST(1))  // HBT1 timeout triped?
		{
			if (HBT_safe && (CAN_sample_counter == 200))  // HBT was present before?
			{
				commands_printf("lost HBT, start sampling");
				CAN_OFF;  // deactivate CAN

				HBT_safe = false;
				CAN_sample_counter = 0;  // and sample resistance
				CAN_sample_timer = chVTGetSystemTimeX();
			}
		}
		else if (!HBT_safe)
		{
			commands_printf("found HBT");
			HBT_safe = true;  // remember HBT was present
		}

		static float UL, UH;
		if (CAN_sample_counter == 200) {}  // chatch deactivated state
		else if ((CAN_sample_counter == 101) && (chVTTimeElapsedSinceX(CAN_sample_timer) >= S2ST(1)))  // last sample was negativ and 1s passed?
		{
	//		commands_printf("restart sampling");
			CAN_sample_counter = 0;  // start new sample
			CAN_sample_timer = chVTGetSystemTimeX();
		}
		else if (CAN_sample_counter == 100)  // CAN sampling finished
		{
			if ((UL > 0.8) && (UH < 3.))  // sample inside allowed window?
			{
				commands_printf("UL %.3f\nUH %.3f", (double) UL, (double) UH);
				commands_printf("HBT found, endable CAN");
				CAN_sample_counter = 200;  // deactivate sampling
				HBT_safe = true;
				CAN_HBT_timeout = chVTGetSystemTimeX();  // reset timeout memory

				CAN_ON;
			}
			else
			{
	//			commands_printf("no HBT found, waiting 1s");
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
	else
		CAN_ON;
}
