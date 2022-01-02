/*
 * temp_compressor.c
 *
 *  Created on: 02.01.2022
 *      Author: main
 */

#include "temp_control.h"

enum
{
	cmp_init,
	cmp_wait_for_start,
	cmp_ramp_up,
	cmp_failed_start,
	cmp_running,
	cmp_equalize
} compressor_state;

void compressor_init(void)
{
	compressor_state = cmp_init;
}

void sm_compressor(void)
{
	static systime_t cmp_timer = 0, cmp_counter = 0, RPM_timer = 0;

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
		if ((T_tank > (T_target + T_hyst_pos)) && !manual_mode)
		{
			cmp_timer = chVTGetSystemTime();
			mc_interface_set_pid_speed(RPM_min*5);
			compressor_state = cmp_ramp_up;
			cmp_counter = 0;
		}
		break;

	case cmp_ramp_up:
		if ((mc_interface_get_fault() != FAULT_CODE_NONE) || (chVTTimeElapsedSinceX(cmp_timer) >= S2ST(1)))
		{
			mc_interface_release_motor();
			cmp_timer = chVTGetSystemTime();
			cmp_counter++;
			compressor_state = cmp_failed_start;
		}
		else if ((chVTTimeElapsedSinceX(cmp_timer) >= MS2ST(500)) && (mc_interface_get_rpm() >= (RPM_min*4)))
		{
			compressor_state = cmp_running;
			RPM_timer = chVTGetSystemTime();
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
	{
		static float P = 0, I = 0, D = 0;

		if (I_Comp >= I_fan_ramp_end)
			U_fan = U_fan_max;
		else if (I_Comp <= I_fan_ramp_start)
			U_fan = U_fan_min;
		else
			U_fan = U_fan_min + (U_fan_max - U_fan_min)*((I_Comp - I_fan_ramp_start)/(I_fan_ramp_end - I_fan_ramp_start));

		if (chVTTimeElapsedSinceX(RPM_timer) >= MS2ST(100))
		{
			float dRPM = T_tank - T_target;
			P = dRPM*RPM_P;

			float dt = ST2MS(chVTTimeElapsedSinceX(RPM_timer))/1000.;
			RPM_timer = chVTGetSystemTime();
			static float T_tank_last = 0, D_filt = 0;
			D_filt -= D_filt/10;
			D_filt += (T_tank - T_tank_last)*dt*RPM_D;
			D = D_filt/10;
			T_tank_last = T_tank;

			float RPM_setpoint = P + I - D + RPM_min;

			float dI = dt*dRPM*RPM_I;
			if ((dI < 0) || (RPM_setpoint < RPM_max))
				I += dI;

			if (RPM_setpoint > RPM_max)
				RPM_setpoint = RPM_max;
			else if (RPM_setpoint < RPM_min)
				RPM_setpoint = RPM_min;

			mc_interface_set_pid_speed(RPM_setpoint*5);
		}

		if (T_tank < (T_target - T_hyst_neg))
		{
			mc_interface_release_motor();
			cmp_timer = chVTGetSystemTime();
			U_fan = U_fan_min;
			I = 0;
			compressor_state = cmp_equalize;
		}
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
