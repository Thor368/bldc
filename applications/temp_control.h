/*
 * temp_control.h
 *
 *  Created on: 02.01.2022
 *      Author: main
 */

#ifndef APPLICATIONS_TEMP_CONTROL_H_
#define APPLICATIONS_TEMP_CONTROL_H_

#include <stdbool.h>

// Some useful includes
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

extern bool manual_mode;

extern float T_tank, T_cond, I_Comp;
extern float U_fan, U_DC;

extern float T_target;
extern float I_fan_ramp_start;
extern float I_fan_ramp_end;
extern float U_fan_min;
extern float U_fan_max;
extern float U_pump_std;
extern float T_hyst_pos;
extern float T_hyst_neg;
extern float RPM_min, RPM_max;
extern float RPM_P, RPM_I, RPM_D, RPM_D_t;
extern float dt_plot;
extern bool FAN_PWM_invert;

#endif /* APPLICATIONS_TEMP_CONTROL_H_ */
