/*
 * SCEN2.h
 *
 *  Created on: 28.03.2019
 *      Author: alexander.schroeder
 */

#ifndef APPLICATIONS_SCEN2_TYPES_H_
#define APPLICATIONS_SCEN2_TYPES_H_


#include <stdbool.h>

struct Analog_t
{
	float temp_water;  // C
	float temp_MOS;  // C
	float temp_motor;  // C
	float U_in;  // V
	float U_charge;  // V
	float I_charge;  // A
	float pressure;  // bar
	float water_ingress;  // V
	float depth;  // m
};
extern struct Analog_t analog;

struct Errors_t
{
	bool water_pressure_error;
	bool water_ingress_error;
};
extern struct Errors_t errors;

#endif /* APPLICATIONS_SCEN2_TYPES_H_ */
