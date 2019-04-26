/*
 * SCEN2.h
 *
 *  Created on: 28.03.2019
 *      Author: alexander.schroeder
 */

#ifndef APPLICATIONS_SCEN2_TYPES_H_
#define APPLICATIONS_SCEN2_TYPES_H_


#include <stdbool.h>
#include <stdint.h>

struct Analog_IO_t
{
	float temp_water;  // C
	float temp_MOS;  // C
	float temp_motor;  // C
	float U_in;  // V
	float U_charge;  // V
	float I_charge_raw;  // A
	float pressure;  // bar
	float water_ingress;  // V
	float depth;  // m
};
extern struct Analog_IO_t analog_IO;

struct Digital_IO_t
{
	union
	{
		struct
		{
			uint8_t silver;
			uint8_t green;
			uint8_t blue;
			uint8_t red;
		};
		uint32_t all;
	} buttons;

	union
	{
		struct
		{
			bool T1A;
			bool T1B;
			bool T2A;
			bool T2B;
		};
		uint32_t all;
	} trigger;
};
extern struct Digital_IO_t digital_IO, can_IO;

struct Errors_t
{
	bool water_pressure_error;
	bool water_ingress_error;
};
extern struct Errors_t errors;

struct Battery_t
{
	uint32_t CAN_offset;

	bool OK_to_charge;
	bool OK_to_discharge;

	float U;
	float SOC;
};
extern struct Battery_t batteries[2];

#endif /* APPLICATIONS_SCEN2_TYPES_H_ */
