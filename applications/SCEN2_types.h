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
#include "hal.h"

typedef struct
{
	float temp_water;  // C
	float temp_MOS;  // C
	float temp_motor;  // C
	float U_in;  // V
	float U_charge;  // V
	float I_charge_raw;  // A
	float I_charge_offset;  // A
	float pressure;  // bar
	float water_ingress;  // V
	float depth;  // m
} Analog_IO_t;
extern Analog_IO_t analog_IO;

typedef struct
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
			bool T1;
			bool T2;
		};
		uint16_t all;
	} trigger;
} Digital_IO_t;
extern Digital_IO_t digital_IO, can_IO;

typedef union
{
	struct
	{
		unsigned water_pressure_error: 1;
		unsigned water_ingress_error: 1;
		unsigned battery_left_error: 1;
		unsigned battery_right_error: 1;
		unsigned trigger_error: 1;
		unsigned charger_error: 1;
	};

	uint8_t all;
} Errors_t;
extern Errors_t errors;

typedef struct
{
	uint32_t offset;
	systime_t time_last_seen;
	bool active;

	bool OK_to_charge;
	bool OK_to_discharge;

	float SOC;
	float total_U;

	union
	{
		struct
		{
			unsigned LTC_config_failed: 1;
			unsigned Cell_temp_discharge_low: 1;
			unsigned Cell_temp_discharge_high: 1;
			unsigned Cell_temp_charge_low: 1;
			unsigned Cell_temp_charge_high: 1;
			unsigned Cell_U_discharge_low: 1;
			unsigned Fuse_fault: 1;
			unsigned reserved: 1;
		};

		uint8_t all;
	} error_flags;

	union
	{
		struct
		{
			unsigned Battery_full: 1;
			unsigned Cell_U_high_warn1: 1;
			unsigned Cell_U_high_warn2: 1;
			unsigned Cell_dU_high: 1;
			unsigned Discharge_I_high: 1;
			unsigned Cell_U_below_flight: 1;
			unsigned Cell_U_over_flight: 1;
			unsigned LTC_ref_U_fault: 1;
		};

		uint8_t all;
	} warning_flags;
} Battery_t;
extern Battery_t batteries[2];

typedef enum
{
	no_charger,
	charger_detected_no_ACK,
	charger_detected_with_ACK
} Charge_mode_t;
extern Charge_mode_t charge_mode;

#endif /* APPLICATIONS_SCEN2_TYPES_H_ */
