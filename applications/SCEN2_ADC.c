/*
 * SCEN2_ADC.c
 *
 *  Created on: 28.03.2019
 *      Author: alexander.schroeder
 */


#include "mc_interface.h"
#include "SCEN2_types.h"
#include "SCEN2_settings.h"
#include <math.h>

#define filter_constant				500


Analog_IO_t analog_IO, filter_analog_IO;
float leakage_threashold = LEAKAGE_THRESHOLD;


void SCEN2_ADC_handler(void)
{
	// Water temperature measurement
	float x = ADC_Value[ADC_IND_TEMP_W];
	filter_analog_IO.temp_water -= 119.5217814001;
	filter_analog_IO.temp_water += x*0.1571592547;
	x *= x;
	filter_analog_IO.temp_water -= x*0.0000345818;
	filter_analog_IO.temp_water -= filter_analog_IO.temp_water/filter_constant;
	analog_IO.temp_water = filter_analog_IO.temp_water/(filter_constant - 1);

	// Water pressure measurement
	uint32_t tmp_L = ADC_Value[ADC_IND_PRESSURE];
	if ((tmp_L < 399) || (tmp_L > 3591))
		errors.water_pressure_error = true;
	else
		errors.water_pressure_error = false;
	filter_analog_IO.pressure += ADC_Value[ADC_IND_PRESSURE]*0.003133051 - 1.249854171;
	filter_analog_IO.pressure -= filter_analog_IO.pressure/filter_constant;
	analog_IO.pressure = filter_analog_IO.pressure/(filter_constant - 1);

	// power stage temperature measurement
	filter_analog_IO.temp_MOS += mc_interface_temp_fet_filtered();
	filter_analog_IO.temp_MOS -= filter_analog_IO.temp_MOS/filter_constant;
	analog_IO.temp_MOS = filter_analog_IO.temp_MOS/(filter_constant - 1);

	// motor temperature measurement
	filter_analog_IO.temp_motor += mc_interface_temp_motor_filtered();
	filter_analog_IO.temp_motor -= filter_analog_IO.temp_motor/filter_constant;
	analog_IO.temp_motor = filter_analog_IO.temp_motor/(filter_constant - 1);

	// intermediate circuit voltage measurement
	filter_analog_IO.U_in += GET_INPUT_VOLTAGE();
	filter_analog_IO.U_in -= filter_analog_IO.U_in/filter_constant;
	analog_IO.U_in = filter_analog_IO.U_in/(filter_constant - 1);

	// charge port voltage and current measurement
	filter_analog_IO.U_charge += ADC_Value[ADC_IND_U_CHG]*0.01855758408368644067796610169492;
	filter_analog_IO.U_charge -= filter_analog_IO.U_charge/filter_constant;
	analog_IO.U_charge = filter_analog_IO.U_charge/(filter_constant - 1);

	analog_IO.I_charge_raw = ADC_Value[ADC_IND_I_CHG]*0.01535475628930817610062893081761;
#ifdef CHARGE_I_INVERTED
	analog_IO.I_charge_raw = ADC_Value[ADC_IND_I_CHG]*-0.01535475628930817610062893081761;
#else
	analog_IO.I_charge_raw = ADC_Value[ADC_IND_I_CHG]*0.01535475628930817610062893081761;
#endif
	filter_analog_IO.I_charge += analog_IO.I_charge_raw - analog_IO.I_charge_offset;
	filter_analog_IO.I_charge -= filter_analog_IO.I_charge/1000;
	analog_IO.I_charge = filter_analog_IO.I_charge/999;

	// water ingress sensor
	analog_IO.water_ingress = ADC_VOLTS(ADC_IND_ING);
	// check if water ingress is tripped
	if (analog_IO.water_ingress < leakage_threashold)
		errors.water_ingress_error = true;
	else
		errors.water_ingress_error = false;
}
