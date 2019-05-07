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


void SCEN2_ADC_handler(void)
{
	// Water temperature measurement -> Sensor unknown
	filter_analog_IO.temp_water += ADC_Value[ADC_IND_TEMP_W]*-0.057035696 + 197.7983105;
	filter_analog_IO.temp_water -= filter_analog_IO.temp_water/filter_constant;
	analog_IO.temp_water = filter_analog_IO.temp_water/(filter_constant - 1);

	// Water pressure and diving depth measurement
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
	analog_IO.temp_MOS += filter_analog_IO.temp_MOS/(filter_constant - 1);

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
	analog_IO.I_charge_raw = ADC_Value[ADC_IND_I_CHG]*0.040283203125;
	filter_analog_IO.I_charge += analog_IO.I_charge_raw - analog_IO.I_charge_offset;
	filter_analog_IO.I_charge -= filter_analog_IO.I_charge/filter_constant;
	analog_IO.I_charge = filter_analog_IO.I_charge/(filter_constant - 1);

	analog_IO.water_ingress = ADC_VOLTS(ADC_IND_ING);
	// check if water ingress is tripped
	if (analog_IO.water_ingress < LEAKAGE_THRESHOLD)
		errors.water_ingress_error = true;
	else
		errors.water_ingress_error = false;
}
