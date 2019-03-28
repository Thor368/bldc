/*
 * SCEN2_ADC.c
 *
 *  Created on: 28.03.2019
 *      Author: alexander.schroeder
 */

#include "mc_interface.h"
#include "SCEN2_types.h"

struct Analog_t analog;


void SCEN2_ADC_handler(void)
{
	// Water temperature measurement -> Sensor unknown
	analog.temp_water = ADC_Value[ADC_IND_TEMP_W];

	// Water pressure and diving depth measurement
	uint32_t tmp_L = ADC_Value[ADC_IND_PRESSURE];
	if ((tmp_L < 399) || (tmp_L > 3591))
		errors.water_pressure_error = true;
	else
		errors.water_pressure_error = false;
	analog.pressure = tmp_L*0.00313313802083 - 1.25;
	analog.depth = analog.pressure*9.807;

	// power stage temperature measurement
	analog.temp_MOS = mc_interface_temp_fet_filtered();

	// motor temperature measurement
	analog.temp_motor = mc_interface_temp_motor_filtered();

	// intermediate circuit voltage measurement
	analog.U_in = GET_INPUT_VOLTAGE();

	// charge port voltage and current measurement
	analog.U_charge = ADC_VOLTS(ADC_IND_U_CHG)*0.0434142752;
	analog.I_charge = ADC_VOLTS(ADC_IND_I_CHG)*50;

	// check if water ingress is tripped
	if (ADC_VOLTS(ADC_IND_ING) < 500)
		errors.water_ingress_error = true;
	else
		errors.water_ingress_error = false;
}
