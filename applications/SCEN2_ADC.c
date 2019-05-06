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

Analog_IO_t analog_IO;


void SCEN2_ADC_handler(void)
{
	// Water temperature measurement -> Sensor unknown
	analog_IO.temp_water = ADC_VOLTS(ADC_IND_TEMP_W);

	// Water pressure and diving depth measurement
	uint32_t tmp_L = ADC_Value[ADC_IND_PRESSURE];
	if ((tmp_L < 399) || (tmp_L > 3591))
		errors.water_pressure_error = true;
	else
		errors.water_pressure_error = false;
	analog_IO.pressure = ADC_VOLTS(ADC_IND_PRESSURE);

	// power stage temperature measurement
	analog_IO.temp_MOS = mc_interface_temp_fet_filtered();

	// motor temperature measurement
	analog_IO.temp_motor = mc_interface_temp_motor_filtered();

	// intermediate circuit voltage measurement
	analog_IO.U_in = GET_INPUT_VOLTAGE();

	// charge port voltage and current measurement
	analog_IO.U_charge = ADC_VOLTS(ADC_IND_U_CHG)*18.727272727272727272727272727272;
	analog_IO.I_charge_raw = ADC_VOLTS(ADC_IND_I_CHG)*50;

	analog_IO.water_ingress = ADC_VOLTS(ADC_IND_ING);
	// check if water ingress is tripped
	if (analog_IO.water_ingress < LEAKAGE_THRESHOLD)
		errors.water_ingress_error = true;
	else
		errors.water_ingress_error = false;
}
