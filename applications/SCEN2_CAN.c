/*
 * SCEN2_CAN.c
 *
 *  Created on: 28.03.2019
 *      Author: alexander.schroeder
 */

#include "SCEN2_types.h"
#include "SCEN2_settings.h"
#include "SCEN2_CAN_IDs.h"

#include "ch.h"
#include "comm_can.h"
#include "mc_interface.h"
#include "timeout.h"

#include "utils.h"
#include <math.h>
#include <stdlib.h>


#define CAN_DELAY_BASIC		100

systime_t can_basic_update;

float speed_save = 0;

void tx_BATTVOLTAGE(void)
{
	uint16_t data[3];
	*((uint16_t *) &data[0]) = (uint16_t) ADC_Value[ADC_IND_VIN_SENS];
	*((uint16_t *) &data[1]) = (uint16_t) lround(analog.U_in*1000);
	*((uint16_t *) &data[2]) = (uint16_t) lround(mc_interface_get_duty_cycle_now()*100);
	comm_can_transmit_eid(ID_POW_BATTVOLTAGE, (uint8_t *) &data, sizeof(data));
}

void tx_CURRENTS_01(void)
{
	uint16_t data[3];
	data[0] = (uint16_t) (ADC_Value[ADC_IND_CURR1]);
	data[1] = (uint16_t) (ADC_Value[ADC_IND_CURR2]);
	data[2] = (uint16_t) (ADC_Value[ADC_IND_CURR3]);
	comm_can_transmit_eid(ID_POW_CURRENTS_01, (uint8_t *) &data, sizeof(data));
}

void tx_TEMPERATURE(void)
{
	uint8_t data[7];
	float temp_max = 1000;
	utils_truncate_number(&temp_max, HW_LIM_TEMP_FET);
	data[0] = (uint8_t) round(temp_max);
	*((uint16_t *) &data[1]) = (uint16_t) lround(analog.temp_MOS);  // PA temp
	*((uint16_t *) &data[3]) = (uint16_t) lround(analog.temp_water);  // ToDo: water temp
	*((uint16_t *) &data[5]) = (uint16_t) lround(analog.temp_motor)*10 + 2732;  // Motor Temp
	comm_can_transmit_eid(ID_POW_TEMPERATURE, (uint8_t *) &data, sizeof(data));
}

void tx_RPS(void)
{
	uint16_t data[3];
	data[1] = 0;
	data[2] = (uint16_t) lround(abs(mc_interface_get_rpm()/POLE_PAIR_COUNT));
	data[0] = data[2]/60;
	comm_can_transmit_eid(ID_POW_RPS, (uint8_t *) &data, sizeof(data));
}

void tx_INPUTPOWER(void)
{
	uint16_t data[1];
	data[0] = (uint16_t) (mc_interface_get_tot_current_in_filtered()*GET_INPUT_VOLTAGE());
	comm_can_transmit_eid(ID_POW_INPUTPOWER, (uint8_t *) &data, sizeof(data));
}

static void rx_callback(uint32_t id, uint8_t *data, uint8_t len)
{
	uint16_t out[4];
	float speed = 0;
	(void) len;

	switch (id)
	{
		case MCL_WR_Motor_Speed_Set:
			speed = *((int16_t *) data)*POLE_PAIR_COUNT;

			if (abs(speed - speed_save) > 1)  // floats...
				mc_interface_set_pid_speed(speed);

			speed_save = speed;
			timeout_reset();
		break;

		case MCL_RD_Motor_Speed:
			out[0] = (uint16_t) mc_interface_get_rpm()/POLE_PAIR_COUNT;
			comm_can_transmit_eid(MCL_RD_Motor_Speed, (uint8_t *) &out, 2);
		break;

		case ID_POW_CURRENTS_01:
			tx_CURRENTS_01();
		break;

		case ID_POW_TEMPERATURE:
			tx_TEMPERATURE();
		break;

		case ID_POW_RPS:
			tx_RPS();
		break;

		case ID_POW_INPUTPOWER:
			tx_INPUTPOWER();
		break;
	}
}

void SCEN2_CAN_handler(void)
{
	if (chVTTimeElapsedSinceX(can_basic_update) > MS2ST(CAN_DELAY_BASIC))
	{
		can_basic_update = chVTGetSystemTime();
		tx_BATTVOLTAGE();
	}

	if (timeout_has_timeout())
		speed_save = 0;
}

void SCEN2_CAN_init(void)
{
	can_basic_update = chVTGetSystemTime();
	comm_can_set_sid_rx_callback(&rx_callback);
}
