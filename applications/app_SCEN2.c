/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include "SCEN2_CAN_IDs.h"
#include <math.h>

// Settings
#define CYCLE_RATE		10

// Threads
static THD_FUNCTION(custom_thread, arg);
static THD_WORKING_AREA(custom_thread_wa, 1024);

// Private variables
static volatile app_configuration config;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile rtcnt_t speed_keep_alive = 0;
static volatile float speed_save = 0;

void app_custom_configure(app_configuration *conf) {
	config = *conf;
}

void app_custom_start(void) {
	stop_now = false;
	chThdCreateStatic(custom_thread_wa, sizeof(custom_thread_wa), NORMALPRIO, custom_thread, NULL);
}

void app_custom_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void tx_BATTVOLTAGE(void)
{
	uint16_t data[4];
	*((uint16_t *) &data[0]) = (uint16_t) ADC_Value[ADC_IND_VIN_SENS];
	*((uint16_t *) &data[2]) = (uint16_t) (GET_INPUT_VOLTAGE()*1000);
	comm_can_transmit_eid(ID_POW_BATTVOLTAGE, (uint8_t *) &data, sizeof(data));
}

void tx_CURRENTS_01(void)
{
	uint16_t data[1];
	data[0] = (uint16_t) (1000*mc_interface_get_tot_current_filtered());
	comm_can_transmit_eid(ID_POW_CURRENTS_01, (uint8_t *) &data, sizeof(data));
}

void tx_TEMPERATURE(void)
{
	uint16_t data[3];
	data[0] = (uint16_t) (100*NTC_TEMP(ADC_Value[ADC_IND_TEMP_MOS]));
	data[1] = 0;
	data[2] = (uint16_t) (100*NTC_TEMP(ADC_Value[ADC_IND_TEMP_MOTOR]));
	comm_can_transmit_eid(ID_POW_TEMPERATURE, (uint8_t *) &data, sizeof(data));
}

void tx_RPS(void)
{
	uint16_t data[3];
	data[0] = (uint16_t) (mc_interface_get_rpm()*60);
	data[1] = 0;
	data[2] = (uint16_t) mc_interface_get_rpm();
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
	uint8_t tmp[1];
	float speed = 0;
	(void) len;

	switch (id)
	{
		case MCL_WR_Motor_Speed_Set:
			speed = *((int16_t *) data);

			if (speed != speed_save)
				mc_interface_set_pid_speed(speed_save);

			speed_save = speed;
			speed_keep_alive = chSysGetRealtimeCounterX() + 200;
		break;

		case MCL_RD_Motor_Speed:
			*((uint16_t *) &tmp[0]) = (uint16_t) mc_interface_get_rpm();
			comm_can_transmit_eid(MCL_RD_Motor_Speed, (uint8_t *) &tmp, 2);
		break;
	}
}

static THD_FUNCTION(custom_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_CUSTOM");

	is_running = true;

	comm_can_set_sid_rx_callback(&rx_callback);

	CAN_CP_ON();

	for(;;)
	{
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / CYCLE_RATE;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		if ((chSysGetRealtimeCounterX() > speed_keep_alive) && (speed_save != 0))
		{
			speed_save = 0;
			mc_interface_set_pid_speed(0);
		}

		tx_BATTVOLTAGE();
		tx_CURRENTS_01();
		tx_TEMPERATURE();
		tx_RPS();
		tx_INPUTPOWER();
	}
}
