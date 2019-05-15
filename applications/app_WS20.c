/*
 * app_WS20.c
 *
 *  Created on: 12.05.2019
 *      Author: Alexander Schroeder
 *    Abstract: Emulate basic Tritium WS20 CAN interface
 */

#include "comm_can.h"
#include "timeout.h"
#include "mc_interface.h"

static THD_FUNCTION(WS20_thread, arg);
static THD_WORKING_AREA(WS20_thread_wa, 1024);

static volatile app_configuration config;
static volatile bool stop_now = true;
static volatile bool is_running = false;

#define CAN_ID_DRIVE_CMD			0x01

#define CAN_ID_INFO					0x00
#define CAN_ID_STATUS				0x01
#define CAN_ID_BUS					0x02
#define CAN_ID_VELOCITY				0x03
#define CAN_ID_HEATSINK				0x0B


uint32_t CAN_base_adr;

void app_custom_start(void)
{
	stop_now = false;
	chThdCreateStatic(WS20_thread_wa, sizeof(WS20_thread_wa), NORMALPRIO, WS20_thread, NULL);
}

void app_custom_stop(void)
{
	stop_now = true;
	while (is_running)
		chThdSleepMilliseconds(1);
}

void rx_callback(uint32_t id, uint8_t *data, uint8_t len)
{
	(void) len;

	switch (id)
	{
	case CAN_ID_DRIVE_CMD:
		mc_interface_set_current(*(float *) &data[4]);
		timeout_reset();
	break;
	}
}

void app_custom_configure(app_configuration *conf)
{
	config = *conf;

	CAN_base_adr = config.controller_id*0x20;
	comm_can_set_sid_rx_callback(&rx_callback);
}

static THD_FUNCTION(WS20_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_WS20");

	is_running = true;

	for(;;) {
		if (stop_now) {
			is_running = false;
			return;
		}

		static systime_t wait_1s = 0;
//		if ()

//		comm_can_transmit_sid(uint32_t id, uint8_t *data, uint8_t len);

		chThdSleep(1);
}
}
