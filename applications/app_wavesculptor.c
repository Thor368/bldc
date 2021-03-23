/*
 * app_wavesculptor.c
 *
 *  Created on: 12.05.2019
 *      Author: Alexander Schroeder
 *    Abstract: Emulate basic Tritium WS20 and WS22 CAN interface
 */

#include "comm_can.h"
#include "timeout.h"
#include "mc_interface.h"
#include "comm_can.h"
#include "terminal.h"
#include "commands.h"

#include <string.h>

static THD_FUNCTION(WS_thread, arg);
static THD_WORKING_AREA(WS_thread_wa, 1024);

static volatile app_configuration app_conf;
static const volatile mc_configuration *mc_conf;
static volatile bool stop_now = true;
static volatile bool is_running = false;

#define ID_DRIVE_CMD				0x01

#define ID_INFO						0x00
#define ID_STATUS					0x01
#define ID_BUS						0x02
#define ID_VELOCITY					0x03
#define ID_TEMP						0x0B


uint32_t CAN_base_adr;
#define DRV_CMD_OFFSET				-0x100
#define CAN_DRIVE_CONTROLS_BASE		(CAN_base_adr + DRV_CMD_OFFSET)
#define CAN_DATA_BASE				CAN_base_adr

#define PACKET_HANDLER				1

static SerialConfig uart_cfg = {
		500000,
		0,
		0,
		0
};

uint8_t if_buffer[2], if_buffer_state;
uint16_t pos1, pos2;
uint8_t temp1, temp2;

enum
{
	ws_none,
	ws_20,
	ws_22
} ws_protocol;

void write_conf(void)
{
	eeprom_var eep_conf;
	int pp = 0;

	eep_conf.as_u32 = 1;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);

	eep_conf.as_u32 = ws_protocol;
	conf_general_store_eeprom_var_custom(&eep_conf, pp++);
}

void read_conf(void)
{
	eeprom_var eep_conf;
	int pp = 0;

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	if (eep_conf.as_u32 != 1)
		write_conf();

	conf_general_read_eeprom_var_custom(&eep_conf, pp++);
	ws_protocol = eep_conf.as_u32;
}

static void ws_set_protocol(int argc, const char **argv)
{
	if (argc == 1)
	{
		switch (ws_protocol)
		{
		case ws_none:
			commands_printf("none");
			break;

		case ws_20:
			commands_printf("ws20");
			break;

		case ws_22:
			commands_printf("ws22");
			break;
		}
	}
	else if (argc == 2)
	{
		if (!strcmp(argv[1], "none"))
			ws_protocol = ws_none;
		else if (!strcmp(argv[1], "ws20"))
			ws_protocol = ws_20;
		else if (!strcmp(argv[1], "ws22"))
			ws_protocol = ws_22;
		else
		{
			commands_printf("Illegal parameter!");
			commands_printf("Allowed parameters: none, ws20, ws22");
		}

		write_conf();
	}
}

void app_custom_start(void)
{
	stop_now = false;

	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	sdStart(&HW_UART_DEV, &uart_cfg);

	terminal_register_command_callback(
				"ws_protocol",
				"Set/Get Wavesculptor protocol",
				"",
				ws_set_protocol);

	ws_protocol = ws_none;

	read_conf();

	chThdCreateStatic(WS_thread_wa, sizeof(WS_thread_wa), NORMALPRIO, WS_thread, NULL);
}

void app_custom_stop(void)
{
	sdStop(&HW_UART_DEV);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);

	stop_now = true;
	while (is_running)
		chThdSleepMilliseconds(1);
}

bool rx_callback(uint32_t id, uint8_t *data, uint8_t len)
{
	(void) len;
	float temp_v, temp_I;

	if ((id < CAN_DRIVE_CONTROLS_BASE) ||
		(id > (CAN_DRIVE_CONTROLS_BASE + 0x20)))
		return false;

	if (ws_protocol == ws_none)
		return false;

	id -= CAN_DRIVE_CONTROLS_BASE;
	switch (id)
	{
	case ID_DRIVE_CMD:
		temp_v = *(float *) &data[0];
		temp_I = *(float *) &data[4];

		if ((temp_I > 1) || (temp_I < 0))
			temp_I = 0;

		if (temp_v > 10.)
			mc_interface_set_current(temp_I*mc_conf->l_current_max);
		else if (temp_v < -10.)
			mc_interface_set_current(-temp_I*mc_conf->l_current_max);
		else
			mc_interface_set_brake_current(temp_I*mc_conf->l_current_max);

		timeout_reset();
		return true;
	break;
	}

	return false;
}

void app_custom_configure(app_configuration *conf)
{
	app_conf = *conf;
	mc_conf = mc_interface_get_configuration();

	CAN_base_adr = app_conf.controller_id*0x20;
	comm_can_set_sid_rx_callback(&rx_callback);
}

void UART_handler(void)
{
	msg_t res = sdGetTimeout(&HW_UART_DEV, TIME_IMMEDIATE);
	while (res != MSG_TIMEOUT)
	{
		if_buffer[0] = if_buffer[1];
		if_buffer[1] = res;
		if ((if_buffer[0] == 10) && (if_buffer[0] == 0))
			if_buffer_state = 1;
		else if (if_buffer_state > 0)
		{
			if_buffer_state++;

			switch (if_buffer_state)
			{
			case 3:
				pos1 = *((uint16_t *) if_buffer);
				break;

			case 4:
				temp1 = if_buffer[1];
				break;

			case 5:
				temp2 = if_buffer[1];
				break;

			case 7:
				pos2 = *((uint16_t *) if_buffer);
				if (pos1 != pos2)
				{
					pos1 = 0;
					pos2 = 0;
					mc_interface_release_motor();
				}

				if_buffer_state = 0;
				break;
			}
		}

		res = sdGetTimeout(&HW_UART_DEV, TIME_IMMEDIATE);
	}
}

static THD_FUNCTION(WS_thread, arg)
{
	(void)arg;

	chRegSetThreadName("APP_WAVESCULPTOR");

	is_running = true;

	for(;;)
	{
		if (stop_now)
		{
			is_running = false;
			return;
		}

		if (ws_protocol != ws_none)
		{
			static systime_t wait_1s = 0;
			if (chVTTimeElapsedSinceX(wait_1s) > MS2ST(1000))
			{
				wait_1s = chVTGetSystemTime();

				uint8_t data_B[8] = {};
				if (ws_protocol == ws_20)
				{
					data_B[0] = 'T';
					data_B[1] = 'R';
					data_B[2] = 'I';
					data_B[3] = 'a';
				}
				else if (ws_protocol == ws_22)
				{
					data_B[0] = 'T';
					data_B[1] = '0';
					data_B[2] = '8';
					data_B[3] = '8';
				}

				*(uint32_t *) &data_B[4] = STM32_UUID[0] + STM32_UUID[1];
				comm_can_transmit_sid(ID_INFO + CAN_DATA_BASE, (uint8_t *) data_B, sizeof(data_B));

				float data_f[2];
				data_f[0] = mc_interface_temp_motor_filtered();
				data_f[1] = mc_interface_temp_fet_filtered();
				comm_can_transmit_sid(ID_TEMP + CAN_DATA_BASE, (uint8_t *) data_f, sizeof(data_f));
			}

			static systime_t wait_200ms = 0;
			if (chVTTimeElapsedSinceX(wait_200ms) > MS2ST(200))
			{
				wait_200ms = chVTGetSystemTime();

				uint8_t data_B[8];
				data_B[0] = 0;
				if (mc_interface_get_duty_cycle_now() >= mc_conf->l_max_duty*0.95)
					data_B[0] |= 0x02;
				else if ((mc_interface_get_tot_current_in_filtered() >= mc_conf->l_in_current_max*0.95) ||
						 (mc_interface_get_tot_current_in_filtered() <= mc_conf->l_in_current_min*0.95))
					data_B[0] |= 0x08;
				else if (GET_INPUT_VOLTAGE() <= mc_conf->l_min_vin*1.05)
					data_B[0] |= 0x20;
				else if (mc_interface_temp_fet_filtered() >= mc_conf->l_temp_fet_start)
					data_B[0] |= 0x40;
				else
					data_B[0] |= 0x01;
				data_B[1] = 0;
				data_B[2] = 0;
				data_B[3] = 0;
				data_B[4] = 0;
				data_B[5] = 0;
				data_B[6] = 0;
				data_B[7] = 0;
				comm_can_transmit_sid(ID_STATUS + CAN_DATA_BASE, (uint8_t *) data_B, sizeof(data_B));

				float data_F[2];
				data_F[0] = GET_INPUT_VOLTAGE();
				data_F[1] = mc_interface_get_tot_current_in_filtered();
				comm_can_transmit_sid(ID_BUS + CAN_DATA_BASE, (uint8_t *) data_F, sizeof(data_F));

				data_F[0] = mc_interface_get_rpm()/(mc_conf->si_motor_poles/2);
				data_F[1] = mc_interface_get_speed();
				comm_can_transmit_sid(ID_VELOCITY + CAN_DATA_BASE, (uint8_t *) data_F, sizeof(data_F));
			}

			UART_handler();
		}
	}
}
