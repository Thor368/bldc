/*
 * SCEN2_CAN.c
 *
 *  Created on: 28.03.2019
 *      Author: alexander.schroeder
 */

#include "SCEN2_types.h"
#include "SCEN2_settings.h"
#include "SCEN2_CAN_IDs.h"
#include "SCEN2_battery.h"
#include "SCEN2_charge.h"

#include "ch.h"
#include "comm_can.h"
#include "mc_interface.h"
#include "timeout.h"

#include "utils.h"
#include <math.h>
#include <stdlib.h>


#define CAN_DELAY_BASIC		100
#define START_UP_TIME		1000

systime_t can_basic_update;
systime_t start_up_timer;
//mc_configuration *c_conf;
Digital_IO_t can_IO;

bool starting_up = false;
float speed_save = 0;

uint32_t CAN_base = 0;

void tx_ChargeCurrent(void)
{
	float data[1];
	data[0] = analog_IO.I_charge_raw;
	comm_can_transmit_eid(MCL_ChargeCurrent + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_LeakageSensor(void)
{
	uint8_t data[5];
	*((float *) &data[0]) = analog_IO.water_ingress;
	data[4] = errors.water_ingress_error;
	comm_can_transmit_eid(MCL_LeakageSensor + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_LeakageSensorThreshold(void)
{
	float data[1];
	data[0] = LEAKAGE_THRESHOLD;
	comm_can_transmit_eid(MCL_LeakageSensorThreshold_rd + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_MC_PID(void)
{
	uint16_t data[3];
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	data[0] = (uint16_t) lround(conf->p_pid_kp)*10000;
	data[1] = (uint16_t) lround(conf->p_pid_ki)*10000;
	data[2] = (uint16_t) lround(conf->p_pid_kd)*10000;
	comm_can_transmit_eid(MCL_MC_PID_rd + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_Voltages(void)
{
	float data[2];
	data[0] = analog_IO.U_in;
	data[1] = analog_IO.U_charge;
	comm_can_transmit_eid(MCL_Voltages + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_MotorCurrents(void)
{
	float data[2];
	data[0] = mc_interface_get_tot_current_filtered();
	data[1] = mc_interface_get_duty_cycle_now();
	comm_can_transmit_eid(MCL_MotorCurrents + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_Temperatures(void)
{
	float data[2];
	data[0] = analog_IO.temp_MOS;
	data[1] = analog_IO.temp_water;
	comm_can_transmit_eid(MCL_Temperatures + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_Temperature_Limits(void)
{
	float data[2];
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	data[0] = conf->l_temp_fet_start;
	data[1] = 0; // Watertemp limit: ToDo
	comm_can_transmit_eid(MCL_Temperature_Limits_rd + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_MotorTemp(void)
{
	float data[1];
	data[0] = analog_IO.temp_motor;
	comm_can_transmit_eid(MCL_MotorTemp + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_MotorTemp_Limit(void)
{
	float data[1];
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	data[0] = conf->l_temp_motor_start;
	comm_can_transmit_eid(MCL_MotorTemp_Limit_rd + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_MotorSpeed(void)
{
	float data[1];
	data[0] = mc_interface_get_rpm()/POLE_PAIR_COUNT;
	comm_can_transmit_eid(MCL_MotorSpeed_rd + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_Preassure(void)
{
	float data[1];
	data[0] = analog_IO.pressure;
	comm_can_transmit_eid(MCL_Pressure + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_DCPower(void)
{
	float data[1];
	data[0] = mc_interface_get_tot_current_in_filtered()*GET_INPUT_VOLTAGE();
	comm_can_transmit_eid(MCL_DC_Power + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_Trigger1(void)
{
	float data[8];

	if (can_IO.trigger.T1A)
		*(float *) &data[0] = 1;
	else
		*(float *) &data[0] = 0;

	if (can_IO.trigger.T1B)
		*(float *) &data[4] = 1;
	else
		*(float *) &data[4] = 0;

	comm_can_transmit_eid(MCL_Trigger1 + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_Trigger2(void)
{
	float data[8];

	if (can_IO.trigger.T2A)
		*(float *) &data[0] = 1;
	else
		*(float *) &data[0] = 0;

	if (can_IO.trigger.T2B)
		*(float *) &data[4] = 1;
	else
		*(float *) &data[4] = 0;

	comm_can_transmit_eid(MCL_Trigger2 + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_Version(void)
{
	uint16_t data[3];
	data[0] = major_release;
	data[1] = minor_release;
	data[2] = branch;
	comm_can_transmit_eid(MCL_Version + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_HASH(void)
{
	uint64_t githash = HASH;
	comm_can_transmit_eid(MCL_Hash + CAN_base, (uint8_t *) &githash, 7);
}

void tx_UID(void)
{
	uint8_t UID[12];
	*((uint32_t *) &UID[0]) = STM32_UID[0];  // fetch UID from system memory
	*((uint32_t *) &UID[4]) = STM32_UID[1];
	*((uint32_t *) &UID[8]) = STM32_UID[2];

	*((uint64_t *) &UID[0]) += *((uint64_t *) &UID[6]);  // shrink UDI from 96bits to 48bits

	comm_can_transmit_eid(MCL_UID + CAN_base, (uint8_t *) &UID, 6);
}

void tx_Buttons(void)
{
	uint8_t buttons[4];

	buttons[0] = can_IO.buttons.silver;
	buttons[1] = can_IO.buttons.green;
	buttons[2] = can_IO.buttons.blue;
	buttons[3] = can_IO.buttons.red;

	can_IO.buttons.all = 0;

	comm_can_transmit_eid(MCL_Buttons + CAN_base, (uint8_t *) &buttons, 6);
}

void tx_Errors(void)
{
	comm_can_transmit_eid(MCL_Errors + CAN_base, (uint8_t *) &errors.all, sizeof(errors.all));
}

void tx_charge_mode(void)
{
	comm_can_transmit_eid(MCL_ChargeMode_rd + CAN_base, &charge_mode, sizeof(charge_mode));
}

void tx_Debugging(void)
{

}

static void rx_callback(uint32_t id, uint8_t *data, uint8_t len, uint8_t rtr)
{
	float speed, limit;
	static mc_configuration conf;
	(void) len;

	if ((id >=  + CAN_base) && (id <  + (CAN_base + 0x1000)))  // frame for us?
		id -= CAN_base;  // subtract base id
	else
		return;

	if ((id >= 0x100) && (id < 0x400))  // frame for BMS?
	{
		SCEN2_Battery_RX(id, data, len, rtr);
		return;
	}

	switch (id)
	{
		case MCL_CAN_ID_base_wr:
			if (data[0] == 15)
			{
				if (data[1] == 0)
					CAN_base = 0;
				else if (data[1] == 1)
					CAN_base = 0x1000;
				else if (data[1] == 2)
					CAN_base = 0x2000;
			}
		break;

		case MCL_ThrowBattOff_wr:
			if (*((uint16_t *) &data[0]) == 666)
			{
				BAT_RIGHT_SPLY_OFF();
				BAT_LEFT_SPLY_OFF();
			}
		break;

		case MCL_UID:
			tx_UID();
		break;

		case MCL_Version:
			tx_Version();
		break;

		case MCL_Hash:
			tx_HASH();
		break;

		case MCL_Reset:
			__disable_irq();  // watchdog will reset system
			while(1);
		break;

		case MCL_Jump_Bootloader:
			if (*((uint16_t *) &data[0]) == 666)
			{
				// jump to bootloader
			}
		break;

		case MCL_Errors:
			tx_Errors();
		break;

		case MCL_Buttons:
			tx_Buttons();
		break;

		case MCL_Trigger1:
			tx_Trigger1();
		break;

		case MCL_Trigger2:
			tx_Trigger2();
		break;

		case MCL_Voltages:
			tx_Voltages();
		break;

		case MCL_Temperatures:
			tx_Temperatures();
		break;

		case MCL_Temperature_Limits_wr:
			conf = *mc_interface_get_configuration();

			limit = *((float *) &data[0]);
			conf.l_temp_fet_start = limit;

			limit = *((float *) &data[4]);
			// Watertemp limit: ToDo
			conf_general_store_mc_configuration(&conf);
			mc_interface_set_configuration(&conf);
			chThdSleepMilliseconds(200);
		break;

		case MCL_Temperature_Limits_rd:
			tx_Temperature_Limits();
		break;

		case MCL_MotorTemp:
			tx_MotorTemp();
		break;

		case MCL_MotorTemp_Limit_wr:
			conf = *mc_interface_get_configuration();

			limit = *((float *) &data[0]);
			conf.l_temp_motor_start = limit;
			conf_general_store_mc_configuration(&conf);
			mc_interface_set_configuration(&conf);
			chThdSleepMilliseconds(200);
		break;

		case MCL_MotorTemp_Limit_rd:
			tx_MotorTemp_Limit();
		break;

		case MCL_Pressure:
			tx_Preassure();
		break;

		case MCL_LeakageSensor:
			tx_LeakageSensor();
		break;

		case MCL_LeakageSensorThreshold_wr:
			// ToDo: NV memory
		break;

		case MCL_LeakageSensorThreshold_rd:
			tx_LeakageSensorThreshold();
		break;

		case MCL_DC_Power:
			tx_DCPower();
		break;

		case MCL_MotorSpeed_wr:
			speed = *((int16_t *) data)*POLE_PAIR_COUNT;

			if (abs(speed - speed_save) > 1)  // floats...
			{
				if (speed_save < 1)  // startup
				{
					mc_interface_set_current(10);
					start_up_timer = chVTGetSystemTime();
					starting_up = true;
				}
				else if (!starting_up)
					mc_interface_set_pid_speed(speed);
			}

			speed_save = speed;
			timeout_reset();
		break;

		case MCL_MotorSpeed_rd:
			tx_MotorSpeed();
		break;

		case MCL_MotorCurrents:
			tx_MotorCurrents();
		break;

		case MCL_MC_PID_wr:
			conf = *mc_interface_get_configuration();

			conf.p_pid_kp = (*(uint16_t *) &data[0])/10000;
			conf.p_pid_ki = (*(uint16_t *) &data[2])/10000;
			conf.p_pid_kd = (*(uint16_t *) &data[4])/10000;

			conf_general_store_mc_configuration(&conf);
			mc_interface_set_configuration(&conf);
			chThdSleepMilliseconds(200);
		break;

		case MCL_MC_PID_rd:
			tx_MC_PID();
		break;

		case MCL_ChargeMode_wr:
			charge_mode = data[0];
		break;

		case MCL_ChargeMode_rd:
			tx_charge_mode();
		break;

		case MCL_ChargeCurrent:
			tx_ChargeCurrent();
		break;
	}
}

void SCEN2_CAN_handler(void)
{
	if (can_IO.buttons.all)
		tx_Buttons();

#ifdef SCEN2_debugging_enable
	if (chVTTimeElapsedSinceX(can_basic_update) > MS2ST(CAN_DELAY_BASIC))
	{
		can_basic_update = chVTGetSystemTime();
		tx_Debugging();
	}
#endif

	if ((starting_up) && (chVTTimeElapsedSinceX(start_up_timer) > MS2ST(START_UP_TIME)))
	{
		starting_up = false;
		mc_interface_set_pid_speed(speed_save);
	}

	if (timeout_has_timeout())
		speed_save = 0;
}

void SCEN2_CAN_init(void)
{
	can_basic_update = chVTGetSystemTime();
	start_up_timer = chVTGetSystemTime();

	comm_can_set_sid_rx_callback(&rx_callback);
//	c_conf = mc_interface_get_configuration();
}
