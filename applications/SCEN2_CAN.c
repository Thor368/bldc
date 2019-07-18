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

#define RIGHT				0
#define LEFT				1

systime_t can_basic_update;
systime_t start_up_timer;

bool starting_up = false;
float speed_save = 0;

uint32_t CAN_base;

void tx_ChargeCurrent(void)
{
#ifdef SCEN2_debugging_enable
	int16_t data[3];
	data[0] = analog_IO.I_charge_raw*1000;
	data[1] = analog_IO.I_charge_offset*1000;
	data[2] = analog_IO.I_charge*1000;
#else
	float data;
	if (charge_mode  == charger_detected_with_ACK)
#ifdef SCEN2_emulate_CHG
		data = 6.5;
#else
		data = analog_IO.I_charge;
#endif
	else
		data = 0;
#endif
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
	comm_can_transmit_eid(MCL_LeakageSensorThreshold + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_MC_PID(void)
{
	uint16_t data[3];
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	data[0] = (uint16_t) lround(conf->p_pid_kp)*10000;
	data[1] = (uint16_t) lround(conf->p_pid_ki)*10000;
	data[2] = (uint16_t) lround(conf->p_pid_kd)*10000;
	comm_can_transmit_eid(MCL_MC_PID + CAN_base, (uint8_t *) &data, sizeof(data));
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
	data[0] = mc_interface_get_tot_current_filtered()/1.414;
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
	comm_can_transmit_eid(MCL_Temperature_Limits + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_MotorTemp_Limit(void)
{
	float data[1];
	const volatile mc_configuration *conf = mc_interface_get_configuration();
	data[0] = conf->l_temp_motor_start;
	comm_can_transmit_eid(MCL_MotorTemp_Limit + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_MotorSpeed_actual(void)
{
	float data[1];
	data[0] = mc_interface_get_rpm()/POLE_PAIR_COUNT;
	if (data[0] < 100)
		data[0] = 0;
	comm_can_transmit_eid(MCL_MotorSpeed_actual + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_DCPower(void)
{
	float data[1];
	data[0] = mc_interface_get_tot_current_in_filtered()*GET_INPUT_VOLTAGE();
	comm_can_transmit_eid(MCL_DC_Power + CAN_base, (uint8_t *) &data, sizeof(data));
}

void tx_Buttons(uint8_t RL)
{
	uint8_t data;

	if (RL&RIGHT)
	{
		data = digital_IO.buttons.all & 0xFFFF;
		comm_can_transmit_eid(MCL_Buttons_right + CAN_base, &data, 2);
	}
	if (RL&LEFT)
	{
		data = digital_IO.buttons.all >> 16;
		comm_can_transmit_eid(MCL_Buttons_left + CAN_base, &data, 2);
	}
}

void tx_Trigger(uint8_t RL)
{
	float data;

	if (RL&RIGHT)
	{
		if (errors.trigger_error_right)
			data = NAN;
		else if (digital_IO.trigger.T1)
			data = 1;
		else
			data = 0;

		comm_can_transmit_eid(MCL_Trigger_right + CAN_base, (uint8_t *) &data, sizeof(data));
	}
	if (RL&LEFT)
	{
		if (errors.trigger_error_left)
			data = NAN;
		else if (digital_IO.trigger.T2)
			data = 1;
		else
			data = 0;

		comm_can_transmit_eid(MCL_Trigger_left + CAN_base, (uint8_t *) &data, sizeof(data));
	}
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

void tx_Debugging(void)
{
	uint16_t data[4];

	data[0] = ADC_Value[0];
	data[1] = ADC_Value[1];
	data[2] = ADC_Value[2];
	data[3] = ADC_Value[3];
	comm_can_transmit_eid(0, (uint8_t *) &data, sizeof(data));

	data[0] = ADC_Value[4];
	data[1] = ADC_Value[5];
	data[2] = ADC_Value[6];
	data[3] = ADC_Value[7];
	comm_can_transmit_eid(1, (uint8_t *) &data, sizeof(data));

	data[0] = ADC_Value[8];
	data[1] = ADC_Value[9];
	data[2] = ADC_Value[10];
	data[3] = ADC_Value[11];
	comm_can_transmit_eid(2, (uint8_t *) &data, sizeof(data));

	data[0] = ADC_Value[12];
	data[1] = ADC_Value[13];
	data[2] = ADC_Value[14];
	data[3] = ADC_Value[15];
	comm_can_transmit_eid(3, (uint8_t *) &data, sizeof(data));
}

void rx_rtr_handler(uint32_t id)
{
	switch (id)
	{
		case MCL_UID:
			tx_UID();
		break;

		case MCL_Version:
			tx_Version();
		break;

		case MCL_Hash:
			tx_HASH();
		break;

		case MCL_Errors:
			comm_can_transmit_eid(MCL_Errors + CAN_base, (uint8_t *) &errors.all, sizeof(errors.all));
		break;

		case MCL_Buttons_right:
			tx_Buttons(RIGHT);
		break;

		case MCL_Buttons_left:
			tx_Buttons(LEFT);
		break;

		case MCL_Trigger_right:
			tx_Trigger(RIGHT);
		break;

		case MCL_Trigger_left:
			tx_Trigger(LEFT);
		break;

		case MCL_Voltages:
			tx_Voltages();
		break;

		case MCL_Temperatures:
			tx_Temperatures();
		break;

		case MCL_Temperature_Limits:
			tx_Temperature_Limits();
		break;

		case MCL_MotorTemp:
			comm_can_transmit_eid(MCL_MotorTemp + CAN_base, (uint8_t *) &analog_IO.temp_motor, sizeof(analog_IO.temp_motor));
		break;

		case MCL_MotorTemp_Limit:
			tx_MotorTemp_Limit();
		break;

		case MCL_Pressure:
			comm_can_transmit_eid(MCL_Pressure + CAN_base, (uint8_t *) &analog_IO.pressure, sizeof(analog_IO.pressure));
		break;

		case MCL_LeakageSensor:
			tx_LeakageSensor();
		break;

		case MCL_LeakageSensorThreshold:
			tx_LeakageSensorThreshold();
		break;

		case MCL_DC_Power:
			tx_DCPower();
		break;

		case MCL_MotorSpeed_actual:
			tx_MotorSpeed_actual();
		break;

		case MCL_MotorSpeed_set:
			comm_can_transmit_eid(MCL_MotorSpeed_set + CAN_base, (uint8_t *) &speed_save, sizeof(speed_save));
		break;


		case MCL_MotorCurrents:
			tx_MotorCurrents();
		break;

		case MCL_MC_PID:
			tx_MC_PID();
		break;

		case MCL_ChargeMode:
			comm_can_transmit_eid(MCL_ChargeMode + CAN_base, &charge_mode, sizeof(charge_mode));
		break;

		case MCL_ChargeCurrent:
			tx_ChargeCurrent();
		break;

		case MCL_12V_control:
			comm_can_transmit_eid(MCL_12V_control + CAN_base, (uint8_t *) &digital_IO.supply_override.all, sizeof(digital_IO.supply_override.all));
		break;
	}
}

void rx_wr_handler(uint32_t id, uint8_t *data)
{
	float speed, limit;
	static mc_configuration conf;

	switch (id)
	{
		case MCL_CAN_ID_base:
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

		case MCL_ThrowBattOff:
			if (*((uint16_t *) &data[0]) == 666)
			{
				digital_IO.supply.AKK_left = false;
				digital_IO.supply.AKK_right = false;
			}
		break;

		case MCL_Temperature_Limits:
			conf = *mc_interface_get_configuration();

			limit = *((float *) &data[0]);
			conf.l_temp_fet_start = limit;

			limit = *((float *) &data[4]);
			// Watertemp limit: ToDo
			conf_general_store_mc_configuration(&conf);
			mc_interface_set_configuration(&conf);
			chThdSleepMilliseconds(200);
		break;

		case MCL_MotorTemp_Limit:
			conf = *mc_interface_get_configuration();

			limit = *((float *) &data[0]);
			conf.l_temp_motor_start = limit;
			conf_general_store_mc_configuration(&conf);
			mc_interface_set_configuration(&conf);
			chThdSleepMilliseconds(200);
		break;

		case MCL_LeakageSensorThreshold:
			// ToDo: NV memory
			leakage_threashold = *((float *) &data[0]);
		break;

		case MCL_MotorSpeed_set:
			if (charge_mode == charger_detected_with_ACK)
				return;

			speed = *((float *) data)*POLE_PAIR_COUNT;

			if (speed < 20)
			{
				mc_interface_release_motor();
				starting_up = false;
			}
			else if (abs(speed - speed_save) > 1)  // floats...
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

		case MCL_MC_PID:
			conf = *mc_interface_get_configuration();

			conf.p_pid_kp = (*(uint16_t *) &data[0])/10000;
			conf.p_pid_ki = (*(uint16_t *) &data[2])/10000;
			conf.p_pid_kd = (*(uint16_t *) &data[4])/10000;

			conf_general_store_mc_configuration(&conf);
			mc_interface_set_configuration(&conf);
			chThdSleepMilliseconds(200);
		break;

		case MCL_ChargeMode:
			charge_mode = data[0];
		break;

		case MCL_Reset:
			if (*((uint16_t *) &data[0]) == 666)
			{
				__disable_irq();  // watchdog will reset system
				while(1);
			}
		break;

		case MCL_Jump_Bootloader:
			if (*((uint16_t *) &data[0]) == 666)
			{
				// jump to bootloader
			}
		break;

		case MCL_12V_control:
			digital_IO.supply_override.power_on = data[0];
			digital_IO.supply_override.HMI_supply = data[1];
			digital_IO.supply_override.AKK_right = data[2];
			digital_IO.supply_override.AKK_left = data[3];
		break;
	}
}

static void rx_callback(CANRxFrame *msg)
{
#ifdef SCEN2_debugging_enable
	SCEN2_Battery_RX(msg);
#endif

	if ((msg->EID >= (MCL_CAN_ID_base + CAN_base)) && (msg->EID < (MCL_CAN_ID_base + CAN_base + 0x1000)))  // frame for us?
	{
		msg->EID -= CAN_base;  // subtract base id
	}
//#ifndef SCEN2_debugging_enable
//	else if ((msg->EID >= 0x100) && (msg->EID))  // frame for BMS?
//	{
//		SCEN2_Battery_RX(msg);
//		return;
//	}
//#endif
	else
	{
		return;
	}

	SCEN2_Battery_RX(msg);
	if (msg->RTR)
		rx_rtr_handler(msg->EID);
	else
		rx_wr_handler(msg->EID, msg->data8);
}

void SCEN2_CAN_handler(void)
{
	static uint32_t buttons = 0;
	if ((buttons != digital_IO.buttons.all))
	{
		buttons = digital_IO.buttons.all;
		tx_Buttons(LEFT|RIGHT);
	}

	static uint16_t trigger = 0;
	if (trigger != digital_IO.trigger.all)
	{
		trigger = digital_IO.trigger.all;
		tx_Trigger(LEFT|RIGHT);
	}

#ifdef SCEN2_debugging_enable
	if (chVTTimeElapsedSinceX(can_basic_update) > MS2ST(CAN_DELAY_BASIC))
	{
		can_basic_update = chVTGetSystemTime();
		tx_Debugging();
	}
#endif

	if (starting_up)
	{
		if (chVTTimeElapsedSinceX(start_up_timer) > MS2ST(START_UP_TIME))
		{
			mc_interface_release_motor();
		}
		else if (abs(mc_interface_get_rpm()) >= 1000)
		{
			starting_up = false;
			mc_interface_set_pid_speed(speed_save);
		}
	}
	if (charge_mode == charger_detected_with_ACK)
	{
		mc_interface_release_motor();
		speed_save = 0;
	}

	if (timeout_has_timeout())
	{
		starting_up = false;
		speed_save = 0;
	}
}

void SCEN2_CAN_init(void)
{
	CAN_base = 0;

	can_basic_update = chVTGetSystemTime();
	start_up_timer = chVTGetSystemTime();

	comm_can_set_app_rx_callback(&rx_callback);
//	c_conf = mc_interface_get_configuration();

	digital_IO.supply_override.all = 0;
}
