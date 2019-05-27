/*
 * SCEN2_battery.c
 *
 *  Created on: 26.04.2019
 *      Author: alexander.schroeder
 */

#include "SCEN2_types.h"
#include "SCEN2_settings.h"
#include "SCEN2_error.h"

#include "hal.h"
#include "mc_interface.h"
#include "comm_can.h"

#define rtr_voltage(offset)			comm_can_transmit_eid_RTR(0x04 + offset)
#define rtr_error_warn(offset)		comm_can_transmit_eid_RTR(0x05 + offset)
#define rtr_SOC(offset)				comm_can_transmit_eid_RTR(0x08 + offset)
#define ping_all()					{rtr_voltage(0x100); rtr_voltage(0x200); rtr_voltage(0x300);}
#define rtr_all_offset(offset)		{rtr_voltage(offset); rtr_error_warn(offset); rtr_SOC(offset);}
#define rtr_all()					{rtr_all_offset(0x200); rtr_all_offset(0x300);}

typedef enum
{
	init,
	wait_for_governor,
	check_batteries,
	switch_to_left,
	configure_left,
	switch_to_right,
	configure_right,
	run
} Battery_state_t;

Battery_state_t batteries_state;

enum Battery_index_t
{
	offset_init = 0x100,
	offset_left = 0x200,
	offset_right = 0x300,
};

Battery_t battery_left, battery_right, battery_init;

void send_state(void)
{
	uint8_t data[1];
	data[0] = batteries_state;
	comm_can_transmit_eid(5, (uint8_t *) &data, sizeof(data));
}

void battery_set_offset(Battery_t *bat, uint32_t old_offset)
{
	uint8_t data[2];
	data[0] = 0x22;
	data[1] = bat->offset >> 8;
	comm_can_transmit_eid(old_offset + 0x0A, (uint8_t *) &data, sizeof(data));
}

void battery_reset(Battery_t *bat, uint32_t offset)
{
	bat->offset = offset;
	bat->time_last_seen = 0;
	bat->active = false;

	bat->OK_to_charge = false;
	bat->OK_to_discharge = false;

	bat->SOC = 0;
	bat->total_U = 0;

	bat->error_flags.all = 0;
	bat->warning_flags.all = 0;
}

void batteries_reset(void)
{
	battery_reset(&battery_init, 0x100);
	battery_reset(&battery_left, 0x200);
	battery_reset(&battery_right, 0x300);
}

void SCEN2_Battery_init(void)
{
	errors.battery_left_error = false;
	errors.battery_right_error = false;
	batteries_reset();
}

uint8_t SCEN2_battey_state(void)
{
	return batteries_state;
}

void SCEN2_Battery_RX(uint32_t id, uint8_t *data, uint8_t len, uint8_t rtr)
{
	(void) len;

	Battery_t *bat;

	if ((id == 0x5) && rtr)
		send_state();

	if (rtr)
		return;

	if ((id >= 0x100) && (id < 0x200))
	{
		bat = &battery_init;
		id -= 0x100;
	}
	else if ((id >= 0x200) && (id < 0x300))
	{
		bat = &battery_left;
		id -= 0x200;
	}
	else if ((id >= 0x300) && (id < 0x400))
	{
		bat = &battery_right;
		id -= 0x300;
	}
	else
		return;

	bat->time_last_seen = chVTGetSystemTime();
	bat->active = true;

	switch (id)
	{
	case 0x04:
		bat->total_U = ((float) *(uint16_t *) &data[0])/1000;
	break;

	case 0x05:
		bat->error_flags.all = data[0];
		bat->warning_flags.all = data[1];
	break;

	case 0x08:
		switch (data[5])
		{
		case 0:
			bat->OK_to_charge = true;
			bat->OK_to_discharge = false;
		break;

		case 1:
			bat->OK_to_charge = false;
			bat->OK_to_discharge = true;
		break;

		case 2:
			bat->OK_to_charge = true;
			bat->OK_to_discharge = true;
		break;
		}

		bat->SOC = ((float) *(uint16_t *) &data[6])/1000;
	}
}

void SCEN2_Battery_handler(void)
{
	static systime_t timer;
	static uint32_t cc = 0;


	switch (batteries_state)
	{
	case init:
		SCEN2_Battery_init();

		BAT_LEFT_SPLY_OFF();
		BAT_RIGHT_SPLY_OFF();

		batteries_state = wait_for_governor;
		send_state();
	break;

	case wait_for_governor:
		if (governor_state == gv_init_BMS)
		{
			BAT_LEFT_SPLY_ON();
			BAT_RIGHT_SPLY_ON();

			cc = 0;
			timer = chVTGetSystemTime();

			batteries_state = check_batteries;
			send_state();
		}
	break;

	case check_batteries:
		if (battery_init.active || (cc >= 10))
		{
			BAT_RIGHT_SPLY_OFF();

			cc = 0;
			timer = chVTGetSystemTime();

			batteries_state = switch_to_left;
			send_state();
		}
		else if (battery_left.active && battery_right.active)
		{
			batteries_state = run;
			send_state();
		}
		else if (chVTTimeElapsedSinceX(timer) > MS2ST(100))
		{
			cc++;
			timer = chVTGetSystemTime();
			ping_all();
		}
	break;

	case switch_to_left:
		if (chVTTimeElapsedSinceX(timer) > MS2ST(10000))
		{
			batteries_reset();
			ping_all();

			cc = 0;
			timer = chVTGetSystemTime();

			batteries_state = configure_left;
			send_state();
		}
	break;

	case configure_left:
		if (battery_init.active)
			battery_set_offset(&battery_left, offset_init);
		else if (battery_left.active || (cc >= 10))
		{
			BAT_RIGHT_SPLY_ON();
			BAT_LEFT_SPLY_OFF();
			if (cc >= 10)
				errors.battery_left_error = true;

			timer = chVTGetSystemTime();

			batteries_state = switch_to_right;
			send_state();
		}
		else if (battery_right.active)
			battery_set_offset(&battery_left, offset_right);
		else if (chVTTimeElapsedSinceX(timer) > MS2ST(100))
		{
			ping_all();
			timer = chVTGetSystemTime();
			cc++;
		}
	break;

	case switch_to_right:
		if (chVTTimeElapsedSinceX(timer) > MS2ST(10000))
		{
			batteries_reset();
			ping_all();

			cc = 0;
			timer = chVTGetSystemTime();

			batteries_state = configure_right;
			send_state();
		}
	break;

	case configure_right:
		if (battery_init.active)
			battery_set_offset(&battery_right, offset_init);
		else if (battery_left.active)
			battery_set_offset(&battery_right, offset_left);
		else if (battery_right.active || (cc >= 10))
		{
			BAT_LEFT_SPLY_ON();
			if (cc >= 10)
				errors.battery_right_error = true;

			timer = chVTGetSystemTime();

			governor_state = gv_run;
			batteries_state = run;
			send_state();
		}
		else if (chVTTimeElapsedSinceX(timer) > MS2ST(100))
		{
			ping_all();
			timer = chVTGetSystemTime();
			cc++;
		}
	break;

	case run:
		if (chVTTimeElapsedSinceX(battery_left.time_last_seen) > MS2ST(1000))
			battery_left.active = false;
		if (chVTTimeElapsedSinceX(battery_right.time_last_seen) > MS2ST(1000))
			battery_right.active = false;

//		errors.battery_left_error |= !battery_left.active;
//		errors.battery_right_error |= !battery_right.active;
	break;
	}
}
