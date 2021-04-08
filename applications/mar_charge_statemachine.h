/*
 * charge_statemachine.h
 *
 *  Created on: 07.10.2020
 *      Author: alexander.schroeder
 */

#ifndef APPLICATIONS_MAR_CHARGE_STATEMACHINE_H_
#define APPLICATIONS_MAR_CHARGE_STATEMACHINE_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
	cpst_init = 0,				// 0
	cpst_wait_for_init,			// 1
	cpst_wait_charge_allowed,	// 2
	cpst_wait_for_enable,		// 3
	cpst_discover,				// 4

	chgst_wait_equalize = 0x10,	// 10
	chgst_wait_settle,			// 11
	chgst_charging,				// 12
	chgst_charge_finished,		// 13
	cpst_error,					// 14
	cpst_wait_for_restart,		// 15
	cpst_wait_for_reset,		// 16

	batst_init = 0x80,			// 80
	batst_connect,				// 81
	batst_run_parallel,			// 82
	batst_run_internal,			// 83
	batst_switch_external,		// 84
	batst_run_external,			// 85
	batst_bat_lost				// 86
} CHG_state_t;

extern CHG_state_t cp_state;
extern bool charger_detected;
extern uint32_t charge_cycles;

extern float external_battery_discharge_limit;


void charge_statemachine(void);

#endif /* APPLICATIONS_MAR_CHARGE_STATEMACHINE_H_ */
