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
	chgst_init = 0,				// 0
	chgst_wait_for_init,		// 1
	chgst_wait_charge_allowed,	// 2
	chgst_wait_for_enable,		// 3
	chgst_wait_for_charger,		// 4
	chgst_wait_equalize,		// 5
	chgst_wait_settle,			// 6
	chgst_charging,				// 7
	chgst_charge_finished,		// 8
	chgst_error,				// 9
	chgst_wait_for_restart,			// A
	chgst_wait_for_reset		// B
} CHG_state_t;

extern CHG_state_t chg_state;
extern bool charger_detected;
extern uint32_t charge_cycles;


void chg_init(void);

void charge_statemachine(void);

#endif /* APPLICATIONS_MAR_CHARGE_STATEMACHINE_H_ */
