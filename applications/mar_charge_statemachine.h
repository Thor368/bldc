/*
 * charge_statemachine.h
 *
 *  Created on: 07.10.2020
 *      Author: alexander.schroeder
 */

#ifndef APPLICATIONS_MAR_CHARGE_STATEMACHINE_H_
#define APPLICATIONS_MAR_CHARGE_STATEMACHINE_H_

#include <stdbool.h>

typedef enum
{
	chgst_init = 0,				// 0
	chgst_wait_for_init,		// 1
	chgst_wait_for_enable,		// 2
	chgst_wait_for_charger,		// 3
	chgst_wait_equalize,		// 4
	chgst_wait_settle,			// 5
	chgst_charging,				// 6
	chgst_charge_finished,		// 7
	chgst_error,				// 8
	chgst_wait_for_reset		// 9
} CHG_state_t;

extern CHG_state_t chg_state;


void chg_init(void);

void charge_statemachine(void);

#endif /* APPLICATIONS_MAR_CHARGE_STATEMACHINE_H_ */
