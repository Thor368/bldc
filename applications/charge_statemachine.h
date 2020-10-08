/*
 * charge_statemachine.h
 *
 *  Created on: 07.10.2020
 *      Author: alexander.schroeder
 */

#ifndef APPLICATIONS_CHARGE_STATEMACHINE_H_
#define APPLICATIONS_CHARGE_STATEMACHINE_H_

#include <stdbool.h>

typedef enum
{
	chgst_init = 0,
	chgst_wait_for_init,
	chgst_wait_for_enable,
	chgst_wait_for_charger,
	chgst_wait_1s,
	chgst_charging,
	chgst_charge_finished,
	chgst_error
} CHG_state_t;

extern CHG_state_t chg_state;


void chg_init(void);

void charge_statemachine(void);

#endif /* APPLICATIONS_CHARGE_STATEMACHINE_H_ */
