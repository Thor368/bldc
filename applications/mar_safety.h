/*
 * mar_safety.h
 *
 *  Created on: 18.02.2021
 *      Author: AUKTORAAlexanderSchr
 */

#ifndef APPLICATIONS_MAR_SAFETY_H_
#define APPLICATIONS_MAR_SAFETY_H_

void safety_Init(void);

void safety_checks(void);

void safety_lock_motor(void);

void safety_reset_sleep_counter(void);

extern float AUX_temp_cutoff;
extern bool motor_lock;

#endif /* APPLICATIONS_MAR_SAFETY_H_ */
