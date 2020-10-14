/*
 * CAN.h
 *
 *  Created on: 14.10.2020
 *      Author: main
 */

#ifndef APPLICATIONS_MAR_CAN_H_
#define APPLICATIONS_MAR_CAN_H_

extern volatile uint32_t CAN_timer;

void CAN_Init(void);

void CAN_Status(void);

#endif /* APPLICATIONS_MAR_CAN_H_ */
