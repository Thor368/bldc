/*
 * SCEN2_battery.h
 *
 *  Created on: 26.04.2019
 *      Author: alexander.schroeder
 */

#ifndef APPLICATIONS_SCEN2_BATTERY_H_
#define APPLICATIONS_SCEN2_BATTERY_H_

void SCEN2_Battery_init(void);

void SCEN2_Battery_RX(uint32_t id, uint8_t *data, uint8_t len, uint8_t rtr);

void SCEN2_Battery_handler(void);

#endif /* APPLICATIONS_SCEN2_BATTERY_H_ */
