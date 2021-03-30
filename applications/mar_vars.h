/*
 * maraneo_vars.h
 *
 *  Created on: 07.10.2020
 *      Author: alexander.schroeder
 */

#ifndef APPLICATIONS_MAR_VARS_H_
#define APPLICATIONS_MAR_VARS_H_

#define MAR_CONF_VERSION	1

extern volatile float U_DC, U_DC_filt;
extern volatile float U_CHG, U_CHG_filt;
extern volatile float I_CHG, I_CHG_filt, I_CHG_offset;
extern volatile float I_BAT;
extern volatile float SoC;
extern volatile float I_CHG_max;

extern volatile uint32_t sleep_time;
extern volatile uint32_t CAN_HBT_timeout;

extern volatile bool charge_enable;
extern volatile float discharge_SoC;
extern volatile bool discharge_enable;

extern volatile bool stand_alone;



#endif /* APPLICATIONS_MAR_VARS_H_ */
