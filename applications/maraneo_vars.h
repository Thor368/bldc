/*
 * maraneo_vars.h
 *
 *  Created on: 07.10.2020
 *      Author: alexander.schroeder
 */

#ifndef APPLICATIONS_MARANEO_VARS_H_
#define APPLICATIONS_MARANEO_VARS_H_

#define MAR_CONF_VERSION	1

extern volatile float U_DC, U_DC_filt;
extern volatile float U_CHG, U_CHG_filt;
extern volatile float I_CHG, I_CHG_filt, I_CHG_offset;
extern volatile float I_BAT;
extern volatile float SoC;

extern volatile uint32_t Sleep_Time;

extern volatile bool charge_en;

extern volatile bool Motor_lock;
extern volatile uint32_t Motor_lock_timer;

extern volatile bool Stand_Alone;



#endif /* APPLICATIONS_MARANEO_VARS_H_ */
