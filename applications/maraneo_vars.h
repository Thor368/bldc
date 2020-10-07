/*
 * maraneo_vars.h
 *
 *  Created on: 07.10.2020
 *      Author: alexander.schroeder
 */

#ifndef APPLICATIONS_MARANEO_VARS_H_
#define APPLICATIONS_MARANEO_VARS_H_

extern volatile float U_DC, U_DC_filt;
extern volatile float U_CHG, U_CHG_filt;
extern volatile float I_CHG, I_CHG_filt, I_CHG_offset;

extern volatile bool charge_en;


#endif /* APPLICATIONS_MARANEO_VARS_H_ */
