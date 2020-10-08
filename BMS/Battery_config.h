/*
 * Battery_config.h
 *
 *  Created on: 31.08.2020
 *      Author: main
 */

#ifndef BMS_BATTERY_CONFIG_H_
#define BMS_BATTERY_CONFIG_H_

#include <stdint.h>
#include "hw.h"
#include "hal.h"

extern uint32_t BMS_cell_count;
extern uint32_t BMS_temp_count;
extern float BMS_OV;
extern float BMS_OV_delay;
extern float BMS_hard_OV;
extern float BMS_OV_recovery;
extern float BMS_Balance_U;
extern float BMS_soft_UV;
extern float BMS_UV_Delay;
extern float BMS_hard_UV;
extern float BMS_UV_recovery;
extern float BMS_soft_COT;
extern float BMS_COT_Delay;
extern float BMS_hard_COT;
extern float BMS_soft_DOT;
extern float BMS_DOT_Delay;
extern float BMS_soft_UT;
extern float BMS_UT_Delay;
extern float BMS_hard_UT;
extern uint32_t BMS_Temp_beta;

#ifndef BMS_ITMP_LIM			// [°C] Temperature limit while balancing
#define BMS_ITMP_LIM			70
#endif

#ifndef BMS_ITMP_HYST			// [°C] Hysteresis for temperature limit
#define BMS_ITMP_HYST			8
#endif

#endif /* BMS_BATTERY_CONFIG_H_ */
