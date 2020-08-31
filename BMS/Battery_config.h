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

extern uint32_t BMS_chip_count;
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
extern float BMS_soft_OC;
extern float BMS_OC_Delay;
extern float BMS_hard_OC;
extern float BMS_soft_COT;
extern float BMS_COT_Delay;
extern float BMS_hard_COT;
extern float BMS_soft_DOT;
extern float BMS_DOT_Delay;
extern float BMS_soft_UT;
extern float BMS_UT_Delay;
extern float BMS_hard_UT;

#ifndef BMS_ITMP_LIM			// [°C] Temperature limit while balancing
#define BMS_ITMP_LIM			70
#endif

#ifndef BMS_ITMP_HYST			// [°C] Hysteresis for temperature limit
#define BMS_ITMP_HYST			8
#endif

#ifndef BMS_I_SENSOR			// [A/c] Counts per mA
#define BMS_I_SENSOR
#define BMS_C_PER_MA			0.0090909090909090909090909090909091
#endif

#ifndef BMS_Temp_beta			// beta of connected NTCs
#define BMS_Temp_beta			3380
#endif

#ifndef BMS_enable_charge		// Define which IO is for charge enable
#define BMS_enable_charge		palSetPad(GPIOA, 6)
#endif

#ifndef BMS_disable_charge
#define BMS_disable_charge		palClearPad(GPIOA, 6)
#endif

#ifndef BMS_fault_delay
#define BMS_fault_delay			10000
#endif

#endif /* BMS_BATTERY_CONFIG_H_ */
