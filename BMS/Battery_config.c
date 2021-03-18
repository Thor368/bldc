#include "Battery_config.h"

#ifdef BMS_cell_count_set						// Number of connected cells
uint32_t BMS_cell_count = BMS_cell_count_set;
#else
uint32_t BMS_cell_count = 12;
#endif

#ifdef BMS_temp_count_set						// Number of connected NTCs
uint32_t BMS_temp_count = BMS_temp_count_set;
#else
uint32_t BMS_temp_count = 2;
#endif

#ifdef BMS_soft_OV_set								// [V] Soft cell overvoltage threshold
float BMS_soft_OV = BMS_soft_OV_set;
#else
float BMS_soft_OV = 4.2;
#endif

#ifdef BMS_OV_delay_set							// [s] Soft cell overvoltage trip time
float BMS_OV_delay = BMS_OC_delay_set;
#else
float BMS_OV_delay = 1;
#endif

#ifdef BMS_hard_OV_set						// [V] Hard cell overvoltage threshold
float BMS_hard_OV = BMS_hard_OV_set;
#else
float BMS_hard_OV = 4.3;
#endif

#ifdef BMS_OV_recovery_set					// [V] Voltage where charging is reenabled
float BMS_OV_recovery = BMS_OV_recovery_set;
#else
float BMS_OV_recovery = 4.15;
#endif

#ifdef BMS_Balance_U_set					// [V] Voltage at which balancing is enabled
float BMS_ = BMS_Balance_U_set;
#else
float BMS_Balance_U = 3.9;
#endif

#ifdef BMS_soft_UV_set						// [V] Soft cell undervoltage threshold
float BMS_soft_UV = BMS_soft_UV_set;
#else
float BMS_soft_UV = 2.8;
#endif

#ifdef BMS_UV_Delay_set						// [s] Soft cell undervoltage delay
float BMS_UV_Delay = BMS_UV_Delay_set;
#else
float BMS_UV_Delay = 5;
#endif

#ifdef BMS_hard_UV_set						// [V] Hard Cell undervoltage Threshold
float BMS_hard_UV = BMS_hard_UV_set;
#else
float BMS_hard_UV = 2.5;
#endif

#ifdef BMS_UV_recovery_set					// [V] Voltage at which discharging is reenabled
float BMS_UV_recovery = BMS_UV_recovery_set;
#else
float BMS_UV_recovery = 3.0;
#endif

#ifdef BMS_soft_COT_set						// [°C] Soft cell overtemperature while charging threshold
float BMS_soft_COT = BMS_soft_COT_set;
#else
float BMS_soft_COT = 35.0;
#endif

#ifdef BMS_COT_Delay_set					// [s] Soft cell overtemperature while charging delay
float BMS_COT_Delay = BMS_COT_Delay_set;
#else
float BMS_COT_Delay = 3.0;
#endif

#ifdef BMS_hard_COT_set						// [°C] Hard cell overtemperature while charging threshold
float BMS_hard_COT = BMS_hard_COT_set;
#else
float BMS_hard_COT = 40.0;
#endif

#ifdef BMS_soft_DOT_set						// [°C] Soft cell overtemperature while discharging threshold
float BMS_soft_DOT = BMS_soft_DOT_set;
#else
float BMS_soft_DOT = 50;
#endif

#ifdef BMS_DOT_Delay_set					// [s] Soft cell overtemperature while discharging delay
float BMS_DOT_Delay = BMS_DOT_Delay_set;
#else
float BMS_DOT_Delay = 3.0;
#endif

#ifdef BMS_hard_DOT_set						// [°C] Soft cell overtemperature while discharging threshold
float BMS_hard_DOT = BMS_hard_DOT_set;
#else
float BMS_hard_DOT = 65;
#endif

#ifdef BMS_soft_UT_set						// [°C] Soft cell undertemperature
float BMS_soft_UT = BMS_soft_UT_set;
#else
float BMS_soft_UT = -0.0;
#endif

#ifdef BMS_UT_Delay_set						// [s] Soft cell undertemperature delay
float BMS_UT_Delay = BMS_UT_Delay_set;
#else
float BMS_UT_Delay = 3.0;
#endif

#ifdef BMS_hard_UT_set						// [°C] Hard cell undertemperature threshold
float BMS_hard_UT = BMS_hard_UT_set;
#else
float BMS_hard_UT = -5.0;
#endif

#ifdef BMS_Temp_beta_set					// B25/50 value of connected sensors
uint23_t BMS_Temp_beta = BMS_Temp_beta_set;
#else
uint32_t BMS_Temp_beta = 3380;
#endif
