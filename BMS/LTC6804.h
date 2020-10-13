#ifndef LTC6804
#define LTC6804

#include "LTC6804_types.h"
#include <stdbool.h>

#define LTC_ADDRESS(x)					(0x8000 | (x << 11))
#define LTC_BROADCAST					0x0000

#define LTC_REGISTER_CFGR				0x001
#define LTC_REGISTER_CVAR				0x002
#define LTC_REGISTER_CVBR				0x004
#define LTC_REGISTER_CVCR				0x008
#define LTC_REGISTER_CVDR				0x010
#define LTC_REGISTER_AVAR				0x020
#define LTC_REGISTER_AVBR				0x040
#define LTC_REGISTER_STAR				0x080
#define LTC_REGISTER_STBR				0x100
#define LTC_REGISTER_COMM				0x200

#define LTC_REGISTER_CV					(LTC_REGISTER_CVAR | LTC_REGISTER_CVBR | LTC_REGISTER_CVCR | LTC_REGISTER_CVDR)
#define LTC_REGISTER_AV					(LTC_REGISTER_AVAR | LTC_REGISTER_AVBR)
#define LTC_REGISTER_ST					(LTC_REGISTER_STAR | LTC_REGISTER_STBR)
#define LTC_ALL_REGISTERS				(LTC_REGISTER_CFGR | LTC_REGISTER_CV | LTC_REGISTER_AV | LTC_REGISTER_ST | LTC_REGISTER_COMM)

#define LTC_START_ADCV					1
#define LTC_START_ADOW					2
#define LTC_START_CVST					3
#define LTC_START_ADAX					4
#define LTC_START_AXST					5
#define LTC_START_ADSTAT				6
#define LTC_START_STATST				7
#define LTC_START_ADCVAX				8
#define LTC_START_PLADC					9
#define LTC_START_DIAGN					10
#define LTC_START_STCOMM				11

#define LTC_VOLT_C1						0
#define LTC_VOLT_C2						1
#define LTC_VOLT_C3						2
#define LTC_VOLT_C4						3
#define LTC_VOLT_C5						4
#define LTC_VOLT_C6						5
#define LTC_VOLT_C7						6
#define LTC_VOLT_C8						7
#define LTC_VOLT_C9						8
#define LTC_VOLT_C10					9
#define LTC_VOLT_C11					10
#define LTC_VOLT_C12					11
#define LTC_VOLT_GPIO1					12
#define LTC_VOLT_GPIO2					13
#define LTC_VOLT_GPIO3					14
#define LTC_VOLT_GPIO4					15
#define LTC_VOLT_GPIO5					16
#define LTC_VOLT_REF					17
#define LTC_VOLT_SOC					18
#define LTC_VOLT_ITMP					19
#define LTC_VOLT_VA						20
#define LTC_VOLT_VD						21


// Voltage conversion Macros
#define LTC_calc_Voltage(raw)			(raw*0.0001)					// Calculate Voltage in V
#define LTC_calc_SOC_Voltage(raw)		(raw*0.002)						// Calculate Sum Of Cell Voltage in V
#define LTC_calc_int_Temp(raw)			(raw*0.013333 - 273)	 		// Calculate internal Temperature in Celsius
#define LTC_TEMP(adc)					((1.0 / ((logf((adc*10000/(33000 - adc))) / 10000.0) / BMS_Temp_beta) + (1.0 / 298.15)) - 273.15)


// INIT AND CONFIG
void LTC_Init(void);


// HIGH LEVEL FUNCTIONS

// Read Register from Target
uint8_t LTC_Read_Register(LTC_DATASET_t* set, uint16_t reg);

// Write Register to Target
uint8_t LTC_Write_Register(LTC_DATASET_t* set, uint16_t reg);

// Send Start Command
uint8_t LTC_Start(LTC_DATASET_t* set, uint8_t action);

// Get raw cell voltage from RAM
uint16_t LTC_get_Voltage_raw(LTC_DATASET_t* chip, uint8_t channel);

// Get raw AUX voltage from RAM
uint16_t LTC_get_AUX_raw(LTC_DATASET_t* chip, uint8_t channel);

#endif
