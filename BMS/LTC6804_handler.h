#include "LTC6804_types.h"

#define BMS_Calc_Current(index, offset)         (((int32_t) chips[index].chip.AVAR.G1V - offset)*BMS_C_PER_MA) // Calculate Current in mA

typedef enum
{
	RES,				// 0x00
	DISCOVER,			// 0x01
	TEST_MUX,			// 0x02
	TEST_CV,			// 0x03
	TEST_GPIO,			// 0x04
	TEST_ST,			// 0x05
	OPEN_SOURCE_TEST1,	// 0x06
	OPEN_SOURCE_TEST2,	// 0x07
	OPEN_SINK_TEST1,	// 0x08
	OPEN_SINK_TEST2,	// 0x09
	TEST_REF,			// 0x0A
	TEST_STATUS,		// 0x0B
	TEST_END,			// 0x0C
	SAMPLE_ITMP,		// 0x0D
	READ_ITMP,			// 0x0E
	SAMPLE_AUX,			// 0x0F
	READ_AUX,			// 0x10
	SAMPLE_CV,			// 0x11
	READ_CV,			// 0x12
	WAIT				// 0x13
} BMS_Status_t;

typedef enum
{
	BMS_Health_OK = 0,
	BMS_Health_Error_Operational = 1,
	BMS_Health_Error_Non_Operational = 2
} BMS_Health_t;

typedef enum
{
	BMS_AUX_None = 0,
	BMS_AUX_Current = 1,
	BMS_AUX_Temp = 2,
	BMS_AUX_Current_Temp = 3
} BMS_AUX_t;

typedef struct
{
	LTC_DATASET_t chip;
	
	BMS_Status_t Status;
	int32_t last_CV, last_TEST;
	
	bool BMS_present;
	
	uint32_t Int_Temp;

	bool Cell_Test_Passed;
	bool GPIO_Test_Passed;
	bool Status_Test_Passed;
	bool MUX_Test_Passed;
	bool Secondary_Reference_OK;
	bool Int_Temp_OK;
	bool VA_OK;
	bool VD_OK;
	bool Open_Cell_Connection[12];
	bool Wrong_Cell_Count;
	BMS_Health_t Health;
	BMS_AUX_t Aux;
	
	uint32_t Cell_U[12];
	uint32_t Cell_Min_U;
	uint32_t Cell_Min_index;
	uint32_t Cell_Max_U;
	uint32_t Cell_Max_index;
	uint32_t Cell_All_U;
	uint32_t Cell_Avr_U;
	uint32_t Cell_Count;
	
	bool Cell_OV[12];
	uint32_t Cell_OV_timer[12];
	bool Cell_UV[12];
	uint32_t Cell_UV_timer[12];
	
	bool Cell_Bleed[12];
	bool Balance_Permission;
	uint32_t Balance_derating;
	uint32_t Balance_timer;
	
	int32_t Cell_Sink_U[12];
	int32_t Cell_Source_U[12];

	int32_t Temp_sensors[5];
} BMS_t;

extern BMS_t chips[5];

extern bool BMS_Balance_Scheduled;

extern float Global_Max_U;
extern float Global_Min_U;

extern bool BMS_Charge_permitted;
extern bool BMS_Discharge_permitted;
extern bool BMS_fault_latch;

void LTC_handler_Init(void);

void LTC_handler(void);
