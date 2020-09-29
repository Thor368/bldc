#include "LTC6804_handler.h"
#include "LTC6804.h"
#include "hw.h"
#include "Battery_config.h"

#include <math.h>
#include <stdlib.h>

BMS_t BMS;

bool BMS_Balance_Scheduled = false;
systime_t BMS_Balance_Timer;

float Global_Max_U = 0;
float Global_Min_U = 0;

#ifdef BMS_I_SENSOR
float Battery_I = 0;
uint32_t Battery_I_offset = 0;
uint32_t Battery_I_offset_coutner = 0;
uint32_t Battery_I_offset_timer = 0;
#endif

bool BMS_Charge_permitted = true;
bool BMS_Discharge_permitted = true;
bool BMS_fault_latch = false;
uint32_t BMS_fault_delay_timer = 0;
uint32_t BMS_charge_delay_timer = 0;
uint32_t BMS_discharge_delay_timer = 0;

void BMS_RES_sets(BMS_t *chip)
{
	chip->chip.address = LTC_BROADCAST;
	
	chip->Int_Temp = 0;

	chip->Status = RES;
	chip->last_CV = chVTGetSystemTimeX();  // 1s
	chip->last_TEST = chVTGetSystemTimeX();  // 10s
	
	chip->BMS_present = false;
	
	chip->Cell_Test_Passed = false;
	chip->GPIO_Test_Passed = false;
	chip->Status_Test_Passed = false;
	chip->MUX_Test_Passed = false;
	chip->Secondary_Reference_OK = false;
	chip->Int_Temp_OK = false;
	chip->VA_OK = false;
	chip->VD_OK = false;
	chip->Wrong_Cell_Count = true;
	chip->Balance_Permission = false;
	chip->Balance_derating = 0;
	chip->Balance_timer = 0;
	chip->Health = BMS_Health_Error_Non_Operational;
	chip->Aux = BMS_AUX_Current_Temp;
	chip->chip.MD = 3;
	chip->chip.DCP = 0;
	chip->chip.CH = 0;
	chip->chip.PUP = 0;
	chip->chip.ST = 1;
	chip->chip.CHG = 0;
	chip->chip.CHST = 0;
	
	chip->Cell_Min_U = 0;
	chip->Cell_Min_index = 0;
	chip->Cell_Max_U = 0;
	chip->Cell_Max_index = 0;
	chip->Cell_All_U = 0;
	chip->Cell_Avr_U = 0;
	chip->Cell_Count = 0;
	
	for (uint8_t j = 0; j < 12; j++)
	{
		chip->Open_Cell_Connection[j] = 0;
		
		chip->Cell_U[j] = 0;
		
		chip->Cell_OV[j] = false;
		chip->Cell_OV_timer[j] = 0;
		chip->Cell_UV[j] = false;
		chip->Cell_UV_timer[j] = 0;
		
		chip->Cell_Bleed[j] = 0;
		
		chip->Cell_Sink_U[j] = 0;
		chip->Cell_Source_U[j] = 0;
	}
	
	for (uint8_t j = 0; j < 5; j++)
		chip->Temp_sensors[j] = 0;

	for (uint8_t j = 0; j < 6; j++)
	{
		chip->chip.CFGR.B[j] = 0;
		chip->chip.CVAR.B[j] = 0;
		chip->chip.CVBR.B[j] = 0;
		chip->chip.CVCR.B[j] = 0;
		chip->chip.CVDR.B[j] = 0;
		chip->chip.AVAR.B[j] = 0;
		chip->chip.AVBR.B[j] = 0;
		chip->chip.STAR.B[j] = 0;
		chip->chip.STBR.B[j] = 0;
		chip->chip.COMM.B[j] = 0;
	}

	chip->chip.CFGR.B[0] = 0xF8;  // Disable Pulldowns on GPIO
}

void LTC_handler_Init()
{
	LTC_Init();
	
	BMS_Balance_Timer = chVTGetSystemTimeX();

	BMS_RES_sets(&BMS);
	BMS.chip.address = LTC_ADDRESS(0);
}

void BMS_Check_Voltage(BMS_t *chip)
{
	for (uint8_t i = 0; i < BMS_cell_count; i++)
	{
		if (chip->Open_Cell_Connection[i])
			chip->Cell_OV[i] = false;
		else if (chip->Cell_U[i] >= BMS_hard_OV)
			chip->Cell_OV[i] = true;
		else if (chip->Cell_U[i] >= BMS_OV)
		{
			if (!(chip->Cell_OV_timer[i]))
				chip->Cell_OV_timer[i] = chVTGetSystemTimeX();
			else if (chVTTimeElapsedSinceX(chip->Cell_OV_timer[i]) > MS2ST(BMS_OV_delay))
			{
				chip->Cell_OV_timer[i] = 0;
				chip->Cell_OV[i] = true;
			}
		}
		else
		{
			chip->Cell_OV_timer[i] = 0;
			chip->Cell_OV[i] = false;
		}
		
		if (chip->Open_Cell_Connection[i])
			chip->Cell_UV[i] = false;
		else if (chip->Cell_U[i] <= BMS_hard_UV)
			chip->Cell_UV[i] = true;
		else if (chip->Cell_U[i] <= BMS_soft_UV)
		{
			if (!(chip->Cell_UV_timer[i]))
				chip->Cell_UV_timer[i] = chVTGetSystemTimeX();
			else if (chVTTimeElapsedSinceX(chip->Cell_UV_timer[i]) > MS2ST(BMS_UV_Delay))
			{
				chip->Cell_UV_timer[i] = 0;
				chip->Cell_UV[i] = true;
			}
		}
		else
		{
			chip->Cell_UV_timer[i] = 0;
			chip->Cell_UV[i] = false;
		}
	}
}

void BMS_Calc_Voltages(BMS_t *chip)
{
	for (uint8_t i = 0; i < BMS_cell_count; i++)
		chip->Cell_U[i] = LTC_calc_Voltage(LTC_get_Voltage_raw(&(chip->chip), i));
	
	chip->Cell_Max_U = 0;
	chip->Cell_Max_index = 0;
	chip->Cell_Min_U = 6.0;
	chip->Cell_Min_index = 0;
	chip->Cell_Count = 0;
	
	for (uint8_t i = 0; i < BMS_cell_count; i++)
	{
		if (!chip->Open_Cell_Connection[i])
		{
			if (chip->Cell_U[i] > chip->Cell_Max_U)
			{
				chip->Cell_Max_index = i;
				chip->Cell_Max_U = chip->Cell_U[i];
			}
			
			if (chip->Cell_U[i] < chip->Cell_Min_U)
			{
				chip->Cell_Min_index = i;
				chip->Cell_Min_U = chip->Cell_U[i];
			}
			
			chip->Cell_Avr_U += chip->Cell_U[i];
			chip->Cell_Count++;
		}
	}
	chip->Cell_All_U = chip->Cell_Avr_U;
	chip->Cell_Avr_U /= chip->Cell_Count;
	
	
	if (chip->Cell_Count != BMS_cell_count)
		chip->Wrong_Cell_Count = true;
	else
		chip->Wrong_Cell_Count = false;
}

void BMS_Calc_Temp(BMS_t *chip)
{
	chip->Temp_sensors[0] = LTC_TEMP(chip->chip.AVAR.G1V);
	chip->Temp_sensors[1] = LTC_TEMP(chip->chip.AVAR.G2V);
	chip->Temp_sensors[2] = LTC_TEMP(chip->chip.AVAR.G3V);
	chip->Temp_sensors[3] = LTC_TEMP(chip->chip.AVBR.G4V);
	chip->Temp_sensors[4] = LTC_TEMP(chip->chip.AVBR.G5V);
}

void BMS_Selfcheck(BMS_t* chip)
{
	if (!chip->Cell_Test_Passed || !chip->MUX_Test_Passed)
		chip->Health = BMS_Health_Error_Non_Operational;
	else if (!chip->GPIO_Test_Passed || !chip->Status_Test_Passed ||
			 !chip->Secondary_Reference_OK || !chip->Int_Temp_OK ||
			 !chip->VA_OK || !chip->VD_OK ||
			 chip->Wrong_Cell_Count)
		chip->Health = BMS_Health_Error_Operational;
	else
		chip->Health = BMS_Health_OK;
}

void LTC_Balancing_handler(void)
{
	if (chVTTimeElapsedSinceX(BMS_Balance_Timer) < S2ST(1))
		return;
	BMS_Balance_Timer = chVTGetSystemTimeX();
	
	Global_Max_U = BMS.Cell_Max_U;
	Global_Min_U = BMS.Cell_Min_U;

	if (BMS.Health == BMS_Health_OK)
		BMS.Balance_Permission = true;
	else
	{
		BMS.Balance_Permission = false;
		BMS.Balance_derating = 0;
		BMS.Balance_timer = 0;
	}
	
	float delta = abs(Global_Max_U - Global_Min_U);
	if ((Global_Max_U < BMS_Balance_U) || (delta <= 0.01))
		BMS_Balance_Scheduled = false;
	else if (delta > 0.02)
		BMS_Balance_Scheduled = true;
	
	bool Local_Balance_Permission = BMS.Balance_Permission;
	if (BMS_Balance_Scheduled && Local_Balance_Permission)
	{
		float Balance_Threashold = Global_Min_U + 0.01;

		if (BMS.Int_Temp > (BMS_ITMP_LIM + BMS_ITMP_HYST*2))
		{
			BMS.Balance_derating = 0;
			BMS.Balance_timer = 0;
		}
		else
		{
			if (BMS.Balance_timer < 10)
				BMS.Balance_timer++;
			else if ((BMS.Balance_derating < BMS_cell_count) && (BMS.Int_Temp < (BMS_ITMP_LIM - BMS_ITMP_HYST/2)))
			{
				BMS.Balance_derating++;
				BMS.Balance_timer = 0;
			}
			else if ((BMS.Balance_derating > 0)              && (BMS.Int_Temp > (BMS_ITMP_LIM + BMS_ITMP_HYST/2)))
			{
				BMS.Balance_derating--;
				BMS.Balance_timer = 0;
			}
		}

		for (uint8_t j = 0; j < 12; j++)
			BMS.Cell_Bleed[j] = false;

		for (uint8_t j = 0; j < BMS.Balance_derating; j++)
		{
			uint8_t highest = 0;
			for (uint8_t x = 1; x < (BMS_cell_count+1); x++)
				if (!BMS.Cell_Bleed[x])
					highest = x;
			if (highest > BMS_cell_count)
				break;

			for (uint8_t x = 0; x < BMS_cell_count; x++)
				if ((BMS.Cell_U[x] > BMS.Cell_U[highest]) && (!BMS.Cell_Bleed[x]))
					highest = x;

			if (BMS.Cell_U[highest] >= Balance_Threashold)
				BMS.Cell_Bleed[highest] = true;
			else
				break;
		}
	}
	else
		for (uint8_t j = 0; j < 12; j++)
			BMS.Cell_Bleed[j] = false;
	
	BMS.chip.CFGR.DCC1 =  BMS.Cell_Bleed[0];
	BMS.chip.CFGR.DCC2 =  BMS.Cell_Bleed[1];
	BMS.chip.CFGR.DCC3 =  BMS.Cell_Bleed[2];
	BMS.chip.CFGR.DCC4 =  BMS.Cell_Bleed[3];
	BMS.chip.CFGR.DCC5 =  BMS.Cell_Bleed[4];
	BMS.chip.CFGR.DCC6 =  BMS.Cell_Bleed[5];
	BMS.chip.CFGR.DCC7 =  BMS.Cell_Bleed[6];
	BMS.chip.CFGR.DCC8 =  BMS.Cell_Bleed[7];
	BMS.chip.CFGR.DCC9 =  BMS.Cell_Bleed[8];
	BMS.chip.CFGR.DCC10 = BMS.Cell_Bleed[9];
	BMS.chip.CFGR.DCC11 = BMS.Cell_Bleed[10];
	BMS.chip.CFGR.DCC12 = BMS.Cell_Bleed[11];

	LTC_Write_Register(&(BMS.chip), LTC_REGISTER_CFGR);
}

void BMS_IO_handler(void)
{
	bool local_Charge_permitted = true;
	bool local_Discharge_permitted = true;
	bool local_BMS_Health = true;

	if (BMS.Health != BMS_Health_OK)
		local_BMS_Health = false;

	for (uint8_t y = 0; y < BMS_cell_count; y++)
	{
		if (BMS.Cell_OV[y])
			local_Charge_permitted = false;
		if (BMS.Cell_UV[y])
			local_Discharge_permitted = false;
	}

	if (local_Charge_permitted && !BMS_Charge_permitted && local_BMS_Health)
	{
		if (Global_Max_U <= (BMS_OV_recovery))
			BMS_Charge_permitted = true;
	}
	else
		BMS_Charge_permitted = false;

	if (local_Discharge_permitted && !BMS_Discharge_permitted && local_BMS_Health)
	{
		if (Global_Min_U >= BMS_UV_recovery)
			BMS_Discharge_permitted = true;
	}
	else
		BMS_Discharge_permitted = local_Discharge_permitted;


	if (BMS_Charge_permitted && (Battery_I_offset != 0))
	{
		BMS_enable_charge;
		BMS_charge_delay_timer = chVTGetSystemTimeX();
	}
	else if (chVTTimeElapsedSinceX(BMS_charge_delay_timer) > MS2ST(BMS_fault_delay))
		BMS_disable_charge;
}

void LTC_handler()
{
	LTC_Balancing_handler();
	BMS_IO_handler();

	switch (BMS.Status)
	{
		case RES:
			BMS_RES_sets(&BMS);
			BMS.chip.address = LTC_ADDRESS(0);

			if (!(LTC_Write_Register(&(BMS.chip), LTC_REGISTER_CFGR | LTC_REGISTER_COMM)))
			{
				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_CFGR))
				{
					BMS.Status = DISCOVER;
					break;
				}

				BMS.BMS_present = true;
				BMS.Status = TEST_MUX;
			}
			else
				BMS.Status = DISCOVER;
		break;

		case DISCOVER:
			if (chVTTimeElapsedSinceX(BMS.last_CV) > S2ST(1))
				BMS.Status = RES;
		break;

		case TEST_MUX:  // Selftest flow
			LTC_Start(&(BMS.chip), LTC_START_DIAGN);
			BMS.Status = TEST_CV;
		break;

		case TEST_CV:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_STBR))
				{
					BMS.Status = RES;
					break;
				}

				BMS.MUX_Test_Passed = !BMS.chip.STBR.MUXFAIL;

				BMS.chip.MD = 1;
				LTC_Start(&(BMS.chip), LTC_START_CVST);
				BMS.Status = TEST_GPIO;
			}
		break;

		case TEST_GPIO:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				uint8_t j;

				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_CV))
				{
					BMS.Status = RES;
					break;
				}

				for (j = 0; (j < 12) && (LTC_get_Voltage_raw(&(BMS.chip), j) == 0x9565); j++);
				if (j < 12)
					BMS.Cell_Test_Passed = 0;
				else
					BMS.Cell_Test_Passed = 1;

				BMS.chip.MD = 1;
				LTC_Start(&(BMS.chip), LTC_START_AXST);
				BMS.Status = TEST_ST;
			}
		break;

		case TEST_ST:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_AV))
				{
					BMS.Status = RES;
					break;
				}

				uint32_t j;
				for (j = 12; (j < 17) && (LTC_get_Voltage_raw(&(BMS.chip), j) == 0x9565); j++);
				if (j < 17)
					BMS.GPIO_Test_Passed = 0;
				else
					BMS.GPIO_Test_Passed = 1;

				LTC_Start(&(BMS.chip), LTC_START_STATST);
				BMS.Status = OPEN_SOURCE_TEST;
			}
		break;

		case OPEN_SOURCE_TEST:
		if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
		{
			if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_ST))
			{
				BMS.Status = RES;
				break;
			}

			uint32_t j;
			for (j = 17; (j < 21) && (LTC_get_Voltage_raw(&(BMS.chip), j) == 0x9565); j++);
			if (j < 21)
				BMS.Status_Test_Passed = 0;
			else
				BMS.Status_Test_Passed = 1;

			BMS.chip.MD = 3;
			BMS.chip.PUP = 1;
			LTC_Start(&(BMS.chip), LTC_START_ADOW);

			BMS.Status = OPEN_SINK_TEST;
		}
		break;

		case OPEN_SINK_TEST:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_CV))
				{
					BMS.Status = RES;
					break;
				}

				for (uint8_t j = 0; j < BMS_cell_count; j++)
					BMS.Cell_Source_U[j] = LTC_calc_Voltage(LTC_get_Voltage_raw(&(BMS.chip), j));

				BMS.chip.MD = 3;
				BMS.chip.PUP = 0;
				LTC_Start(&(BMS.chip), LTC_START_ADOW);

				BMS.Status = TEST_REF;
			}
		break;

		case TEST_REF:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_CV))
				{
					BMS.Status = RES;
					break;
				}

				for (uint32_t j = 0; j < BMS_cell_count; j++)
				{
					BMS.Cell_Sink_U[j] = LTC_calc_Voltage(LTC_get_Voltage_raw(&(BMS.chip), j));

					if (abs(BMS.Cell_Source_U[j] - BMS.Cell_Sink_U[j]) < 0.4)
						BMS.Open_Cell_Connection[j] = false;
					else
						BMS.Open_Cell_Connection[j] = true;
				}

				BMS.chip.MD = 1;
				BMS.chip.CHG = 6;
				LTC_Start(&(BMS.chip), LTC_START_ADAX);
				BMS.Status = TEST_STATUS;
			}
		break;

		case TEST_STATUS:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_AVBR))
				{
					BMS.Status = RES;
					break;
				}

				BMS.Secondary_Reference_OK = (BMS.chip.AVBR.REF > 29800) & (BMS.chip.AVBR.REF < 30200);

				BMS.chip.MD = 1;
				BMS.chip.CHST = 0;
				LTC_Start(&(BMS.chip), LTC_START_ADSTAT);
				BMS.Status = TEST_END;
			}
		break;

		case TEST_END:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_ST))
				{
					BMS.Status = RES;
					break;
				}

				BMS.Int_Temp = LTC_calc_int_Temp(BMS.chip.STAR.ITMP);
				BMS.Int_Temp_OK = BMS.chip.STAR.ITMP < 27975;

				BMS.VA_OK = (BMS.chip.STAR.VA > 45000) & (BMS.chip.STAR.VA < 55000);

				BMS.VD_OK = (BMS.chip.STBR.VD > 27000) & (BMS.chip.STBR.VD < 36000);

				BMS_Selfcheck(&BMS);

				BMS.Status = SAMPLE_ITMP;
			}
		break;

		case SAMPLE_ITMP:  // Normal flow
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if (BMS.Balance_Permission)
				{
					BMS.chip.MD = 1;
					BMS.chip.CHST = 2;
					LTC_Start(&(BMS.chip), LTC_START_ADSTAT);
					BMS.Status = READ_ITMP;
				}
				else
					BMS.Status = SAMPLE_AUX;
			}
		break;

		case READ_ITMP:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_ST))
				{
					BMS.Status = RES;
					break;
				}

				BMS.Int_Temp = LTC_calc_int_Temp(BMS.chip.STAR.ITMP);
				BMS.Int_Temp_OK = BMS.chip.STAR.ITMP < 27975;

				BMS.Status = SAMPLE_AUX;
			}
		break;

		case SAMPLE_AUX:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if ((BMS.Aux == BMS_AUX_Current_Temp) ||
					(BMS.Aux == BMS_AUX_Temp))
				{
					BMS.chip.MD = 3;
					BMS.chip.CHG = 0;
					LTC_Start(&(BMS.chip), LTC_START_ADAX);
					BMS.Status = READ_AUX;
				}
				else
					BMS.Status = SAMPLE_CV;
			}
		break;

		case READ_AUX:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_AV))
				{
					BMS.Status = RES;
					break;
				}

				BMS_Calc_Temp(&BMS);


#ifdef BMS_I_SENSOR
				if (Battery_I_offset_coutner < 10)
				{
					if ((BMS.chip.AVBR.G5V > 15500) &&
						(BMS.chip.AVBR.G5V < 17500))
					{
						if (chVTTimeElapsedSinceX(Battery_I_offset_timer) > S2ST(1))
						{
							Battery_I_offset_timer = chVTGetSystemTimeX();  // 1s
							Battery_I_offset_coutner++;
							Battery_I_offset += BMS.chip.AVBR.G5V;
						}
					}
				}
				else if (Battery_I_offset < 100)
				{
					Battery_I_offset /= 10;
					if ((Battery_I_offset > 15500) && (Battery_I_offset < 17500))
					{
						Battery_I_offset = 0;
						Battery_I_offset_coutner = 0;
						Battery_I_offset_timer = chVTGetSystemTimeX();  // 1s
					}
				}
				else
					Battery_I = BMS_Calc_Current(BMS, Battery_I_offset);
#endif

				BMS.Status = SAMPLE_CV;
			}
		break;

		case SAMPLE_CV:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				BMS.chip.MD = 3;
				BMS.chip.CH = 0;
				LTC_Start(&(BMS.chip), LTC_START_ADCV);

				BMS.Status = READ_CV;
			}
		break;

		case READ_CV:
			if (LTC_Start(&(BMS.chip), LTC_START_PLADC))
			{
				if (LTC_Read_Register(&(BMS.chip), LTC_REGISTER_CV | LTC_REGISTER_AVAR))
				{
					BMS.Status = RES;
					break;
				}

				BMS_Calc_Voltages(&BMS);
				BMS_Check_Voltage(&BMS);

				BMS.Status = WAIT;
			}
		break;

		case WAIT:
			if (chVTTimeElapsedSinceX(BMS.last_TEST) > S2ST(10))
			{
				BMS.last_TEST = chVTGetSystemTimeX();

				BMS.Status = TEST_MUX;
			}
			else if (chVTTimeElapsedSinceX(BMS.last_CV) > S2ST(1))
			{
				BMS.last_CV = chVTGetSystemTimeX();

				if (BMS_Balance_Scheduled)
					BMS.Status = SAMPLE_ITMP;
				else
					BMS.Status = SAMPLE_AUX;
			}
		break;
	}
}
