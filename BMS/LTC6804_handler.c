#include "LTC6804_handler.h"
#include "LTC6804.h"
#include "hw.h"
#include "Battery_config.h"

#include <math.h>
#include <stdlib.h>

BMS_t chips[];

bool BMS_Balance_Scheduled = false;
systime_t BMS_Balance_Timer;

float Global_Max_U = 0;
float Global_Min_U = 0;

#ifdef BMS_I_SENSOR
int32_t Battery_I = 0;
int32_t Battery_I_offset = 0;
int32_t Battery_I_offset_last = 0;
uint32_t Battery_I_offset_timer = 0;
#endif

bool BMS_Charge_permitted = true;
bool BMS_Discharge_permitted = true;
bool BMS_fault_latch = false;
uint32_t BMS_fault_delay_timer = 0;
uint32_t BMS_charge_delay_timer = 0;
uint32_t BMS_discharge_delay_timer = 0;

void BMS_RES_sets(int32_t i)
{
	uint32_t j;
	
	if (i >= 0)
		chips[i].chip.address = LTC_ADDRESS(i);
	else
	{
		chips[0].chip.address = LTC_BROADCAST;
		i = 0;
	}
	
	chips[i].Int_Temp = 0;

	chips[i].Status = RES;
	chips[i].last_CV = chVTGetSystemTimeX();  // 1s
	chips[i].last_TEST = chVTGetSystemTimeX();  // 10s
	
	chips[i].BMS_present = false;
	
	chips[i].Cell_Test_Passed = false;
	chips[i].GPIO_Test_Passed = false;
	chips[i].Status_Test_Passed = false;
	chips[i].MUX_Test_Passed = false;
	chips[i].Secondary_Reference_OK = false;
	chips[i].Int_Temp_OK = false;
	chips[i].VA_OK = false;
	chips[i].VD_OK = false;
	chips[i].Wrong_Cell_Count = true;
	chips[i].Balance_Permission = false;
	chips[i].Balance_derating = 0;
	chips[i].Balance_timer = 0;
	chips[i].Health = BMS_Health_Error_Non_Operational;
	chips[i].Aux = BMS_AUX_Current_Temp;
	chips[i].chip.MD = 3;
	chips[i].chip.DCP = 0;
	chips[i].chip.CH = 0;
	chips[i].chip.PUP = 0;
	chips[i].chip.ST = 1;
	chips[i].chip.CHG = 0;
	chips[i].chip.CHST = 0;
	
	chips[i].Cell_Min_U = 0;
	chips[i].Cell_Min_index = 0;
	chips[i].Cell_Max_U = 0;
	chips[i].Cell_Max_index = 0;
	chips[i].Cell_All_U = 0;
	chips[i].Cell_Avr_U = 0;
	chips[i].Cell_Count = 0;
	
	for (j = 0; j < 12; j++)
	{
		chips[i].Open_Cell_Connection[i] = 0;
		
		chips[i].Cell_U[i] = 0;
		
		chips[i].Cell_OV[j] = false;
		chips[i].Cell_OV_timer[j] = 0;
		chips[i].Cell_UV[j] = false;
		chips[i].Cell_UV_timer[j] = 0;
		
		chips[i].Cell_Bleed[j] = 0;
		
		chips[i].Cell_Sink_U[j] = 0;
		chips[i].Cell_Source_U[j] = 0;
	}
	
	for (j = 0; j < 5; j++)
		chips[i].Temp_sensors[j] = 0;

	for (j = 0; j < 6; j++)
	{
		chips[i].chip.CFGR.B[j] = 0;
		chips[i].chip.CVAR.B[j] = 0;
		chips[i].chip.CVBR.B[j] = 0;
		chips[i].chip.CVCR.B[j] = 0;
		chips[i].chip.CVDR.B[j] = 0;
		chips[i].chip.AVAR.B[j] = 0;
		chips[i].chip.AVBR.B[j] = 0;
		chips[i].chip.STAR.B[j] = 0;
		chips[i].chip.STBR.B[j] = 0;
		chips[i].chip.COMM.B[j] = 0;
	}

	chips[i].chip.CFGR.B[0] = 0xF8;  // Disable Pulldowns on GPIO
}

void LTC_handler_Init()
{
	LTC_Init();
	
	BMS_Balance_Timer = chVTGetSystemTimeX();

	BMS_RES_sets(0);
}

void BMS_Check_Voltage(uint8_t index)
{
	for (uint8_t i = 0; i < BMS_cell_count; i++)
	{
		if (chips[index].Open_Cell_Connection[i])
			chips[index].Cell_OV[i] = false;
		else if (chips[index].Cell_U[i] >= BMS_hard_OV)
			chips[index].Cell_OV[i] = true;
		else if (chips[index].Cell_U[i] >= BMS_OV)
		{
			if (!(chips[index].Cell_OV_timer[i]))
				chips[index].Cell_OV_timer[i] = chVTGetSystemTimeX();
			else if (chVTTimeElapsedSinceX(chips[index].Cell_OV_timer[i]) > MS2ST(BMS_OV_delay))
			{
				chips[index].Cell_OV_timer[i] = 0;
				chips[index].Cell_OV[i] = true;
			}
		}
		else
		{
			chips[index].Cell_OV_timer[i] = 0;
			chips[index].Cell_OV[i] = false;
		}
		
		if (chips[index].Open_Cell_Connection[i])
			chips[index].Cell_UV[i] = false;
		else if (chips[index].Cell_U[i] <= BMS_hard_UV)
			chips[index].Cell_UV[i] = true;
		else if (chips[index].Cell_U[i] <= BMS_soft_UV)
		{
			if (!(chips[index].Cell_UV_timer[i]))
				chips[index].Cell_UV_timer[i] = chVTGetSystemTimeX();
			else if (chVTTimeElapsedSinceX(chips[index].Cell_UV_timer[i]) > MS2ST(BMS_UV_Delay))
			{
				chips[index].Cell_UV_timer[i] = 0;
				chips[index].Cell_UV[i] = true;
			}
		}
		else
		{
			chips[index].Cell_UV_timer[i] = 0;
			chips[index].Cell_UV[i] = false;
		}
	}
}

void BMS_Calc_Voltages(uint8_t index)
{
	for (uint8_t i = 0; i < BMS_cell_count; i++)
		chips[index].Cell_U[i] = LTC_calc_Voltage(LTC_get_Voltage_raw(&(chips[index].chip), i));
	
	chips[index].Cell_Max_U = 0;
	chips[index].Cell_Max_index = 0;
	chips[index].Cell_Min_U = 2147483647;
	chips[index].Cell_Min_index = 0;
	chips[index].Cell_Count = 0;
	
	for (uint8_t i = 0; i < BMS_cell_count; i++)
	{
		if (!chips[index].Open_Cell_Connection[i])
		{
			if (chips[index].Cell_U[i] > chips[index].Cell_Max_U)
			{
				chips[index].Cell_Max_index = i;
				chips[index].Cell_Max_U = chips[index].Cell_U[i];
			}
			
			if (chips[index].Cell_U[i] < chips[index].Cell_Min_U)
			{
				chips[index].Cell_Min_index = i;
				chips[index].Cell_Min_U = chips[index].Cell_U[i];
			}
			
			chips[index].Cell_Avr_U += chips[index].Cell_U[i];
			chips[index].Cell_Count++;
		}
	}
	chips[index].Cell_All_U = chips[index].Cell_Avr_U;
	chips[index].Cell_Avr_U /= chips[index].Cell_Count;
	
	
	if (chips[index].Cell_Count != BMS_cell_count)
		chips[index].Wrong_Cell_Count = true;
	else
		chips[index].Wrong_Cell_Count = false;
}

void BMS_Selfcheck(BMS_t* chip)
{
	if (!chip->Cell_Test_Passed || !chip->MUX_Test_Passed)
		chip->Health = BMS_Health_Error_Non_Operational;
	else if (!chip->GPIO_Test_Passed || !chip->Status_Test_Passed ||
//			 !chip->Secondary_Reference_OK || !chip->Int_Temp_OK ||  chips spinnen
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
	
	Global_Max_U = chips[0].Cell_Max_U;
	Global_Min_U = chips[0].Cell_Min_U;

	for (uint8_t i = 0; i < BMS_chip_count; i++)
	{
		if (chips[i].Cell_Max_U > Global_Max_U)
			Global_Max_U = chips[i].Cell_Max_U;
		
		if (chips[i].Cell_Min_U < Global_Min_U)
			Global_Min_U = chips[i].Cell_Min_U;
		
		if (chips[i].Health == BMS_Health_OK)
			chips[i].Balance_Permission = true;
		else
		{
			chips[i].Balance_Permission = false;
			chips[i].Balance_derating = 0;
			chips[i].Balance_timer = 0;
		}
	}
	
	int32_t delta = Global_Max_U - Global_Min_U;
	if ((Global_Max_U < BMS_Balance_U) || (delta <= 10000))
		BMS_Balance_Scheduled = false;
	else if (delta > 20000)
		BMS_Balance_Scheduled = true;
	
	bool Local_Balance_Permission = true;
	for (uint8_t i = 0; i < BMS_chip_count; i++)
		if (!chips[i].Balance_Permission)
			Local_Balance_Permission = false;

	if (BMS_Balance_Scheduled && Local_Balance_Permission)
	{
		float Balance_Threashold = Global_Min_U + 10000;

		for (uint8_t i = 0; i < BMS_chip_count; i++)
		{
			if (chips[i].Int_Temp > (BMS_ITMP_LIM + BMS_ITMP_HYST*2))
			{
				chips[i].Balance_derating = 0;
				chips[i].Balance_timer = 0;
			}
			else
			{
				if (chips[i].Balance_timer < 10)
					chips[i].Balance_timer++;

				if (chips[i].Int_Temp < (BMS_ITMP_LIM - BMS_ITMP_HYST/2))
				{
					if ((chips[i].Balance_timer >= 10) && (chips[i].Balance_derating < BMS_cell_count))
					{
						chips[i].Balance_derating++;
						chips[i].Balance_timer = 0;
					}
				}
				else if (chips[i].Int_Temp > BMS_ITMP_LIM + BMS_ITMP_HYST/2)
				{
					if (chips[i].Balance_derating < BMS_cell_count)
					{
						if ((chips[i].Balance_timer >= 10) && (chips[i].Balance_derating > 0))
						{
							chips[i].Balance_derating--;
							chips[i].Balance_timer = 0;
						}
					}
				}
			}

			chips[i].Balance_derating = 4;

			for (uint8_t j = 0; j < 12; j++)
				chips[i].Cell_Bleed[j] = false;

			for (uint8_t j = 0; j < chips[i].Balance_derating; j++)
			{
				uint8_t highest = 0;
				for (uint8_t x = 1; x < (BMS_cell_count+1); x++)
					if (!chips[i].Cell_Bleed[x])
						highest = x;
				if (highest > BMS_cell_count)
					break;

				for (uint8_t x = 0; x < BMS_cell_count; x++)
					if ((chips[i].Cell_U[x] > chips[i].Cell_U[highest]) && (!chips[i].Cell_Bleed[x]))
						highest = x;

				if (chips[i].Cell_U[highest] >= Balance_Threashold)
					chips[i].Cell_Bleed[highest] = true;
				else
					break;
			}
		}
	}
	else
	{
		for (uint8_t i = 0; i < BMS_chip_count; i++)
			for (uint8_t j = 0; j < 12; j++)
				chips[i].Cell_Bleed[j] = false;
	}
	
	for (uint8_t i = 0; i < BMS_chip_count; i++)
	{
		chips[i].chip.CFGR.DCC1 =  chips[i].Cell_Bleed[0];
		chips[i].chip.CFGR.DCC2 =  chips[i].Cell_Bleed[1];
		chips[i].chip.CFGR.DCC3 =  chips[i].Cell_Bleed[2];
		chips[i].chip.CFGR.DCC4 =  chips[i].Cell_Bleed[3];
		chips[i].chip.CFGR.DCC5 =  chips[i].Cell_Bleed[4];
		chips[i].chip.CFGR.DCC6 =  chips[i].Cell_Bleed[5];
		chips[i].chip.CFGR.DCC7 =  chips[i].Cell_Bleed[6];
		chips[i].chip.CFGR.DCC8 =  chips[i].Cell_Bleed[7];
		chips[i].chip.CFGR.DCC9 =  chips[i].Cell_Bleed[8];
		chips[i].chip.CFGR.DCC10 = chips[i].Cell_Bleed[9];
		chips[i].chip.CFGR.DCC11 = chips[i].Cell_Bleed[10];
		chips[i].chip.CFGR.DCC12 = chips[i].Cell_Bleed[11];
		
		LTC_Write_Register(&(chips[i].chip), LTC_REGISTER_CFGR);
	}
}

void BMS_IO_handler(void)
{
	bool local_Charge_permitted = true;
	bool local_Discharge_permitted = true;
	bool local_BMS_Health = true;

	for (uint8_t x = 0; x < BMS_chip_count; x++)
	{
		if (chips[x].Health != BMS_Health_OK)
			local_BMS_Health = false;

		for (uint8_t y = 0; y < BMS_cell_count; y++)
		{
			if (chips[x].Cell_OV[y])
				local_Charge_permitted = false;
			if (chips[x].Cell_UV[y])
				local_Discharge_permitted = false;
		}
	}

	if (local_Charge_permitted && !BMS_Charge_permitted)
	{
		if (Global_Max_U <= (BMS_OV_recovery*BMS_OV/100))
			BMS_Charge_permitted = true;
	}
	else
		BMS_Charge_permitted = local_Charge_permitted;

	if (local_Discharge_permitted && !BMS_Discharge_permitted)
	{
		if (Global_Min_U >= BMS_UV_recovery)
			BMS_Discharge_permitted = true;
	}
	else
		BMS_Discharge_permitted = local_Discharge_permitted;


#ifdef BMS_set_fault
	if ((BMS_Charge_permitted && local_BMS_Health) &&
		(BMS_Discharge_permitted && local_BMS_Health))
	{
#ifdef BMS_latch_fault
		if (!BMS_fault_latch)
#endif
			BMS_reset_fault;
		BMS_fault_delay_timer = Tick + BMS_fault_delay;
	}
	else if (BMS_fault_delay_timer < Tick)
	{
		BMS_set_fault;
		BMS_fault_latch = true;
	}
#endif

#ifdef BMS_enable_charge
	if (BMS_Charge_permitted && local_BMS_Health)
	{
		BMS_enable_charge;
		BMS_charge_delay_timer = chVTGetSystemTimeX();
	}
	else if (chVTTimeElapsedSinceX(BMS_charge_delay_timer) > MS2ST(BMS_fault_delay))
		BMS_disable_charge;
#endif

#ifdef BMS_enable_discharge
	if (BMS_Discharge_permitted && local_BMS_Health)
	{
		BMS_enable_discharge;
		BMS_discharge_delay_timer = Tick + BMS_fault_delay;
	}
	else if (BMS_discharge_delay_timer < Tick)
		BMS_disable_discharge;
#endif
}

void LTC_handler()
{
	LTC_Balancing_handler();
	BMS_IO_handler();
	
	for (uint8_t i = 0; i < BMS_chip_count; i++)
	{
		switch (chips[i].Status)
		{
			case RES:
				BMS_RES_sets(i);
				
				if (!(LTC_Write_Register(&(chips[i].chip), LTC_REGISTER_CFGR | LTC_REGISTER_COMM)))
				{
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_CFGR))
					{
						chips[i].Status = DISCOVER;
						break;
					}
					
					chips[i].BMS_present = true;
					chips[i].Status = TEST_MUX;
				}
				else
					chips[i].Status = DISCOVER;
			break;
			
			case DISCOVER:
				if (chVTTimeElapsedSinceX(chips[i].last_CV) > MS2ST(1000))
					chips[i].Status = RES;
			break;
			
			case TEST_MUX:  // Selftest flow
				LTC_Start(&(chips[i].chip), LTC_START_DIAGN);
				chips[i].Status = TEST_CV;
			break;
			
			case TEST_CV:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_STBR))
					{
						chips[i].Status = RES;
						break;
					}
					
					chips[i].MUX_Test_Passed = !chips[i].chip.STBR.MUXFAIL;
						
					chips[i].chip.MD = 1;
					LTC_Start(&(chips[i].chip), LTC_START_CVST);
					chips[i].Status = TEST_GPIO;
				}
			break;
			
			case TEST_GPIO:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					uint8_t j;
					
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_CV))
					{
						chips[i].Status = RES;
						break;
					}
					
					for (j = 0; (j < 12) && (LTC_get_Voltage_raw(&(chips[i].chip), j) == 0x9565); j++);
					if (j < 12)
						chips[i].Cell_Test_Passed = 0;
					else
						chips[i].Cell_Test_Passed = 1;
					
					chips[i].chip.MD = 1;
					LTC_Start(&(chips[i].chip), LTC_START_AXST);
					chips[i].Status = TEST_ST;
				}
			break;
			
			case TEST_ST:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_AV))
					{
						chips[i].Status = RES;
						break;
					}
					
					uint32_t j;
					for (j = 12; (j < 17) && (LTC_get_Voltage_raw(&(chips[i].chip), j) == 0x9565); j++);
					if (j < 17)
						chips[i].GPIO_Test_Passed = 0;
					else
						chips[i].GPIO_Test_Passed = 1;
					
					LTC_Start(&(chips[i].chip), LTC_START_STATST);
					chips[i].Status = OPEN_SOURCE_TEST1;
				}
			break;
			
			case OPEN_SOURCE_TEST1:
			if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
			{
				if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_ST))
				{
					chips[i].Status = RES;
					break;
				}
				
				uint32_t j;
				for (j = 17; (j < 21) && (LTC_get_Voltage_raw(&(chips[i].chip), j) == 0x9565); j++);
				if (j < 21)
					chips[i].Status_Test_Passed = 0;
				else
					chips[i].Status_Test_Passed = 1;

				chips[i].chip.MD = 3;
				chips[i].chip.PUP = 1;
				LTC_Start(&(chips[i].chip), LTC_START_ADOW);
				
				chips[i].Status = OPEN_SOURCE_TEST2;
			}
			break;
			
			case OPEN_SOURCE_TEST2:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					chips[i].chip.MD = 3;
					chips[i].chip.PUP = 1;
					LTC_Start(&(chips[i].chip), LTC_START_ADOW);
				
					chips[i].Status = OPEN_SINK_TEST1;
				}
			break;
			
			case OPEN_SINK_TEST1:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_CV))
					{
						chips[i].Status = RES;
						break;
					}
					
					for (uint8_t j = 0; j < BMS_cell_count; j++)
						chips[i].Cell_Source_U[j] = LTC_calc_Voltage(LTC_get_Voltage_raw(&(chips[i].chip), j));
					
					chips[i].chip.MD = 3;
					chips[i].chip.PUP = 0;
					LTC_Start(&(chips[i].chip), LTC_START_ADOW);
					
					chips[i].Status = OPEN_SINK_TEST2;
				}
			break;
			
			case OPEN_SINK_TEST2:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					chips[i].chip.MD = 3;
					chips[i].chip.PUP = 0;
					LTC_Start(&(chips[i].chip), LTC_START_ADOW);
				
					chips[i].Status = TEST_REF;
				}
			break;
			
			case TEST_REF:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_CV))
					{
						chips[i].Status = RES;
						break;
					}
					
					for (uint32_t j = 0; j < BMS_cell_count; j++)
						chips[i].Cell_Sink_U[j] = LTC_calc_Voltage(LTC_get_Voltage_raw(&(chips[i].chip), j));
					
					int32_t Cell_Delta[BMS_cell_count];
					for (uint8_t j = 1; j < 11; j++)
					{
						Cell_Delta[j] = chips[i].Cell_Source_U[j] - chips[i].Cell_Sink_U[j];
						
						if (Cell_Delta[j] < -400000)
							chips[i].Open_Cell_Connection[j] = true;
						else
							chips[i].Open_Cell_Connection[j] = false;
					}
					
					if (chips[i].Cell_Source_U[0] < 400000)
						chips[i].Open_Cell_Connection[0] = true;
					else
						chips[i].Open_Cell_Connection[0] = false;
					
					if (chips[i].Cell_Sink_U[11] < 400000)
						chips[i].Open_Cell_Connection[11] = true;
					else
						chips[i].Open_Cell_Connection[11] = false;
					
					chips[i].chip.MD = 1;
					chips[i].chip.CHG = 6;
					LTC_Start(&(chips[i].chip), LTC_START_ADAX);
					chips[i].Status = TEST_STATUS;
				}
			break;
			
			case TEST_STATUS:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_AVBR))
					{
						chips[i].Status = RES;
						break;
					}

					chips[i].Secondary_Reference_OK = (chips[i].chip.AVBR.REF > 29800) & (chips[i].chip.AVBR.REF < 30200);

					chips[i].chip.MD = 1;
					chips[i].chip.CHST = 0;
					LTC_Start(&(chips[i].chip), LTC_START_ADSTAT);
					chips[i].Status = TEST_END;
				}
			break;

			case TEST_END:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_ST))
					{
						chips[i].Status = RES;
						break;
					}

					chips[i].Int_Temp = LTC_calc_int_Temp(chips[i].chip.STAR.ITMP);
					chips[i].Int_Temp_OK = chips[i].chip.STAR.ITMP < 27975;

					chips[i].VA_OK = (chips[i].chip.STAR.VA > 45000) & (chips[i].chip.STAR.VA < 55000);

					chips[i].VD_OK = (chips[i].chip.STBR.VD > 27000) & (chips[i].chip.STBR.VD < 36000);

					BMS_Selfcheck(&chips[i]);

					chips[i].Status = SAMPLE_ITMP;
				}
			break;

			case SAMPLE_ITMP:  // Normal flow
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if (chips[i].Balance_Permission)
					{
						chips[i].chip.MD = 1;
						chips[i].chip.CHST = 2;
						LTC_Start(&(chips[i].chip), LTC_START_ADSTAT);
						chips[i].Status = READ_ITMP;
					}
					else
						chips[i].Status = SAMPLE_AUX;
				}
			break;
				
			case READ_ITMP:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_ST))
					{
						chips[i].Status = RES;
						break;
					}

					chips[i].Int_Temp = LTC_calc_int_Temp(chips[i].chip.STAR.ITMP);
					chips[i].Int_Temp_OK = chips[i].chip.STAR.ITMP < 27975;

					chips[i].Status = SAMPLE_AUX;
				}
			break;

			case SAMPLE_AUX:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if ((chips[i].Aux == BMS_AUX_Current_Temp) ||
						(chips[i].Aux == BMS_AUX_Temp))
					{
						chips[i].chip.MD = 3;
						chips[i].chip.CHG = 0;
						LTC_Start(&(chips[i].chip), LTC_START_ADAX);
						chips[i].Status = READ_AUX;
					}
					else
						chips[i].Status = SAMPLE_CV;
				}
			break;

			case READ_AUX:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_AV))
					{
						chips[i].Status = RES;
						break;
					}

#ifdef BMS_Temp_c0
					BMS_Calc_Temp(i);
#endif

					chips[i].Status = SAMPLE_CV;
				}
			break;

			case SAMPLE_CV:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if ((chips[i].Aux == BMS_AUX_Current_Temp) ||
						(chips[i].Aux == BMS_AUX_Current))
					{
						chips[i].chip.MD = 3;
						LTC_Start(&(chips[i].chip), LTC_START_ADCVAX);
					}
					else
					{
						chips[i].chip.MD = 3;
						chips[i].chip.CH = 0;
						LTC_Start(&(chips[i].chip), LTC_START_ADCV);
					}

					chips[i].Status = READ_CV;
				}
			break;

			case READ_CV:
				if (LTC_Start(&(chips[i].chip), LTC_START_PLADC))
				{
					if (LTC_Read_Register(&(chips[i].chip), LTC_REGISTER_CV | LTC_REGISTER_AVAR))
					{
						chips[i].Status = RES;
						break;
					}
					
					BMS_Calc_Voltages(i);
					BMS_Check_Voltage(i);

#ifdef BMS_I_SENSOR
					if ((chips[i].Aux == BMS_AUX_Current_Temp) ||
						(chips[i].Aux == BMS_AUX_Current))
					{
						if (Battery_I_offset == 0)
						{
							if ((chips[i].chip.AVAR.G1V > 23900) &&
								(chips[i].chip.AVAR.G1V < 26100))
							{
								Battery_I_offset_timer = chVTGetSystemTimeX();  // 10s
								Battery_I_offset = (int32_t) chips[i].chip.AVAR.G1V*16;
							}
						}
						else if (chVTTimeElapsedSinceX(Battery_I_offset_timer) > S2ST(10))
						{
							Battery_I_offset_last = Battery_I_offset;
							Battery_I_offset -= Battery_I_offset/16;
							Battery_I_offset += (int32_t) chips[i].chip.AVAR.G1V;

							if (abs(Battery_I_offset - Battery_I_offset_last) < 110)
							{
								Battery_I_offset_timer = chVTGetSystemTimeX();
								Battery_I_offset /= 16;
							}
						}
						else
							Battery_I = BMS_Calc_Current(i, Battery_I_offset);
					}
#endif

					chips[i].Status = WAIT;
				}
			break;
				
			case WAIT:
				if (chVTTimeElapsedSinceX(chips[i].last_TEST) > S2ST(10))
				{
					chips[i].last_TEST = chVTGetSystemTimeX();
					
					chips[i].Status = TEST_MUX;
				}
				else if (chVTTimeElapsedSinceX(chips[i].last_CV) > S2ST(1))
				{
					chips[i].last_CV = chVTGetSystemTimeX();
					
					if (BMS_Balance_Scheduled)
						chips[i].Status = SAMPLE_ITMP;
					else
						chips[i].Status = SAMPLE_AUX;
				}
			break;
		}
	}
}
