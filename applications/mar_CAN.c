/*
 * CAN.c
 *
 *  Created on: 14.10.2020
 *      Author: main
 */

#include <stdint.h>
#include "hw.h"
#include "comm_can.h"
#include "mc_interface.h"
#include "LTC6804_handler.h"
#include "Battery_config.h"

#include "maraneo_vars.h"
#include "mar_CAN.h"
#include "mar_charge_statemachine.h"

volatile uint32_t CAN_timer;
volatile uint32_t CAN_BATI_timeout;
volatile uint32_t CAN_HBT1_timeout, CAN_HBT2_timeout;
volatile float I_BAT;
volatile float SoC;

void mar_calc_SoC(void)
{
	float corrected_U = Global_Min_U + I_BAT*0.1;
	static float SoC_filt;

	SoC_filt -= SoC_filt/100;

	if (corrected_U > BMS_soft_OV)
		SoC_filt = 100;
	else if (corrected_U < BMS_soft_UV)
		SoC_filt = 0;
	else
		SoC_filt += (corrected_U - BMS_soft_UV)/(BMS_soft_OV - BMS_soft_UV);

	SoC = SoC_filt/100;

}

void CAN_callback(uint32_t id, uint8_t *data, uint8_t len)
{
	(void) len;

	if ((id & 0xFFFF) == 0xFFFF)  // maraneo frames
	{
		switch (id >> 16)
		{
		case 0x07:
			CAN_HBT2_timeout = chVTGetSystemTimeX();
			break;

		case 0x21:
			discharge_SoC = *((float *) data);
			discharge_enable = true;
			break;
		}
	}
	else
	{
		switch (id >> 8)  // VESC frames
		{
		case CAN_PACKET_SET_CURRENT_REL:
			CAN_HBT1_timeout = chVTGetSystemTimeX();
			break;

		case CAN_PACKET_STATUS_4:
			CAN_BATI_timeout = chVTGetSystemTimeX();
			I_BAT = mc_interface_get_tot_current_in_filtered() + ((data[4] << 8) | data[5])/10. + I_CHG;

			mar_calc_SoC();
			break;
		}
	}
}

void CAN_Init(void)
{
	comm_can_set_eid_rx_callback(&CAN_callback);

	CAN_timer = chVTGetSystemTimeX();
	CAN_BATI_timeout = chVTGetSystemTimeX();
	CAN_HBT1_timeout = chVTGetSystemTimeX();
	CAN_HBT2_timeout = chVTGetSystemTimeX();

	I_BAT = 0;
	SoC = 0;
}

void CAN_Status(void)
{
	if (chVTTimeElapsedSinceX(CAN_timer) > MS2ST(100))
	{
		CAN_timer = chVTGetSystemTimeX();

		comm_can_transmit_eid(0x00FFFF, (uint8_t *) &Global_Max_U, sizeof(Global_Max_U));
		comm_can_transmit_eid(0x01FFFF, (uint8_t *) &Global_Min_U, sizeof(Global_Min_U));
		comm_can_transmit_eid(0x02FFFF, (uint8_t *) &U_CHG, sizeof(U_CHG));
		comm_can_transmit_eid(0x03FFFF, (uint8_t *) &I_CHG, sizeof(I_CHG));

		uint8_t tmp = chg_state;
		comm_can_transmit_eid(0x04FFFF, (uint8_t *) &tmp, sizeof(tmp));
		comm_can_transmit_eid(0x05FFFF, (uint8_t *) &BMS_Discharge_Limit, sizeof(BMS_Discharge_Limit));
		comm_can_transmit_eid(0x06FFFF, (uint8_t *) &SoC, sizeof(SoC));
	}

	if (chVTTimeElapsedSinceX(CAN_BATI_timeout) > MS2ST(250))
	{
		CAN_BATI_timeout = chVTGetSystemTimeX();
		I_BAT = 0;
	}
}
