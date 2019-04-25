/*
 * SCEN2_CAN_IDs.h
 *
 *  Created on: 11.01.2019
 *      Author: alexander.schroeder
 */

#ifndef SCEN2_CAN_IDS_H_
#define SCEN2_CAN_IDS_H_

#define MCL_CAN_ID_base_wr				0x31000
#define MCL_ThrowBattOff_wr				0x31002
#define MCL_UID							0x31004
#define MCL_Version						0x31006
#define MCL_Hash						0x31008
#define MCL_Reset						0x3100A
#define MCL_Jump_Bootloader				0x3100C

#define MCL_Buttons						0x31011
#define MCL_Trigger1					0x31013
#define MCL_Trigger2					0x31015
#define MCL_Voltages					0x31017
#define MCL_Temperatures				0x31019
#define MCL_Temperature_Limits_wr		0x3101A
#define MCL_Temperature_Limits_rd		0x3101B
#define MCL_MotorTemp					0x3101D
#define MCL_MotorTemp_Limit_wr			0x3101E
#define MCL_MotorTemp_Limit_rd			0x3101F
#define MCL_Pressure					0x31027
#define MCL_LeakageSensor				0x31029
#define MCL_LeakageSensorThreshold_wr	0x3102A
#define MCL_LeakageSensorThreshold_rd	0x3102B
#define MCL_DC_Power					0x3102C
#define MCL_MotorSpeed_wr				0x31030
#define MCL_MotorSpeed_rd				0x31031
#define MCL_MotorCurrents				0x31033
#define MCL_MC_PID_wr					0x31034
#define MCL_MC_PID_rd					0x31035
#define MCL_ChargeMode_wr				0x31040
#define MCL_ChargeMode					0x31041
#define MCL_ChargeCurrent				0x31043

#endif /* SCEN2_CAN_IDS_H_ */
