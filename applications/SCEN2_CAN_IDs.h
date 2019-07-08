/*
 * SCEN2_CAN_IDs.h
 *
 *  Created on: 11.01.2019
 *      Author: alexander.schroeder
 */

#ifndef SCEN2_CAN_IDS_H_
#define SCEN2_CAN_IDS_H_

#define MCL_CAN_ID_base					0x31000

#define MCL_CAN_ID_base					0x31000
#define MCL_ThrowBattOff				0x31002
#define MCL_UID							0x31004
#define MCL_Version						0x31006
#define MCL_Hash						0x31008
#define MCL_Reset						0x3100A
#define MCL_Jump_Bootloader				0x3100C
#define MCL_Errors						0x3100E


#define MCL_Buttons_right				0x31011
#define MCL_Buttons_left				0x31013
#define MCL_Trigger_right				0x31015
#define MCL_Trigger_left				0x31017
#define MCL_Voltages					0x31021
#define MCL_Temperature_Limits			0x31022
#define MCL_Temperatures				0x31023
#define MCL_MotorTemp_Limit				0x31024
#define MCL_MotorTemp					0x31025
#define MCL_Pressure					0x31027
#define MCL_LeakageSensor				0x31029
#define MCL_LeakageSensorThreshold		0x3102A
#define MCL_DC_Power					0x3102C
#define MCL_MotorSpeed					0x31030
#define MCL_MotorCurrents				0x31033
#define MCL_MC_PID						0x31034
#define MCL_ChargeMode					0x31040
#define MCL_ChargeCurrent				0x31043
#define MCL_12V_control					0x310D0

#endif /* SCEN2_CAN_IDS_H_ */
