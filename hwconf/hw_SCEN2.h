/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef HW_SCEN2_H_
#define HW_SCEN2_H_

#define HW_NAME					"SCEN2"

// HW properties
#define HW_HAS_3_SHUNTS

// Macros
#define LED_GREEN_ON()			palSetPad(GPIOE, 2)
#define LED_GREEN_OFF()			palClearPad(GPIOE, 2)
#define LED_RED_ON()			palSetPad(GPIOE, 1)
#define LED_RED_OFF()			palClearPad(GPIOE, 1)

#define CAN_STB_HI()			palSetPad(GPIOE, 0)
#define CAN_STB_LO()			palClearPad(GPIOE, 0)

#define POWER_ON()				palSetPad(GPIOE, 3)
#define POWER_OFF()				palClearPad(GPIOE, 3)

#define BAT_RIGHT_SPLY_ON()		palSetPad(GPIOE, 4)
#define BAT_RIGHT_SPLY_OFF()	palClearPad(GPIOE, 4)

#define BAT_LEFT_SPLY_ON()		palSetPad(GPIOE, 5)
#define BAT_LEFT_SPLY_OFF()		palClearPad(GPIOE, 5)

#define DISP_SPLY_ON()			palSetPad(GPIOD, 11)
#define DISP_SPLY_OFF()			palClearPad(GPIOD, 11)

#define CHG_ENABLE()			palSetPad(GPIOB, 12)
#define CHG_DISABLE()			palClearPad(GPIOB, 12)

#define BUTTON_SILVER()			palReadPad(GPIOD, 3)
#define BUTTON_GREEN()			palReadPad(GPIOD, 2)
#define BUTTON_BLUE()			palReadPad(GPIOA, 0)
#define BUTTON_RED()			palReadPad(GPIOE, 15)

#define TRIGGER_1A()			palReadPad(GPIOC, 10)
#define TRIGGER_1B()			palReadPad(GPIOC, 11)
#define TRIGGER_2A()			palReadPad(GPIOC, 12)
#define TRIGGER_2B()			palReadPad(GPIOD, 0)


/*
 * ADC Vector
 *
 * 0:	IN0		SENS1
 * 1:	IN1		SENS2
 * 2:	IN2		SENS3
 * 3:	IN10	CURR1
 * 4:	IN11	CURR2
 * 5:	IN12	CURR3
 * 6:	IN5		ADC_EXT1
 * 7:	IN6		ADC_EXT2
 * 8:	IN3		TEMP_PCB
 * 9:	IN14	TEMP_MOTOR
 * 10:	IN15	ADC_EXT3
 * 11:	IN13	AN_IN
 * 12:	Vrefint
 * 13:	IN0		SENS1
 * 14:	IN1		SENS2
 */

#define HW_ADC_CHANNELS			15
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5

// ADC Indexes
#define ADC_IND_SENS1			0
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			2

#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5

#define ADC_IND_TEMP_W			6
#define ADC_IND_U_CHG			7
#define ADC_IND_TEMP_MOS		8

#define ADC_IND_PRESSURE		9
#define ADC_IND_VIN_SENS		10
#define ADC_IND_I_CHG			11

#define ADC_IND_TEMP_MOTOR		12
#define ADC_IND_ING				13
#define ADC_IND_U_SENSE2		14

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					39000.0
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.000653
#endif
#ifndef INVERTED_SHUNT_POLARITY
#define INVERTED_SHUNT_POLARITY
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define KTY84_TEMP(val)			(0.1824*val - 190.08)
#define TEMP_MOTOR(adc_val)		KTY84_TEMP(adc_val)
#define TEMP_WATER(adc_val)		KTY84_TEMP(adc_val)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			0
#define READ_HALL2()			0
#define READ_HALL3()			0

// CAN device and port
#define HW_CANH_PORT			GPIOB
#define HW_CANH_PIN_INT			5
#define HW_CANH_PIN_CP			8
#define HW_CANL_PORT			GPIOB
#define HW_CANL_PIN_INT			6
#define HW_CANL_PIN_CP			9
#define HW_CAN_GPIO_AF_CP		GPIO_AF_CAN1
#define HW_CAN_GPIO_AF_INT		GPIO_AF_CAN2
#define HW_CAN_DEV_CP			CAND1
#define HW_CAN_DEV_INT			CAND2

// Default setting overrides
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		80.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif

// Setting limits
#define HW_LIM_CURRENT			-120.0, 120.0
#define HW_LIM_CURRENT_IN		-120.0, 120.0
#define HW_LIM_CURRENT_ABS		0.0, 100.0
#define HW_LIM_VIN				6.0, 75.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0

#endif /* HW_SCEN2_H_ */
