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

#ifndef HW_MARANEO_H_
#define HW_MARANEO_H_

#define HW_NAME						"Maraneo"

#define APP_CUSTOM_TO_USE			"app_maraneo.c"

// BMS settings

#define BMS_cell_count_set		10

#define Sleep_Time_default		10800  // timer in seconds after which MC goes into deep sleep

// HW properties
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS

// Macros
#define LED_GREEN_ON()			palSetPad(GPIOB, 0)
#define LED_GREEN_OFF()			palClearPad(GPIOB, 0)
#define LED_RED_ON()			palSetPad(GPIOB, 1)
#define LED_RED_OFF()			palClearPad(GPIOB, 1)

#define CHRG_ON					palSetPad(GPIOA, 6)
#define CHRG_OFF				palClearPad(GPIOA, 6)

#define BAT_ON					palSetPad(GPIOC, 15)
#define BAT_OFF					palClearPad(GPIOC, 15)

#define SHDN_ON					palSetPad(GPIOC, 5)
#define SHDN_OFF				palClearPad(GPIOC, 5)

#define CAN_ON					palSetPad(GPIOD, 2)
#define CAN_OFF					palClearPad(GPIOD, 2)


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

#define HW_ADC_CHANNELS			12
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			4

// ADC Indexes
#define ADC_IND_SENS1			0
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			2
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_VREFINT			10
#define ADC_IND_VIN_SENS		11
#define ADC_IND_TEMP_MOTOR		10
#define ADC_IND_EXT				10

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
#define CURRENT_SHUNT_RES		0.0005
#endif
#ifndef INVERTED_SHUNT_POLARITY
#define INVERTED_SHUNT_POLARITY
#endif


// Input voltage
#define GET_VOLTAGE_RAW(ch)		((V_REG / 4095.0) * (float)ADC_Value[ch])
#define GET_VOLTAGE(ch)			(GET_VOLTAGE_RAW(ch) * ((VIN_R1 + VIN_R2) / VIN_R2))
#define GET_INPUT_VOLTAGE()		(GET_VOLTAGE_RAW(ADC_IND_VIN_SENS) * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)		(1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val)	0
#define NTC_TEMP_MOTOR(beta)	0

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

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6

// UART Peripheral
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

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

// NRF pins
#define NRF_PORT_CSN			GPIOB
#define NRF_PIN_CSN				12
#define NRF_PORT_SCK			GPIOB
#define NRF_PIN_SCK				4
#define NRF_PORT_MOSI			GPIOB
#define NRF_PIN_MOSI			3
#define NRF_PORT_MISO			GPIOD
#define NRF_PIN_MISO			2

// SPI pins
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// BMS SPI
#define BMS_SPI_DEV				SPID3
#define BMS_SPI_GPIO_AF			GPIO_AF_SPI3
#define BMS_PORT_NSS			GPIOA
#define BMS_PIN_NSS				15
#define BMS_PORT_SCK			GPIOC
#define BMS_PIN_SCK				10
#define BMS_PORT_MOSI			GPIOC
#define BMS_PIN_MOSI			12
#define BMS_PORT_MISO			GPIOC
#define BMS_PIN_MISO			11

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Default setting overrides
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		150.0	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif
#ifndef MCCONF_L_CURRENT_MAX
#define MCCONF_L_CURRENT_MAX			85.0	// Current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN			-0.0	// Current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			60.0	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-0.0	// Input current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_BATTERY_CUT_START
#define MCCONF_L_BATTERY_CUT_START		28.0	// Start limiting the positive current at this voltage
#endif
#ifndef MCCONF_L_BATTERY_CUT_END
#define MCCONF_L_BATTERY_CUT_END		25.0	// Limit the positive current completely at this voltage
#endif
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			20.0	// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			43.0	// Maximum input voltage
#endif
#ifndef MCCONF_FOC_F_SW
#define MCCONF_FOC_F_SW					36000.0
#endif
#ifndef MCCONF_FOC_DT_US
#define MCCONF_FOC_DT_US				0.1 // Microseconds for dead time compensation
#endif
#ifndef MCCONF_L_RPM_MAX
#define MCCONF_L_RPM_MAX				50000.0	// The motor speed limit (Upper)
#endif
#ifndef MCCONF_L_RPM_MIN
#define MCCONF_L_RPM_MIN				-50000.0	// The motor speed limit (Lower)
#endif


// Default app setting override
#ifndef APPCONF_APP_TO_USE
#define APPCONF_APP_TO_USE					APP_NONE
#endif
#ifndef APPCONF_CAN_BAUD_RATE
#define APPCONF_CAN_BAUD_RATE				CAN_BAUD_125K
#endif
#ifndef APPCONF_SEND_CAN_STATUS
#define APPCONF_SEND_CAN_STATUS				CAN_STATUS_1_2_3_4_5
#endif
#ifndef APPCONF_SEND_CAN_STATUS_RATE_HZ
#define APPCONF_SEND_CAN_STATUS_RATE_HZ		10
#endif

// Setting limits
#define HW_LIM_CURRENT			-100.0, 100.0
#define HW_LIM_CURRENT_IN		-100.0, 100.0
#define HW_LIM_CURRENT_ABS		0.0, 160.0
#define HW_LIM_VIN				6.0, 57.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 110.0

#endif /* HW_MARANEO_H_ */
