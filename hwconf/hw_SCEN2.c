/*
	Copyright 2012-2016 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "hw.h"
#ifdef HW_VERSION_SCEN2

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils.h"

// Variables
static volatile bool i2c_running = false;

void hw_init_gpio(void) {
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	// CAN_STB
	CAN_STB_LO();
	palSetPadMode(GPIOE, 0,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// CAN_CP_ON
	CAN_CP_OFF();
	palSetPadMode(GPIOB, 11,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// LEDs
	LED_GREEN_OFF();
	LED_RED_OFF();
	palSetPadMode(GPIOE, 2,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOE, 1,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// Trigger power
	TRIG_SPLY_OFF();
	palSetPadMode(GPIOE, 3,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// Battery right power
	BAT_RIGHT_SPLY_OFF();
	palSetPadMode(GPIOE, 4,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// Battery left power
	BAT_LEFT_SPLY_OFF();
	palSetPadMode(GPIOE, 5,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// Display power
	DISP_SPLY_OFF();
	palSetPadMode(GPIOD, 11,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// Charge Enable/Disable
	CHG_DISABLE();
	palSetPadMode(GPIOB, 10,
			PAL_MODE_OUTPUT_PUSHPULL |
			PAL_STM32_OSPEED_HIGHEST);

	// Button inputs
	palSetPadMode(GPIOD, 3, PAL_MODE_INPUT_PULLUP);  // silver
	palSetPadMode(GPIOD, 2, PAL_MODE_INPUT_PULLUP);  // green
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_PULLUP);  // blue
	palSetPadMode(GPIOE, 15, PAL_MODE_INPUT_PULLUP);  // red

	// Trigger inputs
	palSetPadMode(GPIOC, 10, PAL_MODE_INPUT_PULLUP);  // T1A
	palSetPadMode(GPIOC, 11, PAL_MODE_INPUT_PULLUP);  // T1B
	palSetPadMode(GPIOC, 12, PAL_MODE_INPUT_PULLUP);  // T2A
	palSetPadMode(GPIOD, 0, PAL_MODE_INPUT_PULLUP);  // T2B

	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_FLOATING);

	// ADC Pins
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 7, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOB, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);
}

void hw_setup_adc_channels(void) {
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_15Cycles);			// U_U ADC12
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);			// I_W ADC123
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 3, ADC_SampleTime_15Cycles);			// TEMP_WATER ADC12
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 4, ADC_SampleTime_15Cycles);			// PREASSURE ADC12
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_15Cycles);			// TEMP_MOTOR ADC12

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);			// U_V ADC123
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);			// I_V ADC123
	ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 3, ADC_SampleTime_15Cycles);			// U_CHARGE ADC12
	ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);			// U_ZK ADC12
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 5, ADC_SampleTime_15Cycles);			// WATER_INGRESS ADC12

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);			// U_W ADC123
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);			// I_U ADC123
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 3, ADC_SampleTime_15Cycles);			// TEMP_MOSFET ADC123
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);			// I_CHARGE ADC123
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 5, ADC_SampleTime_15Cycles);			// U_V ADC123

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);		// I_W
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);		// I_V
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);		// I_U
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);		// I_W
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);		// I_V
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);		// I_U
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);		// I_W
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);		// I_V
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles); 		// I_U
}

#endif
