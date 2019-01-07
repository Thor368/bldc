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

#include "encoder.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "hw.h"
#include "utils.h"

// Defines
#define AS5047P_READ_ANGLECOM		(0x3FFF | 0x4000 | 0x8000) // This is just ones
#define AS5047_SAMPLE_RATE_HZ		20000

// Private types
typedef enum {
	ENCODER_MODE_NONE = 0,
	ENCODER_MODE_ABI,
	ENCODER_MODE_AS5047P_SPI
} encoder_mode;

// Private variables
static bool index_found = false;
static uint32_t enc_counts = 10000;
static encoder_mode mode = ENCODER_MODE_NONE;
static float last_enc_angle = 0.0;

// Private functions

void encoder_deinit(void) {
	nvicDisableVector(HW_ENC_TIM_ISR_CH);

	TIM_DeInit(HW_ENC_TIM);

	index_found = false;
	mode = ENCODER_MODE_NONE;
	last_enc_angle = 0.0;
}

void encoder_init_abi(uint32_t counts) {
	// Initialize variables
	index_found = false;
	enc_counts = counts;

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	TIM_EncoderInterfaceConfig (HW_ENC_TIM, TIM_EncoderMode_TI12,
			TIM_ICPolarity_Rising,
			TIM_ICPolarity_Rising);
	TIM_SetAutoreload(HW_ENC_TIM, enc_counts - 1);

	// Filter
	HW_ENC_TIM->CCMR1 |= 6 << 12 | 6 << 4;
	HW_ENC_TIM->CCMR2 |= 6 << 4;

	TIM_Cmd(HW_ENC_TIM, ENABLE);

	mode = ENCODER_MODE_ABI;
}

void encoder_init_as5047p_spi(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	// Set MOSI to 1
#if AS5047_USE_HW_SPI_PINS
	palSetPadMode(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN);
#endif

	// Enable timer clock
	HW_ENC_TIM_CLK_EN();

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ((168000000 / 2 / AS5047_SAMPLE_RATE_HZ) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(HW_ENC_TIM, &TIM_TimeBaseStructure);

	// Enable overflow interrupt
	TIM_ITConfig(HW_ENC_TIM, TIM_IT_Update, ENABLE);

	// Enable timer
	TIM_Cmd(HW_ENC_TIM, ENABLE);

	nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);

	mode = ENCODER_MODE_AS5047P_SPI;
	index_found = true;
}

bool encoder_is_configured(void) {
	return mode != ENCODER_MODE_NONE;
}

float encoder_read_deg(void) {
	static float angle = 0.0;

	switch (mode) {
	case ENCODER_MODE_ABI:
		angle = ((float)HW_ENC_TIM->CNT * 360.0) / (float)enc_counts;
		break;

	case ENCODER_MODE_AS5047P_SPI:
		angle = last_enc_angle;
		break;

	default:
		break;
	}

	return angle;
}

/**
 * Reset the encoder counter. Should be called from the index interrupt.
 */
void encoder_reset(void) {
	// Only reset if the pin is still high to avoid too short pulses, which
	// most likely are noise.
	__NOP();
	__NOP();
	__NOP();
	__NOP();

	const unsigned int cnt = HW_ENC_TIM->CNT;
	static int bad_pulses = 0;
	const unsigned int lim = enc_counts / 20;

	if (index_found) {
		// Some plausibility filtering.
		if (cnt > (enc_counts - lim) || cnt < lim) {
			HW_ENC_TIM->CNT = 0;
			bad_pulses = 0;
		} else {
			bad_pulses++;

			if (bad_pulses > 5) {
				index_found = 0;
			}
		}
	} else {
		HW_ENC_TIM->CNT = 0;
		index_found = true;
		bad_pulses = 0;
	}
}

/**
 * Timer interrupt
 */
void encoder_tim_isr(void) {
	last_enc_angle = 0;
}

/**
 * Set the number of encoder counts.
 *
 * @param counts
 * The number of encoder counts
 */
void encoder_set_counts(uint32_t counts) {
	if (counts != enc_counts) {
		enc_counts = counts;
		TIM_SetAutoreload(HW_ENC_TIM, enc_counts - 1);
		index_found = false;
	}
}

/**
 * Check if the index pulse is found.
 *
 * @return
 * True if the index is found, false otherwise.
 */
bool encoder_index_found(void) {
	return index_found;
}

// Software SPI
