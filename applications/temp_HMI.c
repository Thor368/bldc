/*
 * temp_HMI.c
 *
 *  Created on: 02.01.2022
 *      Author: main
 */

#include "temp_control.h"

#include <string.h>
#include <stdio.h>

static systime_t disp_timer;

static SerialConfig uart_cfg = {
		9600,
		0,
		USART_CR2_LINEN,
		0
};

enum
{
	hst_init,
	hst_wait,
	hst_run
} HMI_state;


void HMI_init(void)
{
	HMI_state = hst_init;

	palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(GPIO_AF_USART6) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(GPIO_AF_USART6) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	sdStart(&SD6, &uart_cfg);
}

char * finddel(char *buf, char *del, uint8_t len)
{
	uint8_t del_len = strlen(del);
	uint8_t cc = 0;
	for (uint8_t i = 0; i < len; i++)
	{
		if (buf[i] == del[cc])
			cc++;
		else
			cc = 0;

		if (cc >= del_len)
			return &buf[i];
	}

	return NULL;
}

void sm_HMI(void)
{
	static char rec_buf[30];
	static uint8_t rec_p = 0;

	msg_t res = sdGetTimeout(&SD6, TIME_IMMEDIATE);
	while (res != MSG_TIMEOUT)
	{
		rec_buf[rec_p++] = (char) res;
		rec_buf[rec_p] = 0;
		if (rec_p >= 29)
			rec_p = 0;

		if (finddel(rec_buf, "\xFF\xFF\xFF", rec_p))
		{
			if ((rec_p == 8) && (rec_buf[0] == 0x71))
				T_target = ((float) *((uint32_t *) &rec_buf[1]))/10;


			rec_p = 0;
		}

		res = sdGetTimeout(&SD6, TIME_IMMEDIATE);
	}

	switch (HMI_state)
	{
	case hst_init:
		disp_timer = chVTGetSystemTime();

		HMI_state = hst_wait;
		break;

	case hst_wait:
		if (chVTTimeElapsedSinceX(disp_timer) >= S2ST(1))
		{
			disp_timer = chVTGetSystemTime();

			char str[30];
			sprintf(str, "tempSet.val=%d\xFF\xFF\xFF", (int) (T_target*10));
			sdWrite(&SD6, (uint8_t *) str, strlen(str));

			HMI_state = hst_run;
		}

		break;

	case hst_run:
		if (chVTTimeElapsedSinceX(disp_timer) >= MS2ST(250))
		{
			disp_timer = chVTGetSystemTime();

			float SoC = (U_DC-14)/2.8;

			char str[30];

			sprintf(str, "SoC.val=%d\xFF\xFF\xFF", (uint8_t) (SoC*100));
			sdWrite(&SD6, (uint8_t *) str, strlen(str));
			sprintf(str, "SoCp.val=%d\xFF\xFF\xFF", (uint8_t) (SoC*100));
			sdWrite(&SD6, (uint8_t *) str, strlen(str));

			sprintf(str, "running.val=%d\xFF\xFF\xFF", (uint8_t) (mc_interface_get_rpm() > 100));
			sdWrite(&SD6, (uint8_t *) str, strlen(str));

			sprintf(str, "tempActual.val=%d\xFF\xFF\xFF", (uint8_t) (T_tank*10));
			sdWrite(&SD6, (uint8_t *) str, strlen(str));

			strcpy(str, "get tempSet.val\xFF\xFF\xFF");
			sdWrite(&SD6, (uint8_t *) str, strlen(str));
		}
		break;
	}
}
