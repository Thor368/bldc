/*
 * temp_HMI.c
 *
 *  Created on: 02.01.2022
 *      Author: main
 */

#include "temp_control.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

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

void decode_CMD(char *cmd, char *val)
{
	if (!strcmp(cmd, "tempTarget"))
	{
		float T_target_tmp = atof(val);

		if (fabs(T_target_tmp - T_target) >= (double) 0.1)
		{
			T_target = T_target_tmp;

			write_conf();
		}
	}
}


void update_HMI_com(void)
{
	static char cmd[30], val[30];
	static uint8_t pp = 0;
	static bool cmdval = false;

	msg_t res = sdGetTimeout(&SD6, TIME_IMMEDIATE);
	while (res != MSG_TIMEOUT)
	{
		if (pp >= 29)
		{
//			commands_printf("Buffer overflow!");
		}
		else if (res == '\n')
		{
			if (cmdval)
				val[pp] = '\0';
			else
			{
				cmd[pp] = '\0';
				val[0] = '\0';
			}

//			commands_printf("%s=%s", cmd, val);
			decode_CMD(cmd, val);

			cmdval = false;
			pp = 0;
			cmd[0] = '\0';
			val[0] = '\0';
		}
		else if (res == '=')
		{
//			commands_printf("Command end, value start");

			cmdval = true;
			cmd[pp] = '\0';
			pp = 0;
		}
		else
		{
//			commands_printf("Appending char %c", res);

			if (cmdval)
				val[pp++] = res;
			else
				cmd[pp++] = res;
		}

		res = sdGetTimeout(&SD6, TIME_IMMEDIATE);
	}
}

void sm_HMI(void)
{
	update_HMI_com();

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
			sprintf(str, "tempTarget=%.1f\n", (double) T_target);
			sdAsynchronousWrite(&SD6, (uint8_t *) str, strlen(str));

			HMI_state = hst_run;
		}

		break;

	case hst_run:
		if (chVTTimeElapsedSinceX(disp_timer) >= MS2ST(250))
		{
			disp_timer = chVTGetSystemTime();

			float SoC = (U_DC-14)/2.8;

			char str[30];

			sprintf(str, "SoC=%.2f\n", (double) SoC);
			sdAsynchronousWrite(&SD6, (uint8_t *) str, strlen(str));

			sprintf(str, "tempTank=%.1f\n", (double) T_tank);
			sdAsynchronousWrite(&SD6, (uint8_t *) str, strlen(str));

			sprintf(str, "tempRet=%.1f\n", (double) T_return);
			sdAsynchronousWrite(&SD6, (uint8_t *) str, strlen(str));

			sprintf(str, "tempCtrlr=%.1f\n", (double) mc_interface_temp_fet_filtered());
			sdAsynchronousWrite(&SD6, (uint8_t *) str, strlen(str));

			sprintf(str, "RPM=%.0f\n", (double) mc_interface_get_rpm()/5);
			sdAsynchronousWrite(&SD6, (uint8_t *) str, strlen(str));

			sprintf(str, "Current=%.1f\n", (double) I_Comp);
			sdAsynchronousWrite(&SD6, (uint8_t *) str, strlen(str));

			sprintf(str, "Ufan=%.1f\n", (double) U_fan);
			sdAsynchronousWrite(&SD6, (uint8_t *) str, strlen(str));
		}
		break;
	}
}
