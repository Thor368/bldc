/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>

// Settings
#define CYCLE_RATE		100

// Threads
static THD_FUNCTION(custom_thread, arg);
static THD_WORKING_AREA(custom_thread_wa, 1024);

// Private variables
static volatile app_configuration config;
static volatile bool stop_now = true;
static volatile bool is_running = false;

void app_custom_configure(app_configuration *conf) {
	config = *conf;
}

void app_custom_start(void) {
	stop_now = false;
	chThdCreateStatic(custom_thread_wa, sizeof(custom_thread_wa), NORMALPRIO, custom_thread, NULL);
}

void app_custom_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

static THD_FUNCTION(custom_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_ADC");

	is_running = true;

	for(;;)
	{
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / CYCLE_RATE;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

	}
}
