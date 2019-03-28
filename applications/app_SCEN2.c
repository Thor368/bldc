#include "app.h"

#include "SCEN2_settings.h"
#include "SCEN2_ADC.h"
#include "SCEN2_error.h"
#include "SCEN2_CAN.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"

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

	chRegSetThreadName("APP_CUSTOM");

	is_running = true;

	SCEN2_CAN_init();

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

		SCEN2_ADC_handler();
		SCEN2_error_handler();
		SCEN2_CAN_handler();
	}
}
