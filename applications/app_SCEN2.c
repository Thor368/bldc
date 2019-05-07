#include "app.h"

#include "SCEN2_settings.h"
#include "SCEN2_ADC.h"
#include "SCEN2_digital_IO.h"
#include "SCEN2_error.h"
#include "SCEN2_CAN.h"
#include "SCEN2_charge.h"
#include "SCEN2_battery.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "SCEN2_types.h"

// Threads
static THD_FUNCTION(custom_thread, arg);
static THD_WORKING_AREA(custom_thread_wa, 1024);

// Private variables
static volatile app_configuration config;
static volatile bool stop_now = true;
static volatile bool is_running = false;

Governor_state_t governor_state = gv_init;

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
	SCEN2_DIO_init();
	SCEN2_Charge_init();
	SCEN2_Battery_init();

	for(;;)
	{
		if (stop_now)
		{
			is_running = false;
			return;
		}

		SCEN2_CAN_handler();
		SCEN2_ADC_handler();
		SCEN2_Error_handler();
		SCEN2_DIO_handler();
		SCEN2_Charge_handler();
		SCEN2_Battery_handler();

		if (governor_state == gv_run)
			DISP_SPLY_ON();

		chThdSleep(1);
	}
}
