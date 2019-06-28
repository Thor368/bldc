/*
 * SCEN2_settings.h
 *
 *  Created on: 28.03.2019
 *      Author: alexander.schroeder
 */

#ifndef APPLICATIONS_SCEN2_SETTINGS_H_
#define APPLICATIONS_SCEN2_SETTINGS_H_

#define major_release		0
#define minor_release		11
#define branch				2				// alpha dev branch

#define LEAKAGE_THRESHOLD	0.7				// 0.7V water ingress threshold
#define CHARGE_U_MAX		55				// 55V maximum allowed charger output voltage
#define CHARGE_U_DETECT		48				// 48V charger detect voltage
#define CHARGE_I_MAX		25				// 25A maximum allowed charge current
#define CHARGE_I_MIN		-0.5			// 0.5A discharge over CP trip level

#define CHARGE_I_INVERTED

#define POLE_PAIR_COUNT		5

#define STM32_UID			((uint32_t *) 0x1FFF7A10)

//#define SCEN2_debugging_enable
//#define SCEN2_emulate_CHG

#endif /* APPLICATIONS_SCEN2_SETTINGS_H_ */
