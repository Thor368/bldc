/*
 * mar_terminal.h
 *
 *  Created on: 14.10.2020
 *      Author: main
 */

#ifndef APPLICATIONS_MAR_TERMINAL_H_
#define APPLICATIONS_MAR_TERMINAL_H_

void mar_Init(void);
void mar_Deinit(void);

void BMS_charg_en(int argc, const char **argv);
void BMS_cb_status(int argc, const char **argv);
void BMS_config(int argc, const char **argv);
void mar_read_config(void);
void mar_write_conf(void);

#endif /* APPLICATIONS_MAR_TERMINAL_H_ */
