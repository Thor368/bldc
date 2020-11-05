/*
 * mar_terminal.c
 *
 *  Created on: 14.10.2020
 *      Author: main
 */

#include <string.h>
#include <stdio.h>

#include "mc_interface.h"
#include "commands.h"
#include "terminal.h"
#include "LTC6804_handler.h"

#include "maraneo_vars.h"
#include "Battery_config.h"
#include "mar_charge_statemachine.h"

static void BMS_charg_en(int argc, const char **argv);
static void BMS_cb_status(int argc, const char **argv);
static void BMS_config(int argc, const char **argv);

void mar_Init(void)
{
	terminal_register_command_callback(
			"bms_status",
			"Show BMS status",
			"",
			BMS_cb_status);

	terminal_register_command_callback(
			"bms_charge_enable",
			"Enable/disable charging (\"\" = report, 0 = off, 1 = on)",
			"[d]",
			BMS_charg_en);

	terminal_register_command_callback(
			"bms_config",
			"set BMS parameter",
			"[s] [f]",
			BMS_config);
}

void mar_Deinit(void)
{
	terminal_unregister_callback(BMS_cb_status);
	terminal_unregister_callback(BMS_charg_en);
	terminal_unregister_callback(BMS_config);
}

void mar_write_conf(void)
{
	eeprom_var mar_conf;
	int pp = 0;

	mar_conf.as_u32 = MAR_CONF_VERSION;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_u32 = BMS_cell_count;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_u32 = BMS_temp_count;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_soft_OV;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_OV_delay;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_hard_OV;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_OV_recovery;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_Balance_U;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_soft_UV;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_UV_Delay;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_hard_UV;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_UV_recovery;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_soft_COT;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_COT_Delay;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_hard_COT;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_soft_DOT;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_DOT_Delay;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_hard_DOT;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_soft_UT;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_UT_Delay;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = BMS_hard_UT;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_u32 = BMS_Temp_beta;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_u32 = Sleep_Time;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);
}

void mar_read_config(void)
{
	eeprom_var mar_conf;
	int pp = 0;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	if (mar_conf.as_u32 != MAR_CONF_VERSION)
		mar_write_conf();

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_cell_count = mar_conf.as_u32;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_temp_count = mar_conf.as_u32;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_soft_OV = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_OV_delay = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_hard_OV = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_OV_recovery = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_Balance_U = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_soft_UV = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_UV_Delay = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_hard_UV = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_UV_recovery = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_soft_COT = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_COT_Delay = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_hard_COT = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_soft_DOT = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_DOT_Delay = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_hard_DOT = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_soft_UT = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_UT_Delay = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_hard_UT = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	BMS_Temp_beta = mar_conf.as_u32;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	Sleep_Time = mar_conf.as_u32;
}

void BMS_config(int argc, const char **argv)
{
	if (argc == 1)  // print all parameters and their setting
	{
		commands_printf("%-20s: %d", "BMS_cell_count", BMS_cell_count);
		commands_printf("%-20s: %d", "BMS_temp_count", BMS_temp_count);
		commands_printf("%-20s: %.3fV", "BMS_soft_OV", (double) BMS_soft_OV);
		commands_printf("%-20s: %.1fs", "BMS_OV_delay", (double) BMS_OV_delay);
		commands_printf("%-20s: %.3fV", "BMS_hard_OV", (double) BMS_hard_OV);
		commands_printf("%-20s: %.2fV", "BMS_OV_recovery", (double) BMS_OV_recovery);
		commands_printf("%-20s: %.3fV", "BMS_Balance_U", (double) BMS_Balance_U);
		commands_printf("%-20s: %.3fV", "BMS_soft_UV", (double) BMS_soft_UV);
		commands_printf("%-20s: %.1fs", "BMS_UV_Delay", (double) BMS_UV_Delay);
		commands_printf("%-20s: %.3fV", "BMS_hard_UV", (double) BMS_hard_UV);
		commands_printf("%-20s: %.3fV", "BMS_UV_recovery", (double) BMS_UV_recovery);
		commands_printf("%-20s: %.1f�C", "BMS_soft_COT", (double) BMS_soft_COT);
		commands_printf("%-20s: %.1fs", "BMS_COT_Delay", (double) BMS_COT_Delay);
		commands_printf("%-20s: %.1f�C", "BMS_hard_COT", (double) BMS_hard_COT);
		commands_printf("%-20s: %.1f�C", "BMS_soft_DOT", (double) BMS_soft_DOT);
		commands_printf("%-20s: %.1fs", "BMS_DOT_Delay", (double) BMS_DOT_Delay);
		commands_printf("%-20s: %.1f�C", "BMS_soft_UT", (double) BMS_soft_UT);
		commands_printf("%-20s: %.1fs", "BMS_UT_Delay", (double) BMS_UT_Delay);
		commands_printf("%-20s: %.1f�C", "BMS_hard_UT", (double) BMS_hard_UT);
		commands_printf("%-20s: %d", "BMS_Temp_beta", BMS_Temp_beta);
		commands_printf("%-20s: %d\n", "Sleep_Time", Sleep_Time);
	}
	else if (argc == 3)
	{
		int ret = 0;

		if (!strcmp(argv[1], "BMS_cell_count"))
			ret = sscanf(argv[2], "%d", (int *) &BMS_cell_count);
		else if (!strcmp(argv[1], "BMS_temp_count"))
			ret = sscanf(argv[2], "%d", (int *) &BMS_temp_count);
		else if (!strcmp(argv[1], "BMS_OV"))
			ret = sscanf(argv[2], "%f", &BMS_soft_OV);
		else if (!strcmp(argv[1], "BMS_OV_delay"))
			ret = sscanf(argv[2], "%f", &BMS_OV_delay);
		else if (!strcmp(argv[1], "BMS_hard_OV"))
			ret = sscanf(argv[2], "%f", &BMS_hard_OV);
		else if (!strcmp(argv[1], "BMS_OV_recovery"))
			ret = sscanf(argv[2], "%f", &BMS_OV_recovery);
		else if (!strcmp(argv[1], "BMS_Balance_U"))
			ret = sscanf(argv[2], "%f", &BMS_Balance_U);
		else if (!strcmp(argv[1], "BMS_soft_UV"))
			ret = sscanf(argv[2], "%f", &BMS_soft_UV);
		else if (!strcmp(argv[1], "BMS_UV_Delay"))
			ret = sscanf(argv[2], "%f", &BMS_UV_Delay);
		else if (!strcmp(argv[1], "BMS_hard_UV"))
			ret = sscanf(argv[2], "%f", &BMS_hard_UV);
		else if (!strcmp(argv[1], "BMS_UV_recovery"))
			ret = sscanf(argv[2], "%f", &BMS_UV_recovery);
		else if (!strcmp(argv[1], "BMS_soft_COT"))
			ret = sscanf(argv[2], "%f", &BMS_soft_COT);
		else if (!strcmp(argv[1], "BMS_COT_Delay"))
			ret = sscanf(argv[2], "%f", &BMS_COT_Delay);
		else if (!strcmp(argv[1], "BMS_hard_COT"))
			ret = sscanf(argv[2], "%f", &BMS_hard_COT);
		else if (!strcmp(argv[1], "BMS_soft_DOT"))
			ret = sscanf(argv[2], "%f", &BMS_soft_DOT);
		else if (!strcmp(argv[1], "BMS_DOT_Delay"))
			ret = sscanf(argv[2], "%f", &BMS_DOT_Delay);
		else if (!strcmp(argv[1], "BMS_soft_UT"))
			ret = sscanf(argv[2], "%f", &BMS_soft_UT);
		else if (!strcmp(argv[1], "BMS_UT_Delay"))
			ret = sscanf(argv[2], "%f", &BMS_UT_Delay);
		else if (!strcmp(argv[1], "BMS_hard_UT"))
			ret = sscanf(argv[2], "%f", &BMS_hard_UT);
		else if (!strcmp(argv[1], "BMS_Temp_beta"))
			ret = sscanf(argv[2], "%d", (int *) &BMS_Temp_beta);
		else if (!strcmp(argv[1], "Sleep_Time"))
			ret = sscanf(argv[2], "%d", (int *) &Sleep_Time);
		else if (!strcmp(argv[1], "BMS_set_cycles"))
		{
			eeprom_var chg_cy;
			sscanf(argv[2], "%d", (int *) &chg_cy.as_u32);
			conf_general_store_eeprom_var_custom(&chg_cy, 63);

			commands_printf("OK\n");
			return;
		}
		else
			commands_printf("Unrecognized parameter name\n");

		if (ret == 1)
		{
			mar_write_conf();
			LTC_handler_Init();

			commands_printf("OK\n");
		}
		else
			commands_printf("Illegal parameter value\n");
	}
	else
		commands_printf("Wrong parameter count\n");
}


void BMS_cb_status(int argc, const char **argv)
{
	(void) argc;
	(void) argv;

	commands_printf("---CHARGEPORT---");
	commands_printf("Port voltage: %.1fV", (double) U_CHG);
	commands_printf("Port current: %.2fA", (double) I_CHG);
	commands_printf("Port current offset: %.2fA", (double) I_CHG_offset/100);

	if (!charge_en)
		commands_printf("Chargeport: Disabled");
	else if (chg_state == chgst_charging)
		commands_printf("Chargeport: Enabled, Charging");
	else
		commands_printf("Chargeport: Enabled, Idle");
	commands_printf("Chargestate: %d", chg_state);

	commands_printf("\n---BMS---");
	commands_printf("State: %d", BMS.Status);
	commands_printf("Present: %d", BMS.BMS_present);
	commands_printf("Balance permission: %d", BMS.Balance_Permission);
	commands_printf("Balance scheduled: %d", BMS_Balance_Scheduled);
	commands_printf("Balance derating: %d", BMS.Balance_derating);

	eeprom_var chg_cy;
	conf_general_read_eeprom_var_custom(&chg_cy, 63);
	commands_printf("Charge cycles: %d", chg_cy.as_u32);

	commands_printf("\nSELFCHECK");
	commands_printf("Cell test passed: %d", BMS.Cell_Test_Passed);
	commands_printf("GPIO test passed: %d", BMS.GPIO_Test_Passed);
	commands_printf("Status test passed: %d", BMS.Status_Test_Passed);
	commands_printf("MUX test passed: %d", BMS.MUX_Test_Passed);
	commands_printf("Secondary reference OK: %d", BMS.Secondary_Reference_OK);
	commands_printf("Internal temperature OK: %d", BMS.Int_Temp_OK);
	commands_printf("VA OK: %d", BMS.VA_OK);
	commands_printf("VD OK: %d", BMS.VD_OK);
	commands_printf("Wrong cell count: %d", BMS.Wrong_Cell_Count);
	commands_printf("Health: %d", BMS.Health);
	for (uint8_t i = 0; i < 12; i++)
		commands_printf("Open test %2d: %.3fV %.3fV", i+1, (double) BMS.Cell_Source_U[i], (double) BMS.Cell_Sink_U[i]);

	commands_printf("\nCELLS");
	commands_printf("Pack voltage: %.2fV", (double) BMS.Cell_All_U);
	commands_printf("Cell average voltage: %.3fV", (double) BMS.Cell_Avr_U);
	commands_printf("Cell count: %d", BMS.Cell_Count);
	for (uint8_t i = 0; i < 12; i++)
	{
		char flags[23] = "";

		if (i == BMS.Cell_Min_index)
			strcat(flags, "MIN ");

		if (i == BMS.Cell_Max_index)
			strcat(flags, "MAX ");

		if (BMS.Cell_OV[i])
			strcat(flags, "OV ");

		if (BMS.Cell_UV[i])
			strcat(flags, "UV ");

		if (BMS.Cell_Bleed[i])
			strcat(flags, "DIS ");

		if (BMS.Open_Cell_Connection[i])
			strcat(flags, "OPEN");

		commands_printf("Cell %2d: %.3fV %s", i+1, (double) BMS.Cell_U[i], flags);
	}

	commands_printf("\nTEMPERATURES");
	commands_printf("Internal temperature:  %.1f�C", (double) BMS.Int_Temp);
	for (uint8_t i = 0; i < 5; i++)
		commands_printf("External temperature %d: %.1f�C", i+1, (double) BMS.Temp_sensors[i]);
	commands_printf("\n");
}

void BMS_charg_en(int argc, const char **argv)
{
	if (argc == 1)
		commands_printf("charge enable = %d", charge_en);
	else if (argc == 2)
	{
		int ret;
		if (!sscanf(argv[1], "%d", &ret))
			commands_printf("Illegal argument!");
		if (ret)
			charge_en = true;
		else
			charge_en = false;
		commands_printf("charge enable = %d", charge_en);
	}
	else
		commands_printf("Wrong argument count!");
}
