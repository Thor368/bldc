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

#include "mar_vars.h"
#include "Battery_config.h"
#include "mar_charge_statemachine.h"
#include "mar_safety.h"

static void BMS_charg_en(int argc, const char **argv);
static void BMS_cb_status(int argc, const char **argv);
static void BMS_config(int argc, const char **argv);
static void BMS_cb_discharge(int argc, const char **argv);

void mar_Init(void)
{
	terminal_register_command_callback(
			"mar_status",
			"Show BMS status",
			"",
			BMS_cb_status);

	terminal_register_command_callback(
			"mar_charge_enable",
			"Enable/disable charging (\"\" = report, 0 = off, 1 = on)",
			"[d]",
			BMS_charg_en);

	terminal_register_command_callback(
			"mar_config",
			"set BMS parameter",
			"[s] [f]",
			BMS_config);

	terminal_register_command_callback(
			"mar_discharge",
			"set SoC to which to discharge internal battery",
			"[f]",
			BMS_cb_discharge);
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

	mar_conf.as_u32 = sleep_time;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_u32 = stand_alone;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = I_CHG_max;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_u32 = charge_cycles;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = AUX_temp_cutoff;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = rpm_upper_limit;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = rpm_lower_limit;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = rpm_trip_max;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_float = rpm_min_I;
	conf_general_store_eeprom_var_custom(&mar_conf, pp++);

	mar_conf.as_u32 = rpm_trip_delay;
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
	sleep_time = mar_conf.as_u32;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	stand_alone = mar_conf.as_u32;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	I_CHG_max = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	charge_cycles = mar_conf.as_u32;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	AUX_temp_cutoff = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	rpm_upper_limit = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	rpm_lower_limit = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	rpm_trip_max = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	rpm_min_I = mar_conf.as_float;

	conf_general_read_eeprom_var_custom(&mar_conf, pp++);
	rpm_trip_delay = mar_conf.as_u32;
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
		commands_printf("%-20s: %.1f°C", "BMS_soft_COT", (double) BMS_soft_COT);
		commands_printf("%-20s: %.1fs", "BMS_COT_Delay", (double) BMS_COT_Delay);
		commands_printf("%-20s: %.1f°C", "BMS_hard_COT", (double) BMS_hard_COT);
		commands_printf("%-20s: %.1f°C", "BMS_soft_DOT", (double) BMS_soft_DOT);
		commands_printf("%-20s: %.1fs", "BMS_DOT_Delay", (double) BMS_DOT_Delay);
		commands_printf("%-20s: %.1f°C", "BMS_soft_UT", (double) BMS_soft_UT);
		commands_printf("%-20s: %.1fs", "BMS_UT_Delay", (double) BMS_UT_Delay);
		commands_printf("%-20s: %.1f°C", "BMS_hard_UT", (double) BMS_hard_UT);
		commands_printf("%-20s: %d", "BMS_Temp_beta", BMS_Temp_beta);
		commands_printf("%-20s: %ds", "sleep_time", sleep_time);
		commands_printf("%-20s: %d", "stand_alone", stand_alone);
		commands_printf("%-20s: %.1fA", "I_CHG_max", (double) I_CHG_max);
		commands_printf("%-20s: %.1f°C", "AUX_temp_cutoff", (double) AUX_temp_cutoff);
		commands_printf("%-20s: %.0f%%", "rpm_upper_limit", (double) rpm_upper_limit*100);
		commands_printf("%-20s: %.0f%%", "rpm_lower_limit", (double) rpm_lower_limit*100);
		commands_printf("%-20s: %.0fRPM", "rpm_trip_max", (double) rpm_trip_max);
		commands_printf("%-20s: %.1fA", "rpm_min_I", (double) rpm_min_I);
		commands_printf("%-20s: %ds\n", "rpm_trip_delay", rpm_trip_delay);
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
		else if (!strcmp(argv[1], "sleep_time"))
			ret = sscanf(argv[2], "%d", (int *) &sleep_time);
		else if (!strcmp(argv[1], "stand_alone"))
			ret = sscanf(argv[2], "%d", (int *) &stand_alone);
		else if (!strcmp(argv[1], "I_CHG_max"))
			ret = sscanf(argv[2], "%f", &I_CHG_max);
		else if (!strcmp(argv[1], "AUX_temp_cutoff"))
			ret = sscanf(argv[2], "%f", &AUX_temp_cutoff);
		else if (!strcmp(argv[1], "rpm_upper_limit"))
			ret = sscanf(argv[2], "%f", &rpm_upper_limit);
		else if (!strcmp(argv[1], "rpm_lower_limit"))
			ret = sscanf(argv[2], "%f", &rpm_lower_limit);
		else if (!strcmp(argv[1], "rpm_trip_max"))
			ret = sscanf(argv[2], "%f", &rpm_trip_max);
		else if (!strcmp(argv[1], "rpm_min_I"))
			ret = sscanf(argv[2], "%f", &rpm_min_I);
		else if (!strcmp(argv[1], "rpm_trip_delay"))
			ret = sscanf(argv[2], "%d", (int *) &rpm_trip_delay);
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
	if (argc == 2)
	{
		if (!strcmp(argv[1], "bms"))
		{
			commands_printf("\n---BMS---");
			commands_printf("SoC: %.1f%%", (double) SoC*100);
			commands_printf("State: %d", BMS.Status);
			commands_printf("Present: %d", BMS.BMS_present);
			commands_printf("Balance permission: %d", BMS.Balance_Permission);
			commands_printf("Balance scheduled: %d", BMS_Balance_Scheduled);
			commands_printf("Balance derating: %d", BMS.Balance_derating);
			commands_printf("Discharge permission: %d", BMS_Discharge_permitted);
			commands_printf("Charge permission: %d", BMS_Charge_permitted);
			commands_printf("Charge_Limit:    %3.0f%%", (double) BMS_Charge_Limit*100);
			commands_printf("Discharge_Limit: %3.0f%%", (double) BMS_Discharge_Limit*100);
			commands_printf("Charge cycles: %d", charge_cycles);

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
			commands_printf("\n");
		}
		else if (!strcmp(argv[1], "cp"))
		{
			commands_printf("---CHARGEPORT---");
			commands_printf("Port voltage: %.1fV", (double) U_CHG);
			commands_printf("Port current: %.2fA", (double) I_CHG);
			commands_printf("Port current offset: %.2fA", (double) I_CHG_offset/100);

			if (!charge_enable)
				commands_printf("Chargeport: Disabled");
			else if (chg_state == chgst_charging)
				commands_printf("Chargeport: Enabled, Charging");
			else
				commands_printf("Chargeport: Enabled, Idle");
			commands_printf("Chargestate: %d", chg_state);
			commands_printf("\n");
		}
		else if (!strcmp(argv[1], "cell"))
		{
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
			commands_printf("\n");
		}
		else if (!strcmp(argv[1], "temp"))
		{
			commands_printf("\nTEMPERATURES");
			commands_printf("Internal temperature:         %-3.1f°C", (double) BMS.Int_Temp);
			commands_printf("Battery temperature #1:       %-3.1f°C", (double) BMS.Temp_sensors[0]);
			commands_printf("Battery temperature #2:       %-3.1f°C", (double) BMS.Temp_sensors[1]);
			commands_printf("Motor conector 1 temperature: %-3.1f°C", (double) BMS.Temp_sensors[2]);
			commands_printf("Motor conector 2 temperature: %-3.1f°C", (double) BMS.Temp_sensors[3]);
			commands_printf("Chargeport temperature:       %-3.1f°C", (double) BMS.Temp_sensors[4]);
			commands_printf("\n");
		}
		else
		{
			commands_printf("Unknown argument given!");
			commands_printf("Possible arguments:");
			commands_printf("bms:  BMS info");
			commands_printf("cp:   Charge port info");
			commands_printf("cell: Cell statistics");
			commands_printf("temp: Temperature sensor readout");
		}
	}
	else
	{
		commands_printf("Wrong number of arguments given!");
		commands_printf("Possible arguments:");
		commands_printf("bms:  BMS info");
		commands_printf("cp:   Charge port info");
		commands_printf("cell: Cell statistics");
		commands_printf("temp: Temperature sensor readout");
	}
}

void BMS_charg_en(int argc, const char **argv)
{
	if (argc == 1)
		commands_printf("charge enable = %d", charge_enable);
	else if (argc == 2)
	{
		int ret;
		if (!sscanf(argv[1], "%d", &ret))
			commands_printf("Illegal argument!");
		if (ret)
			charge_enable = true;
		else
			charge_enable = false;
		commands_printf("charge enable = %d", charge_enable);
	}
	else
		commands_printf("Wrong argument count!");
}

void BMS_cb_discharge(int argc, const char **argv)
{
	if (argc == 1)
	{
		if (discharge_enable)
			commands_printf("discharge_SoC = %.0f%%", (double) discharge_SoC*100);
		else
			commands_printf("discharge_enable = false");
	}
	else if (argc == 2)
	{
		if (!sscanf(argv[1], "%f", &discharge_SoC))
			commands_printf("Illegal argument!");
		else
		{
			discharge_enable = true;
			commands_printf("discharge enable = %d", discharge_enable);
		}
	}
	else
		commands_printf("Wrong argument count!");
}
