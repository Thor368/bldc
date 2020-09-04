#include "LTC6804.h"
#include "PEC.h"
#include "hal.h"

#include <stdint.h>


#define LTC_CMD_WRCFG		0x001				// Write Configuration Register Group
#define LTC_CMD_RDCFG		0x002				// Read Configuration Register Group
#define LTC_CMD_RDCVA		0x004				// Read Cell Voltge Register Group A
#define LTC_CMD_RDCVB		0x006				// Read Cell Voltge Register Group B
#define LTC_CMD_RDCVC		0x008				// Read Cell Voltge Register Group C
#define LTC_CMD_RDCVD		0x00A				// Read Cell Voltge Register Group D
#define LTC_CMD_RDAUXA		0x00C				// Read Auxiliary Register Group A
#define LTC_CMD_RDAUXB		0x00E				// Read Auxiliary Register Group B
#define LTC_CMD_RDSTATA		0x010				// Read Status Register Group A
#define LTC_CMD_RDSTATB		0x012				// Read Status Register Group B
#define LTC_CMD_ADCV		0x260				// Start Cell Voltage ADC Conversion and Poll Status
#define LTC_CMD_ADOW		0x228				// Start Open Wire ADC Conversion and Poll Status
#define LTC_CMD_CVST		0x207				// Start Self-Test Cell Voltage Conversion and Poll Status
#define LTC_CMD_ADAX		0x460				// Start GPIOs ADC Conversion and Poll Status
#define LTC_CMD_AXST		0x407				// Start Self-Test GPIOs Conversion and Poll Status
#define LTC_CMD_ADSTAT		0x468				// Start Status group ADC Conversion and Poll Status
#define LTC_CMD_STATST		0x40F				// Start Self-Test Status group Conversion and Poll Status
#define LTC_CMD_ADCVAX		0x46F				// Start Combined Cell Voltage and GPIO1, GPIO2 Conversion and Poll Status
#define LTC_CMD_CLRCELL		0x711				// Clear Cell Voltage Register Group
#define LTC_CMD_CLRAUX		0x712				// Clear Auxiliary Register Group
#define LTC_CMD_CLRSTAT		0x713				// Clear Status Register Group
#define LTC_CMD_PLADC		0x714				// Poll ADC Conversion Status
#define LTC_CMD_DIAGN		0x715				// Diagnose MUX and Poll Status
#define LTC_CMD_WRCOMM		0x721				// Write COMM Register Group
#define LTC_CMD_RDCOMM		0x722				// Read COMM Register Group
#define LTC_CMD_STCOMM		0x723				// Start I2C/SPI Comminuaction

#define LTC_CS_LO			palClearPad(BMS_PORT_NSS, BMS_PIN_NSS)
#define LTC_CS_HI			palSetPad(BMS_PORT_NSS, BMS_PIN_NSS)
#define LTC_SCK_LO			palClearPad(BMS_PORT_SCK, BMS_PIN_SCK)
#define LTC_SCK_HI			palSetPad(BMS_PORT_SCK, BMS_PIN_SCK)
#define LTC_MOSI_LO			palClearPad(BMS_PORT_MOSI, BMS_PIN_MOSI)
#define LTC_MOSI_HI			palSetPad(BMS_PORT_MOSI, BMS_PIN_MOSI)
#define LTC_MOSI_WR(x)		palWritePad(BMS_PORT_MOSI, BMS_PIN_MOSI, x)
#define LTC_MISO_IN			palReadPad(BMS_PORT_MISO, BMS_PIN_MISO)

#define LTC_SLEEP			for (uint32_t cc = 0; cc < 16; cc++) __ASM volatile ("nop")


void LTC_Init(void)
{
	palSetPadMode(BMS_PORT_MISO, BMS_PIN_MISO,
				PAL_MODE_INPUT |
				PAL_STM32_OSPEED_HIGHEST |
				PAL_STM32_PUDR_FLOATING);

	palSetPadMode(BMS_PORT_SCK, BMS_PIN_SCK,
				PAL_MODE_OUTPUT_PUSHPULL |
				PAL_STM32_OSPEED_HIGHEST |
				PAL_STM32_PUDR_FLOATING);

	palSetPadMode(BMS_PORT_MOSI, BMS_PIN_MOSI,
				PAL_MODE_OUTPUT_PUSHPULL |
				PAL_STM32_OSPEED_HIGHEST |
				PAL_STM32_PUDR_FLOATING);

	palSetPadMode(BMS_PORT_NSS, BMS_PIN_NSS,
				PAL_MODE_OUTPUT_PUSHPULL |
				PAL_STM32_OSPEED_HIGHEST);

	LTC_CS_HI;
	LTC_MOSI_LO;
	LTC_SCK_HI;

//	SPIConfig spiconf;
//	spiconf.end_cb = NULL;
//	spiconf.ssport = BMS_PORT_NSS;
//	spiconf.sspad = BMS_PIN_NSS;
//	spiconf.cr1 = SPI_CR1_BR_0 | SPI_CR1_BR_2;  // 656kHz
//
//	spiStop(&SPID3);
//	spiStart(&SPID3, &spiconf);
}

uint8_t SPI_transceive_byte(uint8_t data)
{
	uint8_t ret = 0;

	for (uint8_t i = 0; i < 8; i++)
	{
		LTC_MOSI_WR(data & (1 << i));
		LTC_SCK_LO;
		LTC_SLEEP;
		LTC_SCK_HI;
		ret |= (LTC_MISO_IN & 1) << i;
		LTC_SLEEP;
	}

	return ret;
}

void SPI_transceive(uint8_t data[], uint8_t length)
{
	for (uint8_t i = 0; i < length; i++)
		data[i] = SPI_transceive_byte(data[i]);
}

void LTC_wake_device(void)
{
	LTC_CS_LO;
	chThdSleep(US2ST(1));
	LTC_CS_HI;
}

bool LTC_read_CMD(uint16_t cmd, uint8_t ret[], uint16_t address)
{
	uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	uint16_t addcmd = cmd | address;  // Add Address to Command
	PEC_Reset();
	uint16_t addcmdpec = __builtin_bswap16(PEC_Compute16b(addcmd));
	addcmd = __builtin_bswap16(addcmd);
	
	LTC_CS_LO;

	SPI_transceive((uint8_t *) &addcmd, 2);			// Send CMD
	SPI_transceive((uint8_t *) &addcmdpec, 2);		// Send CMDs PEC
	SPI_transceive(buf, 8);				// Read register data

	LTC_CS_HI;

	PEC_Reset();  // Calculate Registers PEC
	for (uint8_t i = 0; i < 6; i++)
	{
		PEC_Compute8b(buf[i]);
		ret[i] = buf[i];
	}

	if (PEC_Get() == (buf[7] | (buf[6] << 8)))  // Compare PECs
		return 0;
	else
		return 1;
}

void LTC_write_CMD(uint16_t cmd, uint8_t data[], uint16_t address)
{
	uint16_t addcmd = address | cmd;  // Add Address to Command;
	
	PEC_Reset();
	uint16_t addcmdpec = __builtin_bswap16(PEC_Compute16b(addcmd));
	addcmd = __builtin_bswap16(addcmd);
	
	uint8_t buf[6];
	*((uint32_t *) &buf[0]) = *((uint32_t *) &data[0]);
	*((uint16_t *) &buf[4]) = *((uint16_t *) &data[4]);

 	PEC_Reset();
 	PEC_Compute8b(data[0]);
 	PEC_Compute8b(data[1]);
 	PEC_Compute8b(data[2]);
 	PEC_Compute8b(data[3]);
 	PEC_Compute8b(data[4]);
 	uint16_t datapec = __builtin_bswap16(PEC_Compute8b(data[5]));
 	
	LTC_CS_LO;

	SPI_transceive((uint8_t *) &addcmd, 2);			// Send CMD
	SPI_transceive((uint8_t *) &addcmdpec, 2);		// Send CMDs PEC
	SPI_transceive(buf, 6);							// Send Registers
	SPI_transceive((uint8_t *) &datapec, 2);		// Send Registers PEC

	LTC_CS_HI;
}

uint8_t LTC_start_CMD(uint16_t cmd, uint16_t address)
{
	uint16_t addcmd = address | cmd;	// Add Address to Command;
	
	PEC_Reset();
	uint16_t addcmdpec = __builtin_bswap16(PEC_Compute16b(addcmd));
	addcmd = __builtin_bswap16(addcmd);
	
	LTC_CS_LO;

	SPI_transceive((uint8_t *) &addcmd, 2);			// Send CMD
	SPI_transceive((uint8_t *) &addcmdpec, 2);		// Send CMDs PEC
	
	addcmd = 0;
	SPI_transceive((uint8_t *) &addcmd, 1);	// Read status of device
	
	LTC_CS_HI;
	
	if (addcmd)
		return 1;  // Device busy
	else
		return 0;  // Device ready
}

uint8_t LTC_Read_Register(LTC_DATASET_t* set, uint16_t reg)
{
	LTC_wake_device();

	while (reg)
	{
		uint8_t ret[8];
		
		if (reg & LTC_REGISTER_CFGR)
		{
			reg &= ~LTC_REGISTER_CFGR;
			
			if (LTC_read_CMD(LTC_CMD_RDCFG, ret, set->address))
				return 1;  // PEC code returned didn't match
		
			*((uint32_t *) &(set->CFGR.B[0])) = *((uint32_t *) &ret[0]);
			*((uint16_t *) &(set->CFGR.B[4])) = *((uint16_t *) &ret[4]);
		}
		else if (reg & LTC_REGISTER_CVAR)
		{
			reg &= ~LTC_REGISTER_CVAR;
			
			if (LTC_read_CMD(LTC_CMD_RDCVA, ret, set->address))
				return 1;  // PEC code returned didn't match
		
			*((uint32_t *) &(set->CVAR.B[0])) = *((uint32_t *) &ret[0]);
			*((uint16_t *) &(set->CVAR.B[4])) = *((uint16_t *) &ret[4]);
		}
		else if (reg & LTC_REGISTER_CVBR)
		{
			reg &= ~LTC_REGISTER_CVBR;
			
			if (LTC_read_CMD(LTC_CMD_RDCVB, ret, set->address))
				return 1;  // PEC code returned didn't match
		
			*((uint32_t *) &(set->CVBR.B[0])) = *((uint32_t *) &ret[0]);
			*((uint16_t *) &(set->CVBR.B[4])) = *((uint16_t *) &ret[4]);
		}
		else if (reg & LTC_REGISTER_CVCR)
		{
			reg &= ~LTC_REGISTER_CVCR;
			
			if (LTC_read_CMD(LTC_CMD_RDCVC, ret, set->address))
				return 1;  // PEC code returned didn't match
		
			*((uint32_t *) &(set->CVCR.B[0])) = *((uint32_t *) &ret[0]);
			*((uint16_t *) &(set->CVCR.B[4])) = *((uint16_t *) &ret[4]);
		}
		else if (reg & LTC_REGISTER_CVDR)
		{
			reg &= ~LTC_REGISTER_CVDR;
			
			if (LTC_read_CMD(LTC_CMD_RDCVD, ret, set->address))
				return 1;  // PEC code returned didn't match
		
			*((uint32_t *) &(set->CVDR.B[0])) = *((uint32_t *) &ret[0]);
			*((uint16_t *) &(set->CVDR.B[4])) = *((uint16_t *) &ret[4]);
		}
		else if (reg & LTC_REGISTER_AVAR)
		{
			reg &= ~LTC_REGISTER_AVAR;
			
			if (LTC_read_CMD(LTC_CMD_RDAUXA, ret, set->address))
				return 1;  // PEC code returned didn't match
		
			*((uint32_t *) &(set->AVAR.B[0])) = *((uint32_t *) &ret[0]);
			*((uint16_t *) &(set->AVAR.B[4])) = *((uint16_t *) &ret[4]);
		}
		else if (reg & LTC_REGISTER_AVBR)
		{
			reg &= ~LTC_REGISTER_AVBR;
			
			if (LTC_read_CMD(LTC_CMD_RDAUXB, ret, set->address))
				return 1;  // PEC code returned didn't match
		
			*((uint32_t *) &(set->AVBR.B[0])) = *((uint32_t *) &ret[0]);
			*((uint16_t *) &(set->AVBR.B[4])) = *((uint16_t *) &ret[4]);
		}
		else if (reg & LTC_REGISTER_STAR)
		{
			reg &= ~LTC_REGISTER_STAR;
			
			if (LTC_read_CMD(LTC_CMD_RDSTATA, ret, set->address))
				return 1;  // PEC code returned didn't match
		
			*((uint32_t *) &(set->STAR.B[0])) = *((uint32_t *) &ret[0]);
			*((uint16_t *) &(set->STAR.B[4])) = *((uint16_t *) &ret[4]);
		}
		else if (reg & LTC_REGISTER_STBR)
		{
			reg &= ~LTC_REGISTER_STBR;
			
			if (LTC_read_CMD(LTC_CMD_RDSTATB, ret, set->address))
				return 1;  // PEC code returned didn't match
		
			*((uint32_t *) &(set->STBR.B[0])) = *((uint32_t *) &ret[0]);
			*((uint16_t *) &(set->STBR.B[4])) = *((uint16_t *) &ret[4]);
		}
		else if (reg & LTC_REGISTER_COMM)
		{
			reg &= ~LTC_REGISTER_COMM;
			
			if (LTC_read_CMD(LTC_CMD_RDCOMM, ret, set->address))
				return 1;  // PEC code returned didn't match
		
			*((uint32_t *) &(set->COMM.B[0])) = *((uint32_t *) &ret[0]);
			*((uint16_t *) &(set->COMM.B[4])) = *((uint16_t *) &ret[4]);
		}
		else
			return 2;  // Unkown Register
	}
	
	return 0;
}

uint8_t LTC_Write_Register(LTC_DATASET_t* set, uint16_t reg)
{
	LTC_wake_device();

	while (reg)
	{
		if (reg & LTC_REGISTER_CFGR)
		{
			reg &= ~LTC_REGISTER_CFGR;
			
			LTC_write_CMD(LTC_CMD_WRCFG, set->CFGR.B, set->address);
		}
		else if (reg & LTC_REGISTER_COMM)
		{
			reg &= ~LTC_REGISTER_COMM;
			
			LTC_write_CMD(LTC_CMD_WRCOMM, set->COMM.B, set->address);
		}
		else
			return 3;  // Unkown Register
	}
	
	return 0;
}

uint8_t LTC_Start(LTC_DATASET_t* set, uint8_t action)
{
	uint16_t cmd;
	LTC_wake_device();
	
	switch (action)
	{
		case LTC_START_ADCV:
			cmd = LTC_CMD_ADCV | set->CH | (set->DCP << 4) | (set->MD << 7);
			return LTC_start_CMD(cmd, set->address);
		
		case LTC_START_ADOW:
			cmd = LTC_CMD_ADOW | set->CH | (set->DCP << 4) | (set->PUP << 6) | (set->MD << 7);
			return LTC_start_CMD(cmd, set->address);
		
		case LTC_START_CVST:
			cmd = LTC_CMD_CVST | (set->ST << 5) | (set->MD << 7);
			return LTC_start_CMD(cmd, set->address);
		
		case LTC_START_ADAX:
			cmd = LTC_CMD_ADAX | set->CHG | (set->MD << 7);
			return LTC_start_CMD(cmd, set->address);
		
		case LTC_START_AXST:
			cmd = LTC_CMD_AXST | (set->ST << 5) | (set->MD << 7);
			return LTC_start_CMD(cmd, set->address);
		
		case LTC_START_ADSTAT:
			cmd = LTC_CMD_ADSTAT | set->CHST | (set->MD << 7);
			return LTC_start_CMD(cmd, set->address);
		
		case LTC_START_STATST:
			cmd = LTC_CMD_STATST | (set->ST << 5) | (set->MD << 7);
			return LTC_start_CMD(cmd, set->address);
		
		case LTC_START_ADCVAX:
			cmd = LTC_CMD_ADCVAX | (set->DCP << 4) | (set->MD << 7);
			return LTC_start_CMD(cmd, set->address);
		
		case LTC_START_PLADC:
			return LTC_start_CMD(LTC_CMD_PLADC, set->address);
		
		case LTC_START_DIAGN:
			return LTC_start_CMD(LTC_CMD_DIAGN, set->address);
		
		case LTC_START_STCOMM:
			return LTC_start_CMD(LTC_CMD_STCOMM, set->address);
		
		default:
			return 2;  // Unkown Register
	}
}

uint16_t LTC_get_Voltage_raw(LTC_DATASET_t* chip, uint8_t channel)
{
	switch (channel)
	{
		case 0:
			return (chip->CVAR.C1V);
		
		case 1:
			return (chip->CVAR.C2V);
		
		case 2:
			return (chip->CVAR.C3V);
		
		case 3:
			return (chip->CVBR.C4V);
		
		case 4:
			return (chip->CVBR.C5V);
		
		case 5:
			return (chip->CVBR.C6V);
		
		case 6:
			return (chip->CVCR.C7V);
		
		case 7:
			return (chip->CVCR.C8V);
		
		case 8:
			return (chip->CVCR.C9V);
		
		case 9:
			return (chip->CVDR.C10V);
		
		case 10:
			return (chip->CVDR.C11V);
		
		case 11:
			return (chip->CVDR.C12V);
		
		case 12:
			return (chip->AVAR.G1V);
		
		case 13:
			return (chip->AVAR.G2V);
		
		case 14:
			return (chip->AVAR.G3V);
		
		case 15:
			return (chip->AVBR.G4V);
		
		case 16:
			return (chip->AVBR.G5V);
		
		case 17:
			return (chip->AVBR.REF);
		
		case 18:
			return (chip->STAR.SOC);
		
		case 19:
			return (chip->STAR.ITMP);
		
		case 20:
			return (chip->STAR.VA);
		
		case 21:
			return (chip->STBR.VD);
	}
	
	return 0;
}

uint16_t LTC_get_AUX_raw(LTC_DATASET_t* chip, uint8_t channel)
{
	switch (channel)
	{
		case 0:
			return (chip->AVAR.G1V);

		case 1:
			return (chip->AVAR.G2V);

		case 2:
			return (chip->AVAR.G3V);

		case 3:
			return (chip->AVBR.G4V);

		case 4:
			return (chip->AVBR.G5V);
	}

	return 0;
}
