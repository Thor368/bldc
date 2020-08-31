#ifndef LTC6804_types
#define LTC6804_types

#include <stdint.h>
#include <stdbool.h>

// ADC Mode Selection
#define LTC_MD_Fast_14k							0x1
#define LTC_MD_Normal_3k						0x2
#define LTC_MD_Filtered_2k					0x3

// Discharge Permitted
#define LTC_DCP_Permitted						0x1
#define LTC_DCP_Denied							0x0

// Cell Selection for ADC Conversion
#define LTC_CH_ALL									0x0
#define LTC_CH_1_7									0x1
#define LTC_CH_2_8									0x2
#define LTC_CH_3_9									0x3
#define LTC_CH_4_10									0x4
#define LTC_CH_5_11									0x5
#define LTC_CH_6_12									0x6

// Sink/Source Current for open wire detection
#define LTC_PUP_Down								0x0
#define LTC_PUP_Up									0x1

// Self-Test Mode Selection
#define LTC_ST_Test_1								0x1
#define LTC_ST_Test_2								0x2

// GPIO Selection for ADC Conversion
#define LTC_CHG_ALL									0x0
#define LTC_CHG_G1									0x1
#define LTC_CHG_G2									0x2
#define LTC_CHG_G3									0x3
#define LTC_CHG_G4									0x4
#define LTC_CHG_G5									0x5
#define LTC_CHG_2nd_REF							0x6

// Status Group Selection
#define LTC_CHST_ALL								0x0
#define LTC_CHST_SOC								0x1
#define LTC_CHST_ITMP								0x2
#define LTC_CHST_VA									0x3
#define LTC_CHST_VD									0x4

typedef union
{
	uint8_t B[6];
	
	struct  __attribute__((__packed__))
	{
		// CFGR0
		unsigned ADCOPT: 1;
		unsigned SWTRD: 1;
		unsigned REFON: 1;
		unsigned GPIO1: 1;
		unsigned GPIO2: 1;
		unsigned GPIO3: 1;
		unsigned GPIO4: 1;
		unsigned GPIO5: 1;
		
		//CFGR1&2
		unsigned VUV: 12;
		
		//CFGR2&3
		unsigned VOV: 12;
		
		//CFGR4
		unsigned DCC1: 1;
		unsigned DCC2: 1;
		unsigned DCC3: 1;
		unsigned DCC4: 1;
		unsigned DCC5: 1;
		unsigned DCC6: 1;
		unsigned DCC7: 1;
		unsigned DCC8: 1;
		
		//CFGR5
		unsigned DCC9: 1;
		unsigned DCC10: 1;
		unsigned DCC11: 1;
		unsigned DCC12: 1;
		unsigned DCTO: 4;
	};
} LTC_CFGR_t;


typedef union
{
	uint8_t B[6];
	
	struct  __attribute__((__packed__))
	{
		// CVAR0&1
		unsigned C1V: 16;
		
		// CVAR2&3
		unsigned C2V: 16;
		
		// CVAR4&5
		unsigned C3V: 16;
	};
} LTC_CVAR_t;


typedef union
{
	uint8_t B[6];
	
	struct  __attribute__((__packed__))
	{
		// CVBR0&1
		unsigned C4V: 16;
		
		// CVBR2&3
		unsigned C5V: 16;
		
		// CVBR4&5
		unsigned C6V: 16;
	};
} LTC_CVBR_t;


typedef union
{
	uint8_t B[6];
	
	struct  __attribute__((__packed__))
	{
		// CVCR0&1
		unsigned C7V: 16;
		
		// CVCR2&3
		unsigned C8V: 16;
		
		// CVCR4&5
		unsigned C9V: 16;
	};
} LTC_CVCR_t;


typedef union
{
	uint8_t B[6];
	
	struct  __attribute__((__packed__))
	{
		// CVDR0&1
		unsigned C10V: 16;
		
		// CVDR2&3
		unsigned C11V: 16;
		
		// CVDR4&5
		unsigned C12V: 16;
	};
} LTC_CVDR_t;


typedef union
{
	uint8_t B[6];
	
	struct  __attribute__((__packed__))
	{
		// AVAR0&1
		unsigned G1V: 16;
		
		// AVAR2&3
		unsigned G2V: 16;
		
		// AVAR4&5
		unsigned G3V: 16;
	};
} LTC_AVAR_t;


typedef union
{
	uint8_t B[6];
	
	struct  __attribute__((__packed__))
	{
		// AVBR0&1
		unsigned G4V: 16;
		
		// AVBR2&3
		unsigned G5V: 16;
		
		// AVBR4&5
		unsigned REF: 16;
	};
} LTC_AVBR_t;


typedef union
{
	uint8_t B[6];
	
	struct  __attribute__((__packed__))
	{
		// STAR0&1
		unsigned SOC: 16;
		
		// STAR2&3
		unsigned ITMP: 16;
		
		// STAR4&5
		unsigned VA: 16;
	};
} LTC_STAR_t;


typedef union
{
	uint8_t B[6];
	
	struct  __attribute__((__packed__))
	{
		// STBR0&1
		unsigned VD: 16;
		
		// STBR2
		unsigned C1UV: 1;
		unsigned C1OV: 1;
		unsigned C2UV: 1;
		unsigned C2OV: 1;
		unsigned C3UV: 1;
		unsigned C3OV: 1;
		unsigned C4UV: 1;
		unsigned C4OV: 1;

		// STBR3
		unsigned C5UV: 1;
		unsigned C5OV: 1;
		unsigned C6UV: 1;
		unsigned C6OV: 1;
		unsigned C7UV: 1;
		unsigned C7OV: 1;
		unsigned C8UV: 1;
		unsigned C8OV: 1;

		// STBR4
		unsigned C9UV: 1;
		unsigned C9OV: 1;
		unsigned C10UV: 1;
		unsigned C10OV: 1;
		unsigned C11UV: 1;
		unsigned C11OV: 1;
		unsigned C12UV: 1;
		unsigned C12OV: 1;
		
		//STBR5
		unsigned THSD: 1;
		unsigned MUXFAIL: 1;
		unsigned RSVD: 2;
		unsigned REV: 4;
	};
} LTC_STBR_t;

typedef union
{
	uint8_t B[6];
	
	struct  __attribute__((__packed__))
	{
		// COMM0&1
		unsigned ICOM0: 4;
		unsigned D0: 8;
		
		// COMM1
		unsigned FCOM0: 4;
		
		// COMM2&3
		unsigned ICOM1: 4;
		unsigned D1: 8;
		
		// COMM3
		unsigned FCOM1: 4;
		
		// COMM4&5
		unsigned ICOM2: 4;
		unsigned D2: 8;
		
		// COMM5
		unsigned FCOM2: 4;
	};
} LTC_COMM_t;


typedef struct
{
	uint16_t address;
	
	uint8_t	MD;
	bool DCP;
	uint8_t	CH;
	bool PUP;
	uint8_t	ST;
	uint8_t	CHG;
	uint8_t	CHST;
	
	LTC_CFGR_t CFGR;
	LTC_CVAR_t CVAR;
	LTC_CVBR_t CVBR;
	LTC_CVCR_t CVCR;
	LTC_CVDR_t CVDR;
	LTC_AVAR_t AVAR;
	LTC_AVBR_t AVBR;
	LTC_STAR_t STAR;
	LTC_STBR_t STBR;
	LTC_COMM_t COMM;
} LTC_DATASET_t;

#endif
