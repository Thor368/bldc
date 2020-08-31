#ifndef PEC
#define PEC

#include <stdint.h>

#define PEC_polynom		0x4599
#define PEC_length				16

void PEC_Reset(void);
uint16_t PEC_Get(void);

uint16_t PEC_Compute16b(uint16_t input);
uint16_t PEC_Compute8b(uint8_t input);

#endif
