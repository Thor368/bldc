#include "PEC.h"

#include <stdint.h>
#include <stdbool.h>

uint16_t PEC_code;

uint16_t PEC_Compute1b(uint8_t DIN)
{
  uint16_t inp;
  PEC_code <<= 1;

  if (DIN ^ ((PEC_code >> (PEC_length-1)) > 0))
    inp = PEC_polynom ^ PEC_code;
  else
    inp = PEC_code;

 PEC_code = (PEC_code & !PEC_polynom) | inp;

  return PEC_code;
}

uint16_t PEC_Compute8b(uint8_t input)
{
 	for (uint8_t i = 7; i > 0; i--)
 		PEC_Compute1b((input >> i) & 1);
	
	return (PEC_Compute1b(input & 1) << 1);
}

uint16_t PEC_Compute16b(uint16_t input)
{
	uint8_t i;
	for (i = 15; i > 0; i--)
		PEC_Compute1b((input >> i) & 1);
	
	return (PEC_Compute1b(input & 1) << 1);
}

void PEC_Reset(void)
{
	PEC_code = 0x0010;
}

uint16_t PEC_Get(void)
{
	return (PEC_code << 1);
}
