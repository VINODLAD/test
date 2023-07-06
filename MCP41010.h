#include "stm32f10x.h"

#define CMD_WRITE_DATA 	(uint8_t)0x41
#define CMD_SHDN				(uint8_t)0x81

void Initialize_GPIO(void);
void DigiPots_SetVal(uint8_t val1,uint8_t val2,uint8_t val3);
void Digipot_ShutDown(void);
