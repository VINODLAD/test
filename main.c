#include "stm32f10x.h"
#include "MCP41010.h"
#include "stm32f10x_GPIO.h"
#include "stm32f10x_GPIO.h"
int main(void){
	double Voltage1=5;
	double Voltage2=3.3;
	double Voltage3=12;
	uint8_t val1=(1/(Voltage1*1.25-1))*10000;
	uint8_t val2=(1/(Voltage2*1.25-1))*10000;
	uint8_t val3=(1/(Voltage3*1.25-1))*10000;
	Initialize_GPIO();
	DigiPots_SetVal(val1,val2,val3);
	
}
