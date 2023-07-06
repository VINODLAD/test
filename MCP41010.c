#include "stm32f10x.h"
#include "MCP41010.h"
#include "stm32f103xx_spi.h"
#include "stm32f10x_GPIO.h"
void Initialize_GPIO(void){
	/*
	Data and Clock Pins
	|---- MOSI	A7 ----|---- MISO	A6 ----|---- CLK	A5 ----|
	
	Chip Select Pins
	|---- CS1		A1 ----|---- CS2	A2----|----	CS3		A3 ----|
	*/
	GPIO_Handle_t mosi,clk,cs;
	mosi.pGPIOx=GPIOA;
	clk.pGPIOx=GPIOA;
	cs.pGPIOx=GPIOA;
	GPIO_PCLK_CRT(GPIOA,ENABLE);
	
	mosi.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT50;
	mosi.GPIO_PinConfig.GPIO_Config=GPIO_CNF_OD;
	mosi.GPIO_PinConfig.GPIO_PinNumber=7;
	GPIO_Init(&mosi);
	
	clk.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT50;
	clk.GPIO_PinConfig.GPIO_Config=GPIO_CNF_OD;
	clk.GPIO_PinConfig.GPIO_PinNumber=5;
	GPIO_Init(&clk);
	
	cs.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT50;
	cs.GPIO_PinConfig.GPIO_Config=GPIO_CNF_OD;
	cs.GPIO_PinConfig.GPIO_PinNumber=1;
	GPIO_Init(&cs);
	cs.GPIO_PinConfig.GPIO_PinNumber=2;
	GPIO_Init(&cs);
	cs.GPIO_PinConfig.GPIO_PinNumber=3;
	GPIO_Init(&cs);
}

void DigiPots_SetVal(uint8_t val1,uint8_t val2,uint8_t val3){
	SPI_Handle_t digi_pot;
	digi_pot.pSPIx=SPI1;
	digi_pot.spiconf.CPHA=SPI_CPHA_Dn;
	digi_pot.spiconf.CPOL=SPI_CPOL_Dn;
	digi_pot.spiconf.BaudRate=SPI_BRR_Fpclk_256;
	digi_pot.spiconf.DFF=SPI_DFF_16;
	digi_pot.spiconf.Mode=SPI_MODE_Master;
	digi_pot.spiconf.SlvManage=SPI_SlvManage_Hard;
	uint16_t data1=(CMD_WRITE_DATA<<4)+val1;
	uint16_t data2=(CMD_WRITE_DATA<<4)+val2;
	uint16_t data3=(CMD_WRITE_DATA<<4)+val3;
	
	Initialize_GPIO();
	SPI_PeriClockControl(SPI1,ENABLE);
	GPIO_WriteToOutputPin(GPIOA,1,SET);
	GPIO_WriteToOutputPin(GPIOA,2,SET);
	GPIO_WriteToOutputPin(GPIOA,3,SET);
	SPI_Init(&digi_pot);
	
	SPI_Enable(SPI1,ENABLE);
	GPIO_WriteToOutputPin(GPIOA,1,RESET);
	SPI_SendData(SPI1,(uint8_t*)&data1,2);
	GPIO_WriteToOutputPin(GPIOA,1,SET);
	GPIO_WriteToOutputPin(GPIOA,2,RESET);
	SPI_SendData(SPI1,(uint8_t*)&data2,2);
	GPIO_WriteToOutputPin(GPIOA,2,SET);
	GPIO_WriteToOutputPin(GPIOA,3,RESET);
	SPI_SendData(SPI1,(uint8_t*)&data3,2);
	GPIO_WriteToOutputPin(GPIOA,1,SET);
	SPI_Enable(SPI1,DISABLE);
}

void Digipot_ShutDown(void){
	SPI_Handle_t digi_pot;
	digi_pot.pSPIx=SPI1;
	digi_pot.spiconf.CPHA=SPI_CPHA_Dn;
	digi_pot.spiconf.CPOL=SPI_CPOL_Dn;
	digi_pot.spiconf.BaudRate=SPI_BRR_Fpclk_256;
	digi_pot.spiconf.DFF=SPI_DFF_16;
	digi_pot.spiconf.Mode=SPI_MODE_Master;
	digi_pot.spiconf.SlvManage=SPI_SlvManage_Hard;
	uint16_t data=(CMD_SHDN<<4);
	
	Initialize_GPIO();
	SPI_PeriClockControl(SPI1,ENABLE);
	GPIO_WriteToOutputPin(GPIOA,1,SET);
	GPIO_WriteToOutputPin(GPIOA,2,SET);
	GPIO_WriteToOutputPin(GPIOA,3,SET);
	SPI_Init(&digi_pot);
	
	SPI_Enable(SPI1,ENABLE);
	GPIO_WriteToOutputPin(GPIOA,1,RESET);
	SPI_SendData(SPI1,(uint8_t*)&data,2);
}
