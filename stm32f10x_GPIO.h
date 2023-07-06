#ifndef INC_STM32F103XX_GPIO_H_
#define INC_STM32F103XX_GPIO_H_

#include "stm32f10x.h"

/* Some useful macros*/
#define ENABLE													1
#define DISABLE 												0
#define SET_PIN													ENABLE
#define RESET_PIN												DISABLE
#define GPIO_TRIG_RISING								1
#define GPIO_TRIG_FALLING								2
#define GPIO_TRIG_RISING_FALLING				3
#define GPIOBASEADDR_TO_PORTCODE(x)			(((x)==GPIOA)?0:((x)==GPIOB)?1:((x)==GPIOC)?2:((x)==GPIOD)?3:0)
#define NO_PR_BIT_IMPLEMENTED           4

/* interrupt through EXTI line IRQNumber Macros*/
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40




/*	Arm Cortex M3 NVIC_ISERx Register Address */
#define NVIC_ISER0					(uint32_t*)0xE000E100
#define NVIC_ISER1					(uint32_t*)0xE000E104
#define NVIC_ISER2					(uint32_t*)0xE000E108
#define NVIC_ISER3					(uint32_t*)0xE000E10C

/*	Arm Cortex M3 NVIC_ICERx Register Address */
#define NVIC_ICER0					(uint32_t*)0XE000E180
#define NVIC_ICER1					(uint32_t*)0XE000E184
#define NVIC_ICER2					(uint32_t*)0XE000E188
#define NVIC_ICER3					(uint32_t*)0XE000E18C

/* Arm Cortex M3 NVIC_IPRx Register Address*/
#define NVIC_IPR_BASEADDR		(uint32_t*)0xE000E400


/*   GPIO Pin possible Modes    */
#define GPIO_MODE_IN       	0
#define GPIO_MODE_OUT10     1
#define GPIO_MODE_OUT2      2
#define GPIO_MODE_OUT50     3 

/*   GPIO Pin possible Configurations    */
#define GPIO_CNF_AM					0
#define GPIO_CNF_FI					1
#define GPIO_CNF_PUPD				2
#define GPIO_CNF_RESV				3
#define GPIO_CNF_PP					0
#define GPIO_CNF_OD					1
#define GPIO_CNF_AFPP				2
#define GPIO_CNF_AFOD				3

//#define OFFSET_PIN_CNF			4



/* Deinitialising  GPIO Pins -- Resetting pins*/
#define GPIOA_REG_RESET()      do{(RCC->APB2RSTR |=(1<<2));(RCC->APB2RSTR &=~(1<<2));}while(0)
#define GPIOB_REG_RESET()      do{(RCC->APB2RSTR |=(1<<3));(RCC->APB2RSTR &=~(1<<3));}while(0)
#define GPIOC_REG_RESET()      do{(RCC->APB2RSTR |=(1<<4));(RCC->APB2RSTR &=~(1<<4));}while(0)
/* This is GPIO Pin Configuration structure*/
typedef struct{
    uint32_t GPIO_PinNumber;
    uint32_t GPIO_PinMode;
		uint32_t GPIO_Config;
}GPIO_PinConfig_t;

/* This is a handle structure for a GPIO pin*/
typedef struct{			
	GPIO_TypeDef *pGPIOx;                   // This holds the base address of GPIO port to which pin Belongs
	GPIO_PinConfig_t GPIO_PinConfig;        // This is a member which holds GPIO Pin Configuration structure
}GPIO_Handle_t;

/*
---------------------------------Required GPIO Driver API's---------------------------------
                                1. for GPIO initialization
                                2. for enabling and disabling Clock
                                3. for reading data from a GPIO pin
                                4. for writing data to GPIO pin
                                5. for configuring alternate function mode
                                6. for interrupt handling                       */

/*
 * *********************************API's for GPIO**********************************/

// GPIO API for enabling clock for the respective port.
void GPIO_PCLK_CRT(GPIO_TypeDef *pGPIOx, uint8_t EnorDis);

// GPIO API Initializing a GPIO Port
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIO_TypeDef *pGPIOx);

// GPIO API for read and write
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx,uint8_t PinNumber);
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Value);
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx,uint8_t PinNumber);
void GPIO_SetReset(GPIO_TypeDef *pGPIOx,uint8_t PinNumber,uint8_t state);
void GPIO_Reset(GPIO_TypeDef *pGPIOx,uint8_t PinNumber);

// GPIO API for Interrupt Handling and configuration.
void GPIO_InterrupConfig(uint32_t* pGPIOx,uint32_t PinNumber,uint8_t TrigConfig);
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

//Delay Function
void delay(void);

#endif

