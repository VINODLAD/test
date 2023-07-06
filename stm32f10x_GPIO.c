#include "stm32f10x_GPIO.h"
/*
This is the GPIO Driver Source file containing all the API's for
configuratin and interfacing GPIO pins on stm32f103xx based microcintroller

-------------------------------------Configurable items for user application -------------------------------------
                1. GPIO Port Name
                2. GPIO Pin number
                3. GPIO mode
                4. GPIO speed
                5. GPIO outputtype
                6. Pull-Pulldown
  	            7. GPIO Alternate Function Mode */

/*
---------------------------------Required GPIO Driver API's---------------------------------
                                1. for GPIO initialization
                                2. for enbling and disabling Clock
                                3. for reading data from a GPIO pin
                                4. for writing data to GPIO pin
                                5. for configuring alternate function mode
                                6. for interrupt handling                       */

/**********************************API's for GPIO**********************************/
// GPIO API for enabling clock for the respective port.
void GPIO_PCLK_CRT(GPIO_TypeDef *pGPIOx, uint8_t EnorDir)
{
	if (EnorDir==ENABLE){
		if (pGPIOx==GPIOA){
			RCC->APB2ENR|=(1<<2);
		}
		else if(pGPIOx==GPIOB){
			RCC->APB2ENR|=(1<<3);
		}
		else if(pGPIOx==GPIOC){
			RCC->APB2ENR|=(1<<4);
		}
		else if(pGPIOx==GPIOD){
			RCC->APB2ENR|=(1<<5);
		}
	}
	else if(EnorDir==DISABLE)
	{
		if (pGPIOx==GPIOA){
			RCC->APB2ENR&=~(1<<2);
		}
		else if(pGPIOx==GPIOB){
			RCC->APB2ENR&=~(1<<3);
		}
		else if(pGPIOx==GPIOC){
			RCC->APB2ENR&=~(1<<4);
		}
		else if(pGPIOx==GPIOD){
			RCC->APB2ENR&=~(1<<5);
		}
	}
}

// GPIO API for Initializing a GPIO Port
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	//Enabling the clock for Peripheral
	GPIO_PCLK_CRT(pGPIOHandle->pGPIOx,ENABLE);
	uint32_t temp=0;
	//Configuration of GPIO LOW Register.
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<=7){
		pGPIOHandle->pGPIOx->CRL|=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber*4));
		if (pGPIOHandle->GPIO_PinConfig.GPIO_Config==0){
			pGPIOHandle->pGPIOx->CRL&=~(1<<((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber*4)+2));
		}
		else{
			pGPIOHandle->pGPIOx->CRL&=~(1<<((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber*4)+2));
			pGPIOHandle->pGPIOx->CRL|=(pGPIOHandle->GPIO_PinConfig.GPIO_Config<<((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber*4)+2));
		}	
	}
	
	//Configuration of GPIO HIGH Register.
	else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber<16 && pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber>7){
		temp=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8;
		pGPIOHandle->pGPIOx->CRH|=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(temp*4));
		if (pGPIOHandle->GPIO_PinConfig.GPIO_Config==0){
			pGPIOHandle->pGPIOx->CRH&=~(3<<((temp*4)+2));
		}
		else{
			pGPIOHandle->pGPIOx->CRH&=~(3<<((temp*4)+2));
			pGPIOHandle->pGPIOx->CRH|=(pGPIOHandle->GPIO_PinConfig.GPIO_Config<<((temp*4)+2));
		}
	}
}

//GPIO API for Deinitializing a GPIO Port
void GPIO_Deinit(GPIO_TypeDef *pGPIOx){
	if(pGPIOx==GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx=GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();
	}
}

// GPIO API for Writing Data to GPIO Pin.
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if (Value == RESET_PIN){
		pGPIOx->ODR|=(1<<PinNumber);
	}
	if(Value==SET_PIN){
		pGPIOx->ODR&=~(1<<PinNumber);
	}
}

// GPIO API for Reading Data from GPIO Pin.
uint8_t GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value=((pGPIOx->IDR>>PinNumber) & 1);
	return value;
}

// GPIO API to toggle a PIN
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR^=(1<<PinNumber);
}

//GPIO API to Set and Reset Pin
void GPIO_SetReset(GPIO_TypeDef *pGPIOx,uint8_t PinNumber,uint8_t state){
	if (state==SET_PIN){
		pGPIOx->BSRR|=(1<<PinNumber);
	}
	else if (state==RESET_PIN){
		pGPIOx->BSRR&=~(1<<PinNumber);		
	}
}

void GPIO_Reset(GPIO_TypeDef *pGPIOx,uint8_t PinNumber){
		pGPIOx->BRR|=(1<<PinNumber);
}

//GPIO API for Interrupt Handling.
void GPIO_InterrupConfig(uint32_t* pGPIOx,uint32_t PinNumber,uint8_t TrigConfig){
	RCC->APB2ENR|=(1<<0);
	EXTI->IMR|=(1<<PinNumber);
	if (TrigConfig==GPIO_TRIG_RISING){
		EXTI->FTSR&=~(1<<PinNumber);
		EXTI->RTSR|=(1<<PinNumber);
	}
	else if(TrigConfig==GPIO_TRIG_FALLING){
		EXTI->RTSR&=~(1<<PinNumber);
		EXTI->FTSR|=(1<<PinNumber);
	}
	else if(TrigConfig==GPIO_TRIG_RISING_FALLING){
		EXTI->FTSR|=(1<<PinNumber);
		EXTI->RTSR|=(1<<PinNumber);
	}
	uint32_t port_code = GPIOBASEADDR_TO_PORTCODE(*pGPIOx);
	AFIO->EXTICR[PinNumber/4]|=(port_code<<(4*(PinNumber%4)));
}



void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if (EnorDi==ENABLE){
		NVIC_EnableIRQ(IRQNumber);
	}
	if (EnorDi==DISABLE){
		NVIC_DisableIRQ(IRQNumber);
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){
	//first find out the IPR regiter from IRQNumber.
	uint8_t iprx=IRQNumber/4;
	uint8_t iprx_section=IRQNumber%4;
	uint8_t shift_amount = (iprx_section*8)+(8-NO_PR_BIT_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR+(iprx*4))|=(IRQPriority<<shift_amount );
}
void GPIO_IRQHandling(uint8_t PinNumber){
	//clear the EXTI PR register corresponding to the pin Number.
	if ((EXTI->PR) &(1<<PinNumber)){
		//Clear the bit for that EXTI line
		EXTI->PR|=(1<<PinNumber);
	}
}
//Delay Function
void delay(void){
		for (int i=0;i<1000000;i++);
}
