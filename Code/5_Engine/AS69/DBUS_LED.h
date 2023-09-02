#ifndef __BSP_LED_H_
#define __BSP_LED_H_

#include "stm32f4xx.h"

#define GPIO_LED_ON            do{HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);}while(0)
#define GPIO_LED_OFF           do{HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);}while(0)
#define GPIO_LED_SPARK         do{HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);}while(0)



void LED_GPIOG_Init(void);


#endif
