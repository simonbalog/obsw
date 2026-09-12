#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

void Error_Handler(void);

#define B1_Pin             GPIO_PIN_13
#define B1_GPIO_Port       GPIOC
#define LORA_NSS_Pin       GPIO_PIN_15
#define LORA_NSS_GPIO_Port GPIOD
#define LORA_RST_Pin       GPIO_PIN_3
#define LORA_RST_GPIO_Port GPIOF
#define LORA_DIO0_Pin      GPIO_PIN_12
#define LORA_DIO0_GPIO_Port GPIOG
#define BNO_INT_Pin        GPIO_PIN_9
#define BNO_INT_GPIO_Port  GPIOE
#define SD_CD_Pin          GPIO_PIN_11
#define SD_CD_GPIO_Port    GPIOE

#ifdef __cplusplus
}
#endif

#endif
