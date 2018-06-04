#ifndef __BSP_DS302_H__
#define __BSP_DS302_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#define RST_1 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET)
#define RST_0 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET)
#define IO_1 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET)
#define IO_0 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET)
#define SCLK_1 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
#define SCLK_0 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)

void ds1302_init(void);
void ds1302_write(unsigned char time_tx);
void set_time(void);
void get_time(void);
void get_date(void);
void IO_OUT(void);
void IO_INPUT(void);

#endif
