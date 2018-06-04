#ifndef __BSP_LED_H__
#define __BSP_LED_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* LED0 */
#define LED0A(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET))
#define LED0B(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET))
#define LED0C(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET))
/* LED1 */
#define LED1A(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET))
#define LED1B(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET))
#define LED1C(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET))
/* LED2 */
#define LED2A(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET))
#define LED2B(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET))
#define LED2C(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET))
/* LEDPOWER */
#define LEDPOWERA(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET))
#define LEDPOWERB(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET))
#define LEDPOWERC(x)    (x == 1 ? HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET))

#endif

