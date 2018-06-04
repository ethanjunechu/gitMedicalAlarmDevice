#ifndef __BSP_BUTTON_H__
#define __BSP_BUTTON_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
/* 类型定义 --------------------------------------------------------------*/
typedef enum
{
  BUTTON_UP   = 1,
  BUTTON_DOWN = 0,
}BUTTONState_TypeDef;

/* 宏定义 --------------------------------------------------------------------*/
#define BUTTON_TEST_GPIO_PIN                 GPIO_PIN_10
#define BUTTON_TEST_GPIO                     GPIOC
#define BUTTON_TEST_DOWN_LEVEL               0  /* 根据原理图设计，BUTTON_TEST按下时引脚为低电平，所以这里设置为0 */
#define BUTTON_TEST_EXTI_IRQn                EXTI15_10_IRQn
#define BUTTON_TEST_EXTI_IRQHandler          EXTI15_10_IRQHandler

#define BUTTON_CLEAR_GPIO_PIN                 GPIO_PIN_11
#define BUTTON_CLEAR_GPIO                     GPIOC
#define BUTTON_CLEAR_DOWN_LEVEL               0  /* 根据原理图设计，BUTTON_TEST按下时引脚为低电平，所以这里设置为0 */
#define BUTTON_CLEAR_EXTI_IRQHandler          EXTI15_10_IRQHandler
#define BUTTON_CLEAR_EXTI_IRQn                EXTI15_10_IRQn

/* 扩展变量 ------------------------------------------------------------------*/
/* 函数声明 ------------------------------------------------------------------*/

#endif
















