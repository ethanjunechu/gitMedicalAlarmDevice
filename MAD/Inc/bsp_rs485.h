#ifndef __BSP_RS485_H__
#define __BSP_RS485_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usart.h"

extern uint8_t RS485_RX_BUF[];  	//接收缓冲

//模式控制
#define RS485_TX_ON()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);__HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE)
#define RS485_TX_OFF()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE)


/********************************************************************
*	功能：485初始化
*	参数：bound 波特率
********************************************************************/
void RS485_Init(uint32_t BaudRate);

/**********************************************************
*	功能：发送数据
*	参数：buf 发送缓冲区首地址
*		  len 待发送的字节数
**********************************************************/
void RS485_Send_Data(uint8_t *buf,uint8_t len);

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen) ;

extern uint8_t RS485_RX_BUF[];  	//接收缓冲

#endif
















