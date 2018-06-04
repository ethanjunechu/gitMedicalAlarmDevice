#ifndef __BSP_SPI_FLASH_H__
#define __BSP_SPI_FLASH_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
//#define  SPI_FLASH_ID                       0xEF3015     //W25X16   2MB
#define  SPI_FLASH_ID                       0xEF4016     //W25Q16   4MB
//#define  SPI_FLASH_ID                       0XEF4017     //W25Q64   8MB
//#define  SPI_FLASH_ID                       0XEF4018     //W25Q128  16MB

#define FLASH_SPI_CS_PORT                          GPIOB
#define FLASH_SPI_CS_PIN                           GPIO_PIN_12

#define FLASH_SPI_CS_ENABLE()                      HAL_GPIO_WritePin(FLASH_SPI_CS_PORT, FLASH_SPI_CS_PIN, GPIO_PIN_RESET)
#define FLASH_SPI_CS_DISABLE()                     HAL_GPIO_WritePin(FLASH_SPI_CS_PORT, FLASH_SPI_CS_PIN, GPIO_PIN_SET)

typedef struct {
	/* flashed */
	uint8_t flashed[6];
	/* 上下限 */
	float upper_limit;
	float lower_limit;
	/* 量程 */
	uint8_t rangeIndex;
	/* 压力修正系数 */
	signed int K;
	signed int zero;
	/* 音量 */
	uint8_t volume;
	/* 密码 第一位为密码长度 */
	uint8_t userPassword[13];
	uint8_t omePassword[13];
	uint8_t rootPassword[13];
	/* 名称 */
	uint8_t nameIndex;
	/* modbus地址 */
	uint8_t modbusAddr;
	/* 波特率 */
	uint8_t baudrateIndex;
	/* IP地址 */
	uint8_t IP[4];
	/* 模式 */
	uint8_t modeIndex;
	/* 授权时间 */
	char omeTime[6];
	uint32_t omeDays;
	char rootTime[6];
	uint32_t rootDays;
} SAVEDATA;

/* 扩展变量 ------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;

/* 函数声明 ------------------------------------------------------------------*/

void MX_SPIFlash_Init(void);
void SPI_FLASH_SectorErase(uint32_t SectorAddr);
void SPI_FLASH_BulkErase(void);
void SPI_FLASH_PageWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferWrite(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void SPI_FLASH_BufferRead(uint8_t* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
uint32_t SPI_FLASH_ReadID(void);
uint32_t SPI_FLASH_ReadDeviceID(void);
void SPI_FLASH_StartReadSequence(uint32_t ReadAddr);
void SPI_Flash_PowerDown(void);
void SPI_Flash_WAKEUP(void);

uint8_t SPI_FLASH_ReadByte(void);
uint8_t SPI_FLASH_SendByte(uint8_t byte);
void SPI_FLASH_WriteEnable(void);
void SPI_FLASH_WaitForWriteEnd(void);

#endif  /* __BSP_SPI_FLASH_H__ */

/******************* (C) COPYRIGHT 2017-2020 Ethan ***** END OF FILE ****/
