/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "bsp_spi_flash.h"
#include "bsp_rs485.h"
#include "bsp_ds1302.h"
#include "bsp_led.h"
#include "bsp_button.h"
#include "bsp_usart_lcd.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* printf重定向 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 0xFFFF);
	return ch;
}

#define CMD_MAX_SIZE 60
#define SELFTESTTIME 1000
/* 板子基准电压 */
#define     AD_VAL_WHEN_4MA    774.72723	//4ma,0.6V,
#define     AD_VAL_WHEN_20MA   3723.63636	//20ma,3V,

/* 时钟变量 DS1302 */
//extern volatile char time_tab[];
extern char date_1302[];
//extern char time_1302[];

/* RS485-RS232 参数 */
extern uint8_t RS485_RX_BUF[8];
unsigned char rom485[256];
uint8_t RS232_RX_BUF[CMD_MAX_SIZE];
extern DMA_HandleTypeDef hdma_usart2_rx;
uint8_t rom485[256];
uint8_t RS232_recvEndFlag = 0;
uint8_t RS232_recvLength = 0;

/* UI 相关参数 */
/* 发送AD值和超压欠压状态 */
// EE B1 12 00 01
// 00 06 00 05 2D 31 2E 32 37 AD1
// 00 09 00 05 20 31 2E 32 37 AD2
// 00 0C 00 05 20 31 2E 32 37 AD3
// 00 08 00 04 D5 FD B3 A3	  状态1
// 00 0B 00 04 B3 AC D1 B9	  状态2
// 00 0E 00 04 C7 B7 D1 B9	  状态3
// FF FC FF FF
uint8_t multiUICMD[60] = { 0xEE, 0xB1, 0x12, 0x00, 0x01, 0x00, 0x06, 0x00, 0x05,
		0x2D, 0x31, 0x2E, 0x32, 0x37, 0x00, 0x09, 0x00, 0x05, 0x20, 0x31, 0x2E,
		0x32, 0x37, 0x00, 0x0C, 0x00, 0x05, 0x20, 0x31, 0x2E, 0x32, 0x37, 0x00,
		0x08, 0x00, 0x04, 0xD5, 0xFD, 0xB3, 0xA3, 0x00, 0x0B, 0x00, 0x04, 0xB3,
		0xAC, 0xD1, 0xB9, 0x00, 0x0E, 0x00, 0x04, 0xC7, 0xB7, 0xD1, 0xB9, 0xFF,
		0xFC, 0xFF, 0xFF };
uint8_t lcd_number_x;
uint8_t current_index[3] = { 0, 0, 0 };
uint8_t currentPage = 0;
uint8_t lastPage = 0;
extern uint8_t setTextGreen[];
extern uint8_t setTextRed[];
uint8_t statusColorCMD[3][13] = { { 0xEE, 0xB1, 0x19, 0x00, 0x01, 0x00, 0x08,
		0x07, 0xE0, 0xFF, 0xFC, 0xFF, 0xFF }, { 0xEE, 0xB1, 0x19, 0x00, 0x01,
		0x00, 0x0B, 0x07, 0xE0, 0xFF, 0xFC, 0xFF, 0xFF }, { 0xEE, 0xB1, 0x19,
		0x00, 0x01, 0x00, 0x0E, 0x07, 0xE0, 0xFF, 0xFC, 0xFF, 0xFF } };
uint8_t numColorCMD[3][13] = { { 0xEE, 0xB1, 0x19, 0x00, 0x01, 0x00, 0x06, 0x07,
		0xE0, 0xFF, 0xFC, 0xFF, 0xFF }, { 0xEE, 0xB1, 0x19, 0x00, 0x01, 0x00,
		0x09, 0x07, 0xE0, 0xFF, 0xFC, 0xFF, 0xFF }, { 0xEE, 0xB1, 0x19, 0x00,
		0x01, 0x00, 0x0C, 0x07, 0xE0, 0xFF, 0xFC, 0xFF, 0xFF } };
//能量柱动画帧选择显示命令
//EE B1 23 00 01 00 02 00 FF FC FF FF
uint8_t percentPicCMD[3][12] = { { 0xEE, 0xB1, 0x23, 0x00, 0x01, 0x00, 0x02,
		0x00, 0xFF, 0xFC, 0xFF, 0xFF }, { 0xEE, 0xB1, 0x23, 0x00, 0x01, 0x00,
		0x03, 0x00, 0xFF, 0xFC, 0xFF, 0xFF }, { 0xEE, 0xB1, 0x23, 0x00, 0x01,
		0x00, 0x04, 0x00, 0xFF, 0xFC, 0xFF, 0xFF } };
//音频图标动画帧选择显示命令
uint8_t volumePicCMD[12] = { 0xEE, 0xB1, 0x23, 0x00, 0x01, 0x00, 0x05, 0x00,
		0xFF, 0xFC, 0xFF, 0xFF };
//授权验证提示命令
//授权未到期
//EE B1 10 00 01 00 15 FF FC FF FF
uint8_t licPassedCMD[11] = { 0xEE, 0xB1, 0x10, 0x00, 0x01, 0x00, 0x15, 0xFF,
		0xFC, 0xFF, 0xFF };
//授权到期
//EE B1 10 00 01 00 15 CA DA C8 A8 B5 BD C6 DA FF FC FF FF
uint8_t licFailedCMD[19] = { 0xEE, 0xB1, 0x10, 0x00, 0x01, 0x00, 0x15, 0xD0,
		0xE8, 0xD2, 0xAA, 0xB1, 0xA3, 0xD1, 0xF8, 0xFF, 0xFC, 0xFF, 0xFF };
extern uint8_t cnName[21][15];
extern uint8_t enName[21][10];

/* Led 控制 */
/* 0 - 欠压 | 1 - 正常 | 2 - 超压 */
uint8_t ledFlag[3] = { 1, 1, 1 };

uint8_t testFlag = 0;

#define ADC_NUMOFCHANNEL 3
/* AD转换结果值 */
uint32_t ADC_ConvertedValue[ADC_NUMOFCHANNEL];
volatile int g_adc_Temp_filter[3] = { 0, 0, 0 }, gg_adc_Temp_filter[3] = { 0, 0,
		0 }, adcTemp[3][20], adc_count[3] = { 0, 0, 0 }, adc_index[3] = { 0, 0,
		0 };
int adc_Temp_filter[3] = { 0, 0, 0 };
float float_ADCValue[3] = { 0, 0, 0 };
signed int K[3] = { 0, 0, 0 };
float k[3] = { 0, 0, 0 };
float b[3] = { 0, 0, 0 };
/* 量程 - 根据 rangeIndex 设置进行索引 */
float val_20mA[3];
float val_4mA[3];

/* eepromSaveData */
SAVEDATA saveData[3];
uint8_t currentCHN;

/* 音频相关参数 */
uint8_t bebe = 0;
uint8_t muteFlag = 0;
uint8_t alarmFlag = 0;

/* Timer 相关参数*/
signed long timeStamp = 999;
signed long currentTime = 0;
uint32_t minTick = 0;

/* 密码 第一位为密码长度 */
uint8_t inputPassword[13];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void application(void);
void eepromReadSetting(void);
void eepromWriteSetting(void);
void loadMainPage(void);
void factorySetting(uint8_t permisson);
void selfTest(void);
void updateADC(void);
void updateUI(void);
void updateLed(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* ADCHandle);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void UART_RxIDLECallback(UART_HandleTypeDef* uartHandle);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* tim_baseHandle);
uint32_t calcDays(uint8_t y1, uint8_t m1, uint8_t d1, uint8_t y2, uint8_t m2,
		uint8_t d2);
void FloatToStr5(float data, uint8_t *buf, int size);
float StrToFloat(uint8_t *buf);
void set485rom(uint8_t func);
void change_float_big_485rom(unsigned int j);
void Line_1A_WTN5(unsigned char SB_DATA);
void alarm_on(void);
void alarm_off(void);
void bsp_Delay_Nus(uint16_t time);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_SPI2_Init();
	MX_SPI1_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();

	/* USER CODE BEGIN 2 */
	/* DEBUG 1 开始 */

//  /* RS232 传输测试 */
//  printf("debug start\r\n");
//  /* 12 颗 LED 跑马灯测试 */
//  printf("\r\nLED debug\r\n");
//
//  printf("LEDPOWERA-C\r\n");
//  LEDPOWERA(1);
//  HAL_Delay(500);
//  LEDPOWERA(0);
//
//  LEDPOWERB(1);
//  HAL_Delay(500);
//  LEDPOWERB(0);
//
//  LEDPOWERC(1);
//  HAL_Delay(500);
//  LEDPOWERC(0);
//
//  printf("LED0A-C\r\n");
//  LED0A(1);
//  HAL_Delay(500);
//  LED0A(0);
//
//  LED0B(1);
//  HAL_Delay(500);
//  LED0B(0);
//
//  LED0C(1);
//  HAL_Delay(500);
//  LED0C(0);
//
//  printf("LED1A-C\r\n");
//  LED1A(1);
//  HAL_Delay(500);
//  LED1A(0);
//
//  LED1B(1);
//  HAL_Delay(500);
//  LED1B(0);
//
//  LED1C(1);
//  HAL_Delay(500);
//  LED1C(0);
//
//  printf("LED2A-C\r\n");
//  LED2A(1);
//  HAL_Delay(500);
//  LED2A(0);
//
//  LED2B(1);
//  HAL_Delay(500);
//  LED2B(0);
//
//  LED2C(1);
//  HAL_Delay(500);
//  LED2C(0);
//
//  /* bsp_spi_flash debug */
//  printf("\r\nspi-flash debug\r\n");
//  uint32_t ID;
//  ID = SPI_FLASH_ReadID();
//  printf("ID = %x\r\n", ID);
//  uint8_t flashDebugFlag = 1;
//  uint8_t flashWriteTestBuff[18] = "flash-write-test\0";
//  uint8_t flashReadTestBuff[18];
//  SPI_FLASH_BufferWrite(flashWriteTestBuff, 8888, 18);
//  printf("flash-write addr 8888 = flash-write-test\r\n");
//  SPI_FLASH_BufferRead(flashReadTestBuff, 8888, 18);
//  printf("flash-read  addr 8888 = %s\r\n", flashReadTestBuff);
//  if (ID == SPI_FLASH_ID) {
//    uint8_t i;
//    for (i = 0; i < 18; i++) {
//      if (flashWriteTestBuff[i] != flashReadTestBuff[i]) {
//        flashDebugFlag = 0;
//      }
//    }
//  } else
//    flashDebugFlag = 0;
//  if (flashDebugFlag)
//    printf("spi-flash debug OK!\r\n");
//  else
//    printf("spi-flash debug Err!\r\n");
//
//  /* ds1302 debug */
//  printf("\r\nds1302 debug start\r\n");
//  time_tab[0] = 0x00;
//  time_tab[1] = 0x59;
//  time_tab[2] = 0x23;
//  time_tab[3] = 0x31;
//  time_tab[4] = 0x12;
//  time_tab[5] = 0x07;
//  time_tab[6] = 0x17;
//  time_tab[7] = 0x20;
//  ds1302_init();
//  set_time();
//  HAL_Delay(3000);
//  get_time();
//  get_date();
//  printf("ds1302 read %d%d/%d%d/%d%d %d%d:%d%d:%d%d\r\n", date_1302[5],
//      date_1302[4], date_1302[3], date_1302[2], date_1302[1], date_1302[0],
//      time_1302[5], time_1302[4], time_1302[3], time_1302[2], time_1302[1],
//      time_1302[0]);
//  if ((time_1302[0] - time_tab[0]) == 3)
//    printf("ds1302 ticket 3 sec debug OK!\r\n");
//  else
//    printf("ds1302 debug Err!\r\n");
//
//  /* 3.3为AD转换的参考电压值，stm32的AD转换为12bit，2^12=4096，
//   即当输入为3.3V时，AD转换结果为4096 */
//  ADC_ConvertedValueLocal[0] = (float) (ADC_ConvertedValue[0] & 0xFFF) * 3.3
//      / 4096; // ADC_ConvertedValue[0]只取最低12有效数据
//  ADC_ConvertedValueLocal[1] = (float) (ADC_ConvertedValue[1] & 0xFFF) * 3.3
//      / 4096; // ADC_ConvertedValue[1]只取最低12有效数据
//  ADC_ConvertedValueLocal[2] = (float) (ADC_ConvertedValue[2] & 0xFFF) * 3.3
//      / 4096; // ADC_ConvertedValue[2]只取最低12有效数据
//
//  printf("CH1_PC0 value = %d -> %fV\n", ADC_ConvertedValue[0] & 0xFFF,
//      ADC_ConvertedValueLocal[0]);
//  printf("CH2_PC1 value = %d -> %fV\n", ADC_ConvertedValue[1] & 0xFFF,
//      ADC_ConvertedValueLocal[1]);
//  printf("CH3_PC2 value = %d -> %fV\n", ADC_ConvertedValue[2] & 0xFFF,
//      ADC_ConvertedValueLocal[2]);
//
//  printf("\n");
//  HAL_Delay(3000);
	/* DEBUG 1 结束 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
	/* 主应用 */
	application();
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/**
 * @功能简介 : 主要功能函数
 * @入口参数 : 无
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void application(void) {

	/* 开机指示灯 */
	LED0A(1);
	LED0B(1);
	LED0C(1);
	/* LED1 */
	LED1A(1);
	LED1B(1);
	LED1C(1);
	/* LED2 */
	LED2A(1);
	LED2B(1);
	LED2C(1);
	/* LEDPOWER */
	LEDPOWERA(1);
	LEDPOWERB(1);
	LEDPOWERC(1);

	/* 读取 FLASH 储存的设置 */
	uint8_t i = 0;
	eepromReadSetting();
	for (i = 0; i < 6; i++) {
		if (saveData[0].flashed[i] != '8') {
			i = 8;
		}
	}
	/* 出产设置 */
	if (i != 6) {
		factorySetting(0);
		eepromWriteSetting();
		eepromReadSetting();
	}

	/* 功能初始化 start */
	/* RS485 接收模式 */
	RS485_TX_OFF()
	;

	/* 启动AD转换并使能DMA传输和中断 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, ADC_ConvertedValue, ADC_NUMOFCHANNEL);

	/* 启动Timer1 */
	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
		/* Starting Error */
		while (1)
			;
	}

	/* main 函数 while(1) 前，启动一次 DMA 接收 */
	if (HAL_UART_Receive_DMA(&huart2, (uint8_t *) RS232_RX_BUF,
	CMD_MAX_SIZE) != HAL_OK) {
		Error_Handler();
	}
	/* 开启串口2空闲中断 */
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

	/* 启动检查画面 */
	uint8_t temp[7];
	temp[0] = 0xEE;  			//帧头
	temp[1] = 0xB1;				//命令类型(UPDATE_CONTROL)
	temp[2] = 0x01;
	temp[3] = 0xFF;   			//帧尾
	temp[4] = 0xFC;
	temp[5] = 0xFF;
	temp[6] = 0xFF;
	HAL_UART_Transmit(&huart2, temp, 7, 0xFFFF);
	/* 功能初始化 end */

	/* 正常工作 */
	while (1) {
		if (RS232_recvEndFlag == 1) {
			ProcessUIMessage((PCTRL_MSG) RS232_RX_BUF, RS232_recvLength); //指令处理

			RS232_recvLength = 0;
			RS232_recvEndFlag = 0;
			HAL_UART_Receive_DMA(&huart2, (uint8_t *) RS232_RX_BUF,
			CMD_MAX_SIZE);  //重新使能DMA接收
			/* 开启串口2空闲中断 */
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
		}
		if (currentPage == PAGE_MAIN) {
			/* 自检 */
			if (timeStamp == SELFTESTTIME || testFlag == 1) {
				selfTest();
			}
			updateUI();
			updateLed();
		}
	}
} /* End application() */

/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void eepromReadSetting(void) {
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel7_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
	HAL_Delay(1000);
	SPI_FLASH_BufferRead((uint8_t *) &saveData, 0, 300);
	HAL_Delay(1000);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}/* End eepromReadSetting() */

/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void eepromWriteSetting(void) {
	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel7_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_DisableIRQ(TIM2_IRQn);
	HAL_Delay(1000);
	SPI_FLASH_SectorErase(0);
	SPI_FLASH_BufferWrite((uint8_t *) &saveData, 0, 300);
	HAL_Delay(1000);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}/* End eepromWriteSetting() */

/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void loadMainPage(void) {
	uint8_t i, j;
	//EE B1 12 00 01
	//00 0F 00 01 31
	//00 10 00 01 32
	//00 11 00 01 33
	//00 12 00 01 34
	//00 13 00 01 35
	//00 14 00 01 36
	//00 07 00 01 37
	//00 0A 00 01 38
	//00 0D 00 01 39
	//FF FC FF FF
	uint8_t temp[100];
	temp[0] = 0xEE;
	temp[1] = 0xB1;
	temp[2] = 0x12;
	temp[3] = 0x00;
	temp[4] = 0x01;
	/* 通道 1 名称 */
	temp[5] = 0x00;
	temp[6] = 0x0F;
	j = 7;
	for (i = 0;
			i
					< ((enName[saveData[0].nameIndex][0] << 8)
							+ enName[saveData[0].nameIndex][1] + 2); i++) {
		temp[i + 7] = enName[saveData[0].nameIndex][i];
		j++;
	}

	temp[j++] = 0x00;
	temp[j++] = 0x10;
	for (i = 0;
			i
					< ((cnName[saveData[0].nameIndex][0] << 8)
							+ cnName[saveData[0].nameIndex][1] + 2); i++) {
		temp[i + j] = cnName[saveData[0].nameIndex][i];
	}
	j = j + i;

	/* 通道 2 名称 */
	temp[j++] = 0x00;
	temp[j++] = 0x11;
	for (i = 0;
			i
					< ((enName[saveData[1].nameIndex][0] << 8)
							+ enName[saveData[1].nameIndex][1] + 2); i++) {
		temp[i + j] = enName[saveData[1].nameIndex][i];
	}
	j = j + i;

	temp[j++] = 0x00;
	temp[j++] = 0x12;
	for (i = 0;
			i
					< ((cnName[saveData[1].nameIndex][0] << 8)
							+ cnName[saveData[1].nameIndex][1] + 2); i++) {
		temp[i + j] = cnName[saveData[1].nameIndex][i];
	}
	j = j + i;

	/* 通道 3 名称 */
	temp[j++] = 0x00;
	temp[j++] = 0x13;
	for (i = 0;
			i
					< ((enName[saveData[2].nameIndex][0] << 8)
							+ enName[saveData[2].nameIndex][1] + 2); i++) {
		temp[i + j] = enName[saveData[2].nameIndex][i];
	}
	j = j + i;

	temp[j++] = 0x00;
	temp[j++] = 0x14;
	for (i = 0;
			i
					< ((cnName[saveData[2].nameIndex][0] << 8)
							+ cnName[saveData[2].nameIndex][1] + 2); i++) {
		temp[i + j] = cnName[saveData[2].nameIndex][i];
	}
	j = j + i;

	/* 通道 1 单位 */
	temp[j++] = 0x00;
	temp[j++] = 0x07;
	//00 03 4D 50 61 单位MPa
	if (saveData[0].rangeIndex != 3) {
		temp[j++] = 0x00;
		temp[j++] = 0x03;
		temp[j++] = 0x4D;
		temp[j++] = 0x50;
		temp[j++] = 0x61;
	}
	//00 03 6B 50 61 单位kPa
	else {
		temp[j++] = 0x00;
		temp[j++] = 0x03;
		temp[j++] = 0x6B;
		temp[j++] = 0x50;
		temp[j++] = 0x61;
	}

	/* 通道 2 单位 */
	temp[j++] = 0x00;
	temp[j++] = 0x0A;
	//00 03 4D 50 61 单位MPa
	if (saveData[1].rangeIndex != 3) {
		temp[j++] = 0x00;
		temp[j++] = 0x03;
		temp[j++] = 0x4D;
		temp[j++] = 0x50;
		temp[j++] = 0x61;
	}
	//00 03 6B 50 61 单位kPa
	else {
		temp[j++] = 0x00;
		temp[j++] = 0x03;
		temp[j++] = 0x6B;
		temp[j++] = 0x50;
		temp[j++] = 0x61;
	}

	/* 通道 3 单位 */
	temp[j++] = 0x00;
	temp[j++] = 0x0D;
	//00 03 4D 50 61 单位MPa
	if (saveData[2].rangeIndex != 3) {
		temp[j++] = 0x00;
		temp[j++] = 0x03;
		temp[j++] = 0x4D;
		temp[j++] = 0x50;
		temp[j++] = 0x61;
	}
	//00 03 6B 50 61 单位kPa
	else {
		temp[j++] = 0x00;
		temp[j++] = 0x03;
		temp[j++] = 0x6B;
		temp[j++] = 0x50;
		temp[j++] = 0x61;
	}

	temp[j++] = 0xFF;
	temp[j++] = 0xFC;
	temp[j++] = 0xFF;
	temp[j++] = 0xFF;
	HAL_UART_Transmit(&huart2, temp, j++, 0xFFFF);

	LEDPOWERA((saveData[0].nameIndex != 0));
	LEDPOWERB((saveData[1].nameIndex != 0));
	LEDPOWERC((saveData[2].nameIndex != 0));

	/* rangeIndex 设置0-20ma量程 */
	//0/1MPa | 0/1.6MPa | 0/2.5MPa | -100/300kPa
	for (i = 0; i < 3; i++) {
		switch (saveData[i].rangeIndex) {
		case 0:
			val_20mA[i] = 1;
			val_4mA[i] = 0;
			break;
		case 1:
			val_20mA[i] = 1.6;
			val_4mA[i] = 0;
			break;
		case 2:
			val_20mA[i] = 2.5;
			val_4mA[i] = 0;
			break;
		case 3:
			val_20mA[i] = 300;
			val_4mA[i] = -100;
			break;
		}
	}

	get_date();
	if (saveData[0].omeDays
			> calcDays(saveData[0].omeTime[5] * 10 + saveData[0].omeTime[4],
					saveData[0].omeTime[3] * 10 + saveData[0].omeTime[2],
					saveData[0].omeTime[1] * 10 + saveData[0].omeTime[0],
					date_1302[5] * 10 + date_1302[4],
					date_1302[3] * 10 + date_1302[2],
					date_1302[1] * 10 + date_1302[0])
			&& saveData[0].rootDays
					> calcDays(
							saveData[0].rootTime[5] * 10
									+ saveData[0].rootTime[4],
							saveData[0].rootTime[3] * 10
									+ saveData[0].rootTime[2],
							saveData[0].rootTime[1] * 10
									+ saveData[0].rootTime[0],
							date_1302[5] * 10 + date_1302[4],
							date_1302[3] * 10 + date_1302[2],
							date_1302[1] * 10 + date_1302[0])) {
		HAL_UART_Transmit(&huart2, licPassedCMD, 11, 0xFFFF);
	} else {
		HAL_UART_Transmit(&huart2, licFailedCMD, 19, 0xFFFF);
	}

	//清除密码错误提示
	//EE B1 10 00 02 00 04 20 FF FC FF FF
	uint8_t temp0[12] = { 0xEE, 0xB1, 0x10, 0x00, 0x02, 0x00, 0x04, 0x20, 0xFF,
			0xFC, 0xFF, 0xFF };
	HAL_UART_Transmit(&huart2, temp0, 12, 0xFFFF);
	/* 设置音量 */
	HAL_Delay(500);
	Line_1A_WTN5(0xE0 + ((uint8_t) (saveData[0].volume * 1.5) & 0x0F)); //音量
	HAL_Delay(500);

	set485rom(0);

	huart1.Instance = USART1;
	switch (saveData[0].baudrateIndex) {
	case 0:
		huart1.Init.BaudRate = 2400;
		break;
	case 1:
		huart1.Init.BaudRate = 4800;
		break;
	case 2:
		huart1.Init.BaudRate = 9600;
		break;
	case 3:
		huart1.Init.BaudRate = 19200;
		break;
	case 4:
		huart1.Init.BaudRate = 38400;
		break;
	}
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
	/* 启动一次 DMA 接收 */
	if (HAL_UART_Receive_DMA(&huart1, (uint8_t *) RS485_RX_BUF, 8) != HAL_OK) {
		Error_Handler();
	}
	/* 开启串口1空闲中断 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	//修改音量图标
	if (muteFlag == 1) {
		volumePicCMD[7] = 0;
	} else {
		volumePicCMD[7] = (uint8_t) (saveData[0].volume / 10);
		if (volumePicCMD[7] <= 4) {
			volumePicCMD[7] = 1;
		} else if (volumePicCMD[7] > 4 && volumePicCMD[7] <= 7) {
			volumePicCMD[7] = 2;
		} else if (volumePicCMD[7] > 7) {
			volumePicCMD[7] = 3;
		}
	}
	//修改音量图标
	HAL_UART_Transmit(&huart2, volumePicCMD, 12, 0xFFFF);

}/* End loadMainPage() */

/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void factorySetting(uint8_t permisson) {
	uint8_t i;
	for (i = 0; i < 3; i++) {
		saveData[i].upper_limit = 1;
		saveData[i].lower_limit = 0;
		saveData[i].rangeIndex = 0;
		saveData[i].K = 1;
		saveData[i].zero = 0;
		saveData[i].nameIndex = 0;
	}
	for (i = 0; i < 3; i++) {
		val_20mA[i] = 1;
		val_4mA[i] = 0;
	}
	saveData[0].volume = 50;
	saveData[0].modbusAddr = 1;
	saveData[0].baudrateIndex = 2;
	saveData[0].IP[0] = 192;
	saveData[0].IP[1] = 168;
	saveData[0].IP[2] = 0;
	saveData[0].IP[3] = 1;
	saveData[0].modeIndex = 0;
	for (i = 0; i < 6; i++) {
		saveData[0].flashed[i] = '8';
	}
	/* 授权时间初始化 */
	ds1302_init();
	set_time();
	HAL_Delay(3000);
	get_date();
	/* 初始密码 1级 4个8 | 2级 6个8 | 3级 8个8 |  */
	switch (permisson) {
	/* root */
	case 0:
		saveData[0].rootPassword[0] = 8;
		for (i = 1; i < 9; i++) {
			saveData[0].rootPassword[i] = 0x38;
		}
		for (i = 0; i < 6; i++) {
			saveData[0].rootTime[i] = date_1302[i];
		}
		saveData[0].rootDays = 365;

		saveData[0].omePassword[0] = 6;

		for (i = 1; i < 7; i++) {
			saveData[0].omePassword[i] = 0x38;
		}
		for (i = 0; i < 6; i++) {
			saveData[0].omeTime[i] = date_1302[i];
		}

		saveData[0].omeDays = 365;

		saveData[0].userPassword[0] = 4;
		for (i = 1; i < 5; i++) {
			saveData[0].userPassword[i] = 0x38;
		}
		break;
		/* ome */
	case 1:
		saveData[0].omePassword[0] = 6;

		for (i = 1; i < 7; i++) {
			saveData[0].omePassword[i] = 0x38;
		}
		for (i = 0; i < 6; i++) {
			saveData[0].omeTime[i] = date_1302[i];
		}

		saveData[0].omeDays = 365;

		saveData[0].userPassword[0] = 4;
		for (i = 1; i < 5; i++) {
			saveData[0].userPassword[i] = 0x38;
		}
		break;
		/* user */
	case 2:
		saveData[0].userPassword[0] = 4;
		for (i = 1; i < 5; i++) {
			saveData[0].userPassword[i] = 0x38;
		}
		break;
	}
}
/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void selfTest(void) {
	uint8_t temp[12];
	uint8_t tempAdcASCii[5];
	uint8_t i;
	/* 开启运行指示灯 */
	LEDPOWERA(1);
	LEDPOWERB(1);
	LEDPOWERC(1);
	/* 蜂鸣器报警 */
	alarm_on();
	alarm_on();
	alarm_on();
	alarm_on();
	alarm_on();
//EE 61 0F FF FC FF FF
	temp[0] = 0xEE;	//帧头
	temp[1] = 0x61;	//命令类型(UPDATE_CONTROL)
	temp[2] = 50;
	temp[3] = 0xFF;	//帧尾
	temp[4] = 0xFC;
	temp[5] = 0xFF;
	temp[6] = 0xFF;
	HAL_UART_Transmit(&huart2, temp, 7, 0xFFFF);
	/* 自检欠压 */
	/* 发送AD值和超压欠压状态 */
// EE B1 12 00 01
// 00 06 00 05 2D 31 2E 32 37 AD1
// 00 09 00 05 20 31 2E 32 37 AD2
// 00 0C 00 05 20 31 2E 32 37 AD3
// 00 08 00 04 D5 FD B3 A3	  状态1
// 00 0B 00 04 B3 AC D1 B9	  状态2
// 00 0E 00 04 C7 B7 D1 B9	  状态3
// FF FC FF FF
	FloatToStr5(0, tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 9] = tempAdcASCii[i];
	}
	FloatToStr5(0, tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 18] = tempAdcASCii[i];
	}
	FloatToStr5(0, tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 27] = tempAdcASCii[i];
	}
	/* 通道 1 */
//欠压
	multiUICMD[36] = 0xD7;
	multiUICMD[37] = 0xD4;
	multiUICMD[38] = 0xBC;
	multiUICMD[39] = 0xEC;
	statusColorCMD[0][7] = setTextRed[0];
	statusColorCMD[0][8] = setTextRed[1];
	numColorCMD[0][7] = setTextRed[0];
	numColorCMD[0][8] = setTextRed[1];
	ledFlag[0] = 0;
	/* 通道 2 */
//欠压
	multiUICMD[44] = 0xD7;
	multiUICMD[45] = 0xD4;
	multiUICMD[46] = 0xBC;
	multiUICMD[47] = 0xEC;
	statusColorCMD[1][7] = setTextRed[0];
	statusColorCMD[1][8] = setTextRed[1];
	numColorCMD[1][7] = setTextRed[0];
	numColorCMD[1][8] = setTextRed[1];
	ledFlag[1] = 0;
	/* 通道 3 */
//欠压
	multiUICMD[52] = 0xD7;
	multiUICMD[53] = 0xD4;
	multiUICMD[54] = 0xBC;
	multiUICMD[55] = 0xEC;
	statusColorCMD[2][7] = setTextRed[0];
	statusColorCMD[2][8] = setTextRed[1];
	numColorCMD[2][7] = setTextRed[0];
	numColorCMD[2][8] = setTextRed[1];
	ledFlag[2] = 0;

//能量柱动画帧选择显示命令
//EE B1 23 00 01 00 02 00 FF FC FF FF
	for (i = 0; i < 3; i++) {
		percentPicCMD[i][7] = 0;
	}
	HAL_UART_Transmit(&huart2, multiUICMD, 60, 0xFFFF);
	for (i = 0; i < 3; i++) {
		HAL_UART_Transmit(&huart2, numColorCMD[i], 13, 0xFFFF);
		HAL_UART_Transmit(&huart2, percentPicCMD[i], 12, 0xFFFF);
		HAL_UART_Transmit(&huart2, statusColorCMD[i], 13, 0xFFFF);
	}
	updateLed();
	/* 延时 1500ms*/
	HAL_Delay(1500);

//EE 61 0F FF FC FF FF
	temp[1] = 0x61;	//命令类型(UPDATE_CONTROL)
	temp[2] = 50;
	temp[3] = 0xFF;	//帧尾
	temp[4] = 0xFC;
	temp[5] = 0xFF;
	temp[6] = 0xFF;
	HAL_UART_Transmit(&huart2, temp, 7, 0xFFFF);
	/* 自检欠压 */
	/* 发送AD值和超压欠压状态 */
// EE B1 12 00 01
// 00 06 00 05 2D 31 2E 32 37 AD1
// 00 09 00 05 20 31 2E 32 37 AD2
// 00 0C 00 05 20 31 2E 32 37 AD3
// 00 08 00 04 D5 FD B3 A3	  状态1
// 00 0B 00 04 B3 AC D1 B9	  状态2
// 00 0E 00 04 C7 B7 D1 B9	  状态3
// FF FC FF FF
	FloatToStr5(1, tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 9] = tempAdcASCii[i];
	}
	FloatToStr5(1, tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 18] = tempAdcASCii[i];
	}
	FloatToStr5(1, tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 27] = tempAdcASCii[i];
	}
	/* 通道 1 */
//超压
	multiUICMD[36] = 0xD7;
	multiUICMD[37] = 0xD4;
	multiUICMD[38] = 0xBC;
	multiUICMD[39] = 0xEC;
	statusColorCMD[0][7] = setTextRed[0];
	statusColorCMD[0][8] = setTextRed[1];
	numColorCMD[0][7] = setTextRed[0];
	numColorCMD[0][8] = setTextRed[1];
	ledFlag[0] = 2;
	/* 通道 2 */
//超压
	multiUICMD[44] = 0xD7;
	multiUICMD[45] = 0xD4;
	multiUICMD[46] = 0xBC;
	multiUICMD[47] = 0xEC;
	statusColorCMD[1][7] = setTextRed[0];
	statusColorCMD[1][8] = setTextRed[1];
	numColorCMD[1][7] = setTextRed[0];
	numColorCMD[1][8] = setTextRed[1];
	ledFlag[1] = 2;

	/* 通道 3 */
//超压
	multiUICMD[52] = 0xD7;
	multiUICMD[53] = 0xD4;
	multiUICMD[54] = 0xBC;
	multiUICMD[55] = 0xEC;
	statusColorCMD[2][7] = setTextRed[0];
	statusColorCMD[2][8] = setTextRed[1];
	numColorCMD[2][7] = setTextRed[0];
	numColorCMD[2][8] = setTextRed[1];
	ledFlag[2] = 2;

//能量柱动画帧选择显示命令
//EE B1 23 00 01 00 02 00 FF FC FF FF
	for (i = 0; i < 3; i++) {
		percentPicCMD[i][7] = 9;
	}
	HAL_UART_Transmit(&huart2, multiUICMD, 60, 0xFFFF);
	for (i = 0; i < 3; i++) {
		HAL_UART_Transmit(&huart2, numColorCMD[i], 13, 0xFFFF);
		HAL_UART_Transmit(&huart2, percentPicCMD[i], 12, 0xFFFF);
		HAL_UART_Transmit(&huart2, statusColorCMD[i], 13, 0xFFFF);
	}
	updateLed();
	/* 延时 1500ms*/
	HAL_Delay(1500);

	/* 蜂鸣器报警 */
//EE 61 0F FF FC FF FF
	temp[1] = 0x61;	//命令类型(UPDATE_CONTROL)
	temp[2] = 50;
	temp[3] = 0xFF;	//帧尾
	temp[4] = 0xFC;
	temp[5] = 0xFF;
	temp[6] = 0xFF;
	HAL_UART_Transmit(&huart2, temp, 7, 0xFFFF);
	/* 自检欠压 */
	/* 发送AD值和超压欠压状态 */
// EE B1 12 00 01
// 00 06 00 05 2D 31 2E 32 37 AD1
// 00 09 00 05 20 31 2E 32 37 AD2
// 00 0C 00 05 20 31 2E 32 37 AD3
// 00 08 00 04 D5 FD B3 A3	  状态1
// 00 0B 00 04 B3 AC D1 B9	  状态2
// 00 0E 00 04 C7 B7 D1 B9	  状态3
// FF FC FF FF
	FloatToStr5(0.5, tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 9] = tempAdcASCii[i];
	}
	FloatToStr5(0.5, tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 18] = tempAdcASCii[i];
	}
	FloatToStr5(0.5, tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 27] = tempAdcASCii[i];
	}
	/* 通道 1 */
//正常
	multiUICMD[36] = 0xD7;
	multiUICMD[37] = 0xD4;
	multiUICMD[38] = 0xBC;
	multiUICMD[39] = 0xEC;
	statusColorCMD[0][7] = setTextGreen[0];
	statusColorCMD[0][8] = setTextGreen[1];
	numColorCMD[0][7] = setTextGreen[0];
	numColorCMD[0][8] = setTextGreen[1];
	ledFlag[0] = 1;
	/* 通道 2 */
//正常
	multiUICMD[44] = 0xD7;
	multiUICMD[45] = 0xD4;
	multiUICMD[46] = 0xBC;
	multiUICMD[47] = 0xEC;
	statusColorCMD[1][7] = setTextGreen[0];
	statusColorCMD[1][8] = setTextGreen[1];
	numColorCMD[1][7] = setTextGreen[0];
	numColorCMD[1][8] = setTextGreen[1];
	ledFlag[1] = 1;

	/* 通道 3 */
//正常
	multiUICMD[52] = 0xD7;
	multiUICMD[53] = 0xD4;
	multiUICMD[54] = 0xBC;
	multiUICMD[55] = 0xEC;
	statusColorCMD[2][7] = setTextGreen[0];
	statusColorCMD[2][8] = setTextGreen[1];
	numColorCMD[2][7] = setTextGreen[0];
	numColorCMD[2][8] = setTextGreen[1];
	ledFlag[2] = 1;

//能量柱动画帧选择显示命令
//EE B1 23 00 01 00 02 00 FF FC FF FF
	for (i = 0; i < 3; i++) {
		percentPicCMD[i][7] = 5;
	}
	HAL_UART_Transmit(&huart2, multiUICMD, 60, 0xFFFF);
	for (i = 0; i < 3; i++) {
		HAL_UART_Transmit(&huart2, numColorCMD[i], 13, 0xFFFF);
		HAL_UART_Transmit(&huart2, percentPicCMD[i], 12, 0xFFFF);
		HAL_UART_Transmit(&huart2, statusColorCMD[i], 13, 0xFFFF);
	}
	updateLed();
	timeStamp = currentTime;
	/* 延时 3000ms*/
	HAL_Delay(5000);

	timeStamp = 0;

	/* 加载主界面设置 */
	loadMainPage();
	alarm_off();
	testFlag = 0;
}

/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void updateADC(void) {
	int i = 0, lcd_number_j = 0;
	float aaa;

	for (lcd_number_j = 0; lcd_number_j < 3; lcd_number_j++) {
		adcTemp[lcd_number_j][adc_index[lcd_number_j]] =
				ADC_ConvertedValue[lcd_number_j];
		if (adc_index[lcd_number_j] >= 16)
			adc_index[lcd_number_j] = 0;
		adc_index[lcd_number_j]++;
		if (adc_count[lcd_number_j] < 16)
			adc_count[lcd_number_j]++;

		g_adc_Temp_filter[lcd_number_j] = 0;
		for (i = 1; i < adc_count[lcd_number_j] + 1; i++) {
			g_adc_Temp_filter[lcd_number_j] += adcTemp[lcd_number_j][i];
		}

		if (lcd_number_j == 0) {
			gg_adc_Temp_filter[0] = g_adc_Temp_filter[0] >> 4;
		} else if (lcd_number_j == 1) {
			gg_adc_Temp_filter[1] = g_adc_Temp_filter[1] >> 4;
		} else if (lcd_number_j == 2) {
			gg_adc_Temp_filter[2] = g_adc_Temp_filter[2] >> 4;
		}
		// 编写AD转换后输出的压力数字

		adc_Temp_filter[lcd_number_j] = (gg_adc_Temp_filter[lcd_number_j] + 35
				+ saveData[lcd_number_j].zero)
				* (0.990 + saveData[lcd_number_j].K / 1000.0); //*1.0085+30;	//3屏幕	  01 芯片电压3.31
		if ((adc_Temp_filter[lcd_number_j] < 0))

			adc_Temp_filter[lcd_number_j] = 0;
		else if (adc_Temp_filter[lcd_number_j] >= 4096)
			adc_Temp_filter[lcd_number_j] = 4095;
		if (adc_Temp_filter[lcd_number_j] < 558)	 //采集值小于了0.45V
				{
			float_ADCValue[lcd_number_j] = 0;
		} else {
			if (val_20mA[lcd_number_j] < val_4mA[lcd_number_j]) {
				aaa = val_4mA[lcd_number_j];
				val_4mA[lcd_number_j] = val_20mA[lcd_number_j];
				val_20mA[lcd_number_j] = aaa;
			}
			// y=kx+b
			k[lcd_number_j] = (val_20mA[lcd_number_j] - val_4mA[lcd_number_j])
					/ (AD_VAL_WHEN_20MA - AD_VAL_WHEN_4MA);
			b[lcd_number_j] = val_20mA[lcd_number_j]
					- AD_VAL_WHEN_20MA * k[lcd_number_j];
			float_ADCValue[lcd_number_j] = (adc_Temp_filter[lcd_number_j])
					* k[lcd_number_j] + b[lcd_number_j];
		}
	}  //for
}

/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void updateUI(void) {
	uint8_t tempAdcASCii[5];
	uint8_t i;
	/* 发送AD值和超压欠压状态 */
// EE B1 12 00 01
// 00 06 00 05 2D 31 2E 32 37 AD1
// 00 09 00 05 20 31 2E 32 37 AD2
// 00 0C 00 05 20 31 2E 32 37 AD3
// 00 08 00 04 D5 FD B3 A3	  状态1
// 00 0B 00 04 B3 AC D1 B9	  状态2
// 00 0E 00 04 C7 B7 D1 B9	  状态3
// FF FC FF FF
	for (i = 0; i < 3; i++) {
		if (saveData[i].nameIndex == 0) {
			float_ADCValue[i] = 0;
		}
	}

	FloatToStr5(float_ADCValue[0], tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 9] = tempAdcASCii[i];
	}
	FloatToStr5(float_ADCValue[1], tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 18] = tempAdcASCii[i];
	}
	FloatToStr5(float_ADCValue[2], tempAdcASCii, 5);
	for (i = 0; i < 5; i++) {
		multiUICMD[i + 27] = tempAdcASCii[i];
	}
	/* 通道 1 */
//正常
	if ((float_ADCValue[0] > saveData[0].lower_limit)
			&& (float_ADCValue[0] < saveData[0].upper_limit)
			&& saveData[0].nameIndex != 0) {
		multiUICMD[36] = 0xD5;
		multiUICMD[37] = 0xFD;
		multiUICMD[38] = 0xB3;
		multiUICMD[39] = 0xA3;
		statusColorCMD[0][7] = setTextGreen[0];
		statusColorCMD[0][8] = setTextGreen[1];
		numColorCMD[0][7] = setTextGreen[0];
		numColorCMD[0][8] = setTextGreen[1];
		ledFlag[0] = 1;
	}
//欠压
	if (float_ADCValue[0] <= saveData[0].lower_limit
			&& saveData[0].nameIndex != 0) {
		multiUICMD[36] = 0xC7;
		multiUICMD[37] = 0xB7;
		multiUICMD[38] = 0xD1;
		multiUICMD[39] = 0xB9;
		statusColorCMD[0][7] = setTextRed[0];
		statusColorCMD[0][8] = setTextRed[1];
		numColorCMD[0][7] = setTextRed[0];
		numColorCMD[0][8] = setTextRed[1];
		ledFlag[0] = 0;
	}
//超压
	if (float_ADCValue[0] >= saveData[0].upper_limit
			&& saveData[0].nameIndex != 0) {
		multiUICMD[36] = 0xB3;
		multiUICMD[37] = 0xAC;
		multiUICMD[38] = 0xD1;
		multiUICMD[39] = 0xB9;
		statusColorCMD[0][7] = setTextRed[0];
		statusColorCMD[0][8] = setTextRed[1];
		numColorCMD[0][7] = setTextRed[0];
		numColorCMD[0][8] = setTextRed[1];
		ledFlag[0] = 2;
	}
//未使用
	if (saveData[0].nameIndex == 0) {
		multiUICMD[36] = 0x20;
		multiUICMD[37] = 0x20;
		multiUICMD[38] = 0x20;
		multiUICMD[39] = 0x20;
		statusColorCMD[0][7] = setTextGreen[0];
		statusColorCMD[0][8] = setTextGreen[1];
		numColorCMD[0][7] = setTextGreen[0];
		numColorCMD[0][8] = setTextGreen[1];
		ledFlag[0] = 3;
	}
	/* 通道 2 */
//正常
	if ((float_ADCValue[1] > saveData[1].lower_limit)
			&& (float_ADCValue[1] < saveData[1].upper_limit)
			&& saveData[1].nameIndex != 0) {
		multiUICMD[44] = 0xD5;
		multiUICMD[45] = 0xFD;
		multiUICMD[46] = 0xB3;
		multiUICMD[47] = 0xA3;
		statusColorCMD[1][7] = setTextGreen[0];
		statusColorCMD[1][8] = setTextGreen[1];
		numColorCMD[1][7] = setTextGreen[0];
		numColorCMD[1][8] = setTextGreen[1];
		ledFlag[1] = 1;
	}
//欠压
	if (float_ADCValue[1] <= saveData[1].lower_limit
			&& saveData[1].nameIndex != 0) {
		multiUICMD[44] = 0xC7;
		multiUICMD[45] = 0xB7;
		multiUICMD[46] = 0xD1;
		multiUICMD[47] = 0xB9;
		statusColorCMD[1][7] = setTextRed[0];
		statusColorCMD[1][8] = setTextRed[1];
		numColorCMD[1][7] = setTextRed[0];
		numColorCMD[1][8] = setTextRed[1];
		ledFlag[1] = 0;
	}
//超压
	if (float_ADCValue[1] >= saveData[1].upper_limit
			&& saveData[1].nameIndex != 0) {
		multiUICMD[44] = 0xB3;
		multiUICMD[45] = 0xAC;
		multiUICMD[46] = 0xD1;
		multiUICMD[47] = 0xB9;
		statusColorCMD[1][7] = setTextRed[0];
		statusColorCMD[1][8] = setTextRed[1];
		numColorCMD[1][7] = setTextRed[0];
		numColorCMD[1][8] = setTextRed[1];
		ledFlag[1] = 2;
	}
	//未使用
	if (saveData[1].nameIndex == 0) {
		multiUICMD[44] = 0x20;
		multiUICMD[45] = 0x20;
		multiUICMD[46] = 0x20;
		multiUICMD[47] = 0x20;
		statusColorCMD[1][7] = setTextGreen[0];
		statusColorCMD[1][8] = setTextGreen[1];
		numColorCMD[1][7] = setTextGreen[0];
		numColorCMD[1][8] = setTextGreen[1];
		ledFlag[1] = 3;
	}
	/* 通道 3 */
//正常
	if ((float_ADCValue[2] > saveData[2].lower_limit)
			&& (float_ADCValue[2] < saveData[2].upper_limit)
			&& saveData[2].nameIndex != 0) {
		multiUICMD[52] = 0xD5;
		multiUICMD[53] = 0xFD;
		multiUICMD[54] = 0xB3;
		multiUICMD[55] = 0xA3;
		statusColorCMD[2][7] = setTextGreen[0];
		statusColorCMD[2][8] = setTextGreen[1];
		numColorCMD[2][7] = setTextGreen[0];
		numColorCMD[2][8] = setTextGreen[1];
		ledFlag[2] = 1;
	}
//欠压
	if (float_ADCValue[2] <= saveData[2].lower_limit
			&& saveData[2].nameIndex != 0) {
		multiUICMD[52] = 0xC7;
		multiUICMD[53] = 0xB7;
		multiUICMD[54] = 0xD1;
		multiUICMD[55] = 0xB9;
		statusColorCMD[2][7] = setTextRed[0];
		statusColorCMD[2][8] = setTextRed[1];
		numColorCMD[2][7] = setTextRed[0];
		numColorCMD[2][8] = setTextRed[1];
		ledFlag[2] = 0;
	}
//超压
	if (float_ADCValue[2] >= saveData[2].upper_limit
			&& saveData[2].nameIndex != 0) {
		multiUICMD[52] = 0xB3;
		multiUICMD[53] = 0xAC;
		multiUICMD[54] = 0xD1;
		multiUICMD[55] = 0xB9;
		statusColorCMD[2][7] = setTextRed[0];
		statusColorCMD[2][8] = setTextRed[1];
		numColorCMD[2][7] = setTextRed[0];
		numColorCMD[2][8] = setTextRed[1];
		ledFlag[2] = 2;
	}
	//未使用
	if (saveData[2].nameIndex == 0) {
		multiUICMD[52] = 0x20;
		multiUICMD[53] = 0x20;
		multiUICMD[54] = 0x20;
		multiUICMD[55] = 0x20;
		statusColorCMD[2][7] = setTextGreen[0];
		statusColorCMD[2][8] = setTextGreen[1];
		numColorCMD[2][7] = setTextGreen[0];
		numColorCMD[2][8] = setTextGreen[1];
		ledFlag[2] = 3;
	}
//能量柱动画帧选择显示命令
//EE B1 23 00 01 00 02 00 FF FC FF FF
	for (i = 0; i < 3; i++) {
		percentPicCMD[i][7] = (uint8_t) (float_ADCValue[i] * 10
				/ (val_20mA[i] - val_4mA[i]));
		if (percentPicCMD[i][7] > 8)
			percentPicCMD[i][7] = 8;
		if (percentPicCMD[i][7] < 1)
			percentPicCMD[i][7] = 1;
		switch (ledFlag[i]) {
		case 0:
			percentPicCMD[i][7] = 0;
			break;
		case 2:
			percentPicCMD[i][7] = 9;
			break;
		default:
			break;
		}
	}
	HAL_UART_Transmit(&huart2, multiUICMD, 60, 0xFFFF);
	for (i = 0; i < 3; i++) {
		HAL_UART_Transmit(&huart2, numColorCMD[i], 13, 0xFFFF);
		HAL_UART_Transmit(&huart2, percentPicCMD[i], 12, 0xFFFF);
		HAL_UART_Transmit(&huart2, statusColorCMD[i], 13, 0xFFFF);
	}
	if (ledFlag[0] == 1 && ledFlag[1] == 1 && ledFlag[2] == 1) {
		alarmFlag = 0;
	} else {
		//赋值 4 便于计算音量图标帧数
		alarmFlag = 4;
	}
//修改音量图标
	if (muteFlag == 1 || saveData[0].volume == 0) {
		volumePicCMD[7] = 0 + alarmFlag;
		if (bebe) {
			alarm_off();
			alarm_off();
			alarm_off();
			alarm_off();
			alarm_off();
		}
	} else {
		volumePicCMD[7] = (uint8_t) (saveData[0].volume / 10);
		if (volumePicCMD[7] <= 4) {
			volumePicCMD[7] = 1 + alarmFlag;
		} else if (volumePicCMD[7] > 4 && volumePicCMD[7] <= 7) {
			volumePicCMD[7] = 2 + alarmFlag;
		} else if (volumePicCMD[7] > 7) {
			volumePicCMD[7] = 3 + alarmFlag;
		}
		if (!bebe) {
			alarm_on();
			alarm_on();
			alarm_on();
			alarm_on();
			alarm_on();
		}
	}
//修改音量图标
	HAL_UART_Transmit(&huart2, volumePicCMD, 12, 0xFFFF);

	set485rom(1);
}

/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void updateLed(void) {
	uint8_t i;
	for (i = 0; i < 3; i++) {
		switch (ledFlag[i]) {
		case 0:
			if (i == 0) {
				LED0A(0);
				LED1A(0);
				LED2A(1);
			}
			if (i == 1) {
				LED0B(0);
				LED1B(0);
				LED2B(1);
			}
			if (i == 2) {
				LED0C(0);
				LED1C(0);
				LED2C(1);
			}
			break;
		case 1:
			if (i == 0) {
				LED0A(0);
				LED1A(1);
				LED2A(0);
			}
			if (i == 1) {
				LED0B(0);
				LED1B(1);
				LED2B(0);
			}
			if (i == 2) {
				LED0C(0);
				LED1C(1);
				LED2C(0);
			}
			break;
		case 2:
			if (i == 0) {
				LED0A(1);
				LED2A(0);
				LED1A(0);
			}
			if (i == 1) {
				LED0B(1);
				LED2B(0);
				LED1B(0);
			}
			if (i == 2) {
				LED0C(1);
				LED2C(0);
				LED1C(0);
			}
			break;
		case 3:
			if (i == 0) {
				LED0A(0);
				LED1A(0);
				LED2A(0);
			}
			if (i == 1) {
				LED0B(0);
				LED1B(0);
				LED2B(10);
			}
			if (i == 2) {
				LED0C(0);
				LED1C(0);
				LED2C(0);
			}
			break;
		default:
			break;
		}

	}
}

/**
 * 函数功能: ADC转换完成回调函数
 * 输入参数: ADCHandle：ADC外设设备句柄
 * 返 回 值: 无
 * 说    明: 无
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* ADCHandle) {

}

/**
 * 函数功能: 串口中断回调函数
 * 输入参数: UartHandle：串口外设设备句柄
 * 返 回 值: 无
 * 说    明: 无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uartHandle) {

}
/**
 * 函数功能: 按键外部中断服务函数
 * 输入参数: GPIO_PIN：中断引脚
 * 返 回 值: 无
 * 说    明: 无
 */
void UART_RxIDLECallback(UART_HandleTypeDef* uartHandle) {
	uint32_t temp;

	if (__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET) {
		unsigned char buffer[8];
		uint8_t send_mydata[150];
		unsigned short crc;
		unsigned char i, sendcount;
		unsigned int record_num, record_add;
		//调试用
		RS485_Send_Data(RS485_RX_BUF, 8);

		crc = CRC16(RS485_RX_BUF, 6);
		//  buffer[6]=crc&0xff;
		//  buffer[7]=(crc&0xff00)>>8;
		//  modscan 是校验码高位在前，低位在后
		buffer[7] = crc & 0xff;
		buffer[6] = (crc & 0xff00) >> 8;

		// 判断地址与crc校验
		if ((RS485_RX_BUF[0] == saveData[0].modbusAddr)
				&& (RS485_RX_BUF[1] == 0x03)     //读取命令
				&& (RS485_RX_BUF[6] == buffer[6])
				&& (RS485_RX_BUF[7] == buffer[7])) { // 成功后组合数据 计算 CRC 并发送。�
													 // 获取数据
													 //清空
			memset(send_mydata, 0, sizeof(send_mydata));
			// 地址
			send_mydata[0] = saveData[0].modbusAddr;         //地址
			send_mydata[1] = 0x03;         // 功能码
			send_mydata[2] = RS485_RX_BUF[5] << 1;         // 寄存器个数乘以二

			record_add = RS485_RX_BUF[2] << 8 | RS485_RX_BUF[3];       //组合为复合地址
			if (record_add > 256)
				record_add = 256;
			record_num = RS485_RX_BUF[5] * 2;         //组合为数据长度＠├┱阔啊站 看扩展为2倍 数量�
			if (record_num > 128)
				record_num = 128;

			// rom485[61]//=rom485[81];
			//  rom485[63]//=rom485[83];
			//修改真空报警状态， 上下报警互换

			memcpy((&send_mydata[3]), &rom485[record_add * 2], record_num); //加一大段数据
			i = record_num + 3;

			//添加校验码   modscan 高位在前

			crc = CRC16(send_mydata, i);
			send_mydata[i] = (crc & 0xff00) >> 8;
			send_mydata[i + 1] = crc & 0xff;

			sendcount = i + 2;
			RS485_TX_ON()
			;         //发送使能
			for (i = 0; i < sendcount; i++)
				RS485_Send_Data(send_mydata, sizeof(send_mydata));

			RS485_TX_OFF()
			;
		}
		memset(RS485_RX_BUF, 0xFF, sizeof(RS485_RX_BUF));

		HAL_UART_Receive_DMA(&huart1, (uint8_t *) RS485_RX_BUF, 8); //重新使能DMA接收
		/* 开启串口1空闲中断 */
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	}
	if (__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET) {
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);
		temp = huart2.Instance->SR;
		temp = huart2.Instance->DR;
		HAL_UART_DMAStop(&huart2);
		temp = hdma_usart2_rx.Instance->CNDTR;
		RS232_recvLength = CMD_MAX_SIZE - temp;
		RS232_recvEndFlag = 1;
	}
}

/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* tim_baseHandle) {
	uint8_t temp[12];
	currentTime++;
	if (currentTime > 999) {
		currentTime = 0;
	}
	if (currentTime == 600) {
		minTick++;
		if (minTick > 525600) {
			minTick = 0;
		}
	}
	/* 每小时检查一次授权时间 */
	if (minTick % 60 == 0) {
		get_date();
		if (saveData[0].omeDays
				> calcDays(saveData[0].omeTime[5] * 10 + saveData[0].omeTime[4],
						saveData[0].omeTime[3] * 10 + saveData[0].omeTime[2],
						saveData[0].omeTime[1] * 10 + saveData[0].omeTime[0],
						date_1302[5] * 10 + date_1302[4],
						date_1302[3] * 10 + date_1302[2],
						date_1302[1] * 10 + date_1302[0])
				&& saveData[0].rootDays
						> calcDays(
								saveData[0].rootTime[5] * 10
										+ saveData[0].rootTime[4],
								saveData[0].rootTime[3] * 10
										+ saveData[0].rootTime[2],
								saveData[0].rootTime[1] * 10
										+ saveData[0].rootTime[0],
								date_1302[5] * 10 + date_1302[4],
								date_1302[3] * 10 + date_1302[2],
								date_1302[1] * 10 + date_1302[0])) {
			HAL_UART_Transmit(&huart2, licPassedCMD, 11, 0xFFFF);
		} else {
			HAL_UART_Transmit(&huart2, licFailedCMD, 19, 0xFFFF);
		}
	}
	/* 定时更新ADC */
	if ((currentTime % 3) == 0) {
		updateADC();
	}
	/* 定时获取画面ID */
	//EE B1 01 FF FC FF FF
//	if (currentTime == 500) {
//		temp[0] = 0xEE;  			//帧头
//		temp[1] = 0xB1;				//命令类型(UPDATE_CONTROL)
//		temp[2] = 0x01;
//		temp[3] = 0xFF;   			//帧尾
//		temp[4] = 0xFC;
//		temp[5] = 0xFF;
//		temp[6] = 0xFF;
//		HAL_UART_Transmit(&huart2, temp, 7, 0xFFFF);
//	}
	if (currentPage == PAGE_START && (currentTime - timeStamp) >= 150) {
		//跳转至主屏幕
		//EE B1 00 00 01 FF FC FF FF
		temp[0] = 0xEE;  			//帧头
		temp[1] = NOTIFY_CONTROL;	//命令类型(UPDATE_CONTROL)
		temp[2] = 0x00; 			//CtrlMsgType-指示消息的类型
		temp[3] = 0x00;  			//产生消息的画面ID
		temp[4] = 0x01;
		temp[5] = 0xFF;   			//帧尾
		temp[6] = 0xFC;
		temp[7] = 0xFF;
		temp[8] = 0xFF;
		HAL_UART_Transmit(&huart2, temp, 9, 0xFFFF);
		lastPage = currentPage;
		currentPage = PAGE_MAIN;
		timeStamp = SELFTESTTIME;

		alarm_off();
		alarm_off();
		alarm_off();
		alarm_off();
		alarm_off();
	}
}

/**
 * @功能简介 : 外部按钮中断
 * @入口参数 : 中断
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN) {
	if (GPIO_PIN == BUTTON_TEST_GPIO_PIN) {
		HAL_Delay(200);/* 延时一小段时间，消除抖动 */
		if (HAL_GPIO_ReadPin(BUTTON_TEST_GPIO,
		BUTTON_TEST_GPIO_PIN) == BUTTON_TEST_DOWN_LEVEL) {
			testFlag = 1;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_TEST_GPIO_PIN);
	} else if (GPIO_PIN == BUTTON_CLEAR_GPIO_PIN) {
		HAL_Delay(20);/* 延时一小段时间，消除抖动 */
		if (HAL_GPIO_ReadPin(BUTTON_CLEAR_GPIO,
		BUTTON_CLEAR_GPIO_PIN) == BUTTON_CLEAR_DOWN_LEVEL) {
			//修改音量图标
			if (muteFlag == 0) {
				volumePicCMD[7] = 0 + alarmFlag;
				muteFlag = 1;
			} else {
				volumePicCMD[7] = (uint8_t) (saveData[0].volume / 10);
				if (volumePicCMD[7] <= 4) {
					volumePicCMD[7] = 1 + alarmFlag;
				} else if (volumePicCMD[7] > 4 && volumePicCMD[7] <= 7) {
					volumePicCMD[7] = 2 + alarmFlag;
				} else if (volumePicCMD[7] > 7) {
					volumePicCMD[7] = 3 + alarmFlag;
				}
				muteFlag = 0;
			}
			//修改音量图标
			HAL_UART_Transmit(&huart2, volumePicCMD, 12, 0xFFFF);
		}
		__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_CLEAR_GPIO_PIN);
	}
}

/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
uint32_t calcDays(uint8_t y1, uint8_t m1, uint8_t d1, uint8_t y2, uint8_t m2,
		uint8_t d2) {
	unsigned char x[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	int i, s1 = 0, s2 = 0;
	for (i = 1; i < y1; i++)
		if ((i % 4 == 0 && i % 100 != 0) || i % 400 == 0)
			s1 += 366;	//闰年
		else
			s1 += 365;	//平年

	if ((y1 % 4 == 0 && y1 % 100 != 0) || y1 % 400 == 0)
		x[2] = 29;

	for (i = 1; i < m1; i++)
		s1 += x[i];	//整月的天数
	s1 += d1;	//日的天数

	for (i = 1; i < y2; i++)
		if ((i % 4 == 0 && i % 100 != 0) || i % 400 == 0)
			s2 += 366;	//闰年
		else
			s2 += 365;	//平年

	if ((y2 % 4 == 0 && y2 % 100 != 0) || y2 % 400 == 0)
		x[2] = 29;

	for (i = 1; i < m2; i++)
		s2 += x[i];	//整月的天数
	s2 += d2;	//日的天数
	if (s2 > s1) {
		return s2 - s1;	//返回总天数,相对公元1年
	} else
		return 0;
}

/**
 * @功能简介 : 将浮点数的各个位的数值转换成字符串
 * @入口参数 : data - 浮点数 | *buf - 转换结果保存位置 | 长度
 * @出口参数 : 无
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
void FloatToStr5(float data, uint8_t *buf, int size) {
	int integer = 0, i = 0;
	memset(buf, 0, size);

	if (data < 0) {
		data *= -1.0;
		buf[i] = '-';
	} else
		buf[i] = ' ';
	i++;
	if (data > 999) {
		data = 999;
	}
	if (data >= 100) {
		integer = data / 100;
		buf[i] = integer + '0';
		data = data - integer * 100;
		i++;
	}
	if (data >= 10 || i == 2) {
		if (data >= 10) {
			integer = data / 10;
		} else
			integer = 0;
		buf[i] = integer + '0';
		data = data - integer * 10;
		i++;
	}
	integer = (int) data;
	buf[i] = integer + '0';
	data = data - integer;
	i++;
	if (i < 4) {
		buf[i] = '.';
		i++;
	} else {
		i++;
	}
	if (i < 5) {
		integer = data * 10;
		buf[i] = integer + '0';
		data = data - integer * 0.1;
		i++;
	}
	if (i < 5) {
		integer = data * 100;
		buf[i] = integer + '0';
		data = data - integer * 0.01;
		i++;
	}
}

/**
 * @功能简介 : 将字符串转换成float
 * @入口参数 : *buf - 转换位置
 * @出口参数 : float
 * @历史版本 : V0.0.1 - Ethan - 2018/01/03
 */
float StrToFloat(uint8_t *buf) {
	uint8_t i, j, k, negative = 0;
#define s_temp buf
	float result = 0, result_1 = 0;
	if (buf[0] == ' ') {
		buf++;
	}
	for (i = 0; i < 10; i++) {
		j = buf[i];
		if (j == 0 || ((j < '0' || j > '9') && (j != '.') && (j != '-')))
			break;
	}
	k = j = i;	//数值的个数
	for (i = 0; i < j; i++)	//查找小数点的位置，结束后小数点位于第i+1位
			{
		if (s_temp[i] == '.')
			break;
	}

	for (j = 0; j < i; j++) {
		if (s_temp[j] == '-') {
			negative = 1;
			continue;
		}
		result = result * 10 + (s_temp[j] - '0');
	}
	j++;	//加1后j=i+1，即小数点的位置
	i = j;	//第一个小数的位置
	for (; j < k; j++) {
		if (s_temp[j] < '0' || s_temp[j] > '9')
			break;	//非法字符，返回
		result_1 = result_1 * 10 + (s_temp[j] - '0');
	}
	for (j = 0; j < (k - i); j++)
		result_1 *= 0.1;
	result += result_1;

	if (negative)
		result = -result;
	return result;
}

void set485rom(uint8_t func) {
	uint8_t j;
	if (func == 0) {

		unsigned int bautrate = 0;
		rom485[0] = 0;
		rom485[1] = saveData[0].modbusAddr; //地址
		switch (saveData[0].baudrateIndex) {
		case 0:
			bautrate = 2400;
			break;
		case 1:
			bautrate = 4800;
			break;
		case 2:
			bautrate = 9600;
			break;
		case 3:
			bautrate = 19200;
			break;
		case 4:
			bautrate = 38400;
			break;
		}
		rom485[2] = (bautrate & 0xff00) >> 8; //波特率
		rom485[3] = bautrate & 0xff;

		rom485[20] = 0;  //出厂字节1
		rom485[21] = 0;

		rom485[22] = 0;  //气体索引1
		rom485[23] = saveData[0].nameIndex;

		rom485[40] = 0;  //出厂字节2
		rom485[41] = 0;

		rom485[42] = 0;  //气体索引2
		rom485[43] = saveData[1].nameIndex;

		rom485[60] = 0;  //出厂字节3
		rom485[61] = 0;

		rom485[62] = 0;  //气体索引3
		rom485[63] = saveData[2].nameIndex;

		j = 24;
		memcpy((&rom485[j]), &saveData[0].upper_limit, 8);          //上下限1

		change_float_big_485rom(j);
		change_float_big_485rom(j + 4);

		j = 44;
		memcpy((&rom485[j]), &saveData[1].upper_limit, 8);          //上下限2

		change_float_big_485rom(j);
		change_float_big_485rom(j + 4);

		j = 64;
		memcpy((&rom485[j]), &saveData[1].upper_limit, 8);          //上下限3

		change_float_big_485rom(j);
		change_float_big_485rom(j + 4);
	}

	rom485[4] = 0;	//报警1
	rom485[5] = (ledFlag[0] == 1 ? 0 : 1);
	rom485[6] = 0;	//报警2
	rom485[7] = (ledFlag[1] == 1 ? 0 : 1);
	rom485[8] = 0;	//报警3
	rom485[9] = (ledFlag[2] == 1 ? 0 : 1);

	j = 37;          // 使用8位寄存器作为状态存储量
	switch (ledFlag[0]) {
	case 0:
		rom485[j] = 0x01;
		break;
	case 1:
		rom485[j] = 0x00;
		break;
	case 2:
		rom485[j] = 0x02;
		break;
	}

	j = 57;
	switch (ledFlag[1]) {
	case 0:
		rom485[j] = 0x01;
		break;
	case 1:
		rom485[j] = 0x00;
		break;
	case 2:
		rom485[j] = 0x02;
		break;
	}

	j = 77;
	switch (ledFlag[2]) {
	case 0:
		rom485[j] = 0x01;
		break;
	case 1:
		rom485[j] = 0x00;
		break;
	case 2:
		rom485[j] = 0x02;
		break;
	}

	memcpy((&rom485[32]), &float_ADCValue[0], 4);
	change_float_big_485rom(32);

	memcpy((&rom485[52]), &float_ADCValue[1], 4);
	change_float_big_485rom(52);

	memcpy((&rom485[72]), &float_ADCValue[2], 4);
	change_float_big_485rom(72);
}

void change_float_big_485rom(unsigned int j)  //修改浮点数在 rom 中的存储大小端
{
	char temp_c = 0;
	temp_c = rom485[j + 3];
	rom485[j + 3] = rom485[j + 0];
	rom485[j + 0] = temp_c;

	temp_c = rom485[j + 2];
	rom485[j + 2] = rom485[j + 1];
	rom485[j + 1] = temp_c;
}

//单线声音通讯函数通
void Line_1A_WTN5(unsigned char SB_DATA) {
	unsigned char S_DATA, B_DATA;
	unsigned char j;

	HAL_NVIC_DisableIRQ(USART1_IRQn);
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel4_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel5_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
	HAL_NVIC_DisableIRQ(DMA1_Channel7_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_DisableIRQ(TIM2_IRQn);

	S_DATA = SB_DATA;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);	// 数据位拉低
	HAL_Delay(5);
	B_DATA = S_DATA & 0X01;

	for (j = 0; j < 8; j++) {
		if (B_DATA == 1) {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
			bsp_Delay_Nus(600);

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			bsp_Delay_Nus(200);
		} else {
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
			bsp_Delay_Nus(200);

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
			bsp_Delay_Nus(600);
		}
		S_DATA = S_DATA >> 1;
		B_DATA = S_DATA & 0X01;
	}
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

	HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void alarm_on(void) //播放声音
{
	uint8_t i;
	for (i = 0; i < 5; i++) {
		/* 设置音量 */
		HAL_Delay(50);
		Line_1A_WTN5(0xE0 + ((uint8_t) (saveData[0].volume * 1.5) & 0x0F)); //音量
		HAL_Delay(50);
		Line_1A_WTN5(0xF2); //连续播放
		HAL_Delay(50);
		Line_1A_WTN5(0x00); //播放第零语音
		HAL_Delay(50);
	}
	bebe = 1;
}

void alarm_off(void) //播放声音
{
	uint8_t i;
	for (i = 0; i < 5; i++) {
		Line_1A_WTN5(0xFE); //停止
		HAL_Delay(10);
	}
	bebe = 0;

}

void bsp_Delay_Nus(uint16_t time) {
	uint16_t i = 0;
	while (time--) {
		i = 10;  //自己定义
		while (i--)
			;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/