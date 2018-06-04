#include "bsp_ds1302.h"

/*定义待设置的时间：     秒、 分、  时、 日、 月、 星期、年、控制字*/
volatile char time_tab[8] = { 0x00, 0x00, 0x00, 0x21, 0x01, 0x07, 0x18, 0x20 };
char table[7];
char date_1302[6];
char time_1302[6];
unsigned int year;
unsigned char month;
unsigned char date;
unsigned char hour;
unsigned char min;
unsigned char sec;
/********************************************************************
 函 数 名：ds1302_init()DS1302初始化函数
 功    能：DS1302初始化
 入口参数：无
 返 回 值：无
 ***********************************************************************/
void ds1302_init(void) {
  GPIO_InitTypeDef GPIO_InitStruct; /*定义一个GPIO_InitTypeDef类型的结构体*/
  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  SCLK_0;
  RST_0;
  RST_1;
  ds1302_write(0x8e);
  ds1302_write(0);
  RST_0;
}
/********************************************************************
 函 数 名：ds1302_write()写一个字节数据函数
 功    能：DS1302中写入一个字节
 入口参数：time_tx 要写入的一个字节
 返 回 值：无
 ***********************************************************************/
void ds1302_write(unsigned char time_tx) {
  int j;
  IO_OUT();
  for (j = 0; j < 8; j++) {
    IO_0;
    SCLK_0;
    if ((time_tx & 0x01) != 0)
      IO_1;
    time_tx = time_tx >> 1;
    SCLK_1;
  }
  SCLK_0;
}
/********************************************************************
 函 数 名：ds1302_read()读一个字节函数
 功    能：从DS1302读一个字节
 入口参数：无
 返 回 值：unsigned char :读取的数据
 ***********************************************************************/
unsigned char ds1302_read(void) {
  int j;
  unsigned char time_rx = 0;
  IO_INPUT();
  for (j = 0; j < 8; j++) {
    SCLK_0;
    time_rx = time_rx >> 1;
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1)
      time_rx |= 0x80; //把接收到的数据放到接收寄存器的最高位
    SCLK_1;
  }
  SCLK_0;
  return (time_rx);
}
/********************************************************************
 函 数 名：set_time()设置时间函数
 功    能：设置初始时间
 说    明：使用了多字节写方式
 入口参数：无
 返 回 值：无
 ***********************************************************************/
void set_time(void) {
  int i;
  RST_1;
  ds1302_write(0xbe);
  for (i = 0; i < 8; i++) {
    ds1302_write(time_tab[i]);
  }
  RST_0;
}
/********************************************************************
 函 数 名：get_all()读取全部时间信息函数
 功    能：读取DS1302内部的全部时间信息
 入口参数：无
 返 回 值：无。
 ***********************************************************************/
void get_all(void) {
  int i;
  RST_1;
  ds1302_write(0xbf);
  for (i = 0; i < 7; i++) {
    table[i] = ds1302_read();
  }
  RST_0;
}
/********************************************************************
 函 数 名：get_time()读取时间函数
 功    能：读取DS1302当前时间
 入口参数：无
 返 回 值：无。
 ***********************************************************************/
void get_time(void) {
  get_all();
  time_1302[0] = table[0] & 0x0F; //求秒的个位
  time_1302[1] = table[0] & 0x70; //求秒的十位
  time_1302[1] >>= 4; //右移4位
  time_1302[2] = table[1] & 0x0F; //求分的个位
  time_1302[3] = table[1] & 0x70; //求分的十位
  time_1302[3] >>= 4;
  time_1302[4] = table[2] & 0x0F; //求时的个位
  time_1302[5] = table[2] & 0x70; //求时的十位
  time_1302[5] >>= 4;
}
/********************************************************************
 函 数 名：get_date()读取日期函数
 功    能：读取DS1302当前日期
 入口参数：无
 返 回 值：无。
 ***********************************************************************/
void get_date(void) {
  get_all();
  date_1302[0] = table[3] & 0x0F; //求日的个位
  date_1302[1] = table[3] & 0x30; //求日的十位
  date_1302[1] >>= 4; //右移4位
  date_1302[2] = table[4] & 0x0F;  //求月的个位
  date_1302[3] = table[4] & 0x10; //求月的十位
  date_1302[3] >>= 4;
  date_1302[4] = table[6] & 0x0F; //求年的个位
  date_1302[5] = table[6] & 0xF0; //求年的十位
  date_1302[5] >>= 4;
}
void get_result(void) {
  int i;
  int j;
  get_date();
  i = date_1302[5];
  j = date_1302[4];
  year = i * 10 + j;
  i = date_1302[3];
  j = date_1302[2];
  month = i * 10 + j;
  i = date_1302[1];
  j = date_1302[0];
  date = i * 10 + j;
  get_time();
  i = time_1302[5];
  j = time_1302[4];
  hour = i * 10 + j;
  i = time_1302[3];
  j = time_1302[2];
  min = i * 10 + j;
  i = time_1302[1];
  j = time_1302[0];
  sec = i * 10 + j;
}
void IO_OUT(void) {
  GPIO_InitTypeDef GPIO_InitStruct; /*定义一个GPIO_InitTypeDef类型的结构体*/
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
void IO_INPUT(void) {
  GPIO_InitTypeDef GPIO_InitStruct; /*定义一个GPIO_InitTypeDef类型的结构体*/
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
