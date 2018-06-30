/*
 *********************************************************************************************************
 *
 * 模块名称 : 串口LCD驱动模块
 * 文件名称 : bsp_usart_lcd.c
 * 版   本 : V1.0
 * 说   明 :
 * 修改记录 :
 *   版本号  日期       作者    说明
 *   v0.1    2009-12-27 ethan  创建该文件
 *
 * Copyright (C), 2017-2020, ethan
 *
 *********************************************************************************************************
 */
/* 包含头文件 ----------------------------------------------------------------*/
#include "bsp_usart_lcd.h"
#include "bsp_spi_flash.h"
#include "bsp_ds1302.h"

extern uint32_t calcDays(uint8_t y1, uint8_t m1, uint8_t d1, uint8_t y2,
		uint8_t m2, uint8_t d2);

extern uint8_t currentPage;
extern UART_HandleTypeDef huart2;
extern uint8_t currentPage;
extern uint8_t lastPage;
extern signed long timeStamp;
extern signed long currentTime;
extern SAVEDATA saveData[];
extern uint8_t inputPassword[];
extern uint8_t currentCHN;
extern uint8_t getUIFlag;
extern uint8_t volumePicCMD[];
extern uint8_t muteFlag;
extern uint8_t alarmFlag;
extern uint8_t testFlag;
CTRL_MSG updateUICMD;

/* 颜色代码 */
uint8_t setTextGreen[2] = { 0x04, 0x00 };
uint8_t setTextRed[2] = { 0xF8, 0x00 };

/* 中英文名称 两位长度 + 字符编码 */
uint8_t enName[21][10] = {
//未使用 - Unused
		{ 0x00, 0x06, 0x55, 0x6E, 0x75, 0x73, 0x65, 0x64 },
		//医疗空气 - Med Air
		{ 0x00, 0x07, 0x4D, 0x65, 0x64, 0x20, 0x41, 0x69, 0x72 },
		//器械空气 - Air 800
		{ 0x00, 0x07, 0x41, 0x69, 0x72, 0x20, 0x38, 0x30, 0x30 },
		//牙科空气 - DentAir
		{ 0x00, 0x07, 0x44, 0x65, 0x6E, 0x74, 0x41, 0x69, 0x72 },
		//混合气体 - Syn Gas
		{ 0x00, 0x07, 0x53, 0x79, 0x6E, 0x20, 0x47, 0x61, 0x73 },
		//医用真空 - Vac
		{ 0x00, 0x03, 0x56, 0x61, 0x63 },
		//牙科真空 - DentVac
		{ 0x00, 0x07, 0x44, 0x65, 0x6E, 0x74, 0x56, 0x61, 0x63 },
		//医用氧气 - O2
		{ 0x00, 0x02, 0x4F, 0x32 },
		//氮气 - N2
		{ 0x00, 0x02, 0x4E, 0x32 },
		//二氧化碳 -CO2
		{ 0x00, 0x03, 0x43, 0x4F, 0x32 },
		//氧化亚氮 - N2O
		{ 0x00, 0x03, 0x4E, 0x32, 0x4F },
		//氧/氧化亚氮 - O2/N2O
		{ 0x00, 0x06, 0x4F, 0x32, 0x2F, 0x4E, 0x32, 0x4F },
		//氧/二氧化碳 - O2/CO2
		{ 0x00, 0x06, 0x4F, 0x32, 0x2F, 0x43, 0x4F, 0x32 },
		//氦气/氧气 - He/O2
		{ 0x00, 0x05, 0x48, 0x65, 0x2F, 0x4F, 0x32 },
		//麻醉废气 -TODO
		{ 0x00, 0x01, 0x20 },
		//呼吸废气 -TODO
		{ 0x00, 0x01, 0x20 },
		//输入压力 - Pin
		{ 0x00, 0x03, 0x50, 0x69, 0x6E },
		//输出压力 - Pout
		{ 0x00, 0x04, 0x50, 0x6F, 0x75, 0x74 },
		//通道1 - CHN1
		{ 0x00, 0x04, 0x43, 0x48, 0x4E, 0x31 },
		//通道2 - CHN2
		{ 0x00, 0x04, 0x43, 0x48, 0x4E, 0x32 },
		//通道3 - CHN3
		{ 0x00, 0x04, 0x43, 0x48, 0x4E, 0x33 }, };
uint8_t cnName[21][15] = {
//未使用
		{ 0x00, 0x06, 0xCE, 0xB4, 0xCA, 0xB9, 0xD3, 0xC3 },
		//医疗空气
		{ 0x00, 0x08, 0xD2, 0xBD, 0xC1, 0xC6, 0xBF, 0xD5, 0xC6, 0xF8 },
		//器械空气
		{ 0x00, 0x08, 0xC6, 0xF7, 0xD0, 0xB5, 0xBF, 0xD5, 0xC6, 0xF8 },
		//牙科空气
		{ 0x00, 0x08, 0xD1, 0xC0, 0xBF, 0xC6, 0xBF, 0xD5, 0xC6, 0xF8 },
		//混合气体
		{ 0x00, 0x08, 0xBB, 0xEC, 0xBA, 0xCF, 0xC6, 0xF8, 0xCC, 0xE5 },
		//医用真空
		{ 0x00, 0x08, 0xD2, 0xBD, 0xD3, 0xC3, 0xD5, 0xE6, 0xBF, 0xD5 },
		//牙科真空
		{ 0x00, 0x08, 0xD1, 0xC0, 0xBF, 0xC6, 0xD5, 0xE6, 0xBF, 0xD5 },
		//医用氧气
		{ 0x00, 0x08, 0xD2, 0xBD, 0xD3, 0xC3, 0xD1, 0xF5, 0xC6, 0xF8 },
		//氮气
		{ 0x00, 0x04, 0xB5, 0xAA, 0xC6, 0xF8 },
		//二氧化碳
		{ 0x00, 0x08, 0xB6, 0xFE, 0xD1, 0xF5, 0xBB, 0xAF, 0xCC, 0xBC },
		//氧化亚氮
		{ 0x00, 0x08, 0xD1, 0xF5, 0xBB, 0xAF, 0xD1, 0xC7, 0xB5, 0xAA },
		//氧/氧化亚氮
		{ 0x00, 0x0B, 0xD1, 0xF5, 0x2F, 0xD1, 0xF5, 0xBB, 0xAF, 0xD1, 0xC7,
				0xB5, 0xAA },
		//氧/二氧化碳
		{ 0x00, 0x0B, 0xD1, 0xF5, 0x2F, 0xB6, 0xFE, 0xD1, 0xF5, 0xBB, 0xAF,
				0xCC, 0xBC },
		//氦气/氧气
		{ 0x00, 0x09, 0xBA, 0xA4, 0xC6, 0xF8, 0x2F, 0xD1, 0xF5, 0xC6, 0xF8 },
		//麻醉废气
		{ 0x00, 0x08, 0xC2, 0xE9, 0xD7, 0xED, 0xB7, 0xCF, 0xC6, 0xF8 },
		//呼吸废气
		{ 0x00, 0x08, 0xBA, 0xF4, 0xCE, 0xFC, 0xB7, 0xCF, 0xC6, 0xF8 },
		//输入压力
		{ 0x00, 0x08, 0xCA, 0xE4, 0xC8, 0xEB, 0xD1, 0xB9, 0xC1, 0xA6 },
		//输出压力
		{ 0x00, 0x08, 0xCA, 0xE4, 0xB3, 0xF6, 0xD1, 0xB9, 0xC1, 0xA6 },
		//通道1
		{ 0x00, 0x05, 0xCD, 0xA8, 0xB5, 0xC0, 0x31 },
		//通道2
		{ 0x00, 0x05, 0xCD, 0xA8, 0xB5, 0xC0, 0x32 },
		//通道3
		{ 0x00, 0x05, 0xCD, 0xA8, 0xB5, 0xC0, 0x33 }, };

/* 时钟变量 DS1302 */
//extern volatile char time_tab[];
extern char date_1302[];
//extern char time_1302[];

extern void eepromReadSetting(void);
extern void eepromWriteSetting(void);
void factorySetting(uint8_t permisson);
extern void loadMainPage(void);
extern void FloatToStr5(float data, uint8_t *buf, int size);
extern float StrToFloat(uint8_t *buf);
extern void Line_1A_WTN5(unsigned char SB_DATA);

/*!
 *  \brief  消息处理流程，此处一般不需要更改
 *  \param  msg 待处理消息
 *  \param  size 消息长度
 */
void ProcessUIMessage(PCTRL_MSG msg, uint16_t size) {
	uint8_t cmd_type = msg->cmd_type; //指令类型
	uint8_t ctrl_msg = msg->ctrl_msg;   //消息的类型
	uint16_t screen_id = PTR2U16(&msg->screen_id_high);   //画面ID
	uint16_t control_id = PTR2U16(&msg->control_id_high);   //控件ID
	uint8_t control_type = msg->control_type;   //控件类型
	uint32_t value = PTR2U32(msg->param);   //数值

	switch (cmd_type) {
	case NOTIFY_TOUCH_PRESS:   //触摸屏按下
	case NOTIFY_TOUCH_RELEASE:   //触摸屏松开
		//NotifyTouchXY(cmd_buffer[1],PTR2U16(cmd_buffer+2),PTR2U16(cmd_buffer+4));
		break;
	case NOTIFY_WRITE_FLASH_OK:		//写FLASH成功
		//NotifyWriteFlash(1);
		break;
	case NOTIFY_WRITE_FLASH_FAILD:		//写FLASH失败
		//NotifyWriteFlash(0);
		break;
	case NOTIFY_READ_FLASH_OK:		//读取FLASH成功
		//NotifyReadFlash(1,cmd_buffer+2,size-6);//去除帧头帧尾
		break;
	case NOTIFY_READ_FLASH_FAILD:		//读取FLASH失败
		//NotifyReadFlash(0,0,0);
		break;
	case NOTIFY_READ_RTC:		//读取RTC时间
		//NotifyReadRTC(cmd_buffer[1],cmd_buffer[2],cmd_buffer[3],cmd_buffer[4],cmd_buffer[5],cmd_buffer[6],cmd_buffer[7]);
		break;
	case NOTIFY_CONTROL: {
		if (ctrl_msg == MSG_GET_CURRENT_SCREEN)		//画面ID变化通知
		{
			NotifyScreen(screen_id);
		} else if (ctrl_msg == kCtrlAnimation)		//动画控件变化通知
				{
			NotifyAnimation(screen_id, control_id);
		} else {
			switch (control_type) {
			case kCtrlButton: //按钮控件
				NotifyButton(screen_id, control_id, msg->param[1]);
				break;
			case kCtrlText: //文本控件
				NotifyText(screen_id, control_id, msg->param);
				break;
			case kCtrlProgress: //进度条控件
				NotifyProgress(screen_id, control_id, value);
				break;
			case kCtrlSlider: //滑动条控件
				NotifySlider(screen_id, control_id, value);
				break;
			case kCtrlMeter: //仪表控件
				NotifyMeter(screen_id, control_id, value);
				break;
			case kCtrlMenu: //菜单控件
				NotifyMenu(screen_id, control_id, msg->param[0], msg->param[1]);
				break;
			case kCtrlSelector: //选择控件
				NotifySelector(screen_id, control_id, msg->param[0]);
				break;
			case kCtrlRTC: //倒计时控件
				NotifyTimer(screen_id, control_id);
				break;
			default:
				break;
			}
		}
	}
		break;
	default:
		break;
	}
}

/*!
 *  \brief  画面切换通知
 *  \details  当前画面改变时(或调用GetScreen)，执行此函数
 *  \param screen_id 当前画面ID
 */
void NotifyScreen(uint16_t screen_id) {
	if (getUIFlag == 2) {
		if (currentPage != screen_id) {
			eepromReadSetting();
			loadMainPage();
		}
		getUIFlag = 0;
	} else {
		getUIFlag = 0;
		lastPage = currentPage;
		currentPage = screen_id;
		if (currentPage == PAGE_START) {
			if (timeStamp == 999) {
				uint8_t temp[11];
				/* 播放开机音频 */
				Line_1A_WTN5(0xEF - 5); //音量
				HAL_Delay(50);
				Line_1A_WTN5(0xF3); //断续播放
				HAL_Delay(50);
				Line_1A_WTN5(0x01); //播放第零语音
				HAL_Delay(50);
				//启动开机动画
				//EE B1 20 00 00 00 01 FF FC FF FF
				temp[0] = 0xEE; //帧头
				temp[1] = 0xB1;	//命令类型(UPDATE_CONTROL)
				temp[2] = 0x20;
				temp[3] = 0x00;
				temp[4] = 0x00;
				temp[5] = 0x00;
				temp[6] = 0x01;
				temp[7] = 0xFF;	//帧尾
				temp[8] = 0xFC;
				temp[9] = 0xFF;
				temp[10] = 0xFF;
				HAL_UART_Transmit(&huart2, temp, 11, 0xFFFF);
				//记录定时器时间，8s开机
				timeStamp = currentTime;
			}
		}
		if (currentPage == PAGE_PASSWORD) {
			uint8_t temp[15];
			uint8_t i;
			//清除密码错误提示
			//EE B1 10 00 02 00 04 20 FF FC FF FF
			uint8_t temp0[12] = { 0xEE, 0xB1, 0x10, 0x00, 0x02, 0x00, 0x04,
					0x20, 0xFF, 0xFC, 0xFF, 0xFF };
			HAL_UART_Transmit(&huart2, temp0, 12, 0xFFFF);
			//更新初始显示密码
			//EE B1 10 00 02 00 01 78 78 78 78 FF FC FF FF
			temp[0] = 0xEE; //帧头
			temp[1] = 0xB1;	//命令类型(UPDATE_CONTROL)
			temp[2] = 0x10;
			temp[3] = 0x00;
			temp[4] = 0x02;
			temp[5] = 0x00;
			temp[6] = 0x01;
			temp[7] = 0x78;
			temp[8] = 0x78;
			temp[9] = 0x78;
			temp[10] = 0x78;
			temp[11] = 0xFF;	//帧尾
			temp[12] = 0xFC;
			temp[13] = 0xFF;
			temp[14] = 0xFF;
			HAL_UART_Transmit(&huart2, temp, 15, 0xFFFF);
			for (i = 0; i < 4; i++) {
				inputPassword[i] = 0x78;
			}
		}

		if (currentPage == PAGE_SET1) {
			//获取 set1 当前通道号
			//EE B1 11 00 02 00 01 FF FC FF FF
			uint8_t temp4[11] = { 0xEE, 0xB1, 0x11, 0x00, 0x03, 0x00, 0x01,
					0xFF, 0xFC, 0xFF, 0xFF };
			/* 获取通道号以更新界面 */
			HAL_UART_Transmit(&huart2, temp4, 11, 0xFFFF);
		}

		if (currentPage == PAGE_SET2) {
			uint8_t temp[100];
			uint8_t i = 0;
			//EE B1 12 00 04
			//00 01 00 01 31
			//00 03 00 01 33
			//00 06 00 01 36
			//FF FC FF FF
			temp[0] = 0xEE;
			temp[1] = 0xB1;
			temp[2] = 0x12; 			//CtrlMsgType-指示消息的类型
			temp[3] = 0x00;  			//产生消息的画面ID
			temp[4] = 0x04;

			//加载modbus地址
			temp[5] = 0x00;
			temp[6] = 0x01;
			temp[7] = 0x00;
			temp[8] = 0x03;

			temp[9] = (uint8_t) (saveData[0].modbusAddr / 100);
			temp[10] = saveData[0].modbusAddr - temp[9] * 100;
			temp[10] = (uint8_t) (temp[10] / 10);
			temp[11] = (uint8_t) (saveData[0].modbusAddr - temp[9] * 100
					- temp[10] * 10) + '0';
			temp[9] += '0';
			temp[10] += '0';

			//加载IP地址
			temp[12] = 0x00;
			temp[13] = 0x03;
			temp[14] = 0x00;
			temp[15] = 0x0F;

			temp[16] = (uint8_t) (saveData[0].IP[0] / 100);
			temp[17] = saveData[0].IP[0] - temp[16] * 100;
			temp[17] = (uint8_t) (temp[17] / 10);
			temp[18] = (uint8_t) (saveData[0].IP[0] - temp[16] * 100
					- temp[17] * 10) + '0';
			temp[16] += '0';
			temp[17] += '0';
			temp[19] = '.';

			temp[20] = (uint8_t) (saveData[0].IP[1] / 100);
			temp[21] = saveData[0].IP[1] - temp[20] * 100;
			temp[21] = (uint8_t) (temp[21] / 10);
			temp[22] = (uint8_t) (saveData[0].IP[1] - temp[20] * 100
					- temp[21] * 10) + '0';
			temp[20] += '0';
			temp[21] += '0';
			temp[23] = '.';

			temp[24] = (uint8_t) (saveData[0].IP[2] / 100);
			temp[25] = saveData[0].IP[2] - temp[24] * 100;
			temp[25] = (uint8_t) (temp[25] / 10);
			temp[26] = (uint8_t) (saveData[0].IP[2] - temp[24] * 100
					- temp[25] * 10) + '0';
			temp[24] += '0';
			temp[25] += '0';
			temp[27] = '.';

			temp[28] = (uint8_t) (saveData[0].IP[3] / 100);
			temp[29] = saveData[0].IP[3] - temp[28] * 100;
			temp[29] = (uint8_t) (temp[29] / 10);
			temp[30] = (uint8_t) (saveData[0].IP[3] - temp[28] * 100
					- temp[29] * 10) + '0';
			temp[28] += '0';
			temp[29] += '0';

			//加载密码
			temp[31] = 0x00;
			temp[32] = 0x06;
			temp[33] = 0x00;
			for (i = 0; i <= saveData[0].userPassword[0]; i++) {
				temp[i + 34] = saveData[0].userPassword[i];
			}

			temp[i + 34] = 0xFF;   			//帧尾
			temp[i + 35] = 0xFC;
			temp[i + 36] = 0xFF;
			temp[i + 37] = 0xFF;
			HAL_UART_Transmit(&huart2, temp, i + 38, 0xFFFF);
			//加载波特率
			//EE B1 10 00 04 00 02 00 FF FC FF FF
			temp[3] = 0x10;
			temp[4] = 0x00;
			temp[5] = 0x04;
			temp[6] = 0x00;
			temp[7] = 0x02;
			temp[8] = saveData[0].baudrateIndex;
			temp[9] = 0xFF;
			temp[10] = 0xFC;
			temp[11] = 0xFF;
			temp[12] = 0xFF;
			HAL_UART_Transmit(&huart2, temp, 13, 0xFFFF);
			//加载模式
			temp[7] = 0x04;
			temp[8] = saveData[0].modeIndex;
			HAL_UART_Transmit(&huart2, temp, 13, 0xFFFF);
			//加载音量
			temp[7] = 0x05;
			temp[8] = (uint8_t) (saveData[0].volume / 10);
			HAL_UART_Transmit(&huart2, temp, 13, 0xFFFF);
		}

		if (currentPage == PAGE_BLUETOOTH) {
			//获取蓝牙MAC地址
			//TODO
			//显示蓝牙MAC地址和二维码
			//EE B1 12 00 01 00 03 00 0C 30 30 31 37 45 41 30 39 32 33 41 45 00 01 00 0C 30 30 31 37 45 41 30 39 32 33 41 45 FF FC FF FF
			uint8_t temp4[11] = { 0xEE, 0xB1, 0x11, 0x00, 0x03, 0x00, 0x01,
					0xFF, 0xFC, 0xFF, 0xFF };
			/* 获取通道号以更新界面 */
			HAL_UART_Transmit(&huart2, temp4, 11, 0xFFFF);
		}
	}
}

/*!
 *  \brief  触摸坐标事件响应
 *  \param press 1按下触摸屏，3松开触摸屏
 *  \param x x坐标
 *  \param y y坐标
 */
void NotifyTouchXY(uint8_t press, uint16_t x, uint16_t y) {

}
/*!
 *  \brief  按钮控件通知
 *  \details  当按钮状态改变(或调用GetControlValue)时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param state 按钮状态：0弹起，1按下
 */
void NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state) {
	getUIFlag = 0;
	/* 密码界面确认按钮 */
	if (currentPage == PAGE_PASSWORD && control_id == 2) {
		uint8_t temp[50];
		uint8_t daysASCii[6] = { 0, 0, 0, 0, 0, 0 }; //第一位长度
		uint8_t i = 0, j = 1;
		long days = 0;
		/* 提示密码错误 */
		//EE B1 10 00 02 00 04 C3 DC C2 EB B4 ED CE F3 FF FC FF FF
		uint8_t temp0[19] =
				{ 0xEE, 0xB1, 0x10, 0x00, 0x02, 0x00, 0x04, 0xC3, 0xDC, 0xC2,
						0xEB, 0xB4, 0xED, 0xCE, 0xF3, 0xFF, 0xFC, 0xFF, 0xFF };
		/* 进入set1 */
		//EE B1 00 00 03 FF FC FF FF
		uint8_t temp1[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x03, 0xFF, 0xFC, 0xFF,
				0xFF };
		/* 进入set3 */
		//EE B1 00 00 05 FF FC FF FF
		uint8_t temp2[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x05, 0xFF, 0xFC, 0xFF,
				0xFF };
		/* 进入set4 */
		//EE B1 00 00 06 FF FC FF FF
		uint8_t temp3[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x06, 0xFF, 0xFC, 0xFF,
				0xFF };
		//获取 set1 当前通道号
		//EE B1 11 00 02 00 01 FF FC FF FF
		uint8_t temp4[11] = { 0xEE, 0xB1, 0x11, 0x00, 0x03, 0x00, 0x01, 0xFF,
				0xFC, 0xFF, 0xFF };
		//清除密码错误提示
		//EE B1 10 00 02 00 04 20 FF FC FF FF
		uint8_t temp5[12] = { 0xEE, 0xB1, 0x10, 0x00, 0x02, 0x00, 0x04, 0x20,
				0xFF, 0xFC, 0xFF, 0xFF };

		/* 校验密码 */
		for (i = 0; i < inputPassword[0]; i++) {
			if (inputPassword[i] != saveData[0].userPassword[i]) {
				j = 0;
				break;
			}
			j = 1;
		}
		if (j == 0) {
			for (i = 0; i < inputPassword[0]; i++) {
				if (inputPassword[i] != saveData[0].omePassword[i]) {
					j = 0;
					break;
				}
				j = 2;
			}
		}
		if (j == 0) {
			for (i = 0; i < inputPassword[0]; i++) {
				if (inputPassword[i] != saveData[0].rootPassword[i]) {
					j = 0;
					break;
				}
				j = 3;
			}
		}
		switch (j) {
		case 0:
			/* 提示密码错误 */
			HAL_UART_Transmit(&huart2, temp0, 19, 0xFFFF);
			break;
		case 1:
			/* 清除提示密码错误 */
			HAL_UART_Transmit(&huart2, temp5, 12, 0xFFFF);
			/* 进入set1 */
			HAL_UART_Transmit(&huart2, temp1, 9, 0xFFFF);
			lastPage = currentPage;
			currentPage = PAGE_SET1;
			/* 获取通道号以更新界面 */
			HAL_UART_Transmit(&huart2, temp4, 11, 0xFFFF);
			break;
		case 2:
			/* 清除提示密码错误 */
			HAL_UART_Transmit(&huart2, temp5, 12, 0xFFFF);
			/* 进入set3 */
			HAL_UART_Transmit(&huart2, temp2, 9, 0xFFFF);
			lastPage = currentPage;
			currentPage = PAGE_SET3;

			get_date();
			days = saveData[0].omeDays
					- calcDays(
							saveData[0].omeTime[5] * 10
									+ saveData[0].omeTime[4],
							saveData[0].omeTime[3] * 10
									+ saveData[0].omeTime[2],
							saveData[0].omeTime[1] * 10
									+ saveData[0].omeTime[0],
							date_1302[5] * 10 + date_1302[4],
							date_1302[3] * 10 + date_1302[2],
							date_1302[1] * 10 + date_1302[0]);
			j = 1;
			if (days < 0) {
				days = 0;
			}
			if (days > 10000) {
				daysASCii[j] = (uint8_t) (days / 10000);
				days -= daysASCii[j] * 10000;
				j++;
			}
			if (days > 1000 || j > 1) {
				daysASCii[j] = (uint8_t) (days / 1000);
				days -= daysASCii[j] * 1000;
				j++;
			}
			if (days > 100 || j > 1) {
				daysASCii[j] = (uint8_t) (days / 100);
				days -= daysASCii[j] * 100;
				j++;
			}
			if (days > 10 || j > 1) {
				daysASCii[j] = (uint8_t) (days / 10);
				days -= daysASCii[j] * 10;
				j++;
			}
			daysASCii[j] = days;
			for (i = 0; i < j + 1; i++) {
				daysASCii[i] += '0';
			}
			daysASCii[0] = j;
			//EE B1 12 00 05 00 01 00 03 33 36 35 00 02 00 06 30 30 30 30 30 30 FF FC FF FF
			j = 0;
			temp[j++] = 0xEE;
			temp[j++] = 0xB1;
			temp[j++] = 0x12;
			temp[j++] = 0x00;
			temp[j++] = 0x05;
			temp[j++] = 0x00;
			temp[j++] = 0x01;
			temp[j++] = 0x00;	//数据数量高位
			for (i = 0; i < daysASCii[0] + 1; i++) {
				temp[j++] = daysASCii[i];
			}
			temp[j++] = 0x00;
			temp[j++] = 0x02;
			temp[j++] = 0x00;	//数据数量高位
			for (i = 0; i < saveData[0].omePassword[0] + 1; i++) {
				temp[j++] = saveData[0].omePassword[i];
			}
			temp[j++] = 0xFF;
			temp[j++] = 0xFC;
			temp[j++] = 0xFF;
			temp[j++] = 0xFF;
			HAL_UART_Transmit(&huart2, temp, j, 0xFFFF);
			break;
		case 3:
			/* 清除提示密码错误 */
			HAL_UART_Transmit(&huart2, temp5, 12, 0xFFFF);
			/* 进入set4 */
			HAL_UART_Transmit(&huart2, temp3, 9, 0xFFFF);
			lastPage = currentPage;
			currentPage = PAGE_SET4;

			get_date();

			days = saveData[0].rootDays
					- calcDays(
							saveData[0].rootTime[5] * 10
									+ saveData[0].rootTime[4],
							saveData[0].rootTime[3] * 10
									+ saveData[0].rootTime[2],
							saveData[0].rootTime[1] * 10
									+ saveData[0].rootTime[0],
							date_1302[5] * 10 + date_1302[4],
							date_1302[3] * 10 + date_1302[2],
							date_1302[1] * 10 + date_1302[0]);
			if (days < 0) {
				days = 0;
			}
			if (days > 10000) {
				daysASCii[j] = (uint8_t) (days / 10000);
				days -= daysASCii[j] * 10000;
				j++;
			}
			if (days > 1000 || j > 1) {
				daysASCii[j] = (uint8_t) (days / 1000);
				days -= daysASCii[j] * 1000;
				j++;
			}
			if (days > 100 || j > 1) {
				daysASCii[j] = (uint8_t) (days / 100);
				days -= daysASCii[j] * 100;
				j++;
			}
			if (days > 10 || j > 1) {
				daysASCii[j] = (uint8_t) (days / 10);
				days -= daysASCii[j] * 10;
				j++;
			}
			daysASCii[j] = days;
			for (i = 0; i < j + 1; i++) {
				daysASCii[i] += '0';
			}
			daysASCii[0] = j;
			//EE B1 12 00 05 00 01 00 03 33 36 35 00 02 00 06 30 30 30 30 30 30 FF FC FF FF
			j = 0;
			temp[j++] = 0xEE;
			temp[j++] = 0xB1;
			temp[j++] = 0x12;
			temp[j++] = 0x00;
			temp[j++] = 0x06;
			temp[j++] = 0x00;
			temp[j++] = 0x01;
			temp[j++] = 0x00;	//数据数量高位
			for (i = 0; i < daysASCii[0] + 1; i++) {
				temp[j++] = daysASCii[i];
			}
			temp[j++] = 0x00;
			temp[j++] = 0x02;
			temp[j++] = 0x00;	//数据数量高位
			for (i = 0; i < saveData[0].rootPassword[0] + 1; i++) {
				temp[j++] = saveData[0].rootPassword[i];
			}
			temp[j++] = 0xFF;
			temp[j++] = 0xFC;
			temp[j++] = 0xFF;
			temp[j++] = 0xFF;
			HAL_UART_Transmit(&huart2, temp, j, 0xFFFF);
			break;
		}
	}/* END 密码界面确认按钮 */

	/* 密码界面取消按钮 */
	if (currentPage == PAGE_PASSWORD && control_id == 3) {
		/* 密码界面无参数修改，无需复原参数 */
		//eepromReadSetting();
	}/* END 密码界面取消按钮 */

	/* set1取消按钮 */
	if (currentPage == PAGE_SET1 && control_id == 9) {
		/* 复原参数 */
		eepromReadSetting();
		//恢复至对应主屏幕
		//EE B1 00 00 01 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x01, 0xFF, 0xFC, 0xFF,
				0xFF };
		temp5[4] = lastPage;
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);
		currentPage = lastPage;
		lastPage = PAGE_SET1;
	}/* END set1取消按钮 */

	/* set2取消按钮 */
	if (currentPage == PAGE_SET2 && control_id == 9) {
		/* 复原参数 */
		eepromReadSetting();
		//恢复至对应主屏幕
		//EE B1 00 00 01 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x01, 0xFF, 0xFC, 0xFF,
				0xFF };
		temp5[4] = lastPage;
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);
		currentPage = lastPage;
		lastPage = PAGE_SET2;
	}/* END set2取消按钮 */

	/* set3取消按钮 */
	if (currentPage == PAGE_SET3 && control_id == 9) {
		//恢复至对应主屏幕
		//EE B1 00 00 01 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x01, 0xFF, 0xFC, 0xFF,
				0xFF };
		temp5[4] = lastPage;
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);
		currentPage = lastPage;
		lastPage = PAGE_SET3;
		/* 复原参数 */
		eepromReadSetting();
	}/* END set3取消按钮 */

	/* set4取消按钮 */
	if (currentPage == PAGE_SET4 && control_id == 9) {
		//恢复至对应主屏幕
		//EE B1 00 00 01 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x01, 0xFF, 0xFC, 0xFF,
				0xFF };
		temp5[4] = lastPage;
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);
		currentPage = lastPage;
		lastPage = PAGE_SET4;
		/* 复原参数 */
		eepromReadSetting();
	}/* END set4取消按钮 */

	/* Bluetooth取消按钮 */
	if (currentPage == PAGE_BLUETOOTH && control_id == 4) {
		//恢复至对应主屏幕
		//EE B1 00 00 01 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x01, 0xFF, 0xFC, 0xFF,
				0xFF };
		temp5[4] = lastPage;
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);
		currentPage = lastPage;
		lastPage = PAGE_BLUETOOTH;
	}/* END Bluetooth取消按钮 */

	/* set1 & set2 确认按钮 */
	if ((currentPage == PAGE_SET1 || currentPage == PAGE_SET2)
			&& control_id == 8) {
		//跳转至load屏幕
		//EE B1 00 00 01 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x07, 0xFF, 0xFC, 0xFF,
				0xFF };
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);
		/* 保存参数到 eeprom */
		eepromWriteSetting();
		eepromReadSetting();
		/* 根据设置加载主屏幕 */
		loadMainPage();
	}/* END set1 & set2 确认按钮 */
	/* set3 确认按钮 */
	if (currentPage == PAGE_SET3 && control_id == 8) {
		//跳转至load屏幕
		//EE B1 00 00 01 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x07, 0xFF, 0xFC, 0xFF,
				0xFF };
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);
		/* 保存参数到 eeprom */
		eepromWriteSetting();
		eepromReadSetting();
		/* 根据设置加载主屏幕 */
		loadMainPage();
	}/* END set3取消按钮 */
	/* set4 确认按钮 */
	if (currentPage == PAGE_SET4 && control_id == 8) {
		//跳转至load屏幕
		//EE B1 00 00 01 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x07, 0xFF, 0xFC, 0xFF,
				0xFF };
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);
		/* 保存参数到 eeprom */
		eepromWriteSetting();
		eepromReadSetting();
		/* 根据设置加载主屏幕 */
		loadMainPage();
	}/* END set4 确认按钮 */
	/* 静音按钮 */
	if (control_id == 22) {
		//修改音量图标
		volumePicCMD[4] = currentPage;
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
	}/* END 静音按钮 */
	/* 测试按钮 */
	if (control_id == 24) {
		testFlag = 1;
	}/* END 测试按钮 */
}

/*!
 *  \brief  文本控件通知
 *  \details  当文本通过键盘更新(或调用GetControlValue)时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param str 文本控件内容
 */
void NotifyText(uint16_t screen_id, uint16_t control_id, uint8_t *str) {
	getUIFlag = 0;
	if (currentPage == PAGE_PASSWORD && control_id == 1) {
		uint8_t i, j;
		j = 60;
		inputPassword[0] = j;
		for (i = 0; i < j; i++) {
			if (str[i] != 0) {
				inputPassword[i + 1] = str[i];
			} else
				break;
		}
		inputPassword[0] = i;
	}
	/* 修改设置项 */
	/* 修改上限 */
	if (screen_id == PAGE_SET1 && control_id == 4) {
		saveData[currentCHN].upper_limit = StrToFloat(str);
	}
	/* 修改下限 */
	if (screen_id == PAGE_SET1 && control_id == 5) {
		saveData[currentCHN].lower_limit = StrToFloat(str);
	}
	/* 修改零点 */
	if (screen_id == PAGE_SET1 && control_id == 6) {
		saveData[currentCHN].zero = (signed int) StrToFloat(str);
	}
	/* 修改满点 */
	if (screen_id == PAGE_SET1 && control_id == 7) {
		saveData[currentCHN].K = (signed int) StrToFloat(str);
	}
	/* 修改 modbus 地址 */
	if (screen_id == PAGE_SET2 && control_id == 1) {
		saveData[0].modbusAddr = (uint8_t) StrToFloat(str);
	}
	/* 修改 IP 地址 */
	if (screen_id == PAGE_SET2 && control_id == 3) {
		uint8_t tempArray[4] = { 0, 0, 0, 0 };
		uint8_t i, j = 0;
		for (i = 0; i < 15; i++) {
			if (str[i] != '.') {
				tempArray[i] = str[i];
			} else
				break;
		}
		j += i;
		j++;
		saveData[0].IP[0] = (uint8_t) StrToFloat(tempArray);
		for (i = 0; i < 4; i++) {
			tempArray[i] = 0;
		}
		for (i = 0; i < 15; i++) {
			if (str[j + i] != '.') {
				tempArray[i] = str[j + i];
			} else
				break;
		}
		j += i;
		j++;
		saveData[0].IP[1] = (uint8_t) StrToFloat(tempArray);
		for (i = 0; i < 4; i++) {
			tempArray[i] = 0;
		}
		for (i = 0; i < 15; i++) {
			if (str[j + i] != '.') {
				tempArray[i] = str[j + i];
			} else
				break;
		}
		j += i;
		j++;
		saveData[0].IP[2] = (uint8_t) StrToFloat(tempArray);
		for (i = 0; i < 4; i++) {
			tempArray[i] = 0;
		}
		for (i = 0; i < 15; i++) {
			if (str[j + i] != '.') {
				tempArray[i] = str[j + i];
			} else
				break;
		}
		saveData[0].IP[3] = (uint8_t) StrToFloat(tempArray);
	}
	/* 修改 user 密码 */
	if (screen_id == PAGE_SET2 && control_id == 6) {
		uint8_t i = 0;
		while (str[i] != 0x00) {
			saveData[0].userPassword[i + 1] = str[i];
			i++;
		}
		saveData[0].userPassword[0] = i;
	}
	/* 修改 ome 密码 */
	if (screen_id == PAGE_SET3 && control_id == 2) {
		uint8_t i = 0;
		while (str[i] != 0x00) {
			saveData[0].omePassword[i + 1] = str[i];
			i++;
		}
		saveData[0].omePassword[0] = i;
	}
	/* 修改 ome 密码 */
	if (screen_id == PAGE_SET4 && control_id == 2) {
		uint8_t i = 0;
		while (str[i] != 0x00) {
			saveData[0].rootPassword[i + 1] = str[i];
			i++;
		}
		saveData[0].rootPassword[0] = i;
	}

	/* 修改 ome 授权时间 */
	if (screen_id == PAGE_SET3 && control_id == 1) {
		uint8_t i;
		saveData[0].omeDays = StrToFloat(str);
		get_date();
		for (i = 0; i < 6; i++) {
			saveData[0].omeTime[i] = date_1302[i];
		}
	}
	/* 修改 root 授权时间 */
	if (screen_id == PAGE_SET4 && control_id == 1) {
		uint8_t i;
		saveData[0].rootDays = StrToFloat(str);
		get_date();
		for (i = 0; i < 6; i++) {
			saveData[0].rootTime[i] = date_1302[i];
		}
	}
}

/*!
 *  \brief  进度条控件通知
 *  \details  调用GetControlValue时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param value 值
 */
void NotifyProgress(uint16_t screen_id, uint16_t control_id, uint32_t value) {

}

/*!
 *  \brief  滑动条控件通知
 *  \details  当滑动条改变(或调用GetControlValue)时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param value 值
 */
void NotifySlider(uint16_t screen_id, uint16_t control_id, uint32_t value) {

}

/*!
 *  \brief  仪表控件通知
 *  \details  调用GetControlValue时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param value 值
 */
void NotifyMeter(uint16_t screen_id, uint16_t control_id, uint32_t value) {

}

/*!
 *  \brief  菜单控件通知
 *  \details  当菜单项按下或松开时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param item 菜单项索引
 *  \param state 按钮状态：0松开，1按下
 */
void NotifyMenu(uint16_t screen_id, uint16_t control_id, uint8_t item,
		uint8_t state) {
	getUIFlag = 0;
	/* set2 恢复出厂设置 - 确认按钮 */
	if (currentPage == PAGE_SET2 && control_id == 14 && state == 0) {
		//跳转至load屏幕
		//EE B1 00 00 07 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x07, 0xFF, 0xFC, 0xFF,
				0xFF };
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);

		/* 保存参数到 eeprom */
		factorySetting(2);
		eepromWriteSetting();
		eepromReadSetting();
		loadMainPage();
	}/* END set2 恢复出厂设置 - 确认按钮 */
	/* set3 恢复出厂设置 - 确认按钮 */
	if (currentPage == PAGE_SET3 && control_id == 14 && state == 0) {
		//跳转至load屏幕
		//EE B1 00 00 07 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x07, 0xFF, 0xFC, 0xFF,
				0xFF };
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);
		/* 保存参数到 eeprom */
		factorySetting(1);
		eepromWriteSetting();
		eepromReadSetting();
		loadMainPage();
	}/* END set3 恢复出厂设置 - 确认按钮 */

	/* set4 恢复出厂设置 - 确认按钮 */
	if (currentPage == PAGE_SET4 && control_id == 14 && state == 0) {
		//跳转至load屏幕
		//EE B1 00 00 07 FF FC FF FF
		uint8_t temp5[9] = { 0xEE, 0xB1, 0x00, 0x00, 0x07, 0xFF, 0xFC, 0xFF,
				0xFF };
		HAL_UART_Transmit(&huart2, temp5, 9, 0xFFFF);
		/* 保存参数到 eeprom */
		factorySetting(0);
		eepromWriteSetting();
		eepromReadSetting();
		loadMainPage();
	}/* END set4 恢复出厂设置 - 确认按钮 */
}

/*!
 *  \brief  选择控件通知
 *  \details  当选择控件变化时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 *  \param item 当前选项
 */
void NotifySelector(uint16_t screen_id, uint16_t control_id, uint8_t item) {
	getUIFlag = 0;
	/* 名称 || 量程 */
	uint8_t temp1[12] = { 0xEE, 0xB1, 0x10, 0x00, 0x03, 0x00, 0x02, 0x00, 0xFF,
			0xFC, 0xFF, 0xFF };
	/* 上下限 && 压力修正系数 */
	//EE B1 12 00 03
	//00 04 00 04 31 2E 31 31
	//00 05 00 04 31 2E 31 31
	//00 06 00 04 31 2E 31 31
	//00 07 00 04 31 2E 31 31
	//FF FC FF FF
	uint8_t temp2[45] = { 0xEE, 0xB1, 0x12, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05,
			0x2D, 0x31, 0x2E, 0x31, 0x31, 0x00, 0x05, 0x00, 0x05, 0x2D, 0x31,
			0x2E, 0x31, 0x31, 0x00, 0x06, 0x00, 0x05, 0x2D, 0x31, 0x2E, 0x31,
			0x31, 0x00, 0x07, 0x00, 0x05, 0x2D, 0x31, 0x2E, 0x31, 0x31, 0xFF,
			0xFC, 0xFF, 0xFF };
	/* 根据通道号修改其他设置项 */
	if (screen_id == PAGE_SET1 && control_id == 1) {
		/* 名称 */
		temp1[6] = 2;
		temp1[7] = saveData[item].nameIndex;
		HAL_UART_Transmit(&huart2, temp1, 12, 0xFFFF);
		/* 量程 */
		temp1[6] = 3;
		temp1[7] = saveData[item].rangeIndex;
		HAL_UART_Transmit(&huart2, temp1, 12, 0xFFFF);
		/* 上下限 && 压力修正系数 */
		FloatToStr5(saveData[item].upper_limit, &temp2[9], 5);
		FloatToStr5(saveData[item].lower_limit, &temp2[18], 5);
		FloatToStr5((float) saveData[item].zero, &temp2[27], 5);
		FloatToStr5((float) saveData[item].K, &temp2[36], 5);
		HAL_UART_Transmit(&huart2, temp2, 45, 0xFFFF);
		currentCHN = item;
	}
	/* 修改设置项 */
	/* 修改名称 */
	if (screen_id == PAGE_SET1 && control_id == 2) {
		saveData[currentCHN].nameIndex = item;
	}
	/* 修改量程 */
	if (screen_id == PAGE_SET1 && control_id == 3) {
		saveData[currentCHN].rangeIndex = item;
	}
	/* 修改波特率 */
	if (screen_id == PAGE_SET2 && control_id == 2) {
		saveData[0].baudrateIndex = item;
	}
	/* 修改模式 */
	if (screen_id == PAGE_SET2 && control_id == 4) {
		saveData[0].modeIndex = item;
	}
	/* 修改音量 */
	if (screen_id == PAGE_SET2 && control_id == 5) {
		saveData[0].modeIndex = item * 10;
	}
}

/*!
 *  \brief  定时器超时通知处理
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 */
void NotifyTimer(uint16_t screen_id, uint16_t control_id) {

}

/*!
 *  \brief  读取用户FLASH状态返回
 *  \param status 0失败，1成功
 *  \param _data 返回数据
 *  \param length 数据长度
 */
void NotifyReadFlash(uint8_t status, uint8_t *_data, uint16_t length) {

}

/*!
 *  \brief  写用户FLASH状态返回
 *  \param status 0失败，1成功
 */
void NotifyWriteFlash(uint8_t status) {

}

/*!
 *  \brief  读取RTC时间，注意返回的是BCD码
 *  \param year 年（BCD）
 *  \param month 月（BCD）
 *  \param week 星期（BCD）
 *  \param day 日（BCD）
 *  \param hour 时（BCD）
 *  \param minute 分（BCD）
 *  \param second 秒（BCD）
 */
void NotifyReadRTC(uint8_t year, uint8_t month, uint8_t week, uint8_t day,
		uint8_t hour, uint8_t minute, uint8_t second) {
}
/*!
 *  \brief    动画控件通知
 *  \details  当按钮状态改变(或调用GetControlValue)时，执行此函数
 *  \param screen_id 画面ID
 *  \param control_id 控件ID
 */
void NotifyAnimation(uint16_t screen_id, uint16_t control_id) {

}
