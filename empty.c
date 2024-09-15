/*			
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "oled.h"
#include "bsp_tb6612.h"
#include "srevo.h"
#include "PID.h"
#include "openmv.h"
#include "Encoder.h"
#include "bsp_mpu6050.h"
#include "inv_mpu.h"
#include "JY901.h"
#include "serial.h"
#include <stdio.h>
#include <limits.h>
#include "guang.h"
#include "filter.h"
volatile unsigned char uart_data = 0;
volatile unsigned char uart_data1 = 0;

int Speed_Set = 20; // 999
int SpeedR;
int SpeedL;
float Speed_Left;
float Speed_Right;
float Speed_Current;

int Distance = 0;
int last_position = 0;	// 位置计数
int last_position_ = 0; // 位置计数
int openEncoder = 0;
int currentA;
int currentB;
int lastA = 0;	  // 上一次读取的A通道状态
int lastB = 0;	  // 上一次读取的B通道状态
int position = 0; // 位置计数
int currentA_;
int currentB_;
int lastA_ = 0;	   // 上一次读取的A通道状态
int lastB_ = 0;	   // 上一次读取的B通道状态
int position_ = 0; // 位置计数

extern float Yaw_Now;
extern float Gyro_Now;

float angle_yaw;
int open_yaw = 0;
int open_line = 0;
int yaw_direction = 1;
int line_direction = 0;
float delta_angle = 0.0;
float delta_line = 0.0;

float Yaw_Need;

void uart0_send_char(char ch);
void uart0_send_string(char *str);

void clearEncoder()
{
//	lastA = 0;
//	lastA_ = 0;
//	lastB = 0;
//	lastB_ = 0;
//	position = 0;
//	position_ = 0;
	Distance=0;
	// Speed_Left=Speed_Set;
	// Speed_Right=Speed_Set;
	// Distance=0;
}


int LineL1 = 1, LineL2 = 1, LineR1 = 1, LineR2 = 1;
int pin2 = 1, pin9 = 1,pin21=1,pin14=1;

int led_cnt=0;
int led_flag=0;
int led_cnt_num=0;
void LineWalking(void)
{
	readGuangStatus(); // 获取黑线检测状态
}
void LED_ON(void){
	
	if(led_flag==1){
			led_flag=0;
			led_cnt_num++;
		}
	led_cnt=0;
	 DL_GPIO_setPins(LED_PORT,LED_PIN_27_PIN);  //输出高电平
	DL_GPIO_setPins(BEEP_PORT,BEEP_PIN_26_PIN);
}
void LED_OFF(void){
	if(led_cnt>=50){
		led_flag=1;
	 DL_GPIO_clearPins(LED_PORT,LED_PIN_27_PIN); 
		DL_GPIO_clearPins(BEEP_PORT,BEEP_PIN_26_PIN);
	}
}
volatile int task_cnt = 0;
int flag = 0;
int title = 4;
int road = 0;
int begin = 0;
int adjust_mode = 0;
int lost_cnt = 0;
int straight_cnt = 0;
extern int adjust_cnt;

void readKey(void){
	int a=!!DL_GPIO_readPins(KEY_PORT,KEY_PIN_12_PIN); 
	int b=!!DL_GPIO_readPins(KEY_PORT,KEY_PIN_13_PIN);
	if(a==0&&b==0){
		title=1;
	}
	if(a==0&&b==1){
		title=2;
	}
	if(a==1&&b==0){
		title=3;
	}
	if(a==1&&b==1){
		title=4;
	}
}
int main(void)
{
	SYSCFG_DL_init();
	NVIC_EnableIRQ(Encoder_INT_IRQN); // 开启编码器引脚的GPIOA端口中断
									  // 清除串口中断标志
	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	// 使能串口中断
	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
	// 清除定时器中断标志
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	// 使能定时器中断
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);


	delay_ms(100);
	openEncoder = 0; // 编码器
	open_line = 0;	 // 巡线
	open_yaw = 4;	 // 角度环
	while (1)
	{
	}
}

// 搭配滴答定时器实现的精确ms延时
void delay_ms(unsigned int ms)
{
}

// 串口发送单个字符
void uart1_send_char(char ch)
{
	// 当串口0忙的时候等待，不忙的时候再发送传进来的字符
	while (DL_UART_isBusy(UART_1_INST) == true)
		;
	// 发送单个字符
	DL_UART_Main_transmitData(UART_1_INST, ch);
}
// 串口发送字符串
void uart1_send_string(char *str)
{
	// 当前字符串地址不在结尾 并且 字符串首地址不为空
	while (*str != 0 && str != 0)
	{
		// 发送字符串首地址中的字符，并且在发送完成之后首地址自增
		uart1_send_char(*str++);
	}
}

// Group1的中断服务函数（软件编码器解码）
void GROUP1_IRQHandler(void)
{
	// 读取Group1的中断寄存器并清除中断标志位
	switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
	{
	// 检查是否是KEY的GPIOA端口中断，注意是INT_IIDX，不是PIN_18_IIDX
	case Encoder_INT_IIDX:
		if (openEncoder)
		{
			readEncoder();
		}
		else
		{
			lastA = 0;
			lastA_ = 0;
			lastB = 0;
			lastB_ = 0;
			position = 0;
			position_ = 0;
			Distance = 0;
		}
		break;
	}
}

void UART_1_INST_IRQHandler(void)
{
	// 如果产生了串口中断
	switch (DL_UART_getPendingInterrupt(UART_1_INST))
	{
	case DL_UART_IIDX_RX: // 如果是接收中断
		// 接发送过来的数据保存在变量中
		uart_data1 = DL_UART_Main_receiveData(UART_1_INST);
		// 将保存的数据再发送出去
		// uart1_send_char(uart_data1);
		CopeSerial2Data(uart_data1);
		//IMU_Get();

		break;

	default: // 其他的串口中断
		break;
	}
}
int num = 0;
int startdetect=0;
int runflag=0;
extern int road_cnt;
// 定时器的中断服务函数 已配置为0.2秒的周期
void TIMER_0_INST_IRQHandler(void)
{

	// 如果产生了定时器中断
	switch (DL_TimerG_getPendingInterrupt(TIMER_0_INST))
	{
	case DL_TIMER_IIDX_ZERO: // 如果是0溢出中断
		// Yaw_Now = Kalman_Filter_x(Yaw_Now, Gyro_Now);
		//				// 一阶互补滤波器的调用
		//					Yaw_Now = Complementary_Filter_x(Yaw_Now, Gyro_Now);
		num++;
		led_cnt++;
		LED_OFF();
		IMU_Get();
		Get_Speed();
		Distance+=Speed_Current;
		// 将LED灯的状态翻转
		if (num == 10)
		{
			num = 0;
			//DL_GPIO_togglePins(LED1_PORT, LED1_PIN_14_PIN);
		}
		if ((Yaw_Now >= 0.001 || Yaw_Now <= -0.001) && begin == 0)
		{
			task_cnt++;
			if (task_cnt >= 100)
			{
				begin++;
				Yaw_Need=0;
				readKey();
				openEncoder = 1;
			}
		}
		if (begin)
		{
			if (title == 1)
			{
				if (road == 0)
				{
					open_yaw = 4;
					if (Distance >= 3500)
					{
						LED_ON();
						Speed_Set = 0;
						open_yaw=0;
						road++;
						// clearEncoder();
					}
				}
			}
			if (title == 2)
			{
				if (road == 0&&startdetect==0)
				{
					open_yaw = 4;
					if (Distance >= 3000)
					{
						// Speed_Set=0;
						startdetect=1;
						Speed_Set = 10;
						// clearEncoder();
					}
				}
				else if(road==0&&startdetect==1){
						LineWalking();
						if (pin2 || pin9 || LineL2 || LineR2||LineL1||LineR1){
							startdetect=0;
							road++;
							LED_ON();
							clearEncoder();
						}
				}
				if (road == 1)
				{
					LineWalking();
					if (flag == 0)
					{
						//adjust_mode = 1;
						open_line = 1;
						open_yaw = 0;
						flag = 1;
						//PID_Init(&PIDangle, 6.0f, 0.0f, 1.0f); // PID参数
					}
				}
				if (road == 2&&startdetect==0)
				{
					open_yaw = 4;
					if (Distance >= 3000)
					{
						// Speed_Set=0;
						startdetect=1;
						Speed_Set = 10;
						// clearEncoder();
					}
				}
				else if(road==2&&startdetect==1){
						LineWalking();
						if (pin2 || pin9 || LineL2 || LineR2||LineL1||LineR1){
							startdetect=0;
							road++;
							LED_ON();
							clearEncoder();
						}
				}
				if (road == 3)
				{
					LineWalking();
					if (flag == 0)
					{
						adjust_mode = 1;
						open_line = 1;
						open_yaw = 0;
						flag = 1;
					}
				}
			}
			if (title == 3)
			{
				if(led_cnt_num<=3){
					// LineWalking();
					if (road == 0&&startdetect==0)
					{
						Speed_Set=25;
	//					if(Distance>=500){
	//						Speed_Set=30;
	//					}
						open_yaw = 4;
						if (Distance >= 1000)
						{
							// Speed_Set=0;
							//open_line = 1;
							Speed_Set=30;
							
							if(road_cnt==0&&led_cnt_num==0&&Distance >= 3340){
								startdetect=1;
							//road++;
							Speed_Set = 13;
							Yaw_Need=45;
							}
							else if(road_cnt==0&&led_cnt_num!=0&&Distance>=3500){
								startdetect=1;
							//road++;
							Speed_Set = 13;
							Yaw_Need=45+(float)(led_cnt_num/1.6);
							}
							else if(Distance>=3500&&road_cnt==1){
								startdetect=1;
							//road++;
							Speed_Set = 13;
								Yaw_Need=-135+(float)(led_cnt_num/1.6);
							}
							//clearEncoder();
						}
					}
					if(road==0&&startdetect==1){
						LineWalking();
						if (pin2 || pin9 || LineL2 || LineR2||LineL1||LineR1){
	//						if(pin2||pin9){
	//							open_yaw=0;
	//							open_line=0;
	//							LED_ON();
	//							if(pin2){
	//								delta_line
	//							}
	//							else{
	//							}
	//						}
							//else{
							startdetect=0;
							LED_ON();
							open_yaw=0;
							open_line=1;
							road++;
							clearEncoder();
							//}
						}
					}
					if (road == 1)
					{
						if (flag == 0)
						{
							clearEncoder();
							//adjust_mode = 1;
							flag = 1;
						}
					}
				}
				else {
					delta_line=0;
					open_yaw=0;
					open_line=0;
					Speed_Set=0;
					Motor_straight(0,Left);
					Motor_straight(0,Right);
				}
			}
			// mutex_lock(); // 锁定互斥锁
			if (title == 4)
			{
				if(led_cnt_num<=15){
					// LineWalking();
					if (road == 0&&startdetect==0)
					{
						Speed_Set=25;
	//					if(Distance>=500){
	//						Speed_Set=30;
	//					}
						open_yaw = 4;
						if (Distance >= 1000)
						{
							// Speed_Set=0;
							//open_line = 1;
							Speed_Set=30;
							
							if(road_cnt==0&&led_cnt_num==0&&Distance >= 3300){
								startdetect=1;
							//road++;
							Speed_Set = 15;
							Yaw_Need=45;
							}
							else if(road_cnt==0&&led_cnt_num!=0&&Distance>=3360&&led_cnt_num<=14){
								startdetect=1;
							//road++;
							Speed_Set = 15;
							Yaw_Need=45+(float)(led_cnt_num/1.6);
							}
							else if(road_cnt==0&&led_cnt_num!=0&&Distance>=3100&&led_cnt_num==15){
								startdetect=1;
							//road++;
							Speed_Set = 15;
							Yaw_Need=45+(float)(led_cnt_num/1.6);
							}
							else if(Distance>=3660&&road_cnt==1){
								startdetect=1;
							//road++;
							Speed_Set = 15;
								Yaw_Need=-135+(float)(led_cnt_num/1.6);
							}
							//clearEncoder();
						}
					}
					if(road==0&&startdetect==1){
						LineWalking();
						if (pin2 || pin9 || LineL2 || LineR2||LineL1||LineR1){
	//						if(pin2||pin9){
	//							open_yaw=0;
	//							open_line=0;
	//							LED_ON();
	//							if(pin2){
	//								delta_line
	//							}
	//							else{
	//							}
	//						}
							//else{
							startdetect=0;
							LED_ON();
							open_yaw=0;
							open_line=1;
							road++;
							clearEncoder();
							//}
						}
					}
					if (road == 1)
					{
						if (flag == 0)
						{
							clearEncoder();
							//adjust_mode = 1;
							flag = 1;
						}
					}
				}
				else {
					delta_line=0;
					open_yaw=0;
					open_line=0;
					Speed_Set=0;
					Motor_straight(0,Left);
					Motor_straight(0,Right);
				}
			}
			// mutex_lock(); // 锁定互斥锁
			if((title!=3&&title!=4)||led_cnt_num<=15){
			Motor_Control();
			}
		}
		break;

	default: // 其他的定时器中断

		break;
	}
}