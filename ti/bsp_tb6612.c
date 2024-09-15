/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-31     LCKFB-LP    first version
 */
#include "bsp_tb6612.h"
#include "PID.h"
extern int Speed_Set; // 1000满转
extern int SpeedR;
extern int SpeedL;
extern float Speed_Left;
extern float Speed_Right;
extern float delta_angle;

extern int last_position;  // 位置计数
extern int last_position_; // 位置计数
extern int Distance;	   // 位置计数
extern int position_;	   // 位置计数

extern float delta_angle;
extern float delta_line;
extern int yaw_direction;
extern void clamp(float *value, float min, float max);
extern float map_to_0_to_1000(float Cur);
extern float Yaw_Now;
extern float Gyro_Now;
extern int open_yaw;
extern int open_line;
extern int line_direction;
extern void clearEncoder();
extern int LineL1, LineL2, LineR1, LineR2;
extern int pin2, pin9,pin21,pin14;
extern int adjust_mode;
extern int title;
extern int road;
extern int flag;
extern int lost_cnt;
extern int straight_cnt;
int adjust_cnt = 0;
int road_cnt = 0;
extern void LineWalking(void);
extern void LED_ON(void);
extern void LED_OFF(void);

extern float Speed_Current;

extern float Yaw_Need;
/******************************************************************
 * 函 数 名 称：AO_Control
 * 函 数 说 明：A端口电机控制
 * 函 数 形 参：dir旋转方向 1正转0反转   speed旋转速度，范围（0 ~ per-1）
 * 函 数 返 回：无
 * 作       者：LC
 * 备       注：无
 ******************************************************************/
void AO_Control(uint8_t dir, uint32_t speed)
{
	if (dir == 0)
	{
		AIN1_OUT(0);
		AIN2_OUT(1);
	}
	else
	{
		AIN1_OUT(1);
		AIN2_OUT(0);
	}

	DL_TimerG_setCaptureCompareValue(PWM_CAR_INST, speed, GPIO_PWM_CAR_C0_IDX);
}

void BO_Control(uint8_t dir, uint32_t speed)
{
	if (dir == 0)
	{
		BIN1_OUT(0);
		BIN2_OUT(1);
	}
	else
	{
		BIN1_OUT(1);
		BIN2_OUT(0);
	}

	DL_TimerG_setCaptureCompareValue(PWM_CAR_INST, speed, GPIO_PWM_CAR_C1_IDX);
}

void Motor_straight(int speed, int wheel) // 0是左轮，1是右轮
{
	if (speed >= 0)
	{
		int speed_ = 999 - speed;
		if (wheel == 1)
		{
			AO_Control(1, speed_);
		}
		else if (wheel == 0)
		{
			BO_Control(1, speed_);
		}
	}
	else
	{
		int speed_ = 999 + speed;
		if (wheel == 1)
		{
			AO_Control(0, speed_);
		}
		else if (wheel == 0)
		{
			BO_Control(0, speed_);
		}
	}
}
// 或者，如果你想自己实现，不使用标准库函数，可以这样写：
float my_float_abs(float x) {
    volatile float y = x; // 使用volatile关键字避免编译器优化
    return (y >= 0.0f) ? y : -y;
}
int find_line_flag=0;//判断上一次检测到的直线
int rejust_cnt=0;
int first_beep=0;
extern int led_cnt_num;
extern int Distance;
int fiest_cnt=0;
void Motor_Control(void)
{
	if (open_yaw)
	{ // 角度环
		if (open_yaw == 4)
		{ // 循角度
			//delta_angle = PID_Compute(&PIDangle, Yaw_Now, 1);
			Pid_Yaw(Yaw_Need);
		}
	}
		if(open_line)
		{ 
			// 巡线
			delta_line = 0;
			if (title == 2)
			{
					if(LineL1&&LineR1){
						lost_cnt=0;
						delta_line = -100;
					}
					else if(LineL1&&!LineR1){
						lost_cnt=0;
						delta_line=-300;
					}
					else if(!LineL1&&LineR1){
						lost_cnt=0;
						delta_line=300;
					}
					else if(LineL2&&!LineR2){
						lost_cnt=0;
						delta_line=-400;
					}
					else if(!LineL2&&LineR2){
						lost_cnt=0;
						delta_line=200;
					}
					else if(LineL1==0&&LineR1==0&&LineL2==0&&LineR2==0&&pin9&&pin2==0){
						lost_cnt=0;
						delta_line=-300;
					}
					else if(LineL1==0&&LineR1==0&&LineL2==0&&LineR2==0&&pin9==0&&pin2){
						lost_cnt=0;
						delta_line=200;
					}
					else if (LineR2 == 0 && LineL2 == 0 && LineR1 == 0 && LineL1 == 0&&pin2==0&&pin9==0)
					{
						if(Distance >= 4000){
							lost_cnt++;
						}
							if(lost_cnt<=10){
								delta_line = -110;
							}
							if (title == 2 && road == 1 && lost_cnt >= 5&& Distance >= 4000/*&&my_float_abs(Yaw_Now-180.0)<=10*/)
							{
								if((Yaw_Now+175<=10&&Yaw_Now+175>-10)){
									LED_ON();
									lost_cnt = 0;
									delta_line = 0;
									open_line = 0;
									open_yaw = 4;
									Yaw_Need=-175;
									Speed_Set = 20;
									// Speed_Set=0;
									flag = 0;
									road = 2;
									//PID_SetSetpoint(&PIDangle, -145);
									clearEncoder();
								}
								else{
								lost_cnt=0;
								open_yaw=4;
								Yaw_Need=180;
							}
							}
							if (title == 2 && road == 3 && lost_cnt >= 5&& Distance >= 4000/*&&my_float_abs(Yaw_Now-0)<=10*/)
							{
								if(Yaw_Now+(float)(led_cnt_num/1.6)-0<=10&&Yaw_Now+(float)(led_cnt_num/1.6)-0>-10){
									LED_ON();
									delta_line = 0;
									open_line = 0;
									Speed_Set = 0;
									flag = 0;
									road = 5;
									clearEncoder();
								}
								else{
									lost_cnt=0;
									open_yaw=4;
									Yaw_Need=0;
								}
							}
						
					}
			}
			else if(title==4||title==3){
				Speed_Set=10;
				LineWalking();
				if(road_cnt){//需要右转
					delta_line=-0;
					if(LineL1&&LineR1){
						fiest_cnt=10000;
						fiest_cnt++;
						//Speed_Set=13;
						lost_cnt=0;
						delta_line=-85;
					}
					else if(LineL1&&!LineR1){
						fiest_cnt++;
						lost_cnt=0;
						delta_line=-400;
					}
					else if(!LineL1&&LineR1){
						fiest_cnt=10000;
						fiest_cnt++;
						lost_cnt=0;
						delta_line=400;
					}
//					else if(pin21&&!pin14){
//						lost_cnt=0;
//						delta_line=-300;
//					}
//					else if(!pin21&&pin14){
//						lost_cnt=0;
//						delta_line=300;
//					}
					else if(LineL2&&!LineR2){
						fiest_cnt=10000;
						lost_cnt=0;
						delta_line=-400;
						
					}
					else if(!LineL2&&LineR2){
						lost_cnt=0;
						
						if(fiest_cnt>=10000){
							//delta_line=0;
							delta_line=500;
							fiest_cnt++;
						}
					}
					else if(!pin9&&pin2){
						fiest_cnt++;
						lost_cnt=0;
						if(fiest_cnt>=10000){
						delta_line=600;
						}
						
					}
					else if(!pin2&&pin9){
						fiest_cnt=10000;
						fiest_cnt++;
						lost_cnt=0;
						delta_line=-600;
					}
					else if ((LineR2 == 0 && LineL2 == 0 && LineR1 == 0 && LineL1 == 0)/*&&pin2==0&&pin9==0*/)
					{//完成巡线
						//Speed_Set=0;
						if(Distance>=3920){
						lost_cnt++;
						}
						if(Distance<3920){
						delta_line=-0;
						}
						if (lost_cnt >= 5 && Distance >= 3920)
						{
							fiest_cnt=0;
							if((Yaw_Now+(float)(led_cnt_num/1.6)-45<=10&&Yaw_Now+(float)(led_cnt_num/1.6)-45>-10)){
								LED_ON();
								lost_cnt = 0;
								road_cnt=0;
								delta_line = 0;
								open_line = 0;
								open_yaw = 4;
								if(led_cnt_num==0){
								Yaw_Need=0+(float)(led_cnt_num/1.6);
								}
								else{
									Yaw_Need=-5+(float)(led_cnt_num/1.8);
								}
								Speed_Set = 20;
								//Speed_Set=0;
								road=0;
								flag = 0;
								clearEncoder();
							}
							else{
								lost_cnt=0;
								open_yaw=4;
								Yaw_Need=45;
							}
						}
					}
				}
				else if(road_cnt==0){
					Speed_Set=10;
					delta_line=0;
					//需要左转
					if(LineL1&&LineR1){
						fiest_cnt=10000;
						//Speed_Set=15;
						lost_cnt=0;
						delta_line=85;
					}
					else if(LineL1&&!LineR1){
						fiest_cnt=10000;
						fiest_cnt++;
						lost_cnt=0;
						delta_line=-400;
					}
					else if(!LineL1&&LineR1){
						fiest_cnt++;
						lost_cnt=0;
						delta_line=400;
					}
					else if(LineL2&&!LineR2){
						lost_cnt=0;
						
						if(fiest_cnt>=10000){
							//delta_line=0;
							delta_line=-500;
						}
						
					}
					else if(!LineL2&&LineR2){
						fiest_cnt=10000;
						fiest_cnt++;
						lost_cnt=0;
						delta_line=400;
						
					}
					else if(!pin9&&pin2){
						fiest_cnt=10000;
						lost_cnt=0;
						delta_line=600;
					}
					else if(!pin2&&pin9){
						fiest_cnt++;
						lost_cnt=0;
						if(fiest_cnt>=10000){
						delta_line=-600;
						}
						
					}
//					else if(pin21&&!pin14){
//						lost_cnt=0;
//						delta_line=-300;
//					}
//					else if(!pin21&&pin14){
//						lost_cnt=0;
//						delta_line=800;
//					}
					else if ((LineR2 == 0 && LineL2 == 0 && LineR1 == 0 && LineL1 == 0/*&&pin2==0&&pin9==0*/))
					{//完成巡线
						//Speed_Set=0;
						if(Distance<3920){
						  delta_line=0;
						}
						if(Distance>=3920){
						lost_cnt++;
						}
						
						if (lost_cnt >= 5 && Distance >= 3920)
						{
							fiest_cnt=0;
							if((led_cnt_num>15&&lost_cnt>=30)||(Yaw_Now+135+(float)(led_cnt_num/1.6)<=10&&Yaw_Now+135+(float)(led_cnt_num/1.6)>-10)){
								LED_ON();
								lost_cnt = 0;
								road_cnt=1;
								delta_line = 0;
								open_line = 0;
								open_yaw = 4;
								Yaw_Need= -81+(float)(led_cnt_num/1.6);
								Speed_Set = 20;
								//Speed_Set=0;
								road=0;
								flag = 0;
								clearEncoder();
							}
							else{
								lost_cnt=0;
								open_yaw=4;
								Yaw_Need=-135;
							}
						}
					}
				}
				
			}
			if (!line_direction)
			{
				delta_line = -delta_line; // x取反
			}
		}
	

	// 速度环
	incremental_pidL(Speed_Set,Speed_Current);
	incremental_pidR(Speed_Set,Speed_Current);
	SpeedR = (int)(pwmR+0.5*Output_Yaw- (int)(0.5 * delta_line));
	SpeedL = (int)(pwmL-0.5*Output_Yaw+(int)(0.5 * delta_line));
	
//	if(adjust_cnt&&title==4){
//		if(road_cnt==0){
//			SpeedL=0;
//		}
//		else {
//			SpeedR=0;
//		}
//	}
	
	if(SpeedR>1000)	SpeedR = 1000;
	if(SpeedR<-1000) SpeedR = -1000;
	if(SpeedL>1000)	SpeedL = 1000;
	if(SpeedL<-1000) SpeedL = -1000;	
	Motor_straight(SpeedL,Left);
	Motor_straight(SpeedR,Right);
	// Motor_straight(100,1);
	// Motor_straight(100,0);
}