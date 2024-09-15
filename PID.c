#include "PID.h"
#include "serial.h"
#include "Encoder.h"
float Kp_Yaw= 8.0f;//0.5
float Kd_Yaw= 120.0f;//10.0
float Ki_Yaw = 0;//0.002
float Errplus = 0;
float Yaw_Now_Correct;

float Err_Yaw=0;
float LastErr_Yaw=0;
float Output_Yaw=0;



float RKi = 3.0f,RKp  = 20.0f;//3.0,20.0f
float LKi = 3.0f,LKp  = 20.0f;//3.0,20.0f
float biasR,last_biasR;
float pwmR;
float biasL,last_biasL;
float pwmL;
//-------------------------------------------------

void incremental_pidR(float Tar,float Cur)
{
	biasR = Tar - Cur;
	pwmR += RKp*(biasR - last_biasR) + RKi*biasR;
	last_biasR = biasR;
	if(pwmR>1000)	pwmR = 1000;
	else if(pwmR<-1000)	pwmR=-1000;
}

void incremental_pidL(float Tar,float Cur)
{
	biasL = Tar - Cur;
	pwmL += LKp*(biasL - last_biasL) + LKi*biasL;
	last_biasL = biasL;
		if(pwmL>1000)	pwmL = 1000;
	else if(pwmL<-1000)	pwmL=-1000;
}


void Pid_Yaw(float Target)
{
		Yaw_Now_Correct = Yaw_Now;
		Err_Yaw = Target - Yaw_Now_Correct;
		if(Err_Yaw<-180 && Yaw_Now_Correct>=0)
			Err_Yaw = Target - (Yaw_Now_Correct-360);
		else if(Err_Yaw>180 && Yaw_Now_Correct<=0)
			Err_Yaw = Target - (Yaw_Now_Correct+360);

	Output_Yaw= Kp_Yaw * Err_Yaw + Kd_Yaw * (Err_Yaw - LastErr_Yaw) + Ki_Yaw*Errplus;
	LastErr_Yaw = Err_Yaw;
	if(Output_Yaw>2000)
		Output_Yaw = 2000;
	if(Output_Yaw<-2000)
		Output_Yaw = -2000;
}
