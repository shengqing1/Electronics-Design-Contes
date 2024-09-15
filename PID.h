#ifndef __PID_H
#define __PID_H
#include "ti_msp_dl_config.h"
	void Pid_Yaw(float Target);
	void incremental_pidL(float Tar,float Cur);
	void incremental_pidR(float Tar,float Cur);
	extern float Output_Yaw;
	extern float pwmR;
	extern float pwmL;
#endif
