/*pid算法和速度位置伺服*/

#include "Caculate.h"
#include "math.h"

/**
  *@brief PID增量式算法
	*@param pid：PID结构体
	*@retval 无
	*/
void PID_Calc(PID_t *pid)
{
	//PID误差定义
	pid->cur_error = pid->ref - pid->fdb;
	
	//离散PID误差公式 
	/*  u(k)-u(k-1)=Ae(k)-Be(k-1)+Ce(k-2)
	 *  A=Kp*(1+T/Ti+Td/T)
	 *  B=Kp*(1+2Td/T)
	 *  C=Kp*Td/T
	 */
	pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
	
	//递进
	pid->error[0] = pid->error[1];
	pid->error[1] = pid->ref - pid->fdb;
	
	/*设定输出上限*/
	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;

}

/**
  *@brief PID比例控制算法
	*@param pid：PID结构体
	*@retval 无
	*/
void P_Calc(PID_t *pid){
	pid->cur_error = pid->ref - pid->fdb;
	pid->output = pid->KP * pid->cur_error;
	/*设定输出上限*/
	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;
	
	if(fabs(pid->output)<pid->outputMin)
		pid->output=0;

}

//位置伺服函数
void positionServo(float ref, DJI_t * motor){
	
	motor->posPID.ref = ref;
	motor->posPID.fdb = motor->AxisData.AxisAngle_inDegree;
	PID_Calc(&motor->posPID);
	
	motor->speedPID.ref = motor->posPID.output;
	motor->speedPID.fdb = motor->FdbData.rpm;
	PID_Calc(&motor->speedPID);

}

//速度伺服函数
void speedServo(float ref, DJI_t * motor){
	motor->speedPID.ref = ref;
	motor->speedPID.fdb = motor->FdbData.rpm;
	PID_Calc(&motor->speedPID);
}
