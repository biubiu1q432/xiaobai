#include "pid.h"
#include "main.h"
#include "tim.h"


//位置
Pid local_pid;
//增量
Pid left_incremental_pid;
Pid right_incremental_pid;

extern Motor_Stat Left_Motor;	/*左轮数据*/
extern Motor_Stat Right_Motor;	/*右轮数据*/

//给结构体类型变量赋初值
void PID_init()
{
//位置式
	local_pid.target_dis=0.000;
	local_pid.actual_dis=0.000;
	local_pid.output_val=0.000;
	local_pid.err=0.000;
	local_pid.err_last=0.000;
	local_pid.err_sum=0.000;
	local_pid.Kp=1;
	local_pid.Ki=0.0001;
	local_pid.Kd=8;

	
	//增量式
	left_incremental_pid.actual_val=0.000;
	left_incremental_pid.target_val=0.000;
	left_incremental_pid.output_pwm=0.000;	
	left_incremental_pid.err=0.000;
	left_incremental_pid.err_last=0.000;
	left_incremental_pid.err_pre=0.000;
	left_incremental_pid.err_sum=0.000;
	left_incremental_pid.Kp=5;
	left_incremental_pid.Ki=0.01;
	left_incremental_pid.Kd=1;

	right_incremental_pid.actual_val=0.000;
	right_incremental_pid.target_val=0.000;
	right_incremental_pid.output_pwm=0.000;	
	right_incremental_pid.err=0.000;
	right_incremental_pid.err_last=0.000;
	right_incremental_pid.err_pre=0.000;
	right_incremental_pid.err_sum=0.000;
	right_incremental_pid.Kp=5;
	right_incremental_pid.Ki=0.01;
	right_incremental_pid.Kd=1;


}

/**************************************************************************
@bref: 位置环
@para	：target_dis
@return: void
**************************************************************************/
void Motor_Set_Dis(float target_dis)
{

	//位置式
	float val,pwm;
	float acual_dis = (Left_Motor.Distance + Right_Motor.Distance)/2.000;//当前位置
	local_pid.target_dis  = target_dis;
	val = PID_realize(&local_pid,acual_dis);
	
	//速度式
	Motor_Set_Val(val,val);
	
}

 
/**************************************************************************
@bref: 速度环
@para	：left_val,right_val
@return: void
**************************************************************************/
void Motor_Set_Val(float left_val,float right_val)
{
		float left_pwm,right_pwm;
		//目标赋值
		left_incremental_pid.target_val = left_val;
		right_incremental_pid.target_val = right_val;
		//速度环计算
		left_pwm = Incremental_PID(&left_incremental_pid,Left_Motor.Motorspeed);
		right_pwm = Incremental_PID(&right_incremental_pid,Right_Motor.Motorspeed);
		//pwm
		Motor_Set(left_pwm,right_pwm);
}


/**************************************************************************
@bref: 限幅环(pwm)：采用慢衰减，反比
@para	：float PWM
@return: void
**************************************************************************/
void Motor_Set(float Left_PWM,float Right_PWM)
{
	/*LEFT*/
	
	//T_V<0
	if(Left_PWM <= 0){
		//方向
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,MOTOR_ARR);
		//赋值
		if(Left_PWM<-MOTOR_ARR)Left_PWM=-MOTOR_ARR;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,MOTOR_ARR+Left_PWM);		
	} 
	
	//T_V>0
	if(Left_PWM >0){
		//方向
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,MOTOR_ARR);
		//赋值
		if(Left_PWM>MOTOR_ARR)Left_PWM=MOTOR_ARR;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,MOTOR_ARR-Left_PWM);
	} 

	/*RIGHT*/
	
	//T_V<0
	if(Right_PWM <= 0){
		//方向
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,MOTOR_ARR);
		//赋值
		if(Right_PWM<-MOTOR_ARR)Right_PWM=-MOTOR_ARR;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,MOTOR_ARR+Right_PWM);		
	} 
	
	//T_V>0
	if(Right_PWM >0){
		//方向
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,MOTOR_ARR);
		//赋值
		if(Right_PWM>MOTOR_ARR)Right_PWM=MOTOR_ARR;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,MOTOR_ARR-Right_PWM);
	} 
	

}


//比例P 积分I 控制函数
float PI_realize(Pid * pid,float actual_val)
{
	pid->actual_val = actual_val;//传递真实值
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PI控制 输出=Kp*当前误差+Ki*误差累计值
	pid->output_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
	
	return pid->output_val;
}

/**************************************************************************
@bref: 位置式PID控制器
@para	：当前位置	pid->actual_dis
@return: 指定速度	pid->output_val
**************************************************************************/
float PID_realize(Pid * pid,float actual_dis)
{
	pid->actual_dis = actual_dis;//传递真实值
	pid->err = pid->target_dis - pid->actual_dis;////当前误差=目标值-真实值		
	pid->err_sum += pid->err;//误差累计值 = 当前误差累计和
	//使用PID控制 输出 = Kp*当前误差  +  Ki*误差累计值 + Kd*(当前误差-上次误差)
	pid->output_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);	
	//保存上次误差: 这次误差赋值给上次误差
	pid->err_last = pid->err;
	
	return pid->output_val;
}


/**************************************************************************
函数功能：增量PID控制器
入口参数：实际值，目标值
返回  值：电机PWM
根据增量式离散PID公式 
pwm=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]		对应关系（增量式-》位置式）kp->kd ki->kp kd->ki
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
**************************************************************************/
float Incremental_PID(Pid * pid,float actual_val)
{ 		
	//计算
	pid->actual_val = actual_val;//传递真实值 
	pid->err = pid->target_val - pid->actual_val;//当前误差=目标值-真实值   
	pid->output_pwm  += (pid->Kd*(pid->err - pid->err_last))               /* 比例环节 */
									 + (pid->Kp * pid->err)                           /* 积分环节 */
									 + (pid->Ki*(pid->err - 2*pid->err_last + pid->err_pre));  /* 微分环节 */ 
    
	pid->err_pre=pid->err_last;                                   /* 保存上上次偏差 */
	pid->err_last=pid->err;	                                    /* 保存上一次偏差 */

	return pid->output_pwm;                                            /* 输出结果 */
}

	
