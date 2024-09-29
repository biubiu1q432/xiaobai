#ifndef __PID_H
#define __PID_H

//pid
typedef struct 
{
	//速度
	float target_val;//目标值
	float actual_val;//实际值
	float output_pwm;//输出值
	
	//位置
	float target_dis;
	float actual_dis;
	float output_val;
	float err;//当前偏差
	
	float err_last;//上次偏差
	float err_pre;//上上次偏差
	float err_sum;//误差累计值
	float Kp,Ki,Kd;//比例，积分，微分系数
		 
} Pid;

//实时
typedef struct 
{
	float EncodeCount;
	float Distance;
	float Motorspeed;
} Motor_Stat;



//参数初始化
void PID_init(void);
//位置环
float PI_realize(Pid * pid,float actual_val);
float PID_realize(Pid * pid,float actual_val);
//增量式
float Incremental_PID(Pid * pid,float actual_val);
//速度环
void motorPidSetSpeed(float MotorSetSpeed);
//限幅环
void Motor_Set(float Left_PWM,float Right_PWM);


#endif


