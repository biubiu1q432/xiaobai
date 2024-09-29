#include "pid.h"
#include "main.h"
#include "tim.h"


//λ��
Pid local_pid;
//����
Pid left_incremental_pid;
Pid right_incremental_pid;

extern Motor_Stat Left_Motor;	/*��������*/
extern Motor_Stat Right_Motor;	/*��������*/

//���ṹ�����ͱ�������ֵ
void PID_init()
{
//λ��ʽ
	local_pid.target_dis=0.000;
	local_pid.actual_dis=0.000;
	local_pid.output_val=0.000;
	local_pid.err=0.000;
	local_pid.err_last=0.000;
	local_pid.err_sum=0.000;
	local_pid.Kp=1;
	local_pid.Ki=0.0001;
	local_pid.Kd=8;

	
	//����ʽ
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
@bref: λ�û�
@para	��target_dis
@return: void
**************************************************************************/
void Motor_Set_Dis(float target_dis)
{

	//λ��ʽ
	float val,pwm;
	float acual_dis = (Left_Motor.Distance + Right_Motor.Distance)/2.000;//��ǰλ��
	local_pid.target_dis  = target_dis;
	val = PID_realize(&local_pid,acual_dis);
	
	//�ٶ�ʽ
	Motor_Set_Val(val,val);
	
}

 
/**************************************************************************
@bref: �ٶȻ�
@para	��left_val,right_val
@return: void
**************************************************************************/
void Motor_Set_Val(float left_val,float right_val)
{
		float left_pwm,right_pwm;
		//Ŀ�긳ֵ
		left_incremental_pid.target_val = left_val;
		right_incremental_pid.target_val = right_val;
		//�ٶȻ�����
		left_pwm = Incremental_PID(&left_incremental_pid,Left_Motor.Motorspeed);
		right_pwm = Incremental_PID(&right_incremental_pid,Right_Motor.Motorspeed);
		//pwm
		Motor_Set(left_pwm,right_pwm);
}


/**************************************************************************
@bref: �޷���(pwm)��������˥��������
@para	��float PWM
@return: void
**************************************************************************/
void Motor_Set(float Left_PWM,float Right_PWM)
{
	/*LEFT*/
	
	//T_V<0
	if(Left_PWM <= 0){
		//����
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,MOTOR_ARR);
		//��ֵ
		if(Left_PWM<-MOTOR_ARR)Left_PWM=-MOTOR_ARR;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,MOTOR_ARR+Left_PWM);		
	} 
	
	//T_V>0
	if(Left_PWM >0){
		//����
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_3,MOTOR_ARR);
		//��ֵ
		if(Left_PWM>MOTOR_ARR)Left_PWM=MOTOR_ARR;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_4,MOTOR_ARR-Left_PWM);
	} 

	/*RIGHT*/
	
	//T_V<0
	if(Right_PWM <= 0){
		//����
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,MOTOR_ARR);
		//��ֵ
		if(Right_PWM<-MOTOR_ARR)Right_PWM=-MOTOR_ARR;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,MOTOR_ARR+Right_PWM);		
	} 
	
	//T_V>0
	if(Right_PWM >0){
		//����
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,MOTOR_ARR);
		//��ֵ
		if(Right_PWM>MOTOR_ARR)Right_PWM=MOTOR_ARR;
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,MOTOR_ARR-Right_PWM);
	} 
	

}


//����P ����I ���ƺ���
float PI_realize(Pid * pid,float actual_val)
{
	pid->actual_val = actual_val;//������ʵֵ
	pid->err = pid->target_val - pid->actual_val;//��ǰ���=Ŀ��ֵ-��ʵֵ
	pid->err_sum += pid->err;//����ۼ�ֵ = ��ǰ����ۼƺ�
	//ʹ��PI���� ���=Kp*��ǰ���+Ki*����ۼ�ֵ
	pid->output_val = pid->Kp*pid->err + pid->Ki*pid->err_sum;
	
	return pid->output_val;
}

/**************************************************************************
@bref: λ��ʽPID������
@para	����ǰλ��	pid->actual_dis
@return: ָ���ٶ�	pid->output_val
**************************************************************************/
float PID_realize(Pid * pid,float actual_dis)
{
	pid->actual_dis = actual_dis;//������ʵֵ
	pid->err = pid->target_dis - pid->actual_dis;////��ǰ���=Ŀ��ֵ-��ʵֵ		
	pid->err_sum += pid->err;//����ۼ�ֵ = ��ǰ����ۼƺ�
	//ʹ��PID���� ��� = Kp*��ǰ���  +  Ki*����ۼ�ֵ + Kd*(��ǰ���-�ϴ����)
	pid->output_val = pid->Kp*pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);	
	//�����ϴ����: �����ֵ���ϴ����
	pid->err_last = pid->err;
	
	return pid->output_val;
}


/**************************************************************************
�������ܣ�����PID������
��ڲ�����ʵ��ֵ��Ŀ��ֵ
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]		��Ӧ��ϵ������ʽ-��λ��ʽ��kp->kd ki->kp kd->ki
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
**************************************************************************/
float Incremental_PID(Pid * pid,float actual_val)
{ 		
	//����
	pid->actual_val = actual_val;//������ʵֵ 
	pid->err = pid->target_val - pid->actual_val;//��ǰ���=Ŀ��ֵ-��ʵֵ   
	pid->output_pwm  += (pid->Kd*(pid->err - pid->err_last))               /* �������� */
									 + (pid->Kp * pid->err)                           /* ���ֻ��� */
									 + (pid->Ki*(pid->err - 2*pid->err_last + pid->err_pre));  /* ΢�ֻ��� */ 
    
	pid->err_pre=pid->err_last;                                   /* �������ϴ�ƫ�� */
	pid->err_last=pid->err;	                                    /* ������һ��ƫ�� */

	return pid->output_pwm;                                            /* ������ */
}

	
