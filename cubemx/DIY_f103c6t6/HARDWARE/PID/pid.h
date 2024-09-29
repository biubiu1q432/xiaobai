#ifndef __PID_H
#define __PID_H

//pid
typedef struct 
{
	//�ٶ�
	float target_val;//Ŀ��ֵ
	float actual_val;//ʵ��ֵ
	float output_pwm;//���ֵ
	
	//λ��
	float target_dis;
	float actual_dis;
	float output_val;
	float err;//��ǰƫ��
	
	float err_last;//�ϴ�ƫ��
	float err_pre;//���ϴ�ƫ��
	float err_sum;//����ۼ�ֵ
	float Kp,Ki,Kd;//���������֣�΢��ϵ��
		 
} Pid;

//ʵʱ
typedef struct 
{
	float EncodeCount;
	float Distance;
	float Motorspeed;
} Motor_Stat;



//������ʼ��
void PID_init(void);
//λ�û�
float PI_realize(Pid * pid,float actual_val);
float PID_realize(Pid * pid,float actual_val);
//����ʽ
float Incremental_PID(Pid * pid,float actual_val);
//�ٶȻ�
void motorPidSetSpeed(float MotorSetSpeed);
//�޷���
void Motor_Set(float Left_PWM,float Right_PWM);


#endif


