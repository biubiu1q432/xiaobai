#ifndef __PID_H
#define __PID_H

//pid������
typedef struct 
{
	float target_val;
	float actual_val;//ʵ��ֵ
	float output_pwm;//���ֵ
	
	float target_dis;
	float actual_dis;
	float output_val;
	float err;//��ǰƫ��
	
	float err_last;//�ϴ�ƫ��
	float err_pre;//���ϴ�ƫ��
	float err_sum;//����ۼ�ֵ
	float Kp,Ki,Kd;//���������֣�΢��ϵ��
		 
} Pid;

//ʵʱ����
typedef struct 
{
	float EncodeCount;
	float Distance;
	float Motorspeed;
} Motor_Stat;


//������ʼ��
void PID_init(void);
//λ��ʽ
float PI_realize(Pid * pid,float actual_val);
float PID_realize(Pid * pid,float actual_val);
//����ʽ
float Incremental_PID(Pid * pid,float actual_val);
//�޷�ʽ
void Motor_Set(float Left_PWM,float Right_PWM);

//�ٶȻ�
void Motor_Set_Val(float left_val,float right_val);
//λ�û�
void Motor_Set_Dis(float target_dis);
#endif


