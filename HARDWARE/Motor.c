
#include "stm32f10x.h"
#include "PWM.h"
#include "Motor.h"
#include "timer.h"
///////////////////����������3200ϸ��////////////////////////////////////
int  WipeMotorFlag = 0; //�������ת��Ϊ��״̬ 2 ���� 3ֹͣ
int  fuyang_xch_dir =0;//��������л������־
int  fangwei_xch_dir =0;//��λ����л������־
float  SetSpeed_ref = 0.0;
int  update_accel_flag =0;
float  NowSpeed = 0.0; //��ǰ�ٶ�

float  accel_value = 240;//���ٶ�ֵ��rps,ת/S^2������200Ŀ���ٶȣ�����Ҫ0.5��
float   accel_num = 40;//���ٷֶ���
float  fuyang_dec_num = 40; //���ټ���
float  fangwei_dec_num = 40; //���ټ���
float  add_value = 0;
float  dec_value =0;
u16  t4_pwm_value;//pwm4����ֵ
u16  t5_pwm_value;//pwm5����ֵ

/////////////////////////////////////////////////////////////

u16 PulsePerRound = 3200;


///////////////��������////////////////////////////
int   Mortor_Accel( u32  SetSpeed )
{
	return 1;
	
}


/////////////////�����������ֹͣ//////////////////////////
int   fuyang_Mortor_Decel( u32  SetSpeed )
{
	
	float accel_time = DEC_TIME_VALUE;//���ü���ʱ��2S
	float tim2_accel = 1000*(accel_time / ACCEL_NUM);//������ʱ��ֶλ��֣��õ��������ٶ�ʱ�䣬��λms
	
	fuyang_dec_num = 1;//���ټ���,�𽥼�С
	t4_pwm_value = MOTOR_PWM_ARR*(1+fuyang_dec_num/ACCEL_NUM);
	TIM_Cmd(TIM1, 0);  //�ر�1ms��ʱ��
	decel_cnt=0;
	TIM4_PWM_Init( t4_pwm_value-1 , 0 , (t4_pwm_value-1)/2);//�ٶȵݼ���PWM��װֵ����
	
	TIM3_Int_Init( (u16)(tim2_accel*10.0-1) , 7199 , 1);//����ʱ��Ƭ��ʱ����ʱ��Ƭ�ǹ̶����ļ�����ʱ��/�ֶ�Ƭ��
	
	TIM_Cmd(TIM4, ENABLE);	
	
	Yuntai_jiaodushezhi_flag = 0;
	
	return 1;
	
}

/////////////////��λ�������ֹͣ//////////////////////////
int   fangwei_Mortor_Decel( u32  SetSpeed )
{
	
	float accel_time = DEC_TIME_VALUE;//���ü���ʱ��2S
	float tim2_accel = 1000*(accel_time / ACCEL_NUM);//������ʱ��ֶλ��֣��õ��������ٶ�ʱ�䣬��λms
	
	fangwei_dec_num = 1;//���ټ���,�𽥼�С
	t5_pwm_value = MOTOR_PWM_ARR*(1+fangwei_dec_num/ACCEL_NUM);
	TIM_Cmd(TIM1, 0);  //�ر�1ms��ʱ��
	decel_cnt2 = 0;
	
	TIM5_PWM_Init( t5_pwm_value-1 , 0 , (t5_pwm_value-1)/2);//�ٶȵݼ���PWMֵ����
	
	TIM2_INIT( (u16)(tim2_accel*10-1) , 7199 );//����ʱ��Ƭ��ʱ����ʱ��Ƭ�ǹ̶����ļ�����ʱ��/�ֶ�Ƭ��
	
	TIM_Cmd(TIM5, ENABLE);	
	Yuntai_jiaodushezhi_flag = 0 ;
	
	
	return 1;
	
}
