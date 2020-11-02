
#include "stm32f10x.h"
#include "PWM.h"
#include "Motor.h"
#include "timer.h"
///////////////////驱动器设置3200细分////////////////////////////////////
int  WipeMotorFlag = 0; //期望电机转换为的状态 2 换向 3停止
int  fuyang_xch_dir =0;//俯仰电机切换方向标志
int  fangwei_xch_dir =0;//方位电机切换方向标志
float  SetSpeed_ref = 0.0;
int  update_accel_flag =0;
float  NowSpeed = 0.0; //当前速度

float  accel_value = 240;//加速度值，rps,转/S^2，比如200目标速度，则需要0.5秒
float   accel_num = 40;//加速分段数
float  fuyang_dec_num = 40; //减速计数
float  fangwei_dec_num = 40; //减速计数
float  add_value = 0;
float  dec_value =0;
u16  t4_pwm_value;//pwm4计数值
u16  t5_pwm_value;//pwm5计数值

/////////////////////////////////////////////////////////////

u16 PulsePerRound = 3200;


///////////////加速启动////////////////////////////
int   Mortor_Accel( u32  SetSpeed )
{
	return 1;
	
}


/////////////////俯仰电机减速停止//////////////////////////
int   fuyang_Mortor_Decel( u32  SetSpeed )
{
	
	float accel_time = DEC_TIME_VALUE;//设置减速时间2S
	float tim2_accel = 1000*(accel_time / ACCEL_NUM);//加速总时间分段划分，得到单个加速段时间，单位ms
	
	fuyang_dec_num = 1;//减速计数,逐渐减小
	t4_pwm_value = MOTOR_PWM_ARR*(1+fuyang_dec_num/ACCEL_NUM);
	TIM_Cmd(TIM1, 0);  //关闭1ms定时器
	decel_cnt=0;
	TIM4_PWM_Init( t4_pwm_value-1 , 0 , (t4_pwm_value-1)/2);//速度递减，PWM重装值递增
	
	TIM3_Int_Init( (u16)(tim2_accel*10.0-1) , 7199 , 1);//启动时间片定时器，时间片是固定长的加速总时间/分段片数
	
	TIM_Cmd(TIM4, ENABLE);	
	
	Yuntai_jiaodushezhi_flag = 0;
	
	return 1;
	
}

/////////////////方位电机减速停止//////////////////////////
int   fangwei_Mortor_Decel( u32  SetSpeed )
{
	
	float accel_time = DEC_TIME_VALUE;//设置减速时间2S
	float tim2_accel = 1000*(accel_time / ACCEL_NUM);//加速总时间分段划分，得到单个加速段时间，单位ms
	
	fangwei_dec_num = 1;//减速计数,逐渐减小
	t5_pwm_value = MOTOR_PWM_ARR*(1+fangwei_dec_num/ACCEL_NUM);
	TIM_Cmd(TIM1, 0);  //关闭1ms定时器
	decel_cnt2 = 0;
	
	TIM5_PWM_Init( t5_pwm_value-1 , 0 , (t5_pwm_value-1)/2);//速度递减，PWM值递增
	
	TIM2_INIT( (u16)(tim2_accel*10-1) , 7199 );//启动时间片定时器，时间片是固定长的加速总时间/分段片数
	
	TIM_Cmd(TIM5, ENABLE);	
	Yuntai_jiaodushezhi_flag = 0 ;
	
	
	return 1;
	
}
