
#include "stm32f10x.h"
#include "PWM.h"

int  decel_cnt=0 , decel_cnt2 =0;
////////////////////定时器1初始化////////////////////////////////
void TIM1_Int_Init(u16 arr,u16 psc , FunctionalState NewState)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //时钟使能
	TIM_DeInit(TIM1);
	//定时器TIM1初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //重复计数器计数器清零  
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //注意如果不先清除一下标志位，启动定时器会立即进入一次中断
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

	TIM_Cmd(TIM1, NewState);  //是否使能定时器
						 
}

//通用定时器3中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
// NewState	:是否使能定时器，开始计数
//这里使用的是定时器3!
///////////////////定时器3初始化////////////////////////////////
void TIM3_Int_Init(u16 arr,u16 psc , FunctionalState NewState)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
	TIM_DeInit(TIM3);
	//定时器TIM2初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除更新中断请求位,注意如果不先清除一下标志位，启动定时器会立即进入一次中断
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

	TIM_Cmd(TIM3, NewState);  //是否使能定时器
						 
}

/////////////////////////////////////
/***函数名 TIM4_PWM_Init
		功能：定时器4通道1PWM初始化
		参数：period  重装值
					psc   分频值
					HalfPeriod  比较值
		返回值：无
*/
void TIM4_PWM_Init(u32  Period , u16 psc , u32 HalfPeriod )
{
//	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitType;
	TIM_OCInitTypeDef TIM_OCInitType;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIM_TimeBaseInitType.TIM_ClockDivision = TIM_CKD_DIV1;//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitType.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInitType.TIM_Prescaler   = psc; //分频值
	TIM_TimeBaseInitType.TIM_Period      = Period; //更新值
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitType);

	TIM_OCInitType.TIM_OCIdleState = TIM_OCIdleState_Set; //空闲时状态为set 1 PWM时不需要设置
	TIM_OCInitType.TIM_OCMode = TIM_OCMode_PWM1;	//	
	TIM_OCInitType.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性高
	TIM_OCInitType.TIM_OutputNState = TIM_OutputNState_Disable; //互补输出比较状态禁止 中容量产品TIM2 TIM3 TIM4没有互补脚
	TIM_OCInitType.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能 
	TIM_OCInitType.TIM_Pulse = HalfPeriod; //待装入捕获比较寄存器中的值
	TIM_OC1Init(TIM4,&TIM_OCInitType); //OC1初始化通道1  OC1
	
	TIM_OC1PreloadConfig(TIM4 , TIM_OCPreload_Enable); //使能定时器TIM1 在CCR1上的预装载寄存器
	TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 TIM4 在 ARR 上的预装载寄存器

	//TIM_Cmd(TIM4, ENABLE);  //是否使能定时器

	
}


/////////////////////////////////////
/***函数名 TIM4_PWM_Init
		功能：定时器4通道1PWM初始化
		参数：period  重装值
					psc   分频值
					HalfPeriod  比较值
		返回值：无
*/
void TIM5_PWM_Init(u32  Period , u16 psc , u32 HalfPeriod )
{
//	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitType;
	TIM_OCInitTypeDef TIM_OCInitType;
	GPIO_InitTypeDef GPIO_InitStructure;  

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	TIM_DeInit(TIM5);
	TIM_TimeBaseInitType.TIM_ClockDivision = TIM_CKD_DIV1;//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitType.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
	TIM_TimeBaseInitType.TIM_Prescaler   = psc; //分频值
	TIM_TimeBaseInitType.TIM_Period      = Period; //更新值
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitType);

	TIM_OCInitType.TIM_OCIdleState = TIM_OCIdleState_Set; //空闲时状态为set 1 PWM时不需要设置
	TIM_OCInitType.TIM_OCMode = TIM_OCMode_PWM1;	//	
	TIM_OCInitType.TIM_OCPolarity = TIM_OCPolarity_High;//输出极性高
	TIM_OCInitType.TIM_OutputNState = TIM_OutputNState_Disable; //互补输出比较状态禁止 中容量产品TIM2 TIM3 TIM4没有互补脚
	TIM_OCInitType.TIM_OutputState = TIM_OutputState_Enable;//比较输出使能 
	TIM_OCInitType.TIM_Pulse = HalfPeriod; //待装入捕获比较寄存器中的值
	TIM_OC1Init(TIM5,&TIM_OCInitType); //OC1初始化通道1  OC1
	
	TIM_OC1PreloadConfig(TIM5 , TIM_OCPreload_Enable); //使能定时器TIM1 在CCR1上的预装载寄存器
	TIM_ARRPreloadConfig(TIM5, ENABLE);//ARPE使能 TIM5 在 ARR 上的预装载寄存器
	
//	TIM_Cmd(TIM1, ENABLE);  //是否使能定时器
	
}

int  accel_cnt =0;

//////////////定时器3中断/////////////////////
void TIM3_IRQHandler(void)
{ 

		TIM3->SR&=~(1<<0);
	
		decel_cnt++ ;
		if( 2==fuyang_xch_dir )
		{
		
				if ((fuyang_yundong_daowei_flag == 0) && (fy_ms_count_temp > 0))
				{

					fy_currentdeg += FUYANG_DEG_PER_MS;
				}
				else if ((fuyang_yundong_daowei_flag == 0) && (fy_ms_count_temp < 0))
				{

					fy_currentdeg -= FUYANG_DEG_PER_MS;
				}					
				
				fuyang_dec_num++;//减速计数，逐渐减小。PWM设定中重装值会逐渐增大，从而速度变慢
				t4_pwm_value = MOTOR_PWM_ARR*(1+fuyang_dec_num/ACCEL_NUM);
				
				update_accel_flag =1;//更新PWM标志
				TIM4_PWM_Init( (t4_pwm_value-1) , 0 , (t4_pwm_value-1)/2);//速度递减，PWM重装值递增		
		
				if( decel_cnt >= ACCEL_NUM)//匀速减速处理
				{
						decel_cnt = 0;
	//					TIM_Cmd(TIM1, 0);  //关闭定时器1
						TIM_Cmd(TIM3, 0);  //关闭定时器3
						TIM_Cmd(TIM4, 0);  //关闭定时器4
						update_accel_flag = 0;
						
						fuyang_yundong_daowei_flag = 0;//
						fy_dir = 2;

						fuyang_shezhi_guocheng_flag = 0;		
						
						TIM4_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//直接以设定速度启动		
						if(fuyang_xch_dir == 2)//期望换向
						{
							fuyang_xch_dir=0;
							Yuntai_fuyang_shezhi_chenggong_flag = 1;
							Yuntai_jiaodushezhi_flag = 1;
						}
						TIM_Cmd(TIM3, 0);  //关闭定时器3
						return;
				}
		
		}
		
}





