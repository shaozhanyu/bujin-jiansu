#include "timer.h"
#include "led.h"
#include "Motor.h"
#include "PWM.h"
//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数

u32 guc_10mscnt, fy_timecnt, fw_timecnt; //guc_10mscnt定时器，用于主函数中不常用的函数的判断



void TIM2_INIT(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
	TIM_DeInit(TIM2);
	
	TIM_TimeBaseStructure.TIM_Period = arr;						//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;				//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);				//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除中断标志，否则会直接进入一次中断

	TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE );
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;			  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);							  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_Cmd(TIM2, ENABLE); //使能TIMx外
	
}


////////////////////////////////////////////////////////////////////
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		
		decel_cnt2++;
		
		if(2==fangwei_xch_dir)
		{
					
				if ((fangwei_yundong_daowei_flag == 0) && (fw_ms_count_temp > 0)) //方位电机向螺杆伸出的方向运动
				{

					fw_currentdeg += FANGWEI_DEG_PER_MS;
				}
				else if ((fangwei_yundong_daowei_flag == 0) && (fw_ms_count_temp < 0))
				{

					fw_currentdeg -= FANGWEI_DEG_PER_MS;
				}			
				
				fangwei_dec_num++;//减速计数，逐渐减小。PWM设定中重装值会逐渐增大，从而速度变慢
				t5_pwm_value = MOTOR_PWM_ARR*(1+fangwei_dec_num/ACCEL_NUM);//实际是N次后最终只减速到一半速度
				
				update_accel_flag =1;//更新PWM标志			
				TIM5_PWM_Init( (t5_pwm_value-1) , 0 , (t5_pwm_value-1)/2);//速度递减，PWM值递增	
				
				if( decel_cnt2 >= ACCEL_NUM)//匀速减速处理
				{
					decel_cnt2 = 0;
					TIM_Cmd(TIM2, DISABLE);  //关闭定时器2
					TIM_Cmd(TIM5, DISABLE);  //关闭定时器5

					update_accel_flag = 0;
					
					fangwei_yundong_daowei_flag = 0;//
					fw_dir = 2;
					fangwei_shezhi_guocheng_flag = 0;

					if(fangwei_xch_dir == 2)//期望换向
					{
						fangwei_xch_dir=0;
						Yuntai_fangwei_shezhi_chenggong_flag = 1;
						Yuntai_jiaodushezhi_flag = 1;
						TIM5_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//直接以设定速度启动	
					}
					
					TIM_Cmd(TIM2, DISABLE);  //关闭定时器2
					
					return;
				}		
				
		}
		
		
	}
	
}

/////////////////////////////////////////////////////////
void TIM1_UP_IRQHandler(void)
{ 
		TIM1->SR&=~(1<<0);
	
		if (guc_10mscnt < 15)
		{
			guc_10mscnt++;
		}	
		
		if ((fuyang_yundong_daowei_flag == 0) && (fy_ms_count_temp > 0))
		{
			fy_currentdeg += FUYANG_DEG_PER_MS;
		}
		else if ((fuyang_yundong_daowei_flag == 0) && (fy_ms_count_temp < 0))
		{
			fy_currentdeg -= FUYANG_DEG_PER_MS;
		}
		
		if ((fangwei_yundong_daowei_flag == 0) && (fw_ms_count_temp > 0)) //方位电机向螺杆伸出的方向运动
		{
			fw_currentdeg += FANGWEI_DEG_PER_MS;
		}
		else if ((fangwei_yundong_daowei_flag == 0) && (fw_ms_count_temp < 0))
		{
			fw_currentdeg -= FANGWEI_DEG_PER_MS;
		}
	

	//////////////////////俯仰电机运动中实时位置更新///////////////////////
		if ((fy_reset_flag == 1) && (fuyang_shezhi_guocheng_flag == 1))//判断当前不是复位运动
		{
			if ((fy_timecnt >= fy_ms_count) && (fy_Init_flag == 1)) //必须在复位完成后执行
			{
				TIM_Cmd(TIM4, DISABLE);

				fuyang_yundong_daowei_flag = 1;//
				fy_timecnt = 0;
				fy_dir = 2;
				fuyang_shezhi_guocheng_flag = 0;
		
			}
			
			fy_timecnt++;
			
		}
/////////////////////方位电机运动中实时位置更新////////////////////////
		if ((fw_reset_flag == 1) && (fangwei_shezhi_guocheng_flag == 1))
		{
			if ((fw_timecnt >= fw_ms_count) && (fw_Init_flag == 1)) //必须在复位完成后执行
			{
				TIM_Cmd(TIM5, DISABLE);
				
				fw_timecnt = 0;
				fw_dir = 2;
				fangwei_yundong_daowei_flag = 1; //到位完成了
				fangwei_shezhi_guocheng_flag = 0;			
				
			}
			
			fw_timecnt++;
			
		}
		
		
		
}

