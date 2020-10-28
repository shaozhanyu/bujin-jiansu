#include "Encoder.h"

int x_revolutions;
int y_revolutions;

void Encoder1_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
	  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	  TIM_ICInitTypeDef TIM_ICInitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	  
 	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_TIM8,ENABLE);//使能PORTA,PORTE时钟
//	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM8, ENABLE); //时钟使能
	  
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;   //编码器1的两个接口
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC0,1,2,3(光电检测引脚 )
	
	  TIM_TimeBaseStructure.TIM_Period = 64000;
	  TIM_TimeBaseStructure.TIM_Prescaler = 0;
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	  TIM_TimeBaseStructure.TIM_CounterMode  = TIM_CounterMode_Up;
	  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	  TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);
	  
	  TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	  TIM_ICStructInit(&TIM_ICInitStructure);
	  TIM_ICInitStructure.TIM_ICFilter = 10;
	  TIM_ICInit(TIM8,&TIM_ICInitStructure);
		TIM_ARRPreloadConfig(TIM8,ENABLE);
		
		TIM_ClearFlag(TIM8,TIM_FLAG_Update);
		TIM8->CNT = 0;
    TIM_OC1PreloadConfig(TIM8,TIM_OCPreload_Enable);
				
		TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE);     //使能指定的TIM8中断，允许更新中断
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;    //定时器溢出、下溢
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		TIM_Cmd(TIM8,ENABLE);
}

void TIM8_UP_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM8,TIM_IT_Update) != RESET)
	{
		if(TIM8 ->CR1 & 0x0010)      //向下计数溢出
		{
		   x_revolutions--;
		}
		else
		{
		   x_revolutions++;
		}
	  TIM_ClearITPendingBit(TIM8,TIM_IT_Update);	
	}
}
void Encoder2_Init()
{
   GPIO_InitTypeDef GPIO_InitStructure;
	 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	 TIM_ICInitTypeDef TIM_ICInitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;	 
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	 GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	 TIM_TimeBaseStructure.TIM_Period = 40000;
	 TIM_TimeBaseStructure.TIM_Prescaler = 0;
	 TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	 TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	 TIM_ICStructInit(&TIM_ICInitStructure);
	 TIM_ICInitStructure.TIM_ICFilter = 0;
	 TIM_ICInit(TIM3,&TIM_ICInitStructure);
	 TIM_ARRPreloadConfig(TIM3,ENABLE);
	 
	 TIM_ClearFlag(TIM3,TIM_FLAG_Update);
   TIM3->CNT = 0;
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);     //使能指定的TIM8中断，允许更新中断
	
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;    //定时器溢出、下溢
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	 
	 
	 
   TIM_Cmd(TIM3,ENABLE);	 
	
}
void TIM3_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET)
	{
		if(TIM3 ->CR1 & 0x0010)      //向下计数溢出
		{
		   y_revolutions--;
		}
		else
		{
		   y_revolutions++;
//			 if(y_revolutions )
//			 {
//			    y_revolutions_Init
//			 }
		}
	   TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}







