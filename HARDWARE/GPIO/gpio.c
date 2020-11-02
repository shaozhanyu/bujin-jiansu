#include"gpio.h"

void GPIO_Jiance_Init()
{
   GPIO_InitTypeDef GPIO_InitStructure;
	
	 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOC,ENABLE);//使能PORTA,PORTE时钟
	
	 GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_12|GPIO_Pin_8;//KEY0-KEY2
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
 	 GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC0,1,2,3(光电检测引脚 )
	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOD,&GPIO_InitStructure);
	 
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB,&GPIO_InitStructure);
	 	 
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOC,&GPIO_InitStructure);
	 
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOC,&GPIO_InitStructure);
	 
	 MAX3221_EN = 0;
	 
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB,&GPIO_InitStructure);
	 
	 MAX3221_EN1 = 0;

}


