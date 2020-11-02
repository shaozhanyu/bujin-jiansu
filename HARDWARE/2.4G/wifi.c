#include "wifi.h"

 u8 Uart1RxTemp[20];
 u8 Uart1Cont;
 u8 Uart1Countmax;
 u8 Uart2Ok;
 short shuipingfuyangjiao;
 

void UARST2_Init(u32 bound)
{
	 GPIO_InitTypeDef GPIO_InitStructure;
	 USART_InitTypeDef USART_InitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);	//ʹ��USART1��GPIOAʱ��
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
     
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
		GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.2
	 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA10
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
		GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10 
	
	  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//��ռ���ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;		//�����ȼ�3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
		NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
		USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

		USART_Init(USART2, &USART_InitStructure); //��ʼ������1
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
		USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���1 	
}

void USART2_IRQHandler(void)                	//����1�жϷ������
{

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		USART_ClearFlag(USART2, USART_IT_RXNE);
		Uart1RxTemp[Uart1Cont] =USART_ReceiveData(USART2);	//��ȡ���յ�������
		if(Uart1RxTemp[0] == USART2_Start)
		{
			Uart1Cont++;
			Uart1Countmax = Uart1RxTemp[1];           //��ʾ���ݳ���
		}
		else
		{
			Uart1Cont = 0;
		} 
		if((Uart1Cont>=(Uart1Countmax +1))&&(Uart1Countmax != 0))
		{
			Uart1Cont = 0;					 
			if((Uart1Countmax == 0x05)&&(Uart1RxTemp[5] == (u8)(Uart1RxTemp[1]+Uart1RxTemp[2]+Uart1RxTemp[3]+Uart1RxTemp[4])))
			{
				Uart2Ok =1;
			}  
		if((Uart1Countmax == 0x07)&&(Uart1RxTemp[7] == (u8)(Uart1RxTemp[1]+Uart1RxTemp[2]+Uart1RxTemp[3]+Uart1RxTemp[4]+Uart1RxTemp[5]+Uart1RxTemp[6])))	
		{
				Uart2Ok =1;
			}
		if((Uart1Countmax == 0x0D)&&(Uart1RxTemp[13] == (u8)(Uart1RxTemp[1]+Uart1RxTemp[2]+Uart1RxTemp[3]+Uart1RxTemp[4]+Uart1RxTemp[5]+Uart1RxTemp[6]
					+Uart1RxTemp[7]+Uart1RxTemp[8]+Uart1RxTemp[9]+Uart1RxTemp[10]+Uart1RxTemp[11]+Uart1RxTemp[12])))	
		{
				Uart2Ok =1;
			}			 
		}
	}		 
}


void RS232_Send_Data(u8 *buf,u8 len)
{
  	u8 t;

	for(t = 0;t < len;t++)
	{
	   while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == RESET);
		 USART_SendData(USART2,buf[t]);
	}
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET);

}

short dianziluopan_Anal_Data( )
{

  if((Uart1RxTemp[4]&0xf0) == 0)
	{
	    shuipingfuyangjiao = (Uart1RxTemp[4]&0x0f)*10000+((Uart1RxTemp[4]&0xf0)>>4)*100000+(Uart1RxTemp[5]&0x0f)*100+((Uart1RxTemp[5]&0xf0)>>4)*1000+(Uart1RxTemp[6]&0x0f)+((Uart1RxTemp[6]&0xf0)>>4)*10;
	}
	if(((Uart1RxTemp[4]&0xf0)>>4) == 1)
	{
	  shuipingfuyangjiao = (Uart1RxTemp[4]&0x0f)*10000+(Uart1RxTemp[5]&0x0f)*100+((Uart1RxTemp[5]&0xf0)>>4)*1000
								+(Uart1RxTemp[6]&0x0f)+((Uart1RxTemp[6]&0xf0)>>4)*10;
		shuipingfuyangjiao = 0 - shuipingfuyangjiao;
	}
	 
	return 0;
	 
}
	









