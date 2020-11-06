#include "sys.h"
#include "usart.h"
#include "gpio.h"
#include "AMIS30543.h"
#include "math.h"
#include "stdio.h"
#include "Motor.h"
//////////////////////////////////////////////////////////////////////////////////
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos ʹ��
#endif
//////////////////////////////////////////////////////////////////////////////////
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32������
//����1��ʼ��
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/8/18
//�汾��V1.5
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//��׼����Ҫ��֧�ֺ���
struct __FILE
{
	int handle;
};

FILE __stdout;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ
int _sys_exit(int x)
{
	x = x;
	return x;
}
//�ض���fputc����
int fputc(int ch, FILE *f)
{
	while ((USART1->SR & 0X40) == 0)
		; //ѭ������,ֱ���������
	USART1->DR = (u8)ch;
	return ch;
}
#endif


								 // �������ԭ���ʱ������¼��ֵ
short crc_check;				 //����crc�Ľ��
short crc_result1;				 //�洢��crcֵ

u8 Res;
u8 last_Res;
u8 RS422_receive_str[30]; //���յ�������
u8 RS232_receive_str[30]; //�ظ�
u8 rx_buf[50];
long rx_cnt=0;
long rx_time=0;
u8 RS422_byte_count;
u8 last_RS422_byte_count;
u8 RS232_receive_flag;
u8 Yuntai_ID_flag;						 //��̨ID��ѯ״̬��־
u8 Yuntai_zhuangtai_flag;				 //��̨״̬��ѯ��־
u8 Yuntai_kongzhi_flag;					 //��̨���Ʊ�־
u8 Yuntai_fuwei_flag;					 //��̨��λ��־
u8 Yuntai_jiaodushezhi_flag;			 //��̨�Ƕ����ñ�־
u8 Yuntai_chushishezhi_flag;			 //��̨��ʼ���ñ�־��Ŀǰû����
u8 Yuntai_yingdafangshi_flag;			 //��̨Ӧ�����ñ�־
u8 Yuntai_tingzhi_flag;					 //��ֹ̨ͣ���ñ�־
u8 Yuntai_fangwei_shezhi_chenggong_flag; //��̨���óɹ���־,��ʾ��λ������������Ч������ִ��
u8 Yuntai_fuyang_shezhi_chenggong_flag;

short fw_TargetDeg;		 //��λĿ��Ƕ�
short fy_TargetDeg;		 //����Ŀ��Ƕ�



void uart_init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE); //ʹ��USART1��GPIOAʱ��

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��GPIOA.9

	//USART1_RX	  GPIOA.10��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  //PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //��ʼ��GPIOA.10

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);							  //����ָ���Ĳ�����ʼ��VIC�Ĵ���

	//USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;										//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;								//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//�շ�ģʽ

	USART_Init(USART1, &USART_InitStructure);	   //��ʼ������1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);					   //ʹ�ܴ���1
}

////////////////////////////////////////////////////////////////
void USART1_IRQHandler(void) //����1�жϷ������
{

#if SYSTEM_SUPPORT_OS //���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();
#endif
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
		{
			Res = USART_ReceiveData(USART1); //��ȡ���յ�������
			RS422_receive_str[RS422_byte_count++]=Res;
			rx_time =0;
		}
	}
#if SYSTEM_SUPPORT_OS //���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();
#endif
}

////////////////////////////////////////////////////
void RS485_Send_Data(u8 *buf, u8 len)
{
	uint32_t	t;
	for(t=0;t<len;t++)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=1);//�ȴ����ͽ���
		USART_SendData(USART1, buf[t]);//�򴮿�1��������
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=1);//�ȴ����ͽ���
	}
	
}
//////////////////////////////////////////////////////////////
u16 cal_crc(unsigned char *ptr, unsigned char len)
{
	unsigned char i;
	u16 crc = 0;

	while (len-- != 0)
	{
		for (i = 0x80; i != 0; i >>= 1)
		{
			if ((crc & 0x8000) != 0)
			{
				crc <<= 1;
				crc ^= (crc_mul);
			}
			else
			{
				crc <<= 1;
			}
			if ((*ptr & i) != 0)
			{
				crc ^= (crc_mul);
			}
		}
		ptr++;
	}
	return (crc);
}
///////////////////////��λ���������/////////////////////////////////////////
void RS232_data_Anal()
{
	u8 flag = 0;
	u8 i;
	u8 t = 0;

	if (flag == 0)
	{
		for (i = 0; i < last_RS422_byte_count; i++)
		{
			if ((rx_buf[i] == 0xdb) && (rx_buf[i + 1] == 0xdc))//�ж��м�������0XC0��ת�塣0xdb��0xdc���ϱ�ʾһ��0xc0
			{
				i++;
				RS232_receive_str[t] = 0xc0;
				t++;
			}
			else if ((rx_buf[i] == 0xdb) && (rx_buf[i + 1] == 0xdd))
			{
				i++;
				RS232_receive_str[t] = 0xdb;
				t++;
			}
			else
			{
				RS232_receive_str[t] = rx_buf[i];
				t++;
			}
		}
		crc_check = cal_crc(RS232_receive_str, (t - 2)); 
		crc_result1 = (short)((RS232_receive_str[t - 2] << 8) | RS232_receive_str[t - 1]);

		if (crc_check == crc_result1)
		{
			flag++;
		}
		else
		{
			flag = 0;
		}
	}
	if (flag == 1)//��������ɹ�
	{
		flag=0;

		switch(RS232_receive_str[0])
		{
			case 1:Yuntai_ID_flag = 1;
			break;
			case 2:Yuntai_zhuangtai_flag = 1;
			break;
			case 3:
				Yuntai_kongzhi_flag = 1;
				if (RS232_receive_str[3] == 0x01)
				{
					Yuntai_fuwei_flag = 1;
				}
				if (RS232_receive_str[3] == 0x00)
				{
					if((0==fangwei_xch_dir) &&(0==fuyang_xch_dir) )
						Yuntai_tingzhi_flag = 1;
				}
			break;
			case 4:
				if ( (fy_reset_flag == 1) && (fw_reset_flag == 1)) //�жϵ�ǰ���Ǹ�λ��
				{
					
					Yuntai_jiaodushezhi_flag = 1; //��־����̨���յ���λ�������ýǶȱ�־
					fy_TargetDeg = (short)(RS232_receive_str[6] << 8 | RS232_receive_str[7]);
					Yuntai_yingdafangshi_flag = RS232_receive_str[8];
					fw_TargetDeg = (short)(RS232_receive_str[4] << 8 | RS232_receive_str[5]);
					if ((fw_TargetDeg >= -18000) && (fw_TargetDeg <= 18000))
					{
						Yuntai_fangwei_shezhi_chenggong_flag = 1;
					}
					else
					{
						Yuntai_fangwei_shezhi_chenggong_flag = 0;
					}
					if ((fy_TargetDeg >= -300) && (fy_TargetDeg <= 500))
					{
						Yuntai_fuyang_shezhi_chenggong_flag = 1;
					}
					else
					{
						Yuntai_fuyang_shezhi_chenggong_flag = 0;
					}

					if ((Yuntai_fangwei_shezhi_chenggong_flag == 1) && (Yuntai_fuyang_shezhi_chenggong_flag == 1))//�ж������Ƕ����ö��Ϸ�
					{
						calc_deg_fun(fw_TargetDeg , fy_TargetDeg , t);//�������õĽǶȼ����˶�����

					}
					else if ((Yuntai_fangwei_shezhi_chenggong_flag == 0) || (Yuntai_fuyang_shezhi_chenggong_flag == 0))
					{
						fy_TargetDeg = gul_fuyang_jiaodu;
						fw_TargetDeg = gul_fangwei_jiaodu;
					}
					
				}
			break;
			case 5:Yuntai_chushishezhi_flag = 1;
			break;
			default:
			break;

		}
				
	}
	
}

////////////////////////���ݽǶȼ����˶�����/////////////////////////////////////////////
int  calc_deg_fun( short fw_deg , short fy_deg , int len)
{
	int i;
	for (i = 0; i < len; i++)
	{
		AT24c256_storage[i] = RS232_receive_str[i];
	}
	///////////////////�Ӹ����Ƕȼ����Ƹ˵���˶��յ㳤��/////////////////////
	if(fy_TargetDeg - fy_currentdeg >= 0.002)//�ж��������˶�
	{
		if(fy_dir ==0 )//�жϵ�ǰ״̬�Ƿ�ת�Ǿ�ֹ�У�������Ҫ��ת��˵����Ҫ���ٻ���
		{
			if( fy_currentdeg > 5.0)//�жϷ������ֹͣ�����㹻��������Ϊ����ʱ��̶���һ��û�о����ײ��
			{
				fuyang_xch_dir =2;//���û����־2
				Yuntai_fuyang_shezhi_chenggong_flag=2;//�޸����ýǶȱ�־����ʱת�����ֹͣ���Ȳ�ִ��ʵ�ʽǶ��˶�
				Yuntai_jiaodushezhi_flag =3;
			}
			else
			{
				Yuntai_fuyang_shezhi_chenggong_flag=0;								
			}
		}
	}
	else  if(fy_TargetDeg - fy_currentdeg < -0.002)//�жϷ����˶�
	{
		if(fy_dir ==1 )//�жϵ�ǰ״̬����ת�Ǿ�ֹ�У�������Ҫ��ת��˵����Ҫ���ٻ���
		{
			if( fy_currentdeg < 33)//�ж��������ֹͣ�����㹻��������Ϊ����ʱ��̶���һ��û�о����ײ��
			{
				fuyang_xch_dir =2;//���û����־2
			
				Yuntai_fuyang_shezhi_chenggong_flag=2;//�޸����ýǶȱ�־����ʱת�����ֹͣ���Ȳ�ִ��ʵ�ʽǶ��˶�
				Yuntai_jiaodushezhi_flag =3;
			}
			else
			{
				Yuntai_fuyang_shezhi_chenggong_flag=0;								
			}
		}
	}	
	
	if( fw_TargetDeg - fw_currentdeg >= 0.002)//�ж��������˶�
	{
		if(fw_dir ==0 )//�жϵ�ǰ״̬�Ƿ�ת�Ǿ�ֹ�У�������Ҫ��ת��˵����Ҫ���ٻ���
		{
			if( fw_currentdeg < 5 )//�жϷ������ֹͣ�����㹻��������Ϊ����ʱ��̶���һ��û�о����ײ��
			{
				Yuntai_fangwei_shezhi_chenggong_flag=2;//�޸����ýǶȱ�־����ʱת�����ֹͣ���Ȳ�ִ��ʵ�ʽǶ��˶�
				Yuntai_jiaodushezhi_flag =4;
				fangwei_xch_dir =2;//���û����־2
			}
			else
			{
				Yuntai_fangwei_shezhi_chenggong_flag = 0;
			}
			
		}
	}
	else  if(fw_TargetDeg - fw_currentdeg < -0.002)//�жϷ����˶�
	{
		if(fw_dir ==1 )//�жϵ�ǰ״̬����ת�Ǿ�ֹ�У�������Ҫ��ת��˵����Ҫ���ٻ���
		{
			if( fw_currentdeg >5.0)//�ж��������ֹͣ�����㹻��������Ϊ����ʱ��̶���һ��û�о����ײ��
			{
				fangwei_xch_dir =2;//���û����־2
				Yuntai_fangwei_shezhi_chenggong_flag=2;//�޸����ýǶȱ�־����ʱת�����ֹͣ���Ȳ�ִ��ʵ�ʽǶ��˶�
				Yuntai_jiaodushezhi_flag =4;
			}
			else
			{
				Yuntai_fangwei_shezhi_chenggong_flag=0;
			}
		}
	}	
	
	return 0;
	
}

////////////////////////���͸���λ������/////////////////////////////////////////////
void RS422_Send_Data(u8 *buf, u8 len)
{
	u8 t;
	for (t = 0; t < len; t++)
	{
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
			;
		USART_SendData(USART1, buf[t]);
	}
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		;
	FORCEON = 0;
}





