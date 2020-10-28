#include "sys.h"
#include "usart.h"
#include "gpio.h"
#include "AMIS30543.h"
#include "math.h"
#include "stdio.h"
#include "Motor.h"
//////////////////////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h" //ucos 使用
#endif
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE
{
	int handle;
};

FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
int _sys_exit(int x)
{
	x = x;
	return x;
}
//重定义fputc函数
int fputc(int ch, FILE *f)
{
	while ((USART1->SR & 0X40) == 0)
		; //循环发送,直到发送完毕
	USART1->DR = (u8)ch;
	return ch;
}
#endif

#if EN_USART1_RX				 //如果使能了接收
								 // 电机到达原点的时候电机记录的值
short crc_check;				 //计算crc的结果
short crc_result1;				 //存储的crc值
extern short gul_fuyang_jiaodu;	 //俯仰角度值
extern short gul_fangwei_jiaodu; //方位角度值
u8 Res;
u8 last_Res;
u8 RS422_receive_str[30]; //接收到的数据
u8 RS232_receive_str[30]; //回复
u8 RS422_byte_count;
u8 last_RS422_byte_count;
u8 RS232_receive_flag;
u8 Yuntai_ID_flag;						 //云台ID问询状态标志
u8 Yuntai_zhuangtai_flag;				 //云台状态查询标志
u8 Yuntai_kongzhi_flag;					 //云台控制标志
u8 Yuntai_fuwei_flag;					 //云台复位标志
u8 Yuntai_jiaodushezhi_flag;			 //云台角度设置标志
u8 Yuntai_chushishezhi_flag;			 //云台初始设置标志，目前没有用
u8 Yuntai_yingdafangshi_flag;			 //云台应答设置标志
u8 Yuntai_tingzhi_flag;					 //云台停止设置标志
u8 Yuntai_fangwei_shezhi_chenggong_flag; //云台设置成功标志,表示上位机设置数据有效，可以执行
u8 Yuntai_fuyang_shezhi_chenggong_flag;
double fangwei_TargetPos;		 //方位目标位置
double fuyang_TargetPos;		 //俯仰目标位置
short fangwei_Targetjiaodu;		 //方位目标角度
double fangwei_Targethudu;		 //方位目标弧度
short fuyang_Targetjiaodu;		 //俯仰目标角度
double fuyang_Targethudu;		 //俯仰目标弧度
extern unsigned char guc_StepM1; //细分数
extern float gf_MotorStep;		 //电机单步步长
double fangwei_TargetPos_temp;	 //方位目标角度中间值
double fuyang_TargettPos_temp;	 //俯仰目标角度中间值
extern u8 AT24c256_storage[30];	 //要存储的数据
extern char In_place_flag1;
extern char In_place_flag;
extern u8 fangwei_yundong_daowei_flag;
extern u8 fuyang_yundong_daowei_flag;
extern int gul_Targettimeout_temp;
extern u32 guc_timejishi;


void uart_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE); //使能USART1，GPIOA时钟

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化GPIOA.9

	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			  //PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);				  //初始化GPIOA.10

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		  //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);							  //根据指定的参数初始化VIC寄存器

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;										//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;								//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					//收发模式

	USART_Init(USART1, &USART_InitStructure);	   //初始化串口1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启串口接受中断
	USART_Cmd(USART1, ENABLE);					   //使能串口1
}

void USART1_IRQHandler(void) //串口1中断服务程序
{

#if SYSTEM_SUPPORT_OS //如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();
#endif
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
		{
			Res = USART_ReceiveData(USART1); //读取接收到的数据
		}
		if ((Res == 0xc0) && ((last_Res == 0) || (last_Res == 0xC0)))
		{
			RS422_byte_count = 0;
		}
		else if (Res != 0xc0)
		{
			RS422_receive_str[RS422_byte_count] = Res;
			RS422_byte_count++;
		}
		else if ((Res == 0xc0) && (RS422_byte_count != 0) && (RS422_byte_count != 1))
		{
			last_RS422_byte_count = RS422_byte_count;
			RS422_byte_count = 0;
			RS232_receive_flag = 1;
			//				  RS232_data_Anal();
		}
		last_Res = Res;
	}
#if SYSTEM_SUPPORT_OS //如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();
#endif
}

void RS485_Send_Data(u8 *buf, u8 len)
{
	uint32_t	t;
	for(t=0;t<len;t++)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=1);//等待发送结束
		USART_SendData(USART1, buf[t]);//向串口1发送数据
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=1);//等待发送结束
	}
	
}

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

void RS232_data_Anal()
{
	u8 flag = 0;
	u8 i;
	u8 t = 0;

	if (flag == 0)
	{
		for (i = 0; i < last_RS422_byte_count; i++)
		{
			if ((RS422_receive_str[i] == 0xdb) && (RS422_receive_str[i + 1] == 0xdc))
			{
				i++;
				RS232_receive_str[t] = 0xc0;
				t++;
			}
			else if ((RS422_receive_str[i] == 0xdb) && (RS422_receive_str[i + 1] == 0xdd))
			{
				i++;
				RS232_receive_str[t] = 0xdb;
				t++;
			}
			else
			{
				RS232_receive_str[t] = RS422_receive_str[i];
				t++;
			}
		}
		crc_check = cal_crc(RS232_receive_str, (t - 2)); //RS232_receive_str数组是完全没有转义之前的数组
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
	if (flag == 1)
	{
		flag=0;
		if (RS422_receive_str[0] == 0x01)
		{
			Yuntai_ID_flag = 1;
		}
		if (RS422_receive_str[0] == 0x02)
		{
			Yuntai_zhuangtai_flag = 1;
		}
		if (RS422_receive_str[0] == 0x03)
		{
			Yuntai_kongzhi_flag = 1;
			if (RS422_receive_str[3] == 0x01)
			{
				Yuntai_fuwei_flag = 1;
			}
			if (RS422_receive_str[3] == 0x00)
			{
				if((0==fangwei_xch_dir) &&(0==fuyang_xch_dir) )
					Yuntai_tingzhi_flag = 1;
			}
		}
		if ((RS422_receive_str[0] == 0x04) && (In_place_flag1 == 1) && (In_place_flag == 1)) //&& (gb_SHUN_NI == 2) && (gb_SHUN_NI1 == 2))
		{
			Yuntai_jiaodushezhi_flag = 1; //标志着云台接收到方位俯仰设置角度标志
			fuyang_Targetjiaodu = (short)(RS232_receive_str[6] << 8 | RS232_receive_str[7]);
			Yuntai_yingdafangshi_flag = RS232_receive_str[8];
			fangwei_Targetjiaodu = (short)(RS232_receive_str[4] << 8 | RS232_receive_str[5]);
			if ((fangwei_Targetjiaodu >= -1500) && (fangwei_Targetjiaodu <= 1500))
			{
				Yuntai_fangwei_shezhi_chenggong_flag = 1;
			}
			else
			{
				Yuntai_fangwei_shezhi_chenggong_flag = 0;
			}
			if ((fuyang_Targetjiaodu >= -300) && (fuyang_Targetjiaodu <= 500))
			{
				Yuntai_fuyang_shezhi_chenggong_flag = 1;
			}
			else
			{
				Yuntai_fuyang_shezhi_chenggong_flag = 0;
			}
			if ((Yuntai_fangwei_shezhi_chenggong_flag == 1) && (Yuntai_fuyang_shezhi_chenggong_flag == 1))//判断两个角度设置都合法
			{
				for (i = 0; i < t; i++)
				{
					AT24c256_storage[i] = RS232_receive_str[i];
				}
				
				fangwei_Targethudu = (double)((1500 - fangwei_Targetjiaodu + 4389.97574) * 3.14159265 / 18000); //计算弧度
				fangwei_TargetPos_temp = cos(fangwei_Targethudu);
				fangwei_TargetPos_temp = (double)(sqrt(57.2713 * 57.2713 + 120.6785 * 120.6785 - fangwei_TargetPos_temp * 2 * 57.2713 * 120.6785) - 87.625);
				if ((fangwei_TargetPos_temp >= 0) && (fangwei_TargetPos_temp <= 38))//要求的新长度位置
				{
					fangwei_TargetPos = fangwei_TargetPos_temp;//推杆实际需要伸出绝对长度值
				}
				fuyang_Targethudu = 0;
				fuyang_Targethudu = (double)((fuyang_Targetjiaodu + 2162.1881+339.81465) * 3.14159265 / 18000);

				fuyang_TargettPos_temp = cos(fuyang_Targethudu);
				fuyang_TargettPos_temp = (double)(sqrt(553.5480 * 553.5480 + 409.1304 * 409.1304 - fuyang_TargettPos_temp * 2 * 553.5480 * 409.1304) - 229.625);
				if ((fuyang_TargettPos_temp >= 0) && (fuyang_TargettPos_temp <= 58))//判断坐标在允许范围内
				{
					fuyang_TargetPos = fuyang_TargettPos_temp;
				}
				if(fuyang_TargetPos - gl_currentPos >= 0.002)//判断是正向运动
				{
						if(gb_SHUN_NI ==0 )//判断当前状态是反转非静止中，本次需要正转，说明需要急速换向
						{
							if( gl_currentPos > 5.0)//判断反向减速停止距离足够，否则因为减速时间固定，一旦没有距离会撞机
							{
								fuyang_xch_dir =2;//设置换向标志2
								Yuntai_fuyang_shezhi_chenggong_flag=2;//修改设置角度标志，暂时转入减速停止，先不执行实际角度运动
								Yuntai_jiaodushezhi_flag =3;
							}
							else
							{
								Yuntai_fuyang_shezhi_chenggong_flag=0;								
							}
						}
				}
				else  if(fuyang_TargetPos - gl_currentPos < -0.002)//判断反向运动
				{
						if(gb_SHUN_NI ==1 )//判断当前状态是正转非静止中，本次需要反转，说明需要急速换向
						{
							if( gl_currentPos < 33)//判断正向减速停止距离足够，否则因为减速时间固定，一旦没有距离会撞机
							{
								fuyang_xch_dir =2;//设置换向标志2
							
								Yuntai_fuyang_shezhi_chenggong_flag=2;//修改设置角度标志，暂时转入减速停止，先不执行实际角度运动
								Yuntai_jiaodushezhi_flag =3;
							}
							else
							{
								Yuntai_fuyang_shezhi_chenggong_flag=0;								
							}
						}
				}	
				
				if(fangwei_TargetPos - gl_currentPos1 >= 0.002)//判断是正向运动
				{
						if(gb_SHUN_NI1 ==0 )//判断当前状态是反转非静止中，本次需要正转，说明需要急速换向
						{
							if( gl_currentPos1 < 25.0 )//判断反向减速停止距离足够，否则因为减速时间固定，一旦没有距离会撞机
							{
								Yuntai_fangwei_shezhi_chenggong_flag=2;//修改设置角度标志，暂时转入减速停止，先不执行实际角度运动
								Yuntai_jiaodushezhi_flag =4;
								fangwei_xch_dir =2;//设置换向标志2
							}
							else
							{
								Yuntai_fangwei_shezhi_chenggong_flag = 0;
							}
							
						}
				}
				else  if(fangwei_TargetPos - gl_currentPos1 < -0.002)//判断反向运动
				{
						if(gb_SHUN_NI1 ==1 )//判断当前状态是正转非静止中，本次需要反转，说明需要急速换向
						{
							if( gl_currentPos1 >5.0)//判断正向减速停止距离足够，否则因为减速时间固定，一旦没有距离会撞机
							{
								fangwei_xch_dir =2;//设置换向标志2
								Yuntai_fangwei_shezhi_chenggong_flag=2;//修改设置角度标志，暂时转入减速停止，先不执行实际角度运动
								Yuntai_jiaodushezhi_flag =4;
							}
							else
							{
								Yuntai_fangwei_shezhi_chenggong_flag=0;
							}
						}
				}	
				
			}
			else if ((Yuntai_fangwei_shezhi_chenggong_flag == 0) || (Yuntai_fuyang_shezhi_chenggong_flag == 0))
			{
				fuyang_Targetjiaodu = gul_fuyang_jiaodu;
				fangwei_Targetjiaodu = gul_fangwei_jiaodu;
			}
		}
		if (RS422_receive_str[0] == 0x05)
		{
			Yuntai_chushishezhi_flag = 1;
		}
		
	}
	
}

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

#endif
