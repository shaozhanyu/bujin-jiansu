#include "24c02.h"
#include "delay.h"


void IIC_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIOB时钟 模拟IIC通讯，两个引脚都设为输出方式

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;          //普通输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;     //100MHz
  GPIO_Init(GPIOB,&GPIO_InitStructure);                 //初始化IO
	IIC_SCL=1;        //定义B8引脚为高电平
	IIC_SDAOUT=1;   
}


void IIC_Start(void)          //开始信号：SCL为高电平时，SDA由高电平向低电平跳变，开始传送数据
{
	SDA_OUT();     //配置SDA线为输出    PB9作为数据传输引脚，有输入和输出两种模式
	IIC_SDAOUT=1;	  	  
	IIC_SCL=1;
	delay_us(10);
 	IIC_SDAOUT=0;
	delay_us(10);
	IIC_SCL=0;     //准备发送或接收数据 
}	  

void IIC_Stop(void)    //结束信号：SCL为高电平时，SDA由低电平向高电平跳变，结束传送数据。
{
	SDA_OUT();    //配置SDA线为输出
	IIC_SCL=0;
	IIC_SDAOUT=0; 
 	delay_us(10);
	IIC_SCL=1;
  delay_us(10);				
	IIC_SDAOUT=1; //发送I2C总线结束信号				   	
}

u8 MCU_Wait_Ack(void)       //MCU等待从设备应答信号到来    这种情况MCU作为发送端，IIC作为接收端
	                         //等待IIC发出一个低电平的信号
{
	u8 WaitTime=0;
	SDA_IN();      //配置SDA线为输入  
	IIC_SDAOUT=1;
	delay_us(10);	   
	IIC_SCL=1;
	delay_us(10);	 
	while(IIC_SDAIN)    //判断输入SDA为1
	{
		WaitTime++;
		if(WaitTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0; 
	return 0;  
}

void MCU_Send_Ack(void)     //MCU产生ACK应答，告知24Cxx，应答信号：
{
	IIC_SCL=0;   //信号电平置低，脉冲电平置低时，才能更改数据线的电平状态
	SDA_OUT();    //数据线规定STM32主机作为主器件，SDA是输出状态
	IIC_SDAOUT=0;  //输出SDA线上的状态为0  
	delay_us(10);
	IIC_SCL=1;     //时钟信号电平置高
	delay_us(10);
	IIC_SCL=0;       
}

void MCU_NOAck(void)    //MCU不产生ACK应答
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDAOUT=1;
	delay_us(10);
	IIC_SCL=1;
	delay_us(10);
	IIC_SCL=0;
}	 

void IIC_Send_Byte(u8 txd)
{                        
	u8 t;   
	SDA_OUT(); 	    
	IIC_SCL=0;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
			//IIC_SDA=(txd&0x80)>>7;
	if((txd&0x80)>>7)
		IIC_SDAOUT=1;
	else
		IIC_SDAOUT=0;
	txd<<=1; 	  
	delay_us(10);   //对TEA5767这三个延时都是必须的
	IIC_SCL=1;
	delay_us(10); 
	IIC_SCL=0;	
	delay_us(10);
	}	 
} 	


u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
    IIC_SCL=0; 
    delay_us(10);
		IIC_SCL=1;
    receive<<=1;
    if(IIC_SDAIN)receive++;   
		delay_us(10); 
  }					 
	if (!ack)
			MCU_NOAck();//发送nACK
	else
			MCU_Send_Ack(); //发送ACK   
	return receive;
}

/*******************************IO口模拟IIC*************************************
*******************************************************************************/


/*******************************************************************************
*************************以下为EEPROM24C02读写操作******************************
*******************************************************************************/
void AT24CXX_Init(void)
{
	IIC_Init();
}



//初始化24c02的IIC接口
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=0;		  	    																 
    IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(0XA0);	   //发送写命令
		MCU_Wait_Ack();
		IIC_Send_Byte(ReadAddr>>8);//发送高地址
		//IIC_Wait_Ack();		 
	}else IIC_Send_Byte(0XA0+((ReadAddr/256)<<1));   //发送器件地址0XA0,写数据 	 

	MCU_Wait_Ack(); 
  IIC_Send_Byte(ReadAddr%256);   //发送低地址
	MCU_Wait_Ack();	    
	IIC_Start();  	 	   
	IIC_Send_Byte(0XA1);           //进入接收模式			   
	MCU_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();//产生一个停止条件	    
	return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{				   	  	    																 
    IIC_Start();  
	if(EE_TYPE>AT24C16)
	{
		IIC_Send_Byte(0XA0);	    //发送写命令
		MCU_Wait_Ack();
		IIC_Send_Byte(WriteAddr>>8);//发送高地址
 	}else
	{
		IIC_Send_Byte(0XA0+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据 
	}	 
	MCU_Wait_Ack();	   
    IIC_Send_Byte(WriteAddr%256);   //发送低地址
	MCU_Wait_Ack(); 	 										  		   
	IIC_Send_Byte(DataToWrite);     //发送字节							   
	MCU_Wait_Ack();  		    	   
    IIC_Stop();//产生一个停止条件 
	delay_ms(7);	 
}
//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
	}												    
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(255);//避免每次开机都写AT24CXX			   
	if(temp==0X55)return 0;		   
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(255,0X55);
	    temp=AT24CXX_ReadOneByte(255);	  
		if(temp==0X55)return 0;
	}
	return 1;											  
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	while(NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}

/****************************************************************************
* 名    称: u32 Buf_4Byte(u8 *pBuffer,u32 Date_4Byte,u8 Byte_num,u8 mode)
* 功    能：多位数与字节互转
* 入口参数：mode：1:多位数转分成字节   0:字节合并成一个多位数
            Byte_num：需要转化的字节数
            *pBuffer：字节接收数组或字节所在数组
            Date_4Byte：多位数数
* 返回参数：mode为0时，返回多位数
* 说    明：Byte_num最大为4个字节，该函数在后面的触摸屏校正时存取校正值所用到
****************************************************************************/
u32 Buf_4Byte(u8 *pBuffer,u32 Date_4Byte,u8 Byte_num,u8 mode)
{
   u8 i; u32 middata=0;
	if(mode)    //多位数转分成字节
	 {
	   for(i=0;i<Byte_num;i++)
	     {
	       *pBuffer++ =(Date_4Byte>>(8*i))&0xff;
	     }
			return 0; 
	 } 
	 else       //字节合并成一个多位数
	 {
	    Date_4Byte=0;
		  pBuffer+=(Byte_num-1);
		  for(i=0;i<Byte_num;i++)
	      { 		
		      middata<<=8;
		      middata+= *pBuffer--;			   
	      }
			return middata;	
	 }
}

//void AT24CXXX_IPNET( )
//{
//		AT24CXX_Read(0,(u8*)IPbuffer,5);
//		if(IPbuffer[4]==crc_high_first((u8 *)IPbuffer,4))
//			{
//			sprintf((char *)strIP,"IP=%d.%d.%d.%d\r\n",IPbuffer[0],IPbuffer[1],IPbuffer[2],IPbuffer[3]);
//			uart1SendChars((u8*)strIP,25);
//			IP1=IPbuffer[0];
//			IP2=IPbuffer[1];
//			IP3=IPbuffer[2];
//			IP4=IPbuffer[3];
//			}		
//		else
//			{
//			AT24CXX_Read(40,(u8*)IPbuffer,5);
//			if(IPbuffer[4]==crc_high_first((u8 *)IPbuffer,4))
//				{
//			  sprintf((char *)strIP,"IP=%d.%d.%d.%d\r\n",IPbuffer[0],IPbuffer[1],IPbuffer[2],IPbuffer[3]);
//			  uart1SendChars((u8*)strIP,25);	
//			  IP1=IPbuffer[0];
//			  IP2=IPbuffer[1];
//			  IP3=IPbuffer[2];
//			  IP4=IPbuffer[3];
//				}
//			else
//				{
//			  IP1=192;
//			  IP2=168;
//			  IP3=1;
//			  IP4=240;
//			  }		
//			} 
//		AT24CXX_Read(8,(u8*)Netbuffer,5);
//		if(Netbuffer[4]==crc_high_first((u8 *)Netbuffer,4))
//			{
//			sprintf((char *)strNET,"NET=%d.%d.%d.%d\r\n",Netbuffer[0],Netbuffer[1],Netbuffer[2],Netbuffer[3]);
//			uart1SendChars((u8*)strNET,25);
//			NET1=Netbuffer[0];
//			NET2=Netbuffer[1];
//			NET3=Netbuffer[2];
//			NET4=Netbuffer[3];
//			}		
//		else
//			{
//			AT24CXX_Read(48,(u8*)Netbuffer,5);
//		  if(Netbuffer[4]==crc_high_first((u8 *)Netbuffer,4))
//			  {
//			  sprintf((char *)strNET,"NET=%d.%d.%d.%d\r\n",Netbuffer[0],Netbuffer[1],Netbuffer[2],Netbuffer[3]);
//			  uart1SendChars((u8*)strNET,25);	
//			  NET1=Netbuffer[0];
//			  NET2=Netbuffer[1];
//			  NET3=Netbuffer[2];
//			  NET4=Netbuffer[3];
//			  }
//		 else
//			  {
//			  NET1=255;
//			  NET2=255;
//			  NET3=255;
//			  NET4=0;
//			  }
//			}
//		AT24CXX_Read(16,(u8*)WGbuffer,5);
//		if(WGbuffer[4]==crc_high_first((u8 *)WGbuffer,4))
//			{
//			sprintf((char *)strWG,"WG=%d.%d.%d.%d\r\n",WGbuffer[0],WGbuffer[1],WGbuffer[2],WGbuffer[3]);
//			uart1SendChars((u8*)strWG,25);
//			WG1=WGbuffer[0];
//			WG2=WGbuffer[1];
//			WG3=WGbuffer[2];
//			WG4=WGbuffer[3];
//			}		
//		else
//			{
//			AT24CXX_Read(56,(u8*)WGbuffer,5);
//			if(WGbuffer[4]==crc_high_first((u8 *)WGbuffer,4))
//			  {
//			  sprintf((char *)strWG,"WG=%d.%d.%d.%d\r\n",WGbuffer[0],WGbuffer[1],WGbuffer[2],WGbuffer[3]);
//			  uart1SendChars((u8*)strWG,25);	
//			  WG1=WGbuffer[0];
//			  WG2=WGbuffer[1];
//			  WG3=WGbuffer[2];
//			  WG4=WGbuffer[3];
//			  }
//			else
//			 {
//			  WG1=192;
//			  WG2=168;
//			  WG3=1;
//			  WG4=1;
//			 }		
//			}
//		AT24CXX_Read(24,(u8*)PORTbuffer,3);    			
//		if(PORTbuffer[2]==crc_high_first((u8 *)PORTbuffer,2))
//		{
//     PORT1=(PORTbuffer[1]<<8)+PORTbuffer[0];
//		 sprintf((char *)strPORT,"PORT=%d\r\n",PORT1); 
//     uart1SendChars((u8 *)strPORT,25);			
//		}
//		else
//		{
//		 	AT24CXX_Read(64,(u8*)PORTbuffer,3); 
//      if(PORTbuffer[2]==crc_high_first((u8 *)PORTbuffer,2))
//      {
//			 PORT1=(PORTbuffer[1]<<8)+PORTbuffer[0];
//		   sprintf((char *)strPORT,"PORT=%d\r\n",PORT1); 
//       uart1SendChars((u8 *)strPORT,25);				
//			}
//      else
//      {
//			 PORT1=2040;
//			}						
//		}
//    AT24CXX_Read(32,(u8*)RS485buffer,3);
//    if(RS485buffer[1]==crc_high_first((u8 *)RS485buffer,1))
//		{
//		  RS485_ID=RS485buffer[0];
//		  sprintf((char *)strRS485ID,"ID=%d\r\n",RS485_ID); 
//      uart1SendChars((u8 *)strRS485ID,25);	      			
//		}			
//		else
//		{
//		 AT24CXX_Read(72,(u8*)RS485buffer,3);
//    if(RS485buffer[1]==crc_high_first((u8 *)RS485buffer,1))
//		{
//		  RS485_ID=RS485buffer[0];
//		  sprintf((char *)strRS485ID,"ID=%d\r\n",RS485_ID); 
//      uart1SendChars((u8 *)strRS485ID,25);	      			
//		}		
//    else	
//    {			
//		  RS485_ID=0x01;
//		}
//		}
//}


















