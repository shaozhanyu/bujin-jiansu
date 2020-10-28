#ifndef __24C02_H
#define __24C02_H
#include "sys.h"  

////////////////////////////////////////////////////////////////////////////////// 

#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	    8191
#define AT24C128	16383
#define AT24C256	32767  
//Mini STM32开发板使用的是24c08，所以定义EE_TYPE为AT24C08
#define EE_TYPE AT24C256



//IIC_SDA线IO方向配置
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(u32)8<<12;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(u32)3<<12;}
//IO操作函数	 
#define IIC_SCL      PBout(10) //SCL
#define IIC_SDAOUT   PBout(11) //输出SDA	 
#define IIC_SDAIN    PBin(11)  //输入SDA 

//IIC相关函数
void IIC_Init(void);          //初始化IIC的IO口				 
void IIC_Start(void);				  //发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
u8 MCU_Wait_Ack(void); 				//IIC等待ACK信号
void MCU_Send_Ack(void);					  //IIC发送ACK信号
void MCU_NOAck(void);				  //IIC不发送ACK信号
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_OneByte(u8 ack);
	
//EEPROM24c02相关函数
//u8 AT24C02_ReadByte(u8 ReadAddr);							     //指定地址读取一个字节
//void AT24C02_WriteByte(u8 WriteAddr,u8 DataToWrite);		//指定地址写入一个字节

u32 Buf_4Byte(u8 *pBuffer,u32 Date_4Byte,u8 Byte_num,u8 mode);

//void AT24C02_Write(u8 WriteAddr,u8 *pBuffer,u8 WriteNum);	//从指定地址开始写入指定长度的数据
//void AT24C02_Read(u8 ReadAddr,u8 *pBuffer,u8 ReadNum);   	//从指定地址开始读出指定长度的数据

//u8 AT24C02_Test(void);  //检查器件
//void AT24C02_Init(void); //初始化IIC

u8 AT24CXX_ReadOneByte(u16 ReadAddr);							//指定地址读取一个字节
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite);		//指定地址写入一个字节
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//指定地址开始写入指定长度的数据
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//从指定地址开始写入指定长度的数据
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//从指定地址开始读出指定长度的数据

u8 AT24CXX_Check(void);  //检查器件
void AT24CXX_Init(void); //初始化IIC

void AT24CXXX_IPNET(void);

#endif

