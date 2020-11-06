/*
 * @,@Author: ,: your name
 * @,@Date: ,: 2020-09-17 09:17:15
 * @,@LastEditTime: ,: 2020-10-30 09:03:40
 * @,@LastEditors: ,: Please set LastEditors
 * @,@Description: ,: In User Settings Edit
 * @,@FilePath: ,: \USERe:\test2-da\SYSTEM\usart\usart.h
 */
#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
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
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define crc_mul 0x1021

extern  u8 RS422_byte_count;
extern long rx_time;
extern long rx_cnt;
extern u8 last_RS422_byte_count;
extern  u8 rx_buf[50];

union IntToChar
{
    int16_t m_Int;
	  uint16_t m_UInt;
	  struct Char
		{
			int8_t m_CharL;
			int8_t m_CharH;		
		}m_Char;
		struct UChar
		{
		   uint8_t m_UCharL;
			 uint8_t m_UCharH;
		}m_UChar;
};
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void RS485_Send_Data(u8 *buf,u8 len);
void Receive_data_Anal(void);
void RS422_Send_Data(u8 *buf,u8 len);
void RS232_data_Anal(void);
u16 cal_crc(unsigned char *ptr,unsigned char len);
double mycos(double x);
int  calc_deg_fun( short fw_deg , short fy_deg , int len);

#endif


