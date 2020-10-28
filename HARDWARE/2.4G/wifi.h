#ifndef _WIFI_H
#define _WIFI_H

#include "sys.h"
#define USART2_Start    0x68
#define USART1_Len 7

void UARST2_Init(u32 bound);
void RS232_Send_Data(u8 *buf,u8 len);
short dianziluopan_Anal_Data(void);

#endif

