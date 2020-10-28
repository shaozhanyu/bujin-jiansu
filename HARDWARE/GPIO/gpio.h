#ifndef  _GPIO_H
#define  _GPIO_H
#include"sys.h"


#define RS485_TX_EN PDout(2)
#define Fuyang_Lmt1   PBin(8)
#define Fangwei_Lmt1   PBin(9)
#define Zuozhuan   PCin(13)
#define Youzhuan   PCin(14)
#define Fuyang_Err PCin(12)
#define Fangwei_Err PCin(8)
#define Tingzhiyundong PCin(15)
#define MAX3221_EN PCout(5)
#define MAX3221_EN1 PBout(0)
#define FORCEON PAout(4)

void GPIO_Jiance_Init(void);


#endif


