#ifndef _TIME_C
#define _TIME_C

#include "stm32f10x.h"
#include "Motor.h"




extern   int  decel_cnt , decel_cnt2 ;


typedef void (*Process)(void);



void TIM4_PWM_Init(u32  Period , u16 psc , u32 HalfPeriod );
void TIM5_PWM_Init(u32  Period , u16 psc , u32 HalfPeriod );
//void TIM1_Init(void);
void TIM1_SetOC1Process(Process p);
void TIM1_Config(u32 Prescaler,u32 Period,u8 Ratio);
void TIM1_Int_Init(u16 arr,u16 psc , FunctionalState NewState);
void TIM3_Int_Init(u16 arr,u16 psc , FunctionalState NewState);
#endif
