
#include "stm32f10x.h"
#include "PWM.h"

int  decel_cnt=0 , decel_cnt2 =0;
////////////////////��ʱ��1��ʼ��////////////////////////////////
void TIM1_Int_Init(u16 arr,u16 psc , FunctionalState NewState)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //ʱ��ʹ��
	TIM_DeInit(TIM1);
	//��ʱ��TIM1��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; //�ظ�����������������  
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update); //ע������������һ�±�־λ��������ʱ������������һ���ж�
	
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM1, NewState);  //�Ƿ�ʹ�ܶ�ʱ��
						 
}

//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
// NewState	:�Ƿ�ʹ�ܶ�ʱ������ʼ����
//����ʹ�õ��Ƕ�ʱ��3!
///////////////////��ʱ��3��ʼ��////////////////////////////////
void TIM3_Int_Init(u16 arr,u16 psc , FunctionalState NewState)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
	TIM_DeInit(TIM3);
	//��ʱ��TIM2��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //��������ж�����λ,ע������������һ�±�־λ��������ʱ������������һ���ж�
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM3�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���

	TIM_Cmd(TIM3, NewState);  //�Ƿ�ʹ�ܶ�ʱ��
						 
}

/////////////////////////////////////
/***������ TIM4_PWM_Init
		���ܣ���ʱ��4ͨ��1PWM��ʼ��
		������period  ��װֵ
					psc   ��Ƶֵ
					HalfPeriod  �Ƚ�ֵ
		����ֵ����
*/
void TIM4_PWM_Init(u32  Period , u16 psc , u32 HalfPeriod )
{
//	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitType;
	TIM_OCInitTypeDef TIM_OCInitType;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIM_TimeBaseInitType.TIM_ClockDivision = TIM_CKD_DIV1;//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseInitType.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseInitType.TIM_Prescaler   = psc; //��Ƶֵ
	TIM_TimeBaseInitType.TIM_Period      = Period; //����ֵ
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitType);

	TIM_OCInitType.TIM_OCIdleState = TIM_OCIdleState_Set; //����ʱ״̬Ϊset 1 PWMʱ����Ҫ����
	TIM_OCInitType.TIM_OCMode = TIM_OCMode_PWM1;	//	
	TIM_OCInitType.TIM_OCPolarity = TIM_OCPolarity_High;//������Ը�
	TIM_OCInitType.TIM_OutputNState = TIM_OutputNState_Disable; //��������Ƚ�״̬��ֹ ��������ƷTIM2 TIM3 TIM4û�л�����
	TIM_OCInitType.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ�� 
	TIM_OCInitType.TIM_Pulse = HalfPeriod; //��װ�벶��ȽϼĴ����е�ֵ
	TIM_OC1Init(TIM4,&TIM_OCInitType); //OC1��ʼ��ͨ��1  OC1
	
	TIM_OC1PreloadConfig(TIM4 , TIM_OCPreload_Enable); //ʹ�ܶ�ʱ��TIM1 ��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPEʹ�� TIM4 �� ARR �ϵ�Ԥװ�ؼĴ���

	//TIM_Cmd(TIM4, ENABLE);  //�Ƿ�ʹ�ܶ�ʱ��

	
}


/////////////////////////////////////
/***������ TIM4_PWM_Init
		���ܣ���ʱ��4ͨ��1PWM��ʼ��
		������period  ��װֵ
					psc   ��Ƶֵ
					HalfPeriod  �Ƚ�ֵ
		����ֵ����
*/
void TIM5_PWM_Init(u32  Period , u16 psc , u32 HalfPeriod )
{
//	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitType;
	TIM_OCInitTypeDef TIM_OCInitType;
	GPIO_InitTypeDef GPIO_InitStructure;  

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
//	TIM_DeInit(TIM5);
	TIM_TimeBaseInitType.TIM_ClockDivision = TIM_CKD_DIV1;//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseInitType.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
	TIM_TimeBaseInitType.TIM_Prescaler   = psc; //��Ƶֵ
	TIM_TimeBaseInitType.TIM_Period      = Period; //����ֵ
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitType);

	TIM_OCInitType.TIM_OCIdleState = TIM_OCIdleState_Set; //����ʱ״̬Ϊset 1 PWMʱ����Ҫ����
	TIM_OCInitType.TIM_OCMode = TIM_OCMode_PWM1;	//	
	TIM_OCInitType.TIM_OCPolarity = TIM_OCPolarity_High;//������Ը�
	TIM_OCInitType.TIM_OutputNState = TIM_OutputNState_Disable; //��������Ƚ�״̬��ֹ ��������ƷTIM2 TIM3 TIM4û�л�����
	TIM_OCInitType.TIM_OutputState = TIM_OutputState_Enable;//�Ƚ����ʹ�� 
	TIM_OCInitType.TIM_Pulse = HalfPeriod; //��װ�벶��ȽϼĴ����е�ֵ
	TIM_OC1Init(TIM5,&TIM_OCInitType); //OC1��ʼ��ͨ��1  OC1
	
	TIM_OC1PreloadConfig(TIM5 , TIM_OCPreload_Enable); //ʹ�ܶ�ʱ��TIM1 ��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM5, ENABLE);//ARPEʹ�� TIM5 �� ARR �ϵ�Ԥװ�ؼĴ���
	
//	TIM_Cmd(TIM1, ENABLE);  //�Ƿ�ʹ�ܶ�ʱ��
	
}

int  accel_cnt =0;

//////////////��ʱ��3�ж�/////////////////////
void TIM3_IRQHandler(void)
{ 

		TIM3->SR&=~(1<<0);
	
		decel_cnt++ ;
		if( 2==fuyang_xch_dir )
		{
		
				if ((fuyang_yundong_daowei_flag == 0) && (fy_ms_count_temp > 0))
				{

					fy_currentdeg += FUYANG_DEG_PER_MS;
				}
				else if ((fuyang_yundong_daowei_flag == 0) && (fy_ms_count_temp < 0))
				{

					fy_currentdeg -= FUYANG_DEG_PER_MS;
				}					
				
				fuyang_dec_num++;//���ټ������𽥼�С��PWM�趨����װֵ�������󣬴Ӷ��ٶȱ���
				t4_pwm_value = MOTOR_PWM_ARR*(1+fuyang_dec_num/ACCEL_NUM);
				
				update_accel_flag =1;//����PWM��־
				TIM4_PWM_Init( (t4_pwm_value-1) , 0 , (t4_pwm_value-1)/2);//�ٶȵݼ���PWM��װֵ����		
		
				if( decel_cnt >= ACCEL_NUM)//���ټ��ٴ���
				{
						decel_cnt = 0;
	//					TIM_Cmd(TIM1, 0);  //�رն�ʱ��1
						TIM_Cmd(TIM3, 0);  //�رն�ʱ��3
						TIM_Cmd(TIM4, 0);  //�رն�ʱ��4
						update_accel_flag = 0;
						
						fuyang_yundong_daowei_flag = 0;//
						fy_dir = 2;

						fuyang_shezhi_guocheng_flag = 0;		
						
						TIM4_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//ֱ�����趨�ٶ�����		
						if(fuyang_xch_dir == 2)//��������
						{
							fuyang_xch_dir=0;
							Yuntai_fuyang_shezhi_chenggong_flag = 1;
							Yuntai_jiaodushezhi_flag = 1;
						}
						TIM_Cmd(TIM3, 0);  //�رն�ʱ��3
						return;
				}
		
		}
		
}





