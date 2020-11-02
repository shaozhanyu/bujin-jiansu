#include "AMIS30543.h"
#include "delay.h"

extern unsigned char guc_WorkCurrent;
extern unsigned char guc_StepDivisor;
extern unsigned char guc_StepDivisor1;
extern u8 fy_dir;
extern u8 fw_dir;
extern unsigned char guc_StepM; //ϸ�����ã�1��2��8��16
extern unsigned char guc_StepM1;
extern float gf_MotorStep;			//�������һ����ת�Ķ���1.8
extern unsigned long gul_FirstFreq; //��һƵ�ʣ�ͨ����ֱ�ӵ�����ٶȼ���õ��Ŀ�ֱ�ӵ����Ƶ��
extern unsigned long gul_FirstFreq1;
extern unsigned int gui_BeiYong; //����
extern unsigned int gui_BeiYong1;
extern unsigned char guc_WorkCurrent1;
unsigned char guc_AMIS30543_SPI_WR_data1;
unsigned char guc_AMIS30543_SPI_CR0_data1;
unsigned char guc_AMIS30543_SPI_CR1_data1;
unsigned char guc_AMIS30543_SPI_CR2_data1;
unsigned char guc_AMIS30543_SPI_CR3_data1;
unsigned char guc_AMIS30543_SPI_WR_data2;
unsigned char guc_AMIS30543_SPI_CR0_data2;
unsigned char guc_AMIS30543_SPI_CR1_data2;
unsigned char guc_AMIS30543_SPI_CR2_data2;
unsigned char guc_AMIS30543_SPI_CR3_data2;
//unsigned char guc_AMIS30543_data2;
AMIS30543_CR AMIS30543_CR_SPI1;
AMIS30543_CR AMIS30543_CR_SPI2;

void AMIS30543_Init1()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//	 GPIO_ResetBits(GPIOC,GPIO_Pin_11);      //DIR1�͵�ƽΪ0���͵�ƽ˳ʱ����ת��
	//	 AMIS30543_DIR1 = 0;               //DIR1��ƽΪ1��ʱ�򣬸��������0��λ��ת������λ����  //DIR1��ƽΪ0��ʱ�򣬸��������ǰת��ȥ������λ����
	//	 fy_dir = 0;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOB, GPIO_Pin_7);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; //SPI3������ͨѶ��ʹ��GPIOB.3,GPIOB.4��GPIOB.5
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA, GPIO_Pin_15); //�����ģʽ�£�δ�ã�����Ϊͨ��IO

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	 //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //ָ�����ݴ����MSBλ����LSBλ��ʼ�����ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI3, &SPI_InitStructure);

	SPI_Cmd(SPI3, ENABLE);
}
//////////////////////////////////////////////////////
void AMIS30543_Init2()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, GPIO_Pin_12);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//	 AMIS30543_DIR2 = 0;     //�͸����ķ�����ͬ
	//	 fw_dir = 0;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 //����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;	 //NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //ָ�����ݴ����MSBλ����LSBλ��ʼ�����ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_Cmd(SPI2, ENABLE);
}

/////////////////////////////////////////////////////////////
void AMIS30543_NXT1_init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	//����ΪPWMģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = psc;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

//	TIM_Cmd(TIM4, DISABLE);
}
//////////////////////////////////////////////////////////
void AMIS30543_NXT2_init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	//����ΪPWMģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = psc;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM5, ENABLE);

}
void SPI2_SetSpeed(u8 SpeedSet)
{
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);
}

u8 SPI2_ReadWriteByte(u8 TxData)
{
	u8 retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ������� �����ͻ���ձ�־λ
	{
		retry++;
		if (retry > 200)
			return 0;
	}
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ������
	retry = 0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	{
		retry++;
		if (retry > 200)
			return 0;
	}
	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����
}
u8 SPI3_ReadWriteByte(u8 TxData)
{
	u8 retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if (retry > 200)
		{
			return 0;
		}
	}
	SPI_I2S_SendData(SPI3, TxData); //ͨ������SPIx����һ������
	retry = 0;

	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET)
	{
		retry++;
		if (retry > 200)
			return 0;
	}

	return SPI_I2S_ReceiveData(SPI3); //����ͨ������SPIx������յ�������
}
u8 SPI2_Byte_Write(unsigned char addr, AMIS30543_CR DATA)
{
	u8 save;
	unsigned char  k, k1;
	if (addr == 0x00)
	{
		save = DATA.AMIS30543_WR.bitsWDT << 3 | DATA.AMIS30543_WR.bitsWDEN << 7;
	}
	if (addr == 0x01)
	{
		save = DATA.AMIS30543_CR0.bitsCUR | DATA.AMIS30543_CR0.bitsSM << 5;
	}
	if (addr == 0x02)
	{
		save = DATA.AMIS30543_CR1.bitsEMC | DATA.AMIS30543_CR1.bitsPWMJ << 2 | DATA.AMIS30543_CR1.bitsPWMF << 3 | DATA.AMIS30543_CR1.bitsNXTP << 6 | DATA.AMIS30543_CR1.bitsDIRCTRL << 7;
	}
	if (addr == 0x03)
	{
		save = DATA.AMIS30543_CR2.bitsSLAT << 4 | DATA.AMIS30543_CR2.bitsSLAG << 5 | DATA.AMIS30543_CR2.bitsSLP << 6 | DATA.AMIS30543_CR2.bitsMOTEN << 7;
	}
	if (addr == 0x09)
	{
		save = DATA.AMIS30543_CR3.bitsESM;
	}
	AMIS30543_SPI2_NSS = 0;

	k = SPI2_ReadWriteByte(0x80 | addr);

	k1 = SPI2_ReadWriteByte(save);

	//	AMIS30543_SPI2_NSS = 1;
	//	delay_us(10);
	//	AMIS30543_SPI2_NSS = 0;
	delay_us(10);
	//
	//	k2 = SPI2_ReadWriteByte(0x00);
	AMIS30543_SPI2_NSS = 1;
	delay_us(10);
	AMIS30543_SPI2_NSS = 0;
	k1 = SPI2_ReadWriteByte(0x00 | addr);

	k1 = SPI2_ReadWriteByte(0x00);
	AMIS30543_SPI2_NSS = 1;
	delay_us(10);

	if (k1 == save)
	{
		return save;
	}
	else
	{
		return 0;
	}
	
}

u8 SPI3_Byte_Write(unsigned char addr, AMIS30543_CR DATA)
{
	u8 save;
	unsigned char  k,k1;
	if (addr == 0x00)
	{
		save = DATA.AMIS30543_WR.bitsWDT << 3 | DATA.AMIS30543_WR.bitsWDEN << 7;
	}
	if (addr == 0x01)
	{
		save = DATA.AMIS30543_CR0.bitsCUR | DATA.AMIS30543_CR0.bitsSM << 5;
	}
	if (addr == 0x02)
	{
		save = DATA.AMIS30543_CR1.bitsEMC | DATA.AMIS30543_CR1.bitsPWMJ << 2 | DATA.AMIS30543_CR1.bitsPWMF << 3 | DATA.AMIS30543_CR1.bitsNXTP << 6 | DATA.AMIS30543_CR1.bitsDIRCTRL << 7;
	}
	if (addr == 0x03)
	{
		save = DATA.AMIS30543_CR2.bitsSLAT << 4 | DATA.AMIS30543_CR2.bitsSLAG << 5 | DATA.AMIS30543_CR2.bitsSLP << 6 | DATA.AMIS30543_CR2.bitsMOTEN << 7;
	}
	if (addr == 0x09)
	{
		save = DATA.AMIS30543_CR3.bitsESM;
	}
	AMIS30543_SPI3_NSS = 0;
	delay_us(10);
	k = SPI3_ReadWriteByte(0x80 | addr);

	k1 = SPI3_ReadWriteByte(save);
	AMIS30543_SPI3_NSS = 1;
	delay_us(10);
	AMIS30543_SPI3_NSS = 0;
	k1 = SPI3_ReadWriteByte(0x00 | addr);

	k1 = SPI3_ReadWriteByte(0x00);

	AMIS30543_SPI3_NSS = 1;
	delay_us(100);

	if (k1 == save)
	{
		return save;
	}
	else
	{
		return 0;
	}
}
void AMIS30543_INIT1()
{
	switch (motrotype) //�����������
	{
	case 1:
		AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1060;
		break;
	case 2:
		AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
		break;
	case 3:
		AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_2440;
		break;
	default:
		AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
		break;
	}
	AMIS30543_CR_SPI1.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_32;
	AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
	AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Disable; //���δ����

	AMIS30543_CR_SPI1.AMIS30543_CR1.bitsEMC = AMIS30543_SPI_EMC_VeryFast;
	AMIS30543_CR_SPI1.AMIS30543_CR1.bitsPWMJ = AMIS30543_SPI_PWMJ_Enable;
	AMIS30543_CR_SPI1.AMIS30543_CR1.bitsPWMF = AMIS30543_SPI_PWMF_Double;
	AMIS30543_CR_SPI1.AMIS30543_CR1.bitsNXTP = AMIS30543_SPI_NXTP_Rise;
	AMIS30543_CR_SPI1.AMIS30543_CR1.bitsDIRCTRL = AMIS30543_SPI_DIRCTRL_CW;

	switch (guc_WorkCurrent)
	{
	case 0: //0.25
		switch (motrotype)
		{
		case 1:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_245;
			break;
		case 2:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_395;
			break;
		case 3:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_640;
			break;
		default:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_395;
			break;
		}
		break;
	case 1: //0.5
		switch (motrotype)
		{
		case 1:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_540;
			break;
		case 2:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_780;
			break;
		case 3:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
			break;
		default:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_780;
			break;
		}
		break;
	case 2: //0.75
		switch (motrotype)
		{
		case 1:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_780;
			break;
		case 2:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1150;
			break;
		case 3:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1850;
			break;
		default:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1150;
			break;
		}
		break;
	case 3: //1
		switch (motrotype)
		{
		case 1:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1060;
			break;
		case 2:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
			break;
		case 3:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_2440;
			break;
		default:
			AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
			break;
		}
		break;
	default:
		AMIS30543_CR_SPI1.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
		break;
	}
	switch (guc_StepDivisor)
	{
	case 0:
		AMIS30543_CR_SPI1.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Compenstated2;
		guc_StepM = 1;
		//				    gul_FirstFreq = (6.0/gf_MotorStep) * 1 * gui_BeiYong;
		break;
	case 1:
		AMIS30543_CR_SPI1.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Other;
		AMIS30543_CR_SPI1.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_Half;
		guc_StepM = 2;
		//				    gul_FirstFreq = (6.0/gf_MotorStep) * 2 * gui_BeiYong;
		break;
	case 2:
		AMIS30543_CR_SPI1.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Other;
		AMIS30543_CR_SPI1.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_4;
		guc_StepM = 4;
		//				    gul_FirstFreq = (6.0/gf_MotorStep) * 4 * gui_BeiYong;
		break;
	case 3:
		AMIS30543_CR_SPI1.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Other;
		AMIS30543_CR_SPI1.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_8;
		guc_StepM = 8;
		//				     gul_FirstFreq = (6.0/gf_MotorStep) * 8 * gui_BeiYong;
		break;
	case 4:
		AMIS30543_CR_SPI1.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Other;
		AMIS30543_CR_SPI1.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_16;
		guc_StepM = 16;
		//				    gul_FirstFreq = (6.0/gf_MotorStep) * 16 * gui_BeiYong;
		break;
	case 5:
		AMIS30543_CR_SPI1.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Other;
		AMIS30543_CR_SPI1.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_32;
		guc_StepM = 32;
		//				    gul_FirstFreq = (6.0/gf_MotorStep) * 32 * gui_BeiYong;
		break;
	case 6:
		AMIS30543_CR_SPI1.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_64;
		guc_StepM = 64;
		//				    gul_FirstFreq = (6.0/gf_MotorStep) * 64 * gui_BeiYong;
		break;
	case 7:
		AMIS30543_CR_SPI1.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_128;
		guc_StepM = 128;
		//				    gul_FirstFreq = (6.0/gf_MotorStep) * 128 * gui_BeiYong;
		break;
	default:
		AMIS30543_CR_SPI1.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_128;
		guc_StepM = 128;
		//				    gul_FirstFreq = (6.0/gf_MotorStep) * 128 * gui_BeiYong;
		break;
	}
	guc_AMIS30543_SPI_WR_data1 = SPI3_Byte_Write(AMIS30543_SPI_WR, AMIS30543_CR_SPI1);
	guc_AMIS30543_SPI_CR0_data1 = SPI3_Byte_Write(AMIS30543_SPI_CR0, AMIS30543_CR_SPI1);
	guc_AMIS30543_SPI_CR1_data1 = SPI3_Byte_Write(AMIS30543_SPI_CR1, AMIS30543_CR_SPI1);
	guc_AMIS30543_SPI_CR2_data1 = SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);
	guc_AMIS30543_SPI_CR3_data1 = SPI3_Byte_Write(AMIS30543_SPI_CR3, AMIS30543_CR_SPI1);
}
void AMIS30543_INIT2()
{
	switch (motrotype) //�����������
	{
	case 1:
		AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1060;
		break;
	case 2:
		AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
		break;
	case 3:
		AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_2440;
		break;
	default:
		AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
		break;
	}
	//	 AMIS30543_CR_SPI2.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_32;
	//	 AMIS30543_CR_SPI2.AMIS30543_CR3.bitsESM  = AMIS30543_SPI_ESM_128;
	AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
	AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Disable; //���δ����

	AMIS30543_CR_SPI2.AMIS30543_CR1.bitsEMC = AMIS30543_SPI_EMC_VeryFast;
	AMIS30543_CR_SPI2.AMIS30543_CR1.bitsPWMJ = AMIS30543_SPI_PWMJ_Enable;
	AMIS30543_CR_SPI2.AMIS30543_CR1.bitsPWMF = AMIS30543_SPI_PWMF_Double;
	AMIS30543_CR_SPI2.AMIS30543_CR1.bitsNXTP = AMIS30543_SPI_NXTP_Rise;
	AMIS30543_CR_SPI2.AMIS30543_CR1.bitsDIRCTRL = AMIS30543_SPI_DIRCTRL_CW;

	switch (guc_WorkCurrent1)
	{
	case 0: //0.25
		switch (motrotype)
		{
		case 1:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_245;
			break;
		case 2:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_395;
			break;
		case 3:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_640;
			break;
		default:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_395;
			break;
		}
		break;
	case 1: //0.5
		switch (motrotype)
		{
		case 1:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_540;
			break;
		case 2:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_780;
			break;
		case 3:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
			break;
		default:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_780;
			break;
		}
		break;
	case 2: //0.75
		switch (motrotype)
		{
		case 1:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_780;
			break;
		case 2:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1150;
			break;
		case 3:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1850;
			break;
		default:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1150;
			break;
		}
		break;
	case 3: //1
		switch (motrotype)
		{
		case 1:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1060;
			break;
		case 2:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
			break;
		case 3:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_2440;
			break;
		default:
			AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
			break;
		}
		break;
	default:
		AMIS30543_CR_SPI2.AMIS30543_CR0.bitsCUR = AMIS30543_SPI_CUR_1405;
		break;
	}
	switch (guc_StepDivisor1) //	 guc_StepDivisor=7,Ϊ128ϸ��
	{
	case 0:
		AMIS30543_CR_SPI2.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Compenstated2;
		guc_StepM1 = 1;
		gul_FirstFreq1 = (6.0 / gf_MotorStep) * 1 * gui_BeiYong1;
		break;
	case 1:
		AMIS30543_CR_SPI2.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Other;
		AMIS30543_CR_SPI2.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_Half;
		guc_StepM1 = 2;
		gul_FirstFreq1 = (6.0 / gf_MotorStep) * 2 * gui_BeiYong1;
		break;
	case 2:
		AMIS30543_CR_SPI2.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Other;
		AMIS30543_CR_SPI2.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_4;
		guc_StepM1 = 4;
		gul_FirstFreq1 = (6.0 / gf_MotorStep) * 4 * gui_BeiYong1;
		break;
	case 3:
		AMIS30543_CR_SPI2.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Other;
		AMIS30543_CR_SPI2.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_8;
		guc_StepM1 = 8;
		gul_FirstFreq1 = (6.0 / gf_MotorStep) * 8 * gui_BeiYong1;
		break;
	case 4:
		AMIS30543_CR_SPI2.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Other;
		AMIS30543_CR_SPI2.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_16;
		guc_StepM1 = 16;
		gul_FirstFreq1 = (6.0 / gf_MotorStep) * 16 * gui_BeiYong1;
		break;
	case 5:
		AMIS30543_CR_SPI2.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_Other;
		AMIS30543_CR_SPI2.AMIS30543_CR0.bitsSM = AMIS30543_SPI_SM_32;
		guc_StepM1 = 32;
		gul_FirstFreq1 = (6.0 / gf_MotorStep) * 32 * gui_BeiYong1;
		break;
	case 6:
		AMIS30543_CR_SPI2.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_64;
		guc_StepM1 = 64;
		gul_FirstFreq1 = (6.0 / gf_MotorStep) * 64 * gui_BeiYong1;
		break;
	case 7:
		AMIS30543_CR_SPI2.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_128;
		guc_StepM1 = 128;
		gul_FirstFreq1 = (6.0 / gf_MotorStep) * 128 * gui_BeiYong1;
		break;
	default:
		AMIS30543_CR_SPI2.AMIS30543_CR3.bitsESM = AMIS30543_SPI_ESM_128;
		guc_StepM1 = 128;
		gul_FirstFreq1 = (6.0 / gf_MotorStep) * 128 * gui_BeiYong1;
		break;
	}
	guc_AMIS30543_SPI_WR_data2 = SPI2_Byte_Write(AMIS30543_SPI_WR, AMIS30543_CR_SPI2);
	guc_AMIS30543_SPI_CR0_data2 = SPI2_Byte_Write(AMIS30543_SPI_CR0, AMIS30543_CR_SPI2);
	guc_AMIS30543_SPI_CR1_data2 = SPI2_Byte_Write(AMIS30543_SPI_CR1, AMIS30543_CR_SPI2);
	guc_AMIS30543_SPI_CR2_data2 = SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);
	guc_AMIS30543_SPI_CR3_data2 = SPI2_Byte_Write(AMIS30543_SPI_CR3, AMIS30543_CR_SPI2);
}
