#include "delay.h"
#include "sys.h"
#include "timer.h"
#include "AMIS30543.h"
#include "usart.h"
#include "gpio.h"
#include "stdlib.h"
#include "math.h"
#include "wifi.h"
#include "24c02.h"
#include "Motor.h"
#include "PWM.h"
#include "stdio.h"
#include "string.h"

#define LMTMAXCOUNT 2

char fy_reset_flag; //�������������̱�־
char fy_Init_flag;
char fw_Init_flag;
char fw_reset_flag;				  //��λ���������̱�־
unsigned char guc_WorkCurrent;		  //��ʾ���������������
unsigned char guc_WorkCurrent1;		  //��ʾ��λ�����������
unsigned char guc_StepDivisor;		  //���ø��������ϸ����
unsigned char guc_StepDivisor1;		  //���÷�λ�����ϸ����
extern u8 Uart1RxTemp[20];			  //�������̽�����������
extern u32 guc_1mscnt;				  //1ms��ʱ��
extern u32 guc_2mscnt;				  //2ms��ʱ��
extern u32 guc_10mscnt;				  //10ms��ʱ��

extern u8 Uart2Ok;					  //�������̽������ݱ�־
extern u8 RS232_receive_flag;		  //��λ���������ݱ�־λ
extern u8 Yuntai_fuwei_flag;		  //��̨��λ��־
extern short shuipingfuyangjiao;	  //ˮƽ������
extern short crc_check;				  //crc������
extern short crc_result1;			  //���յ���crc���

extern short fw_TargetDeg;	  //��λĿ��Ƕ�

u8 shuipingfuyangjiao_diwei;		  //ˮƽ�����ǵ�λ
u8 fuyang_shezhi_guocheng_flag;		  //�������ù��̱�־
u8 fangwei_shezhi_guocheng_flag;	  //��λ���ù��̱�־

extern short fy_TargetDeg;	  //����Ŀ��Ƕ�
extern u32 fw_timecnt;			  //��λ��ʱ��־

extern u8 Yuntai_tingzhi_flag;		  //��ֹ̨ͣ��־

short gul_fuyang_jiaodu;			  //�����Ƕȱ���
short gul_fangwei_jiaodu;			  //��λ�Ƕȱ���
short crc_fasong_Result;			  //crc����洢����
u8 crc_fasong_gaowei;				  //crc���͸�λ
u8 crc_fasong_diwei;				  //crc���͵�λ
u8 fangwei_jiaodu_diwei;			  //��λ�Ƕȵ�λ
u8 fuyang_jiaodu_diwei;				  //�����Ƕȵ�λ

extern u8 Yuntai_fangwei_shezhi_chenggong_flag; //��̨��λ���óɹ���־
extern u8 Yuntai_fuyang_shezhi_chenggong_flag;	//��̨�������óɹ���־


u32 fy_ms_count;			 //����Ŀ��ʱ��
int fy_ms_count_temp;		 //����Ŀ��ʱ���м�ֵ
u32 fw_ms_count;			 //��λĿ��ʱ��
int fw_ms_count_temp;	 //��λĿ��ʱ��
extern u8 RS422_receive_str[30]; //���ڽ�������
u8 AT24c256_storage[30];		 //��洢оƬ�������

u8 fy_dir;	   //�����������ת��ת��ֹͣ
u8 fw_dir;	   //����λ�����ת��ת��ֹͣ
u8 AT240c_str[30]; // �ڴ洢оƬ�д��������
extern u8 Yuntai_ID_flag;
extern u8 Yuntai_zhuangtai_flag;
extern u8 Yuntai_kongzhi_flag;
extern u8 Yuntai_jiaodushezhi_flag;
extern u8 Yuntai_chushishezhi_flag;
extern u8 Yuntai_yingdafangshi_flag;

///////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
double fw_currentdeg=0.0; //��λ��ǰ�Ƕ�
double fy_currentdeg=0.0; //������ǰ�Ƕ�
double last_fw_currentdeg=0.0; //�ϴη�λ�Ƕ�
double last_fy_currentdeg=0.0; //�ϴθ����Ƕ�
double fw_deg_offset=0;//��λƫ��Ƕ�
double fy_deg_offset=0;//����ƫ��Ƕ�
short fuyang_Targetjiaodu;		 //����Ŀ��Ƕ�
short fangwei_Targetjiaodu;		 //��λĿ��Ƕ�
///////////////////////////////////////////////////////
unsigned long gul_FirstFreq;  //��������ĵ�һƵ�ʣ�ͨ����ֱ�ӵ�����ٶȼ���õ��Ŀ�ֱ�ӵ����Ƶ��
unsigned long gul_FirstFreq1; //��λ����ĵ�һƵ��
unsigned char guc_StepM;	  //ϸ�����ã�1��2��8��16
unsigned char guc_StepM1;
unsigned int gui_BeiYong; //����
unsigned int gui_BeiYong1;
float gf_MotorStep;					   //�������һ����ת�Ķ���1.8
double gul_jiaodu;					   //�����Ƕ�ֵ
double fy_cos_value;				   //�����Ƕ�cosֵ
double fw_cos_value;				   //��λ�Ƕ�cosֵ
double gul_jiaodu1;					   //��λ�Ƕ�ֵ
u8 fuyang_yundong_daowei_flag;		   //�����˶���λ��־1��ʾ�˶�ִ�����2��ʾ�������
u8 fangwei_yundong_daowei_flag;		   //��λ�˶���λ��־0��ʾ�����˶���1��ʾ��λ
unsigned short psc;					   //���ø��������Ƶ��
unsigned short psc1;				   //���÷�λ�����Ƶ��
extern AMIS30543_CR AMIS30543_CR_SPI1; //���ø�������Ľṹ��
extern AMIS30543_CR AMIS30543_CR_SPI2; //���÷�λ����Ľṹ��
extern double fuyang_TargettPos_temp;  //����Ŀ��λ�ü����м�ֵ
u8 RS232_send_str[30];				   //�洢��ѯ������
u8 RS232_send_zhuanyi_str[30];		   //�洢ת����ȫ������
u8 RS232_tingzhi_str[30];
u8 RS232_Auto_output[6] = {0x68, 0x05, 0x00, 0x0C, 0x01, 0x12};														//���õ������̵ķ�������
u8 RS232_yuntaiID_huifu[14] = {0xc0, 0x81, 0x00, 0x07, 0x00, 0x00, 0x01, 0x14, 0x08, 0x01, 0x00, 0xAD, 0xFE, 0xc0}; //�ظ���̨ID������
u8 RS232_yuntaijiaodushezhi_huifu[8] = {0xc0, 0x84, 0x00, 0x01, 0x01, 0x34, 0xD9, 0xc0};							//�Ƕ����ûظ�����
u8 RS232_yuntaikongzhi_huifu[8] = {0xc0, 0x83, 0x00, 0x01, 0x01, 0x65, 0xF4, 0xc0};									//ת̨���ƻظ�����
u8 RS232_yuntaikongzhi_shibai_huifu[8] = {0xc0, 0x83, 0x00, 0x01, 0x00, 0x75, 0xD5, 0xc0};							//ת̨����ʧ������
u8 RS232_lingwei_shezhi_huifu[11] = {0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3D, 0xCA};
extern u32 fy_timecnt;	  //��time��ʱ���м���λ�õļ�ʱ����
void jiaodu_send(void);		  //�ظ���λ�Ͳ�ѯ�ĺ���
void AT24c256_dta_Anal(void); //�����洢���ݵĺ���
int  Yuntai_kongzhi(void);
int  Yuntai_jiaodushezhi(void);
int  SysInit(void);
////////////////////////////////////////////////

//////////////////////////////////////////////
int  SysInit(void)
{
		fy_dir = 2;															 //�ϵ��ʾ�����������ֹͣ״̬
		fw_dir = 2;														  //�ϵ��ʾ��λ�������ֹͣ״̬
		fy_reset_flag = 1;														  //�����ܲ�����λ�õ�һ����������Ϊ��ռ��Ҫ����������λ�ã������������Ϊ1
		fw_reset_flag = 1;														  //����λ���һ�ϵ��ǲ�����λ��
		gf_MotorStep = 1.8;														  //����Ĳ����Ƕ�Ϊ1.8��
		guc_StepDivisor1 = 7;													  //ѡ��128ϸ��
		guc_WorkCurrent = 3;													  //���ø�������Ĺ�������
		guc_WorkCurrent1 = 3;													  //���÷�λ����Ĺ�������
		guc_StepDivisor = 7;													  //ѡ�񲽽����������ϸ����
		gui_BeiYong = 60;														  //�ٶ�5������ٶȣ��ڼ���Ƶ�ʵ�ʱ��û������������������������õ�û�����壬��ʼ��0 �Ĺ���Ҳ�������óɼӼ��ٵĹ���
	
		AMIS30543_INIT1();														  //����������ò������ó�ʼ��
		AMIS30543_INIT2();														  //��λ������ò������ó�ʼ��
		AT24CXX_Init();															  //�洢оƬ���ó�ʼ��
		AT24CXX_Read(0, (u8 *)AT240c_str, 30);									  //���洢оƬ�����ݶ�����
		//gul_FirstFreq = (unsigned int)((6.0 / gf_MotorStep) * guc_StepM * 120);	  //���������Ƶ������Ϊ��Ӧ��120rpm    //gul_setting_TargetFreq;	//(unsigned int)(6.0/gf_MotorStep)*guc_StepM*gui_BeiYong;
		//gul_FirstFreq1 = (unsigned int)((6.0 / gf_MotorStep) * guc_StepM1 * 120); //��λ�����Ƶ������Ϊ��Ӧ��150rpm
		
		gul_FirstFreq = 120.0/60*(360.0/gf_MotorStep)*guc_StepM;//ÿ����������	  //���������Ƶ������Ϊ��Ӧ��120rpm    //gul_setting_TargetFreq;	//(unsigned int)(6.0/gf_MotorStep)*guc_StepM*gui_BeiYong;
		gul_FirstFreq1 = 120.0/60*(360/gf_MotorStep)*guc_StepM1;	  //���������Ƶ������Ϊ��Ӧ��120rpm    //gul_setting_TargetFreq;	//(unsigned int)(6.0/gf_MotorStep)*guc_StepM*gui_BeiYong;
		
		
		psc = (unsigned short)(72000000 / gul_FirstFreq);	//ÿ������PWM��Ӧ��װ���ڶ�ʱֵ	1406//���㸩������ٶȶ�Ӧ�Ķ�ʱ�����������
		psc1 = (unsigned short)(72000000 / gul_FirstFreq1);						  //���㷽λ����ٶȶ�Ӧ�Ķ�ʱ�����������
		RS232_Send_Data(RS232_Auto_output, 6);									  //��������ǵĺ���
		AT24c256_dta_Anal();													  //�����洢������
		AMIS30543_NXT1_init(MOTOR_PWM_ARR - 1, (u16)((MOTOR_PWM_ARR - 1) / 2));//����PWM��ÿ��51208����������
		AMIS30543_NXT2_init(MOTOR_PWM_ARR - 1, (u16)((MOTOR_PWM_ARR - 1) / 2));
		
		return 0;
		
}
///////////////////������//////////////////////////////////////
int main(void)
{
	
	u8 count=0;																  //���ڸ�ֵ��λ�ļ���
	
	delay_init();															  //��ʱ������ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);							  //����ϵͳ�ж����ȼ�����2
	uart_init(115200);														  //����λ����ͨѶ����������Ϊ115200
	UARST2_Init(9600);														  //��������̵�ͨѶ����������Ϊ9600
	GPIO_Jiance_Init();														  //��λ����ʼ��
		
	AMIS30543_Init1();														  //�������������ʼ��
	AMIS30543_Init2();														  //��λ�����ʼ��
	TIM1_Int_Init(9, 7199 , ENABLE);														  //1ms��ʱ����ʼ��
	
	SysInit();
	
	
	while (1)
	{
		
		///////////////��λ��ָ�����//////////////////
		if (RS232_receive_flag == 1) //��ʾ���յ���λ��������
		{
				RS232_receive_flag = 0;		  //��־λ����
				RS232_data_Anal();			  //���ݽ�������
				memset(RS422_receive_str , 0 ,sizeof(RS422_receive_str));
			  RS422_byte_count=0;				
			
		}//end ��λ��ָ�����
	
		////////////��λ����̨����ָ��////////////////////
		Yuntai_kongzhi();				
		
		////////////��λ����ѯIDָ��//////////////////////////
		if (Yuntai_ID_flag == 1) //�豸ID��־
		{
			Yuntai_ID_flag = 0;						   //��־����
			RS485_Send_Data(RS232_yuntaiID_huifu, 14); //�ظ���λ����ǰ���豸ID
		}
		//////////////��λ����ѯ��ǰ״ָ̬��////////////////////////
		if (Yuntai_zhuangtai_flag == 1) //��ʾ���յ���λ���Ĳ�ѯָ��
		{
			Yuntai_zhuangtai_flag = 0; //��־����
			jiaodu_send();			   //��ѯ�ظ�����
		}
				
		////////////////��λ���Ƕ�����ָ��////////////////
					
		switch(Yuntai_jiaodushezhi_flag)
		{
			case 1:
				Yuntai_jiaodushezhi();	
				break;
			case 2:
				break;
			case 3:
				fuyang_Mortor_Decel(0);//���������������ֹͣ�˶�

				break;
			case 4:

				fangwei_Mortor_Decel(0);//��������ֹͣ�˶�
				
				break;
			default:break;
			
		}
		
		///////////////////�����������ݽ���/////////////////////////////////////////////
		if (Uart2Ok == 1) //�������̽��յ����ݵı�־
		{
			Uart2Ok = 0;
			dianziluopan_Anal_Data(); //���̽�������
		}
		//////////////////��λ�˶�ѭ���ж�/////////////////////////////////////
		if (guc_10mscnt >= 2) //ʱ��Խ�̵��ڵ�Խ�죬���ٶȹ���Խ��
		{
				guc_10mscnt = 0;
			
				//////////////////////��λ�����λ����////////////////////////////////////
				if ((0 == Fangwei_Lmt1) && (fw_dir == 1)&&(!fw_reset_flag)) //gb_SHUN_NI1�����жϣ���Ϊֻ�е��������������̵ķ����˶�Ȼ������0λ�˶��Ž���γ����������һֱ����һ�γ��򣬵��µ���޷����
				{
						TIM_Cmd(TIM5, DISABLE);	 //���ֹͣ
						fw_dir = 2;				 //��־�ŵ������ֹͣ״̬
						fangwei_yundong_daowei_flag = 1; //��ʾ�������ֹͣ״̬
						fw_ms_count_temp=0;
						fw_currentdeg = 0;				 //��ʾ��λ����ĳ��᳤��ʵ��Ϊ0
						if (fw_reset_flag == 1)
						{
								AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
								AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Disable; //���δ����
								SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);
						}
						else  if (fw_reset_flag == 0) ////�жϵ�ǰ��Ҫ��λ�����0
						{
								FW_DIR = 0;				 //�Ƹ��������������ʹ��
								fw_dir = 0;				 //��λ�����ת
								fw_timecnt = 0;				 //��λ�����ʱ����ʼ
								fangwei_yundong_daowei_flag = 0; //��ʾ��������˶�״̬
								fw_reset_flag = 1;				 //�˳������λ�ı�־
								fw_Init_flag = 1;

								fw_ms_count_temp = 0.0/FANGWEI_DEG_PER_MS; //PWM�������ms��
						
								fw_ms_count = fw_ms_count_temp;
								
								fw_TargetDeg = 0;
								for (count = 0; count < 12; count++)
								{
									AT24c256_storage[count] = RS232_lingwei_shezhi_huifu[count];
								}
								TIM_Cmd(TIM5, ENABLE);			 //����ٶ����ú���
								fangwei_shezhi_guocheng_flag = 1;
								
						}
				}
				
				//////////////////////���������λ����////////////////////////////////////
				if ((0 == Fuyang_Lmt1) && (fy_dir == 1)&&(!fy_reset_flag)) 
				{
						TIM_Cmd(TIM4, DISABLE);
						fuyang_yundong_daowei_flag = 1;
						fy_ms_count_temp=0;
						fy_dir = 2;
						fy_currentdeg = 0;//��ʾ��������ĳ��᳤��ʵ��Ϊ0
						fy_timecnt = 0;
						if (fy_reset_flag == 1)
						{
								AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
								AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Disable; //���δ����
								SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);
						}
						else  if (fy_reset_flag == 0)//�жϵ�ǰ��Ҫ���������0
						{
								fuyang_yundong_daowei_flag = 0;					
								fy_reset_flag = 1;
								FY_DIR = 0;
								fy_dir = 0;
								fy_timecnt = 0;
								fy_Init_flag = 1;
								fy_ms_count_temp = 0.0/FUYANG_DEG_PER_MS; //PWM�������ms��
						
								for (count = 0; count < 12; count++)
								{
									AT24c256_storage[count] = RS232_lingwei_shezhi_huifu[count];
								}
								fy_ms_count = fy_ms_count_temp;
								fuyang_shezhi_guocheng_flag = 1;
								TIM_Cmd(TIM4, ENABLE);
								
						}
				}
				
		}//end if (guc_10mscnt >= 2)��λ�˶���ʱ�ж�

		
	}
	
}





///////////////////������λ���Ƕ�����ָ��////////////////////////////////////////
int Yuntai_jiaodushezhi()
{
		if (Yuntai_jiaodushezhi_flag == 1)
		{
			if (Yuntai_yingdafangshi_flag == 0)
			{
				Yuntai_jiaodushezhi_flag = 0; //��λ�������˶��ǶȺ󣬻������ֻ��һ��
				
				if ((Yuntai_fangwei_shezhi_chenggong_flag == 1) && (Yuntai_fuyang_shezhi_chenggong_flag == 1))
				{
					RS485_Send_Data(RS232_yuntaijiaodushezhi_huifu, 8);
				}
				else if ((Yuntai_fangwei_shezhi_chenggong_flag == 0) || (Yuntai_fuyang_shezhi_chenggong_flag == 0))
				{
					RS485_Send_Data(RS232_yuntaikongzhi_shibai_huifu, 8);
				}
				
				/////////////////���ø�������///////////////////////
				if (Yuntai_fuyang_shezhi_chenggong_flag == 1)
				{
						Yuntai_fuyang_shezhi_chenggong_flag=0;
					
						if (fuyang_yundong_daowei_flag == 1)
						{
							fy_timecnt = 0;//��������˶�1ms�ж��жϼ���
							last_fy_currentdeg = fy_currentdeg;
						}
						
						fy_deg_offset = fy_TargetDeg-fy_currentdeg;
						
						fy_ms_count_temp = fy_deg_offset/FUYANG_DEG_PER_MS;
						
						if (fy_ms_count_temp == 0)
						{
							TIM_Cmd(TIM4, DISABLE);
							TIM_Cmd(TIM3, 0);  //�رն�ʱ��3
							fy_dir = 2;//�������״ֹ̬ͣ
							fy_ms_count = 0;
							fuyang_yundong_daowei_flag = 1;//�˶���ϱ�־
							fy_timecnt = 0;
							fuyang_shezhi_guocheng_flag = 0;

						}						
						else  if (fy_ms_count_temp > 0)
						{
								TIM_Cmd(TIM4, 0);  //�رն�ʱ��4
								TIM_Cmd(TIM3, 0);  //�رն�ʱ��3
								
								fy_ms_count = fy_ms_count_temp;														
								AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
								AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //���δ����
								SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);
								FY_DIR = 1;
								fy_dir = 1;//�������״̬��ת
								TIM4_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//ֱ�����趨�ٶ�����
								fuyang_yundong_daowei_flag = 0;//��0�˶���ɱ�־����ʼ�˶�
								TIM_Cmd(TIM4, ENABLE);
								TIM_Cmd(TIM1, ENABLE);
								
								fy_timecnt = 0;
								fuyang_shezhi_guocheng_flag = 1;
															
						}
						else  if (fy_ms_count_temp < 0)
						{
							TIM_Cmd(TIM4, 0);  //�رն�ʱ��4
							TIM_Cmd(TIM3, 0);  //�رն�ʱ��3
							fy_ms_count = abs(fy_ms_count_temp);
							AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
							AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //���δ����
							SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);
							TIM4_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//ֱ�����趨�ٶ�����					
							FY_DIR = 0;
							fy_dir = 0;//�������״̬��ת
							fuyang_yundong_daowei_flag = 0;
							
							fy_timecnt = 0;
							fuyang_shezhi_guocheng_flag = 1;
							TIM_Cmd(TIM4, ENABLE);
							TIM_Cmd(TIM1, ENABLE);
						}			
				}
				/////////////////���÷�λ����///////////////////////
				if (Yuntai_fangwei_shezhi_chenggong_flag == 1)
				{
						Yuntai_fangwei_shezhi_chenggong_flag=0;	
						TIM_Cmd(TIM2, 0);  //�رն�ʱ��2
						if (fangwei_yundong_daowei_flag == 1)
						{
							fw_timecnt = 0;
							last_fw_currentdeg = fw_currentdeg;
						}
						
						fw_deg_offset = fw_TargetDeg-fw_currentdeg;
						
						fw_ms_count_temp = fw_deg_offset/FANGWEI_DEG_PER_MS;
						if (fw_ms_count_temp == 0)
						{
							fw_dir = 2;//��λ���״ֹ̬ͣ
							TIM_Cmd(TIM5, DISABLE); //��λֹͣ
							TIM_Cmd(TIM2, DISABLE);
							TIM_DeInit(TIM2);
							fw_ms_count = 0; //��λĿ��ʱ������
							fw_timecnt = 0;
							fangwei_yundong_daowei_flag = 1;
							fangwei_shezhi_guocheng_flag = 0;
		
						}						
						else if (fw_ms_count_temp > 0)
						{
							TIM_Cmd(TIM5, DISABLE); //��λֹͣ
							TIM_Cmd(TIM2, DISABLE);
							TIM_DeInit(TIM2);
							fw_ms_count = fw_ms_count_temp;
							AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
							AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //���δ����
							SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);

							FW_DIR = 1;
							fw_dir = 1;//��λ���״̬��ת
							TIM5_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//ֱ�����趨�ٶ�����	
							fangwei_yundong_daowei_flag = 0;
							fw_timecnt = 0; //��λ��ʱ����
							fangwei_shezhi_guocheng_flag = 1;
														
							TIM_Cmd(TIM5, ENABLE);
							TIM_Cmd(TIM1, ENABLE);
							
						}						
						else if (fw_ms_count_temp < 0)
						{
							TIM_Cmd(TIM5, DISABLE); //��λֹͣ
							TIM_Cmd(TIM2, DISABLE);
							TIM_DeInit(TIM2);
							fw_ms_count = abs(fw_ms_count_temp);
							AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
							AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //���δ����
							SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);
							
							FW_DIR = 0;

							TIM5_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//ֱ�����趨�ٶ�����		

							fw_dir = 0;//��λ���״̬��ת
							
							fangwei_yundong_daowei_flag = 0;
							fw_timecnt = 0; //��λ��ʱ����
							fangwei_shezhi_guocheng_flag = 1;							
				
							TIM_Cmd(TIM5, ENABLE);
							TIM_Cmd(TIM1, ENABLE);	
							
						}			
					
				}
				
				
			}
		}//end ��λ���Ƕ�����ָ��
		
		return 0;
}

/////////////////////������λ����̨����//////////////////////////////////////
int  Yuntai_kongzhi()
{
		if (Yuntai_kongzhi_flag == 1) //���յ���̨����ָ��
		{
			Yuntai_kongzhi_flag = 0;	//����ָ���־����
			if (Yuntai_fuwei_flag == 1) //��λ����
			{
				RS485_Send_Data(RS232_yuntaikongzhi_huifu, 8); //����Ӧ��û������
				AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
				AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //���δ����
				SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);					//ʹ�ܸ������
				AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
				AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //���δ����
				SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);					//ʹ�ܷ�λ���
				Yuntai_fuwei_flag = 0;													//��λָ��ֻ��һ��û������
				FW_DIR = 1;														//�͸����ķ�����ͬ����λ�����DIRΪ0ʱ�������λ����
				fw_dir = 1;														//��־�����ת
				FY_DIR = 1;														//�͸����ķ�����ͬ�����������DIRΪ0ʱ�������λ����
				fy_dir = 1;															//��־���������ת
				fy_reset_flag = 0;														//��־��������˶����븴λ״̬
				fw_reset_flag = 0;														//��־��λ������˶������˸�λ״̬
				if (Fuyang_Lmt1 == 1)													//��ʾ�������û������λ
				{
					TIM_Cmd(TIM4, ENABLE); //�����������
				
					fuyang_yundong_daowei_flag = 0;//��������˶���־
				}
				else
				{
					TIM_Cmd(TIM4, DISABLE); //����ֹͣ
				}
				
				if (Fangwei_Lmt1 == 1)
				{
					TIM_Cmd(TIM5, ENABLE); //������λ���
					fangwei_yundong_daowei_flag = 0;//��λ����˶���־

				}
				else
				{
					TIM_Cmd(TIM5, DISABLE);
				}
				
			}
			
			
			if (Yuntai_tingzhi_flag == 1)//ָֹͣ��
			{
				Yuntai_tingzhi_flag = 0;
				RS485_Send_Data(RS232_yuntaikongzhi_huifu, 8);
				if ((fw_reset_flag == 1) && (fy_reset_flag == 1))//�жϵ�ǰ���Ǹ�λ�˶���,���������ֶ�ֹͣ
				{
					if (fw_ms_count_temp > 0) //ֹͣʱ��ķ�λ�Ƕ�
					{
						TIM_Cmd(TIM5, DISABLE);
						fangwei_yundong_daowei_flag =1;						
						fw_ms_count = fw_timecnt;		 //��λֹͣʱ��ʱʱ���¼����
						fangwei_yundong_daowei_flag = 1;			 //��λ�˶���λ��־
						fw_dir = 2;							 //��ʾֹͣ
					}
					else if (fw_ms_count_temp < 0)
					{
						TIM_Cmd(TIM5, DISABLE);
						
						fangwei_yundong_daowei_flag =1;					
						fw_ms_count = fw_timecnt; //��λ��ʱʱ��
						fw_dir = 2;
					}
					if (fy_ms_count_temp > 0)
					{
						TIM_Cmd(TIM4, DISABLE);
						fuyang_yundong_daowei_flag =1;						
						fy_ms_count = fy_timecnt;
						fy_dir = 2;
					}
					else if (fy_ms_count_temp < 0)
					{
						TIM_Cmd(TIM4, DISABLE);
						fuyang_yundong_daowei_flag =1;						
						fy_ms_count = fy_timecnt;
						fy_dir = 2;
					}
	
					AT24c256_storage[0] = 0x04;
					AT24c256_storage[1] = 0x00;
					AT24c256_storage[2] = 0x06;
					AT24c256_storage[3] = 0x00;
					AT24c256_storage[4] = (u8)((gul_fangwei_jiaodu & 0xff00) >> 8);
					AT24c256_storage[5] = (u8)(gul_fangwei_jiaodu & 0x00ff);
					AT24c256_storage[6] = (u8)((gul_fuyang_jiaodu & 0xff00) >> 8);
					AT24c256_storage[7] = (u8)(gul_fuyang_jiaodu & 0x00ff);
					AT24c256_storage[8] = 00;
					crc_check = cal_crc(AT24c256_storage, 9);
					AT24c256_storage[9] = (crc_check & 0xFF00) >> 8;
					AT24c256_storage[10] = (u8)(crc_check & 0x00FF);
				}
			}
		}//end ��λ����̨����ָ��
		return  0;
}

//////////////////�ظ���λ���ǶȲ�ѯָ��////////////////////////////
void jiaodu_send()
{
	u8 i;
	u8 t;
	i = 0;
	t = 0;
	u8  fy_stat=0,fw_stat=0 ;
	RS232_send_str[0] = 0xc0;												 //֡ͷ
	RS232_send_str[1] = 0x82;												 //�����֣��̶��ֽ�
	RS232_send_str[2] = 0x00;												 //���ݳ��� ���̶��ֽ�

	gul_fangwei_jiaodu = fw_currentdeg;//��λ�Ƕ�

	gul_fuyang_jiaodu = fy_currentdeg;//�����Ƕ�
										
	if ((fuyang_yundong_daowei_flag == 1) && (fangwei_yundong_daowei_flag == 1))//�ж�����������˶���ϣ��ر�ʹ��ʡ��
	{
		AT24CXX_Write(0, (u8 *)AT24c256_storage, 11);//����λ������
		fuyang_yundong_daowei_flag = 2;//���ø����˶���־Ϊ2.�˶���ɣ�ʵʱλ���Ѿ�д��EEPROM
		fangwei_yundong_daowei_flag = 2;
		AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
		AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Disable; //�رո������ʹ��
		SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);
		AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
		AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Disable; //�رշ�λ���ʹ��
		SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);
	}

	RS232_send_str[4] = (u8)((gul_fangwei_jiaodu & 0xFF00) >> 8);//��λ�Ƕ�
	fangwei_jiaodu_diwei = (u8)(gul_fangwei_jiaodu & 0xFF);
	RS232_send_str[5] = fangwei_jiaodu_diwei;
	RS232_send_str[6] = 0;//������λ
	RS232_send_str[7] = 0;//������λ
	RS232_send_str[8] = (u8)((gul_fuyang_jiaodu & 0xFF00) >> 8); //�����Ƕȸ�8λ
	fuyang_jiaodu_diwei = (u8)(gul_fuyang_jiaodu & 0x00FF);
	RS232_send_str[9] = fuyang_jiaodu_diwei;
	RS232_send_str[10] = (u8)((shuipingfuyangjiao & 0xff00) >> 8);//ˮƽ������
	shuipingfuyangjiao_diwei = (u8)(shuipingfuyangjiao & 0xff);
	RS232_send_str[11] = shuipingfuyangjiao_diwei;

	if ((0 == Fuyang_Lmt1) && (0 == Fangwei_Lmt1))//��λ״̬
	{
		RS232_send_str[12] = 0x09;
	}
	else if ((0 == Fuyang_Lmt1) && (1 == Fangwei_Lmt1))
	{
		RS232_send_str[12] = 0x01;
	}
	else if ((1 == Fuyang_Lmt1) && (0 == Fangwei_Lmt1))
	{
		RS232_send_str[12] = 0x08;
	}
	else if ((1 == Fuyang_Lmt1) && (1 == Fangwei_Lmt1))
	{
		RS232_send_str[12] = 0x00;
	}
	if ((AMIS30543_ERR1 == 0) && (AMIS30543_ERR2 == 0))//�澯��Ϣ
	{
		RS232_send_str[13] = 0x03;
	}
	else if ((AMIS30543_ERR1 == 1) && (AMIS30543_ERR2 == 1))
	{
		RS232_send_str[13] = 0x00;
	}
	else if ((AMIS30543_ERR1 == 1) && (AMIS30543_ERR2 == 0))
	{
		RS232_send_str[13] = 0x02;//��������쳣
	}
	else if ((AMIS30543_ERR1 == 0) && (AMIS30543_ERR2 == 1))
	{
		RS232_send_str[13] = 0x01;//��λ����쳣
	}
	switch(fw_dir)
	{
		case 0:fy_stat = 1;//���������ת
		break;
		case 1:fy_stat = 2;//���������ת
		break;
		case 2:fy_stat = 0;//�������ֹͣ
		break;
		default:break;
	}

	switch(fy_dir)
	{
		case 0:fw_stat = 1;//��λ�����ת
		break;
		case 1:fw_stat = 2;//��λ�����ת
		break;
		case 2:fw_stat = 0;//��λ���ֹͣ
		break;
		default:break;
	}
	
	RS232_send_str[14] = (fy_stat<<2) + fw_stat;//�ϲ������ͷ�λ���״̬

	
	if (((fy_reset_flag == 0) || (fw_reset_flag == 0)) && (AMIS30543_ERR1 == 1) && (AMIS30543_ERR2 == 1))
	{
		RS232_send_str[15] = 0x02;//���߹���״̬��λ
	}
	else if (((fy_dir != 2) || (fw_dir != 2)) && (AMIS30543_ERR1 == 1) && (AMIS30543_ERR2 == 1))
	{
		RS232_send_str[15] = 0x03;//��ת
	}
	else if ((fy_dir == 2) && (fw_dir == 2) && (AMIS30543_ERR1 == 1) && (AMIS30543_ERR2 == 1))
	{
		RS232_send_str[15] = 0x04;//���߹���״̬-ֹͣ
	}
	else if ((AMIS30543_ERR1 == 0) || (AMIS30543_ERR2 == 0))
	{
		RS232_send_str[15] = 0x05;
	}
	RS232_send_str[3] = 12;
	crc_fasong_Result = cal_crc(&(RS232_send_str[1]), 15);
	crc_fasong_gaowei = (u8)((crc_fasong_Result >> 8) & 0xff);
	RS232_send_str[16] = crc_fasong_gaowei;
	crc_fasong_diwei = (u8)(crc_fasong_Result & 0xff);
	RS232_send_str[17] = crc_fasong_diwei;
	RS232_send_zhuanyi_str[0] = 0xc0;
	t = t + 1;
	for (i = 1; i < 18; i++) //�жϷ��͵��ֽ����Ƿ���0xc0;
	{
		if (RS232_send_str[i] == 0xc0)
		{
			RS232_send_zhuanyi_str[t] = 0xdb;
			t++;
			RS232_send_zhuanyi_str[t] = 0xdc;
			t++;
		}
		else if (RS232_send_str[i] == 0xdb)
		{
			RS232_send_zhuanyi_str[t] = 0xdb;
			t++;
			RS232_send_zhuanyi_str[t] = 0xdd;
			t++;
		}
		else
		{
			RS232_send_zhuanyi_str[t] = RS232_send_str[i];
			t++;
		}
	}
	RS232_send_zhuanyi_str[t] = 0xc0;
	RS485_Send_Data(RS232_send_zhuanyi_str, (t + 1));

}

/////////////////////��ȡEEPROM�洢������//////////////////////////////////////////
void AT24c256_dta_Anal()
{
	u8 flag = 0;
	short data_changdu;
	data_changdu = (short)((short)(AT240c_str[1] << 8) | AT240c_str[2]);
	if (flag == 0)
	{
		crc_check = cal_crc(AT240c_str, (data_changdu + 3));
		crc_result1 = (short)((AT240c_str[data_changdu + 3] << 8) | AT240c_str[data_changdu + 4]);
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
		if (AT240c_str[0] == 0x04)
		{
			flag++;
		}
		else
		{
			flag = 0;
		}
	}
	if (flag == 2)
	{
		fw_TargetDeg = (short)((AT240c_str[4] << 8) | AT240c_str[5]);
		if ((fw_TargetDeg >= -18000) && (fw_TargetDeg <= 18000))
		{
			fangwei_yundong_daowei_flag = 1; //��λ�����
		}
		fy_TargetDeg = (short)((AT240c_str[6] << 8) | AT240c_str[7]);
				
		if ((fy_TargetDeg >= -300) && (fy_TargetDeg <= 500))
		{
			
			fuyang_yundong_daowei_flag = 1; //��λ�����
		}
		fy_Init_flag = 1;
		fw_Init_flag = 1;
		
	}
}


