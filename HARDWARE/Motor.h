#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stm32f10x.h"
#include "AMIS30543.h"
////////////////////////////////////////////////////////////
#define  MAX_SPEED   120.00 //��������ٶ�120ת/���� ���ڼӼ��ٵ���Ҫ��
//ÿ���޸�PWMʱ��Ϊ�˸�׼ȷ������С���㣬��PWM�趨ֵ���Ϊ������������ת�ٵ��²�������ֵ��û��ϵ
#define  DEC_TIME_VALUE  2.0
#define  ACCEL_NUM   10.0  //�ٶȷֶμ��ٷ�Ƭ����
#define  MOTOR_PWM_ARR          1400.0  //128ϸ�֣�200����2Ȧ/S��51200������/�롣��Ӧ72000000/51200 =1406.25��Ϊ�˼����趨������С��ȡ1400
#define  ACCEL_TIME_COUNT_VALUE   (72.0*(DEC_TIME_VALUE/ACCEL_NUM)*1000000)  //����Ƶ72Mʱ0.2S����ʱ��Ƭ��PWM�������ֵ�����ں�PWM��װֵ�Ƚϣ�����֪��ʱ��Ƭ������˶���PWM����
#define  ONE_MS_TIME_COUNT_VALUE  (72.0*1000/MOTOR_PWM_ARR)  //1msʱ�䣬PWM����û�з�Ƶ����1msʱ���߹�PWM�����������MOTOR_PWM_ARR��N��
#define  MM_PER_STEP              ((1.27/200)/128.0)
#define  STEP_COUNT_PER_MM        (128.0*200.0/1.27)    //ÿmm�������������������嵱��
#define  FUYANG_STEP_COUNT_PER_MM        (128.0*200.0/1.5875)    //ÿmm�������������������嵱��
#define  FUYANG_MM_PER_STEP              ((1.5875/200.0)/128.0)

#define  FANGWEI_DEG_PER_MS               (2.0*360/90.0/1000.0*100)   //ÿms��λ���ת���ĽǶ�0.01�ȷֱ��ʡ�120r/m
#define  FUYANG_DEG_PER_MS                (2.0*360/252.0/1000.0*100)   //ÿms�������ת���ĽǶȡ�120r/m

////////////////////////////////////////////////////////
extern  double fw_currentdeg; //��λ��ǰ�Ƕ�
extern  double fy_currentdeg; //������ǰ�Ƕ�
extern  double last_fw_currentdeg; //�ϴη�λ�Ƕ�
extern  double last_fy_currentdeg; //�ϴθ����Ƕ�
extern  double fw_deg_offset;//��λƫ��Ƕ�
extern  double fy_deg_offset;//����ƫ��Ƕ�
extern  short fuyang_Targetjiaodu;		 //����Ŀ��Ƕ�
extern  short fangwei_Targetjiaodu;		 //��λĿ��Ƕ�
/////////////////////////////////////////////////////////////
extern  int   WipeMotorFlag ;//�������ת��Ϊ��״̬
extern  float  SetSpeed_ref ;
extern  int  update_accel_flag ;
extern  float  NowSpeed ; //��ǰ�ٶ�
extern  float  accel_value;//���ٶ�ֵ����rpm��
extern  float   accel_num ;//���ٷֶ���
extern  float   dec_num;//���ټ���
extern  float  add_value ;
extern  float  dec_value ;
extern  u32  every_accel;
extern  u16  t4_pwm_value;//pwm4����ֵ
extern  u16  t5_pwm_value;//pwm5����ֵ

extern  float  fuyang_dec_num ; //���ټ���
extern  float  fangwei_dec_num ; //���ټ���
extern  int  fuyang_xch_dir ;//��������л������־
extern  int  fangwei_xch_dir ;//��λ����л������־
/////////////////////////////////////////////////////////////
//extern  u8 fangwei_daowei_flag;
//extern  u8 fuyang_daowei_flag;
extern double fy_currentPos;					//������ǰλ��
extern double fw_currentPos;					//��λ��ǰλ��
extern u8 fuyang_yundong_daowei_flag;
extern u8 fangwei_yundong_daowei_flag;			//��λ�˶���λ��־
extern int fw_ms_count_temp;				//��λĿ��ʱ��
extern int fy_ms_count_temp;				//��λĿ��ʱ��
extern double last_fy_currentPos;
extern double last_fw_currentPos;
extern double fy_currentPos;
extern AMIS30543_CR AMIS30543_CR_SPI1;
extern AMIS30543_CR AMIS30543_CR_SPI2;
extern int fw_ms_count_temp;
extern double fw_currentPos;
extern u32 guc_timejishi1;



extern char fy_reset_flag;						//���ڸ���������̵ı�־
extern char fw_reset_flag;						//���ڷ�λ������̵ı�־
extern u8 fuyang_shezhi_guocheng_flag;			//�������óɹ����̱�־
extern u8 fangwei_shezhi_guocheng_flag;			//��λ���óɹ���־
extern char fw_Init_flag;					//�ϵ縴λǰ�ı�־
extern u32 fw_ms_count;					//��λĿ��ʱ��
extern u8 fw_dir;							//����λ�����ת��ת��ֹͣ
extern u8 fy_dir;

extern u32 fy_ms_count;
extern char fy_Init_flag;
////////////////////////////////////////////////////
extern u8 AT24c256_storage[30];	 //Ҫ�洢������
extern char In_place_flag1;
extern char In_place_flag;
extern u8 fangwei_yundong_daowei_flag;
extern u8 fuyang_yundong_daowei_flag;
extern int fy_ms_count_temp;
extern u32 guc_timejishi;
//////////////////////////////////////////////////
extern u8 Yuntai_fangwei_shezhi_chenggong_flag; //��̨��λ���óɹ���־
extern u8 Yuntai_fuyang_shezhi_chenggong_flag;
extern u8 Yuntai_jiaodushezhi_flag;
extern short gul_fuyang_jiaodu;
extern short gul_fangwei_jiaodu;
/////////////////////////////////////////////////////////////
#define  MotorTIMPeriod     (float)(150.0/(NowSpeed/200.0))
#define  MotorHalfTIMPeriod   (float)(MotorTIMPeriod/2.0)
	
#define MOTOR_EN_PORT		GPIOA
#define MOTOR_EN_PIN		GPIO_Pin_10
#define MOTOR_DIR_PORT		GPIOA
#define MOTOR_DIR_PIN		GPIO_Pin_8

#define MOTOR_EN(x)			GPIO_WriteBit(MOTOR_EN_PORT,MOTOR_EN_PIN,(BitAction)(x))//1����״̬ 0����״̬
#define MOTOR_DIR(x)		GPIO_WriteBit(MOTOR_DIR_PORT,MOTOR_DIR_PIN,(BitAction)(x))
#define MOTOR_DIR_CH    GPIO_WriteBit(MOTOR_DIR_PORT,MOTOR_DIR_PIN, !GPIO_ReadOutputDataBit(MOTOR_DIR_PORT,MOTOR_DIR_PIN));

void MOTOR_Init(void);
//void MOTOR_SetSpeed(u16 Speed);
void Motor_SetSpeed(void);
int   Mortor_Accel( u32  SetSpeed );
int   fuyang_Mortor_Decel( u32  SetSpeed );
int   fangwei_Mortor_Decel( u32  SetSpeed );
extern u16 PulsePerRound;
extern u16 RevolveSpeed;
extern u16 OldRevolveSpeed;

#endif
