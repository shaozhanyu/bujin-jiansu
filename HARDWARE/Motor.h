#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "stm32f10x.h"
#include "AMIS30543.h"
////////////////////////////////////////////////////////////
#define  MAX_SPEED   120.00 //最大运行速度120转/分钟 由于加减速的需要，
//每次修改PWM时，为了更准确不产生小数点，从PWM设定值设计为整数倍，所以转速导致不是整数值，没关系
#define  DEC_TIME_VALUE  2.0
#define  ACCEL_NUM   10.0  //速度分段加速分片次数
#define  MOTOR_PWM_ARR          2800.0  //128细分，200步，2圈/S则51200脉冲数/秒。对应72000000/51200 =1406.25。为了加速设定不产生小数取1400
#define  FY_MOTOR_PWM_ARR       2800.0  //128细分，200步，2圈/S则51200脉冲数/秒。对应72000000/51200 =1406.25。为了加速设定不产生小数取1400

#define  ACCEL_TIME_COUNT_VALUE   (72.0*(DEC_TIME_VALUE/ACCEL_NUM)*1000000)  //不分频72M时0.2S加速时间片的PWM输出次数值，用于和PWM重装值比较，可以知道时间片内输出了多少PWM脉冲
#define  ONE_MS_TIME_COUNT_VALUE  (72.0*1000/MOTOR_PWM_ARR)  //1ms时间，PWM由于没有分频，则1ms时间走过PWM输出的脉冲数MOTOR_PWM_ARR的N倍
#define  MM_PER_STEP              ((1.27/200)/128.0)
#define  STEP_COUNT_PER_MM        (128.0*200.0/1.27)    //每mm包含的脉冲数，即脉冲当量
#define  FUYANG_STEP_COUNT_PER_MM        (128.0*200.0/1.5875)    //每mm包含的脉冲数，即脉冲当量
#define  FUYANG_MM_PER_STEP              ((1.5875/200.0)/128.0)

#define  FANGWEI_DEG_PER_MS               (1.0*360/90.0/1000.0*100)   //每ms方位电机转动的角度0.01度分辨率。120r/m
#define  FUYANG_DEG_PER_MS                (1.0*360/252.0/1000.0*100)   //每ms俯仰电机转动的角度。120r/m

////////////////////////////////////////////////////////
extern  double fw_currentdeg; //方位当前角度
extern  double fy_currentdeg; //俯仰当前角度
extern  double last_fw_currentdeg; //上次方位角度
extern  double last_fy_currentdeg; //上次俯仰角度
extern  double fw_deg_offset;//方位偏差角度
extern  double fy_deg_offset;//俯仰偏差角度
extern  short fuyang_Targetjiaodu;		 //俯仰目标角度
extern  short fangwei_Targetjiaodu;		 //方位目标角度
/////////////////////////////////////////////////////////////
extern  int   WipeMotorFlag ;//期望电机转换为的状态
extern  float  SetSpeed_ref ;
extern  int  update_accel_flag ;
extern  float  NowSpeed ; //当前速度
extern  float  accel_value;//加速度值，按rpm算
extern  float   accel_num ;//加速分段数
extern  float   dec_num;//减速计数
extern  float  add_value ;
extern  float  dec_value ;
extern  u32  every_accel;
extern  u16  t4_pwm_value;//pwm4计数值
extern  u16  t5_pwm_value;//pwm5计数值

extern  float  fuyang_dec_num ; //减速计数
extern  float  fangwei_dec_num ; //减速计数
extern  int  fuyang_xch_dir ;//俯仰电机切换方向标志
extern  int  fangwei_xch_dir ;//方位电机切换方向标志
/////////////////////////////////////////////////////////////
//extern  u8 fangwei_daowei_flag;
//extern  u8 fuyang_daowei_flag;
extern double fy_currentPos;					//俯仰当前位置
extern double fw_currentPos;					//方位当前位置
extern u8 fuyang_yundong_daowei_flag;
extern u8 fangwei_yundong_daowei_flag;			//方位运动到位标志
extern int fw_ms_count_temp;				//方位目标时间
extern int fy_ms_count_temp;				//方位目标时间
extern double last_fy_currentPos;
extern double last_fw_currentPos;
extern double fy_currentPos;
extern AMIS30543_CR AMIS30543_CR_SPI1;
extern AMIS30543_CR AMIS30543_CR_SPI2;
extern int fw_ms_count_temp;
extern double fw_currentPos;
extern u32 guc_timejishi1;



extern char fy_reset_flag;						//用于俯仰找零过程的标志
extern char fw_reset_flag;						//用于方位找零过程的标志
extern u8 fuyang_shezhi_guocheng_flag;			//俯仰设置成功过程标志
extern u8 fangwei_shezhi_guocheng_flag;			//方位设置成功标志
extern char fw_Init_flag;					//上电复位前的标志
extern u32 fw_ms_count;					//方位目标时间
extern u8 fw_dir;							//代表方位电机正转反转或停止
extern u8 fy_dir;

extern u32 fy_ms_count;
extern char fy_Init_flag;
////////////////////////////////////////////////////
extern u8 AT24c256_storage[30];	 //要存储的数据
extern char In_place_flag1;
extern char In_place_flag;
extern u8 fangwei_yundong_daowei_flag;
extern u8 fuyang_yundong_daowei_flag;
extern int fy_ms_count_temp;
extern u32 guc_timejishi;
//////////////////////////////////////////////////
extern u8 Yuntai_fangwei_shezhi_chenggong_flag; //云台方位设置成功标志
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

#define MOTOR_EN(x)			GPIO_WriteBit(MOTOR_EN_PORT,MOTOR_EN_PIN,(BitAction)(x))//1自由状态 0自锁状态
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
