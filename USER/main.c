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

char In_place_flag; //俯仰电机找零过程标志
char Init_first_flag;
char Init1_first_flag;
char In_place_flag1;				  //方位电机找零过程标志
unsigned char guc_WorkCurrent;		  //表示俯仰电机工作电流
unsigned char guc_WorkCurrent1;		  //表示方位电机工作电流
unsigned char guc_StepDivisor;		  //设置俯仰电机的细分数
unsigned char guc_StepDivisor1;		  //设置方位电机的细分数
extern u8 Uart1RxTemp[20];			  //电子罗盘接收数据数组
extern u32 guc_1mscnt;				  //1ms定时器
extern u32 guc_2mscnt;				  //2ms定时器
extern u32 guc_10mscnt;				  //10ms定时器
extern u32 guc_50mscnt;				  //50ms定时器
extern u32 guc_100mscnt;			  //100ms定时器
extern u32 guc_1000mscnt;			  //1000ms定时器
extern u8 Uart2Ok;					  //电子罗盘接收数据标志
extern u8 RS232_receive_flag;		  //上位机发送数据标志位
extern u8 Yuntai_fuwei_flag;		  //云台复位标志
extern short shuipingfuyangjiao;	  //水平俯仰角
extern short crc_check;				  //crc计算结果
extern short crc_result1;			  //接收到的crc结果
extern double fangwei_Targethudu;	  //方位目标弧度
extern short fangwei_Targetjiaodu;	  //方位目标角度
extern double fangwei_TargetPos_temp; //方位目标位置中间值
u8 shuipingfuyangjiao_diwei;		  //水平俯仰角地位
u8 fuyang_shezhi_guocheng_flag;		  //俯仰设置过程标志
u8 fangwei_shezhi_guocheng_flag;	  //方位设置过程标志
extern double fuyang_Targethudu;	  //俯仰目标弧度
extern short fuyang_Targetjiaodu;	  //俯仰目标角度
extern u32 guc_timejishi1;			  //方位计时标志
extern double fangwei_TargetPos;	  //方位目标位置
extern double fuyang_TargetPos;		  //俯仰目标位置
extern u8 Yuntai_tingzhi_flag;		  //云台停止标志
double fangwei_xiangdui_TargetPos;	  //方位电机相对目标位置
double fuyang_xiangdui_TargetPos;	  //俯仰电机相对目标位置
short gul_fuyang_jiaodu;			  //俯仰角度变量
short gul_fangwei_jiaodu;			  //方位角度变量
short crc_fasong_Result;			  //crc计算存储变量
u8 crc_fasong_gaowei;				  //crc发送高位
u8 crc_fasong_diwei;				  //crc发送低位
u8 fangwei_jiaodu_diwei;			  //方位角度低位
u8 fuyang_jiaodu_diwei;				  //俯仰角度地位

extern u8 Yuntai_fangwei_shezhi_chenggong_flag; //云台方位设置成功标志
extern u8 Yuntai_fuyang_shezhi_chenggong_flag;	//云台俯仰设置成功标志


u32 gul_Targettimeout;			 //俯仰目标时间
int gul_Targettimeout_temp;		 //俯仰目标时间中间值
u32 gul_Targettimeout1;			 //方位目标时间
int gul_Targettimeout1_temp;	 //方位目标时间
extern u8 RS422_receive_str[30]; //串口接收数组
u8 AT24c256_storage[30];		 //向存储芯片存的数据

u8 gb_SHUN_NI;	   //代表俯仰电机正转反转或停止
u8 gb_SHUN_NI1;	   //代表方位电机正转反转或停止
u8 AT240c_str[30]; // 在存储芯片中储存的数据
extern u8 Yuntai_ID_flag;
extern u8 Yuntai_zhuangtai_flag;
extern u8 Yuntai_kongzhi_flag;
extern u8 Yuntai_jiaodushezhi_flag;
extern u8 Yuntai_chushishezhi_flag;
extern u8 Yuntai_yingdafangshi_flag;
double gl_currentPos; //当前位置
double gl_currentPos1;
double last_gl_currentPos;
double last_gl_currentPos1;
unsigned long gul_FirstFreq;  //俯仰电机的第一频率，通过可直接到达的速度计算得到的可直接到达的频率
unsigned long gul_FirstFreq1; //方位电机的第一频率
unsigned char guc_StepM;	  //细分设置，1，2，8，16
unsigned char guc_StepM1;
unsigned int gui_BeiYong; //备用
unsigned int gui_BeiYong1;
float gf_MotorStep;					   //步进电机一步旋转的度数1.8
double gul_jiaodu;					   //俯仰角度值
double cos_gul_jiaodu;				   //俯仰角度cos值
double cos_gul_jiaodu1;				   //方位角度cos值
double gul_jiaodu1;					   //方位角度值
u8 fuyang_yundong_daowei_flag;		   //俯仰运动到位标志1表示运动执行完成2表示保存完成
u8 fangwei_yundong_daowei_flag;		   //方位运动到位标志0表示正在运动。1表示到位
unsigned short psc;					   //设置俯仰电机的频率
unsigned short psc1;				   //设置方位电机的频率
extern AMIS30543_CR AMIS30543_CR_SPI1; //设置俯仰电机的结构体
extern AMIS30543_CR AMIS30543_CR_SPI2; //设置方位电机的结构体
extern double fuyang_TargettPos_temp;  //俯仰目标位置计算中间值
u8 RS232_send_str[30];				   //存储查询的数据
u8 RS232_send_zhuanyi_str[30];		   //存储转义完全的数据
u8 RS232_tingzhi_str[30];
u8 RS232_Auto_output[6] = {0x68, 0x05, 0x00, 0x0C, 0x01, 0x12};														//设置电子罗盘的发送数组
u8 RS232_yuntaiID_huifu[14] = {0xc0, 0x81, 0x00, 0x07, 0x00, 0x00, 0x01, 0x14, 0x08, 0x01, 0x00, 0xAD, 0xFE, 0xc0}; //回复云台ID的数组
u8 RS232_yuntaijiaodushezhi_huifu[8] = {0xc0, 0x84, 0x00, 0x01, 0x01, 0x34, 0xD9, 0xc0};							//角度设置回复数组
u8 RS232_yuntaikongzhi_huifu[8] = {0xc0, 0x83, 0x00, 0x01, 0x01, 0x65, 0xF4, 0xc0};									//转台控制回复数组
u8 RS232_yuntaikongzhi_shibai_huifu[8] = {0xc0, 0x83, 0x00, 0x01, 0x00, 0x75, 0xD5, 0xc0};							//转台控制失败数组
u8 RS232_lingwei_shezhi_huifu[11] = {0x04, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3D, 0xCA};
extern u32 guc_timejishi;	  //在time定时器中计算位置的计时变量
void jiaodu_send(void);		  //回复上位就查询的函数
void AT24c256_dta_Anal(void); //解析存储数据的函数
int  Yuntai_kongzhi(void);
int  Yuntai_jiaodushezhi(void);
int  SysInit(void);
////////////////////////////////////////////////
int  fangwei_ccw[5];
//////////////////////////////////////////////
int  SysInit(void)
{
		gb_SHUN_NI = 2;															  //上电表示俯仰电机处于停止状态
		gb_SHUN_NI1 = 2;														  //上电表示方位电机处于停止状态
		In_place_flag = 1;														  //代表能不能找位置的一个变量，因为何占林要求上来就找位置，所以这个变量为1
		In_place_flag1 = 1;														  //代表方位电机一上电是不是有位置
		gf_MotorStep = 1.8;														  //电机的步进角度为1.8度
		guc_StepDivisor1 = 7;													  //选择128细分
		guc_WorkCurrent = 3;													  //设置俯仰电机的工作电流
		guc_WorkCurrent1 = 3;													  //设置方位电机的工作电流
		guc_StepDivisor = 7;													  //选择步进电机驱动的细分数
		gui_BeiYong = 60;														  //速度5代表的速度，在计算频率的时候没有用这个变量，所以这里设置的没有意义，初始找0 的过程也可以设置成加减速的过程
	
		AMIS30543_INIT1();														  //俯仰电机配置参数设置初始化
		AMIS30543_INIT2();														  //方位电机配置参数设置初始化
		AT24CXX_Init();															  //存储芯片配置初始化
		AT24CXX_Read(0, (u8 *)AT240c_str, 30);									  //将存储芯片的数据读出来
		//gul_FirstFreq = (unsigned int)((6.0 / gf_MotorStep) * guc_StepM * 120);	  //俯仰电机的频率设置为对应的120rpm    //gul_setting_TargetFreq;	//(unsigned int)(6.0/gf_MotorStep)*guc_StepM*gui_BeiYong;
		//gul_FirstFreq1 = (unsigned int)((6.0 / gf_MotorStep) * guc_StepM1 * 120); //方位电机的频率设置为对应的150rpm
		
		gul_FirstFreq = 120.0/60*(360.0/gf_MotorStep)*guc_StepM;//每秒脉冲总数	  //俯仰电机的频率设置为对应的120rpm    //gul_setting_TargetFreq;	//(unsigned int)(6.0/gf_MotorStep)*guc_StepM*gui_BeiYong;
		gul_FirstFreq1 = 120.0/60*(360/gf_MotorStep)*guc_StepM1;	  //俯仰电机的频率设置为对应的120rpm    //gul_setting_TargetFreq;	//(unsigned int)(6.0/gf_MotorStep)*guc_StepM*gui_BeiYong;
		
		
		psc = (unsigned short)(72000000 / gul_FirstFreq);	//每个脉冲PWM对应重装周期定时值	1406//计算俯仰电机速度对应的定时器的输入参数
		psc1 = (unsigned short)(72000000 / gul_FirstFreq1);						  //计算方位电机速度对应的定时器的输入参数
		RS232_Send_Data(RS232_Auto_output, 6);									  //设置倾角仪的函数
		AT24c256_dta_Anal();													  //解析存储的数据
		AMIS30543_NXT1_init(MOTOR_PWM_ARR - 1, (u16)((MOTOR_PWM_ARR - 1) / 2));//设置PWM，每秒51208个方波脉冲
		AMIS30543_NXT2_init(MOTOR_PWM_ARR - 1, (u16)((MOTOR_PWM_ARR - 1) / 2));
		
		return 0;
		
}
///////////////////主函数//////////////////////////////////////
int main(void)
{
	
	u8 count=0;																  //用于赋值零位的计数
	
	delay_init();															  //延时函数初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);							  //设置系统中断优先级分组2
	uart_init(115200);														  //与上位机的通讯波特率设置为115200
	UARST2_Init(9600);														  //与电子罗盘的通讯波特率设置为9600
	GPIO_Jiance_Init();														  //限位检测初始化
		
	AMIS30543_Init1();														  //俯仰电机驱动初始化
	AMIS30543_Init2();														  //方位电机初始化
	TIM1_Int_Init(9, 7199 , ENABLE);														  //1ms定时器初始化
	
	SysInit();
	
	
	while (1)
	{
		
		///////////////上位机指令解析//////////////////
		if (RS232_receive_flag == 1) //表示接收到上位机的数据
		{
				RS232_receive_flag = 0;		  //标志位清零
				RS232_data_Anal();			  //数据解析函数
				memset(RS422_receive_str , 0 ,sizeof(RS422_receive_str));
			  RS422_byte_count=0;				
			
		}//end 上位机指令解析
	
		////////////上位机云台控制指令////////////////////
		Yuntai_kongzhi();				
		
		////////////上位机查询ID指令//////////////////////////
		if (Yuntai_ID_flag == 1) //设备ID标志
		{
			Yuntai_ID_flag = 0;						   //标志清零
			RS485_Send_Data(RS232_yuntaiID_huifu, 14); //回复上位机当前的设备ID
		}
		//////////////上位机查询当前状态指令////////////////////////
		if (Yuntai_zhuangtai_flag == 1) //表示接收到上位机的查询指令
		{
			Yuntai_zhuangtai_flag = 0; //标志清零
			jiaodu_send();			   //查询回复函数
		}
				
		////////////////上位机角度设置指令////////////////
					
		switch(Yuntai_jiaodushezhi_flag)
		{
			case 1:
				Yuntai_jiaodushezhi();	
				break;
			case 2:
				break;
			case 3:
				fuyang_Mortor_Decel(0);//俯仰电机启动减速停止运动

				break;
			case 4:

				fangwei_Mortor_Decel(0);//启动减速停止运动
				
				break;
			default:break;
			
		}
		
		///////////////////电子罗盘数据解析/////////////////////////////////////////////
		if (Uart2Ok == 1) //电子罗盘接收到数据的标志
		{
			Uart2Ok = 0;
			dianziluopan_Anal_Data(); //罗盘解析程序
		}
		//////////////////复位运动循环判断/////////////////////////////////////
		if (guc_10mscnt >= 2) //时间越短调节的越快，加速度过程越好
		{
				guc_10mscnt = 0;
			
				//////////////////////方位电机限位触发////////////////////////////////////
				if ((0 == Fangwei_Lmt1) && (gb_SHUN_NI1 == 0)) //gb_SHUN_NI1必须判断，因为只有电机先向电机出轴最短的方向运动然后再向0位运动才进这段程序否则程序会一直进这一段程序，导致电机无法伸出
				{
						TIM_Cmd(TIM5, DISABLE);	 //电机停止
						gb_SHUN_NI1 = 2;				 //标志着电机处于停止状态
						fangwei_yundong_daowei_flag = 1; //表示电机处于停止状态
						gul_Targettimeout1_temp=0;
						gl_currentPos1 = 0;				 //表示方位电机的出轴长度实际为0
						fangwei_TargetPos=0;
						if (In_place_flag1 == 1)
						{
								AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
								AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Disable; //电机未启动
								SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);
						}
						else  if (In_place_flag1 == 0) ////判断当前是要求方位电机归0
						{
								AMIS30543_DIR2 = 1;				 //推杆向外伸出，方向使能
								gb_SHUN_NI1 = 1;				 //方位电机逆转
								guc_timejishi1 = 0;				 //方位电机计时器开始
								gl_currentPos1 = 0;				 //表示电机的出轴长度
								fangwei_yundong_daowei_flag = 0; //表示电机处于运动状态
								In_place_flag1 = 1;				 //退出电机复位的标志
								fangwei_TargetPos = 15.831697;	 //这是0位标志,复位后，开始归0运动
								 //	gul_Targettimeout1 = (u32)(fangwei_TargetPos*5/2/0.00635);   //这是目标时间标志
								
								Init1_first_flag = 1;
								last_gl_currentPos1 = 0;

								gul_Targettimeout1_temp = fangwei_TargetPos*STEP_COUNT_PER_MM/ONE_MS_TIME_COUNT_VALUE; //PWM输出持续ms数
						
								gul_Targettimeout1 = gul_Targettimeout1_temp;
								
								fangwei_Targetjiaodu = 0;
								for (count = 0; count < 12; count++)
								{
									AT24c256_storage[count] = RS232_lingwei_shezhi_huifu[count];
								}
								TIM_Cmd(TIM5, ENABLE);			 //电机速度设置函数
								fangwei_shezhi_guocheng_flag = 1;
								
						}
				}
				
				//////////////////////俯仰电机限位触发////////////////////////////////////
				if ((0 == Fuyang_Lmt1) && (gb_SHUN_NI == 0)) 
				{
						TIM_Cmd(TIM4, DISABLE);
						fuyang_yundong_daowei_flag = 1;
						gul_Targettimeout_temp=0;
						gb_SHUN_NI = 2;
						gl_currentPos = 0;//表示俯仰电机的出轴长度实际为0
						fuyang_TargetPos = 0;
						guc_timejishi = 0;
						if (In_place_flag == 1)
						{
								AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
								AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Disable; //电机未启动
								SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);
						}
						else  if (In_place_flag == 0)//判断当前是要求俯仰电机归0
						{
								fuyang_yundong_daowei_flag = 0;					
								In_place_flag = 1;
								AMIS30543_DIR1 = 1;
								gb_SHUN_NI = 1;
								guc_timejishi = 0;
								gl_currentPos = 0;
								Init_first_flag = 1;
								fuyang_TargetPos = 22.091; // gl_TargetPos = (long)(Low_limit_initial -(310000/9));                            //  gl_TargetPos = (long)(Low_limit_initial -(100002/3));
								//	 gul_Targettimeout = (u32)(fuyang_TargetPos*5/2/0.00635);    //这个值是电机要走的步数,当
								
								Init_first_flag = 1;
								fuyang_Targetjiaodu = 0;
								last_gl_currentPos = 0;
								gul_Targettimeout_temp = fuyang_TargetPos*FUYANG_STEP_COUNT_PER_MM/ONE_MS_TIME_COUNT_VALUE; //PWM输出持续ms数
						
								for (count = 0; count < 12; count++)
								{
									AT24c256_storage[count] = RS232_lingwei_shezhi_huifu[count];
								}
								gul_Targettimeout = gul_Targettimeout_temp;
								fuyang_shezhi_guocheng_flag = 1;
								TIM_Cmd(TIM4, ENABLE);
								
						}
				}
				
		}//end if (guc_10mscnt >= 2)复位运动定时判断

		
	}
	
}





///////////////////处理上位机角度设置指令////////////////////////////////////////
int Yuntai_jiaodushezhi()
{
		if (Yuntai_jiaodushezhi_flag == 1)
		{
			if (Yuntai_yingdafangshi_flag == 0)
			{
				Yuntai_jiaodushezhi_flag = 0; //上位机设置运动角度后，会进这里只进一次
				
				if ((Yuntai_fangwei_shezhi_chenggong_flag == 1) && (Yuntai_fuyang_shezhi_chenggong_flag == 1))
				{
					RS485_Send_Data(RS232_yuntaijiaodushezhi_huifu, 8);
				}
				else if ((Yuntai_fangwei_shezhi_chenggong_flag == 0) || (Yuntai_fuyang_shezhi_chenggong_flag == 0))
				{
					RS485_Send_Data(RS232_yuntaikongzhi_shibai_huifu, 8);
				}
				
				/////////////////设置俯仰命令///////////////////////
				if (Yuntai_fuyang_shezhi_chenggong_flag == 1)
				{
						Yuntai_fuyang_shezhi_chenggong_flag=0;
					
						if (fuyang_yundong_daowei_flag == 1)
						{
							guc_timejishi = 0;//俯仰电机运动1ms中断判断计数
							last_gl_currentPos = gl_currentPos;
						}
						if(fabs(fuyang_TargetPos - gl_currentPos) >= 0.002)
							fuyang_xiangdui_TargetPos = fuyang_TargetPos - gl_currentPos;
						else
							fuyang_xiangdui_TargetPos = 0;

						gul_Targettimeout_temp = fuyang_xiangdui_TargetPos*FUYANG_STEP_COUNT_PER_MM/ONE_MS_TIME_COUNT_VALUE; //PWM输出持续ms数
						
						if (gul_Targettimeout_temp == 0)
						{
							TIM_Cmd(TIM4, DISABLE);
							TIM_Cmd(TIM3, 0);  //关闭定时器3
							gb_SHUN_NI = 2;//俯仰电机状态停止
							gul_Targettimeout = 0;
							fuyang_yundong_daowei_flag = 1;//运动完毕标志
							guc_timejishi = 0;
							gl_currentPos = fuyang_TargetPos;
							fuyang_shezhi_guocheng_flag = 0;

						}						
						else  if (gul_Targettimeout_temp > 0)
						{
								TIM_Cmd(TIM4, 0);  //关闭定时器4
								TIM_Cmd(TIM3, 0);  //关闭定时器3
								
								gul_Targettimeout = gul_Targettimeout_temp;														
								AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
								AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //电机未启动
								SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);
								AMIS30543_DIR1 = 1;
								gb_SHUN_NI = 1;//俯仰电机状态正转
								TIM4_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//直接以设定速度启动
								fuyang_yundong_daowei_flag = 0;//清0运动完成标志，开始运动
								TIM_Cmd(TIM4, ENABLE);
								TIM_Cmd(TIM1, ENABLE);
								
								guc_timejishi = 0;
								fuyang_shezhi_guocheng_flag = 1;
															
						}
						else  if (gul_Targettimeout_temp < 0)
						{
							TIM_Cmd(TIM4, 0);  //关闭定时器4
							TIM_Cmd(TIM3, 0);  //关闭定时器3
							gul_Targettimeout = abs(gul_Targettimeout_temp);
							AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
							AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //电机未启动
							SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);
							TIM4_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//直接以设定速度启动					
							AMIS30543_DIR1 = 0;
							gb_SHUN_NI = 0;//俯仰电机状态反转
							fuyang_yundong_daowei_flag = 0;
							
							guc_timejishi = 0;
							fuyang_shezhi_guocheng_flag = 1;
							TIM_Cmd(TIM4, ENABLE);
							TIM_Cmd(TIM1, ENABLE);
						}			
				}
				/////////////////设置方位命令///////////////////////
				if (Yuntai_fangwei_shezhi_chenggong_flag == 1)
				{
						Yuntai_fangwei_shezhi_chenggong_flag=0;	
						TIM_Cmd(TIM2, 0);  //关闭定时器2
						if (fangwei_yundong_daowei_flag == 1)
						{
							guc_timejishi1 = 0;
							last_gl_currentPos1 = gl_currentPos1;
						}
						if(fabs(fangwei_TargetPos - gl_currentPos1) >= 0.002)
							fangwei_xiangdui_TargetPos = fangwei_TargetPos - gl_currentPos1;
						else
							fangwei_xiangdui_TargetPos = 0;
						//gul_Targettimeout1_temp = (int)((fangwei_xiangdui_TargetPos * 5 / 2 / 0.00635)); //38厘米的杆，一个整步0.00635
						//计算运动指定长度，需要1ms中断几次。PWM是51.2次脉冲/ms ，脉冲当量20157.48/mm,需要393.6ms，10mm运动就需要3936ms。中断就是10次
						gul_Targettimeout1_temp = fangwei_xiangdui_TargetPos*STEP_COUNT_PER_MM/ONE_MS_TIME_COUNT_VALUE;
						if (gul_Targettimeout1_temp == 0)
						{
							gb_SHUN_NI1 = 2;//方位电机状态停止
							TIM_Cmd(TIM5, DISABLE); //方位停止
							TIM_Cmd(TIM2, DISABLE);
							TIM_DeInit(TIM2);
							gul_Targettimeout1 = 0; //方位目标时间清零
							guc_timejishi1 = 0;
							fangwei_yundong_daowei_flag = 1;
							gl_currentPos1 = fangwei_TargetPos;
							fangwei_shezhi_guocheng_flag = 0;
		
						}						
						else if (gul_Targettimeout1_temp > 0)
						{
							TIM_Cmd(TIM5, DISABLE); //方位停止
							TIM_Cmd(TIM2, DISABLE);
							TIM_DeInit(TIM2);
							gul_Targettimeout1 = gul_Targettimeout1_temp;
							AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
							AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //电机未启动
							SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);

							AMIS30543_DIR2 = 1;
							gb_SHUN_NI1 = 1;//方位电机状态正转
							TIM5_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//直接以设定速度启动	
							fangwei_yundong_daowei_flag = 0;
							guc_timejishi1 = 0; //方位计时清零
							fangwei_shezhi_guocheng_flag = 1;
														
							TIM_Cmd(TIM5, ENABLE);
							TIM_Cmd(TIM1, ENABLE);
							
						}						
						else if (gul_Targettimeout1_temp < 0)
						{
							TIM_Cmd(TIM5, DISABLE); //方位停止
							TIM_Cmd(TIM2, DISABLE);
							TIM_DeInit(TIM2);
							gul_Targettimeout1 = abs(gul_Targettimeout1_temp);
							AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
							AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //电机未启动
							SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);
							
							AMIS30543_DIR2 = 0;

							TIM5_PWM_Init( MOTOR_PWM_ARR-1 , 0 , (MOTOR_PWM_ARR-1)/2);//直接以设定速度启动		

							gb_SHUN_NI1 = 0;//方位电机状态反转
							
							fangwei_yundong_daowei_flag = 0;
							guc_timejishi1 = 0; //方位计时清零
							fangwei_shezhi_guocheng_flag = 1;							
				
							TIM_Cmd(TIM5, ENABLE);
							TIM_Cmd(TIM1, ENABLE);	
							
						}			
					
				}
				
				
			}
		}//end 上位机角度设置指令
		
		return 0;
}

/////////////////////处理上位机云台控制//////////////////////////////////////
int  Yuntai_kongzhi()
{
		if (Yuntai_kongzhi_flag == 1) //接收到云台控制指令
		{
			Yuntai_kongzhi_flag = 0;	//控制指令标志清零
			if (Yuntai_fuwei_flag == 1) //复位过程中找到零位前没有位置
			{
				RS485_Send_Data(RS232_yuntaikongzhi_huifu, 8); //立即应答没有问题
				AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
				AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //电机未启动
				SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);					//使能俯仰电机
				AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
				AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Enable; //电机未启动
				SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);					//使能方位电机
				Yuntai_fuwei_flag = 0;													//复位指令只进一次没有问题
				AMIS30543_DIR2 = 0;														//和俯仰的方向相同，方位电机的DIR为0时电机向零位靠近
				gb_SHUN_NI1 = 0;														//标志电机正转
				AMIS30543_DIR1 = 0;														//和俯仰的方向相同，俯仰电机的DIR为0时电机向零位靠近
				gb_SHUN_NI = 0;															//标志俯仰电机下转
				In_place_flag = 0;														//标志俯仰电机运动进入复位状态
				In_place_flag1 = 0;														//标志方位电机的运动进入了复位状态
				if (Fuyang_Lmt1 == 1)													//表示俯仰电机没有在限位
				{
					TIM_Cmd(TIM4, ENABLE); //俯仰电机向零位方向运动        //表示电机在运动的标志
				
					fuyang_yundong_daowei_flag = 0;
				}
				else
				{
					TIM_Cmd(TIM4, DISABLE); //否则停止
				}
				
				if (Fangwei_Lmt1 == 1)
				{
					TIM_Cmd(TIM5, ENABLE); //设置方位电机速度
					fangwei_yundong_daowei_flag = 0;

				}
				else
				{
					TIM_Cmd(TIM5, DISABLE);
				}
				
			}
			
			
			if (Yuntai_tingzhi_flag == 1)
			{
				Yuntai_tingzhi_flag = 0;
				RS485_Send_Data(RS232_yuntaikongzhi_huifu, 8);
				if ((In_place_flag1 == 1) && (In_place_flag == 1))//判断当前不是复位运动中,否则不允许手动停止
				{
					if (gul_Targettimeout1_temp > 0) //停止时算的方位角度
					{
						TIM_Cmd(TIM5, DISABLE);
						fangwei_yundong_daowei_flag =1;
						//gl_currentPos1 = last_gl_currentPos1 + (guc_timejishi1 * 2 * 0.00635 / 5);
						cos_gul_jiaodu1 = (double)(((57.2713 * 57.2713) + (120.6785 * 120.6785) - (87.625 + gl_currentPos1) * (87.625 + gl_currentPos1)) / (2 * 57.2713 * 120.6785)); //计算出具体的物理角度实际角度没有乘以100
						gul_jiaodu1 = 1500 - (acos(cos_gul_jiaodu1) * 18000 / 3.14159265 - 4389.97573);
						fangwei_Targetjiaodu = (short)(gul_jiaodu1); //当前方位目标
						gul_fangwei_jiaodu = fangwei_Targetjiaodu;	 //当前目标角度
						fangwei_TargetPos = gl_currentPos1;			 //方位目标位置
						gul_Targettimeout1 = guc_timejishi1;		 //方位停止时计时时间记录下来
						fangwei_yundong_daowei_flag = 1;			 //方位运动到位标志
						gb_SHUN_NI1 = 2;							 //表示停止
					}
					else if (gul_Targettimeout1_temp < 0)
					{
						TIM_Cmd(TIM5, DISABLE);
						
						fangwei_yundong_daowei_flag =1;
						//gl_currentPos1 = last_gl_currentPos1 - (guc_timejishi1 * 2 * 0.00635 / 5);																					  //方位负方向
						cos_gul_jiaodu1 = (double)(((57.2713 * 57.2713) + (120.6785 * 120.6785) - (87.625 + gl_currentPos1) * (87.625 + gl_currentPos1)) / (2 * 57.2713 * 120.6785)); //计算出具体的物理角度
						gul_jiaodu1 = 1500 - (acos(cos_gul_jiaodu1) * 18000 / 3.14159265 - 4389.97573);																				  //方位角度
						fangwei_Targetjiaodu = (short)(gul_jiaodu1);																												  //乘以100
						gul_fangwei_jiaodu = fangwei_Targetjiaodu;
						fangwei_TargetPos = gl_currentPos1;
						gul_Targettimeout1 = guc_timejishi1; //方位计时时间
						fangwei_yundong_daowei_flag = 1;
						gb_SHUN_NI1 = 2;
					}
					if (gul_Targettimeout_temp > 0)
					{
						TIM_Cmd(TIM4, DISABLE);
						fuyang_yundong_daowei_flag =1;
						//gl_currentPos = last_gl_currentPos + (guc_timejishi * 0.00635 * 2 / 5);
						cos_gul_jiaodu = (double)(((553.5480 * 553.5480) + (409.1304 * 409.1304) - (229.625 + gl_currentPos) * (229.625 + gl_currentPos)) / (2 * 553.5480 * 409.1304)); //计算出具体的物理角度
									
						gul_jiaodu = acos(cos_gul_jiaodu) * 180 / 3.14159265 - 21.6219;
						fuyang_Targetjiaodu = (short)(gul_jiaodu * 100); //停止的时候算俯仰目标角度
						gul_fuyang_jiaodu = fuyang_Targetjiaodu;		 //停止的时候算俯仰目标角度
						fuyang_TargetPos = gl_currentPos;
						gul_Targettimeout = guc_timejishi;
						fuyang_yundong_daowei_flag = 1;
						gb_SHUN_NI = 2;
					}
					else if (gul_Targettimeout_temp < 0)
					{
						TIM_Cmd(TIM4, DISABLE);
						fuyang_yundong_daowei_flag =1;
						//gl_currentPos = last_gl_currentPos - (guc_timejishi * 0.00635 * 2 / 5);
						cos_gul_jiaodu = (double)(((553.5480 * 553.5480) + (409.1304 * 409.1304) - (229.625 + gl_currentPos) * (229.625 + gl_currentPos)) / (2 * 553.5480 * 409.1304)); //计算出具体的物理角度
						gul_jiaodu = acos(cos_gul_jiaodu) * 180 / 3.14159265 - 21.6219;
						fuyang_Targetjiaodu = (short)(gul_jiaodu * 100); //停止的时候算俯仰目标角度
						gul_fuyang_jiaodu = fuyang_Targetjiaodu;		 //停止的时候算俯仰目标角度
						fuyang_TargetPos = gl_currentPos;
						gul_Targettimeout = guc_timejishi;
						fuyang_yundong_daowei_flag = 1;
						gb_SHUN_NI = 2;
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
		}//end 上位机云台控制指令
		return  0;
}
////////////////////////////////////////////////////////////////////
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
		fangwei_Targetjiaodu = (short)((AT240c_str[4] << 8) | AT240c_str[5]);
		fangwei_Targethudu = (double)((1500 - fangwei_Targetjiaodu + 4389.97574) * 3.14159265 / 18000);
		fangwei_TargetPos_temp = cos(fangwei_Targethudu);
		fangwei_TargetPos_temp = (double)(sqrt(57.2713 * 57.2713 + 120.6785 * 120.6785 - fangwei_TargetPos_temp * 2 * 57.2713 * 120.6785) - 87.625);
		if ((fangwei_TargetPos_temp >= 0) && (fangwei_TargetPos_temp <= 38))
		{
			fangwei_TargetPos = fangwei_TargetPos_temp;
			gl_currentPos1 = fangwei_TargetPos;
			last_gl_currentPos1 = fangwei_TargetPos;
			fangwei_yundong_daowei_flag = 1; //到位完成了
		}
		fuyang_Targetjiaodu = (short)((AT240c_str[6] << 8) | AT240c_str[7]);
		fuyang_Targethudu = (double)((fuyang_Targetjiaodu-500.0 + 2162.1881) * 3.14159265 / 18000);
		fuyang_TargettPos_temp = cos(fuyang_Targethudu);
		fuyang_TargettPos_temp = (double)(sqrt(553.5480 * 553.5480 + 409.1304 * 409.1304 - fangwei_TargetPos_temp * 2 * 553.5480 * 409.1304) - 229.625);
				
		if ((fuyang_TargettPos_temp >= 0) && (fuyang_TargettPos_temp <= 38))
		{
			fuyang_TargetPos = fuyang_TargettPos_temp;
			gl_currentPos = fuyang_TargetPos;
			last_gl_currentPos = fuyang_TargetPos;
			fuyang_yundong_daowei_flag = 1; //到位完成了
		}
		Init_first_flag = 1;
		Init1_first_flag = 1;
	}
}
//////////////////回复上位机角度查询指令////////////////////////////
void jiaodu_send()
{
	u8 i;
	u8 t;
	i = 0;
	t = 0;
	RS232_send_str[0] = 0xc0;												 //帧头
	RS232_send_str[1] = 0x82;												 //命令字，固定字节
	RS232_send_str[2] = 0x00;												 //数据长度 ，固定字节
	if ((fangwei_yundong_daowei_flag == 0) && (gul_Targettimeout1_temp > 0)) //方位电机向螺杆伸出的方向运动
	{
		//gl_currentPos1 = last_gl_currentPos1 + (guc_timejishi1 * 2 * 0.00635 / 5);
		cos_gul_jiaodu1 = (double)(((57.2713 * 57.2713) + (120.6785 * 120.6785) - (87.625 + gl_currentPos1) * (87.625 + gl_currentPos1)) / (2 * 57.2713 * 120.6785)); //计算出具体的物理角度
		gul_fangwei_jiaodu = (short)(1500 - (acos(cos_gul_jiaodu1) * 18000 / 3.14159265 - 4389.97574));
	}
	else if ((fangwei_yundong_daowei_flag == 0) && (gul_Targettimeout1_temp < 0))
	{
		//gl_currentPos1 = last_gl_currentPos1 - (guc_timejishi1 * 2 * 0.00635 / 5);																					  //负方向
		cos_gul_jiaodu1 = (double)(((57.2713 * 57.2713) + (120.6785 * 120.6785) - (87.625 + gl_currentPos1) * (87.625 + gl_currentPos1)) / (2 * 57.2713 * 120.6785)); //计算出具体的物理角度
		gul_fangwei_jiaodu = (short)(1500 - (acos(cos_gul_jiaodu1) * 18000 / 3.14159265 - 4389.97574));																  //方位角度   //乘以100
	}
	else if (fangwei_yundong_daowei_flag == 1)
	{
		gul_fangwei_jiaodu = fangwei_Targetjiaodu; //乘以100
	}
	RS232_send_str[4] = (u8)((gul_fangwei_jiaodu & 0xFF00) >> 8);
	fangwei_jiaodu_diwei = (u8)(gul_fangwei_jiaodu & 0xFF);
	RS232_send_str[5] = fangwei_jiaodu_diwei;
	RS232_send_str[6] = 0;
	RS232_send_str[7] = 0;
	if ((fuyang_yundong_daowei_flag == 0) && (gul_Targettimeout_temp > 0))
	{
		//gl_currentPos = last_gl_currentPos + (guc_timejishi * 0.00635 * 2 / 5);
		cos_gul_jiaodu = (double)(((553.5480 * 553.5480) + (409.1304 * 409.1304) - (229.625 + gl_currentPos) * (229.625 + gl_currentPos)) / (2 * 553.5480 * 409.1304)); //计算出具体的物理角度						
		gul_fuyang_jiaodu = (short)(acos(cos_gul_jiaodu) * 18000 / 3.14159265 - 2162.188);
										
	}
	else if ((fuyang_yundong_daowei_flag == 0) && (gul_Targettimeout_temp < 0))
	{
		cos_gul_jiaodu = (double)(((553.5480 * 553.5480) + (409.1304 * 409.1304) - (229.625 + gl_currentPos) * (229.625 + gl_currentPos)) / (2 * 553.5480 * 409.1304)); //计算出具体的物理角度						
		gul_fuyang_jiaodu = (short)(acos(cos_gul_jiaodu) * 18000 / 3.14159265 - 2162.188);
		
	}
	else if (fuyang_yundong_daowei_flag == 1)
	{
		gul_fuyang_jiaodu = fuyang_Targetjiaodu;
		if ((fuyang_yundong_daowei_flag == 1) && (fangwei_yundong_daowei_flag == 1))//判断两个电机都运动完毕，关闭使能省电
		{
			AT24CXX_Write(0, (u8 *)AT24c256_storage, 11);//保存位置数据
			fuyang_yundong_daowei_flag = 2;//设置俯仰运动标志为2.运动完成，实时位置已经写入EEPROM
			fangwei_yundong_daowei_flag = 2;
			AMIS30543_CR_SPI1.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
			AMIS30543_CR_SPI1.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Disable; //电机未启动
			SPI3_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI1);
			AMIS30543_CR_SPI2.AMIS30543_CR2.bitsSLAG = AMIS30543_SPI_SLAG_gain1;
			AMIS30543_CR_SPI2.AMIS30543_CR2.bitsMOTEN = AMIS30543_SPI_MOTEN_Disable; //电机未启动
			SPI2_Byte_Write(AMIS30543_SPI_CR2, AMIS30543_CR_SPI2);
		}
	}
	RS232_send_str[8] = (u8)((gul_fuyang_jiaodu & 0xFF00) >> 8); //预留
	fuyang_jiaodu_diwei = (u8)(gul_fuyang_jiaodu & 0x00FF);
	RS232_send_str[9] = fuyang_jiaodu_diwei;
	RS232_send_str[10] = (u8)((shuipingfuyangjiao & 0xff00) >> 8);
	shuipingfuyangjiao_diwei = (u8)(shuipingfuyangjiao & 0xff);
	RS232_send_str[11] = shuipingfuyangjiao_diwei;
	if ((0 == Fuyang_Lmt1) && (0 == Fangwei_Lmt1))
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
	if ((AMIS30543_ERR1 == 0) && (AMIS30543_ERR2 == 0))
	{
		RS232_send_str[13] = 0x03;
	}
	else if ((AMIS30543_ERR1 == 1) && (AMIS30543_ERR2 == 1))
	{
		RS232_send_str[13] = 0x00;
	}
	else if ((AMIS30543_ERR1 == 1) && (AMIS30543_ERR2 == 0))
	{
		RS232_send_str[13] = 0x02;
	}
	else if ((AMIS30543_ERR1 == 0) && (AMIS30543_ERR2 == 1))
	{
		RS232_send_str[13] = 0x01;
	}
	if (gb_SHUN_NI == 0) //俯仰反转1000
	{
		if (gb_SHUN_NI1 == 0) //方位顺转0001
		{
			RS232_send_str[14] = 0x09;
		}
		if (gb_SHUN_NI1 == 1) //方位逆转0010
		{
			RS232_send_str[14] = 0x0A;
		}
		if (gb_SHUN_NI1 == 2) //方位停止0000
		{
			RS232_send_str[14] = 0x08;
		}
	}
	else if (gb_SHUN_NI == 1) //俯仰正转0100
	{
		if (gb_SHUN_NI1 == 0) //方位顺转0001
		{
			RS232_send_str[14] = 0x05;
		}
		if (gb_SHUN_NI1 == 1) //方位逆转0010
		{
			RS232_send_str[14] = 0x06;
		}
		if (gb_SHUN_NI1 == 2) //方位停止
		{
			RS232_send_str[14] = 0x04;
		}
	}
	else if (gb_SHUN_NI == 2) //俯仰停止0000
	{
		if (gb_SHUN_NI1 == 0)
		{
			RS232_send_str[14] = 0x01;
		}
		if (gb_SHUN_NI1 == 1)
		{
			RS232_send_str[14] = 0x02;
		}
		if (gb_SHUN_NI1 == 2)
		{
			RS232_send_str[14] = 0x00;
		}
	}
	if (((In_place_flag == 0) || (In_place_flag1 == 0)) && (AMIS30543_ERR1 == 1) && (AMIS30543_ERR2 == 1))
	{
		RS232_send_str[15] = 0x02;//复位
	}
	else if (((gb_SHUN_NI != 2) || (gb_SHUN_NI1 != 2)) && (AMIS30543_ERR1 == 1) && (AMIS30543_ERR2 == 1))
	//else if (((gb_SHUN_NI1 != 2) || (gb_SHUN_NI1 != 2)))	
	{
		RS232_send_str[15] = 0x03;//运转
	}
	else if ((gb_SHUN_NI == 2) && (gb_SHUN_NI1 == 2) && (AMIS30543_ERR1 == 1) && (AMIS30543_ERR2 == 1))
	{
		RS232_send_str[15] = 0x04;//天线工作状态-停止
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
	for (i = 1; i < 18; i++) //判断发送的字节中是否有0xc0;
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
