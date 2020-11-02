#ifndef _AMIS30543_H
#define _AMIS30543_H
#include "sys.h"

#define  AMIS30543_SPI_WR    0x00
#define  AMIS30543_SPI_CR0   0x01
#define  AMIS30543_SPI_CR1   0x02
#define  AMIS30543_SPI_CR2   0x03
#define  AMIS30543_SPI_CR3   0x09
#define  AMIS30543_SPI_SR0   0x04
#define  AMIS30543_SPI_SR1   0x05
#define  AMIS30543_SPI_SR2   0x06
#define  AMIS30543_SPI_SR3   0x07
#define  AMIS30543_SPI_SR4   0x0A
#define  AMIS30543_SPI2_NSS   PBout(12)
#define  AMIS30543_SPI3_NSS   PAout(15)
#define  FY_DIR    PCout(11)
#define  FW_DIR    PCout(9)
#define  AMIS30543_ERR1   PCin(12)
#define  AMIS30543_ERR2   PCin(8)
#define motrotype 2

typedef struct
{
	unsigned char AMIS30543_SPI_ADDR;
	unsigned char AMIS30543_SPI_WDEN;
	unsigned char AMIS30543_SPI_SM;
	unsigned char AMIS30543_SPI_CUR;
	unsigned char AMIS30543_SPI_DIRCTRL;
	unsigned char AMIS30543_SPI_NXTR;
	unsigned char AMIS30543_SPI_PWMF;
	unsigned char AMIS30543_SPI_PWMJ;
	unsigned char AMIS30543_SPI_EMC;
	unsigned char AMIS30543_SPI_MOTEN;
	unsigned char AMIS30543_SPI_SLP;
	unsigned char AMIS30543_SPI_SLAG;
	unsigned char AMIS30543_SPI_SLAT;
	unsigned char AMIS30543_SPI_ESM;
}AMIS30543_RegsCRStruct_TypeDef;

typedef struct
{
unsigned char AMIS30543_SPI_TW;
unsigned char AMIS30543_SPI_CPFAIL;
unsigned char AMIS30543_SPI_WD;
}AMIS30543_RegsSRStrust_TypeDef;

typedef struct 
{
   struct WR
   {
    unsigned char bitsDefault:3;
    unsigned char bitsWDT:4;
    unsigned char bitsWDEN:1;
    }AMIS30543_WR;
   struct  CR0
   {
    unsigned char bitsCUR:5;
    unsigned char bitsSM:3;
    }AMIS30543_CR0;
   struct  CR1
   {
    unsigned char bitsEMC:2;
    unsigned char bitsPWMJ:1;
    unsigned char bitsPWMF:1;
    unsigned char bitsDefaults:2;
    unsigned char bitsNXTP:1;
    unsigned char bitsDIRCTRL:1;
   }AMIS30543_CR1;
   struct  CR2
   {
    unsigned char bitsDefalt4:4;
    unsigned char bitsSLAT:1;
    unsigned char bitsSLAG:1;
    unsigned char bitsSLP:1;
    unsigned char bitsMOTEN:1;
    }AMIS30543_CR2;
   struct  CR3
   {
    unsigned char bitsESM:3;
    unsigned char bitsDefault5:5;
    }AMIS30543_CR3;
}AMIS30543_CR;
typedef enum {
    AMIS30543_SPI_CUR_132 = ((unsigned char)0x00),
    AMIS30543_SPI_CUR_245 = ((unsigned char)0x01),
    AMIS30543_SPI_CUR_355 = ((unsigned char)0x02),
	  AMIS30543_SPI_CUR_395 = ((unsigned char)0x03),
    AMIS30543_SPI_CUR_445 = ((unsigned char)0x04),
    AMIS30543_SPI_CUR_485 = ((unsigned char)0x05),
    AMIS30543_SPI_CUR_540 = ((unsigned char)0x06),
    AMIS30543_SPI_CUR_585 = ((unsigned char)0x07),
    AMIS30543_SPI_CUR_640 = ((unsigned char)0x08),
    AMIS30543_SPI_CUR_715 = ((unsigned char)0x09),
    AMIS30543_SPI_CUR_780 = ((unsigned char)0x0A),
    AMIS30543_SPI_CUR_870 = ((unsigned char)0x0B),
    AMIS30543_SPI_CUR_955 = ((unsigned char)0x0C),
    AMIS30543_SPI_CUR_1060 = ((unsigned char)0x0D),
    AMIS30543_SPI_CUR_1150 = ((unsigned char)0x0E),
    AMIS30543_SPI_CUR_1260 = ((unsigned char)0x0F),
    AMIS30543_SPI_CUR_1405 = ((unsigned char)0x10),
	  AMIS30543_SPI_CUR_1520 = ((unsigned char)0x11),
    AMIS30543_SPI_CUR_1695 = ((unsigned char)0x12),
    AMIS30543_SPI_CUR_1850 = ((unsigned char)0x13),
    AMIS30543_SPI_CUR_2070 = ((unsigned char)0x14),
    AMIS30543_SPI_CUR_2240 = ((unsigned char)0x15),
    AMIS30543_SPI_CUR_2440 = ((unsigned char)0x16),
    AMIS30543_SPI_CUR_2700 = ((unsigned char)0x17),
    AMIS30543_SPI_CUR_2845 = ((unsigned char)0x18),
    AMIS30543_SPI_CUR_3000 = ((unsigned char)0x19)
}AMIS30543_SPI_CUR;
typedef enum {
    AMIS30543_SPI_SM_32 = ((unsigned char)0x00),
	AMIS30543_SPI_SM_16 = ((unsigned char)0x01),
	AMIS30543_SPI_SM_8 = ((unsigned char)0x02),
	AMIS30543_SPI_SM_4 = ((unsigned char)0x03),
	AMIS30543_SPI_SM_Half = ((unsigned char)0x04),
	AMIS30543_SPI_SM_Unhaif = ((unsigned char)0x05),
	AMIS30543_SPI_SM_Unfull = ((unsigned char)0x06)    
}AMIS30543_SPI_SM;
typedef enum {
    AMIS30543_SPI_MOTEN_Disable = ((unsigned char)0x00),
    AMIS30543_SPI_MOTEN_Enable = ((unsigned char)0x01)
}AMIS30543_SPI_MOTEN;
typedef enum {
    AMIS30543_SPI_SLP_Active = ((unsigned char)0x00),
    AMIS30543_SPI_SLP_sleep = ((unsigned char)0x01)
}AMIS30543_SPI_SLP;
typedef enum {
    AMIS30543_SPI_SLAG_gain = ((unsigned char)0x00),
    AMIS30543_SPI_SLAG_gain1 = ((unsigned char)0x01)
}AMIS30543_SPI_SLAG;
typedef enum {
    AMIS30543_SPI_SLAT_ntransp = ((unsigned char)0x00),
    AMIS30543_SPI_SLAT_transp = ((unsigned char)0x01)
}AMIS30543_SPI_SLAT;
typedef enum{
    AMIS30543_SPI_WDT_32ms = ((unsigned char)0x00),
    AMIS30543_SPI_WDT_64ms = ((unsigned char)0x01),
    AMIS30543_SPI_WDT_96ms = ((unsigned char)0x02),
    AMIS30543_SPI_WDT_128ms = ((unsigned char)0x03),
    AMIS30543_SPI_WDT_160ms = ((unsigned char)0x04),
    AMIS30543_SPI_WDT_192ms = ((unsigned char)0x05),
    AMIS30543_SPI_WDT_224ms = ((unsigned char)0x06),
    AMIS30543_SPI_WDT_256ms = ((unsigned char)0x07),
    AMIS30543_SPI_WDT_288ms = ((unsigned char)0x08),
    AMIS30543_SPI_WDT_320ms = ((unsigned char)0x09),
    AMIS30543_SPI_WDT_352ms = ((unsigned char)0x0A),
    AMIS30543_SPI_WDT_384ms = ((unsigned char)0x0B),
    AMIS30543_SPI_WDT_416ms = ((unsigned char)0x0C),
    AMIS30543_SPI_WDT_448ms = ((unsigned char)0x0D),
    AMIS30543_SPI_WDT_480ms = ((unsigned char)0x0E),
    AMIS30543_SPI_WDT_512ms = ((unsigned char)0x0F)
}AMIS30543_SPI_WDT;
typedef enum{
    AMIS30543_SPI_WDEN_DISABLE = ((unsigned char)0x00),
    AMIS30543_SPI_WDEN_ENABLE = ((unsigned char)0x01)
}AMIS30543_SPI_WDEN;
typedef enum{
    AMIS30543_SPI_ESM_Other = ((unsigned char)0x00),
    AMIS30543_SPI_ESM_128 = ((unsigned char)0x01),
    AMIS30543_SPI_ESM_64 = ((unsigned char)0x02),
	AMIS30543_SPI_ESM_Compenstated2 = ((unsigned char)0x03),
    AMIS30543_SPI_ESM_Compenstated1 = ((unsigned char)0x04),
}AMIS30543_SPI_ESM;
typedef enum{
    AMIS30543_SPI_EMC_VeryFast = ((unsigned char)0x00),
	AMIS30543_SPI_EMC_Fast = ((unsigned char)0x01),
	AMIS30543_SPI_EMC_Slow = ((unsigned char)0x02),
    AMIS30543_SPI_EMC_VerySlow = ((unsigned char)0x03)
}AMIS30543_SPI_EMC;
typedef enum{
    AMIS30543_SPI_PWMJ_Disable = ((unsigned char)0x00),
	AMIS30543_SPI_PWMJ_Enable = ((unsigned char)0x01)
}AMIS30543_SPI_PWMJ;
typedef enum{
    AMIS30543_SPI_PWMF_Default = ((unsigned char)0x00),
    AMIS30543_SPI_PWMF_Double = ((unsigned char)0x01)
}AMIS30543_SPI_PWMF;
typedef enum{
    AMIS30543_SPI_NXTP_Rise = ((unsigned char)0x00),
    AMIS30543_SPI_NXTP_Fall = ((unsigned char)0x01)
}AMIS30543_SPI_NXTP;
typedef enum{
    AMIS30543_SPI_DIRCTRL_CW = ((unsigned char)0x00),
    AMIS30543_SPI_DIRCTRL_CCW = ((unsigned char)0x01)
}AMIS30543_SPI_DIRCTRL;




void AMIS30543_Init1(void);
void AMIS30543_Init2(void);
void AMIS30543_NXT1_init(u16 arr,u16 psc);
void AMIS30543_NXT2_init(u16 arr,u16 psc);
void SPI2_SetSpeed(u8 SpeedSet);
u8 SPI2_ReadWriteByte(u8 TxData);
u8 SPI3_ReadWriteByte(u8 TxData);
u8 SPI2_Byte_Write(unsigned char addr,AMIS30543_CR DATA);
u8 SPI3_Byte_Write(unsigned char addr,AMIS30543_CR DATA);
void AMIS30543_INIT1(void);
void AMIS30543_INIT2(void);


#endif

