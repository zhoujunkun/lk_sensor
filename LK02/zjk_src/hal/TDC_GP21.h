#ifndef _TDC_GP21_H
#define _TDC_GP21_H
#include "stm32f1xx_hal.h"
#include "main.h"
#include "spi.h"

//REGISTER 0
#define RER0_ANZ_FIRE(x)      ((uint32_t)x)<<28
#define RER0_DIV_FIRE(x)      ((uint32_t)x)<<24
#define RER0_ANZ_PER_CALRES(x)((uint32_t)x)<<22
#define RER0_DIV_CLKHS(x)     ((uint32_t)x)<<20
#define RER0_START_CLKHS(x)   ((uint32_t)x)<<18
#define RER0_ANZ_PORT(x)      ((uint32_t)x)<<17
#define RER0_TCYCLE(x)        ((uint32_t)x)<<16
#define RER0_ANZ_FAKE(x)      ((uint32_t)x)<<15
#define RER0_SEL_ECLK_TEMP(x) ((uint32_t)x)<<14
#define RER0_CALIBRATE(x)     ((uint32_t)x)<<13
#define RER0_NO_CAL_AUTO(x)   ((uint32_t)x)<<12
#define RER0_MESSB2(x)        ((uint32_t)x)<<11
#define RER0_NEG_STOP2(x)     ((uint32_t)x)<<10
#define RER0_NEG_STOP1(x)     ((uint32_t)x)<<9
#define RER0_NEG_START(x)     ((uint32_t)x)<<8



//REGISTER 1
#define REG1_HIT2(x)          ((uint32_t)x)<<28
#define REG1_HIT1(x)          ((uint32_t)x)<<24
#define REG1_EN_FAST_INIT(x)  ((uint32_t)x)<<23
#define REG1_HITIN2(x)        ((uint32_t)x)<<19
#define REG1_HITIN1(x)        ((uint32_t)x)<<16
#define REG1_CURR32K(x)       ((uint32_t)x)<<15
#define REG1_SEL_START_FIRE(x)  ((uint32_t)x)<<14
#define REG1_SEL_TSTO2(x)     ((uint32_t)x)<<11
#define REG1_SEL_TSTO1(x)     ((uint32_t)x)<<8

//REGISTER 2
#define REG2_EN_INT(x)        ((uint32_t)x)<<29
#define REG2_RFEDGE2(x)       ((uint32_t)x)<<28
#define REG2_RFEDGE1(x)       ((uint32_t)x)<<27
#define REG2_DELVAL1(x)       ((uint32_t)x)<<8

//REGISTER 3 - EN_FIRST_WAVE = 0
#define REG3_EN_AUTOCALC_MB2(x)    ((uint32_t)x)<<31
#define REG3_EN_EN_FIRST_WAVE(x)   ((uint32_t)x)<<30
#define REG3_EN_EN_ERR_VAL(x)      ((uint32_t)x)<<29
#define REG3_EN_SEL_TIMO_MB2(x)    ((uint32_t)x)<<27
#define REG3_EN_DELVAL2(x)         ((uint32_t)x)<<8

//REGISTER 3 - EN_FIRST_WAVE = 1
//#define REGISTER_3_EN_AUTOCALC_MB2(x)  ((uint32_t)x)<<31
//#define REGISTER_3_EN_FIRST_WAVE(x)   ((uint32_t)x)<<30
//#define REGISTER_3_EN_ERR_VAL(x)  ((uint32_t)x)<<29
//#define REGISTER_3_SEL_TIMO_MB2(x)  ((uint32_t)x)<<27
#define REG3_EN_DELREL3(x)       ((uint32_t)x)<<20
#define REG3_EN_DELREL2(x)       ((uint32_t)x)<<14
#define REG3_EN_DELREL1(x)       ((uint32_t)x)<<8

//REGISTER 4 - EN_FIRST_WAVE = 0
#define REG4_DELVAL3(x)       ((uint32_t)x)<<8
//REGISTER 4 - EN_FIRST_WAVE = 1
#define REG4_DIS_PW(x)        ((uint32_t)x)<<16
#define REG4_EDGE_PW(x)       ((uint32_t)x)<<15
#define REG4_OFFSRNG2(x)      ((uint32_t)x)<<14
#define REG4_OFFSRNG1(x)      ((uint32_t)x)<<13
#define REG4_OFFS(x)          ((uint32_t)x)<<8

//REGISTER 5
#define REG5_CONF_FIRE(x)       ((uint32_t)x)<<29
#define REG5_EN_STARTNOISE(x)   ((uint32_t)x)<<28
#define REG5_DIS_PHASESHIFT(x)  ((uint32_t)x)<<27
#define REG5_REPEAT_FIRE(x)     ((uint32_t)x)<<24
#define REG5_PHFIRE(x)          (x & 0xEFFF)<<8

//REGISTER 6
#define REG6_EN_ANALOG(x)     ((uint32_t)x)<<31
#define REG6_NEG_STOP_TEMP(x) ((uint32_t)x)<<30
#define REG6_DA_KORR(x)       ((uint32_t)x)<<25
#define REG6_TW2(x)           ((uint32_t)x)<<22
#define REG6_EN_INT(x)        ((uint32_t)x)<<21
#define REG6_START_CLKHS(x)   ((uint32_t)x)<<20
#define REG6_CYCLE_TEMP(x)    ((uint32_t)x)<<18
#define REG6_CYCLE_TOF(x)     ((uint32_t)x)<<16
#define REG6_HZ60(x)          ((uint32_t)x)<<15
#define REG6_FIREO_DEF(x)     ((uint32_t)x)<<14 //Set to 1 if internal analog section is used
#define REG6_QUAD_RES(x)      ((uint32_t)x)<<13
#define REG6_DOUBLE_RES(x)    ((uint32_t)x)<<12
#define REG6_TEMP_PORTDIR(x)  ((uint32_t)x)<<11
#define REG6_ANZ_FIRE(x)      ((uint32_t)x)<<8


///
#define OPC_START_TOF            0x01
#define OPC_INIT                 0x70
#define OPC_RESET                0x50
#define OPC_ID                   0xB7
#define OPC_Reg1highByte          0xB5
#define OP_CODE_WR(addr)         (0x80 | (addr))
#define OP_CODE_RD(addr)         (0xB0 | (addr))
////
//--------------------------------------------------------------------------
// 1m(��) = 10dm(����) = 100cm(����) = 1000mm(����)
//#define   C_VELOCITY           299792458 // velocity of light (m/s)
//#define   C_VELOCITY          299792.458 // velocity of light (m/ms)
//#define   C_VELOCITY          299.792458 // velocity of light (m/us)
//#define   C_VELOCITY         0.299792458 // velocity of light (m/ns)
#define C_VELOCITY            29.9792458 // velocity of light (cm/ns)


#define gp21_select()    HAL_GPIO_WritePin(GP21_NSS_GPIO_Port,GP21_NSS_Pin,GPIO_PIN_RESET)
#define gp21_release()   HAL_GPIO_WritePin(GP21_NSS_GPIO_Port,GP21_NSS_Pin,GPIO_PIN_SET)

#define gp21_rstn_idle()  HAL_GPIO_WritePin(GP21_RSTN_GPIO_Port,GP21_RSTN_Pin,GPIO_PIN_SET)
#define gp21_rstn_rsow()  HAL_GPIO_WritePin(GP21_RSTN_GPIO_Port,GP21_RSTN_Pin,GPIO_PIN_RESET)

#define gp21_en_startSignal()   HAL_GPIO_WritePin(GP21_EN_Start_GPIO_Port,GP21_EN_Start_Pin,GPIO_PIN_SET)
#define gp21_close_startSignal()   HAL_GPIO_WritePin(GP21_EN_Start_GPIO_Port,GP21_EN_Start_Pin,GPIO_PIN_RESET)


#define gp21_en_stop1Signal()   HAL_GPIO_WritePin(GP21_EN_Stop1_GPIO_Port,GP21_EN_Stop1_Pin,GPIO_PIN_SET)
#define gp21_close_stop1Signal()   HAL_GPIO_WritePin(GP21_EN_Stop1_GPIO_Port,GP21_EN_Stop1_Pin,GPIO_PIN_RESET)


#define gp21_read_intn()     HAL_GPIO_ReadPin(GP21_INTN_GPIO_Port,GP21_INTN_Pin)

#define TDC_Signal_high()    tx_signalGpio->BSRR = TDC_Signal_Pin;
#define TDC_Signal_low()     tx_signalGpio->BSRR = (uint32_t)TDC_Signal_Pin << 16U;



void GP21_Init(void);
void gp21_write(uint8_t reg);
void gp21_write_cfg(uint8_t op_code, uint32_t cfg32);
void gp21_get_id(uint8_t *id);
uint16_t  get_gp21_statu(void);
void gp21_hard_rst(void);
void gp21_defaultcofg(void);
void gp21_startOneSignal(void);
uint32_t   gp21_read_diatance(void);
 void tdc_delay(uint32_t cval);
 

typedef struct 
{
   void (*init)(void);    /*init tdc gp21*/
   void (*write)(uint8_t reg);
}_tdc_gp21;

#define GP21_STATU_CH1      0x0038U
#define GP21_STATU_TIMEOUT  0x0100U
#endif
