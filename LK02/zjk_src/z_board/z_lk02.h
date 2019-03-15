#ifndef _Z_LK03_H
#define _Z_LK03_H
#include "spi.h"
#include "tim.h"
#include "main.h"
#include "z_include.h"

#define tdc_rx_voltge_high()      HAL_GPIO_WritePin(TDC_Sighal_AngleReles_GPIO_Port,TDC_Sighal_AngleReles_Pin,GPIO_PIN_SET)
#define tdc_rx_voltge_low()       HAL_GPIO_WritePin(TDC_Sighal_AngleReles_GPIO_Port,TDC_Sighal_AngleReles_Pin,GPIO_PIN_RESET)


#define tdc_wire_toggle()         HAL_GPIO_TogglePin(wire_ctl_GPIO_Port,wire_ctl_Pin)
/*≤Œ ˝≈‰÷√*/
#define AD603_AGC_DEFAULT   300  
#define AD603_AGC_MIN     200 //0.16V -10DB
#define AD603_AGC_MAX     1200//0.720V 20DB
#define  TX_HIGH_VOL_TLC5618   1600  //

#define PID_KP      2.5
#define PID_KI      0.15
#define PID_SETPOINT 1100

 void tdc_board_init(void);
float gp21_distance_cal(uint32_t *dit,uint8_t dislens);

 void tdc_rx_voltge_relese(void);

#endif

