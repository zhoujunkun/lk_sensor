#ifndef _Z_LK03_H
#define _Z_LK03_H
#include "spi.h"
#include "tim.h"
#include "tdc_gp21.h"
#include "tlc5618.h"
#include "z_analog.h"
#include "z_serial.h"
#include "main.h"
#include "tim.h"
#define DISTANCE_RCV_SIZE   5


typedef struct {

	uint16_t vol;
	uint8_t vol_statu;

}_tdc_voltage ;


typedef struct {

	float Kp;
	float Ki;
	float ki_sum;
	int16_t setpoint;

}_tdc_pid ;

typedef struct{

 uint8_t time_out;
 uint16_t txSignalCnt;

}_tdc_statu;
typedef struct   
{
	uint32_t gp21_statu_INT;
	float gp21_dist_reult;
	uint32_t gp21_distance[DISTANCE_RCV_SIZE];
	uint16_t z_analog_erro_cnt;
	uint8_t isGp21Complete;
	uint16_t tdc_distance;
	uint16_t tlc_resualt;
 _tdc_voltage rev_siganl;
	_tdc_statu statu;   
	_tdc_pid pid;
	
}_TDC_typ;

#define tdc_laser_light_on()      HAL_GPIO_WritePin(Laser_Light_GPIO_Port,Laser_Light_Pin,GPIO_PIN_SET)
#define tdc_laser_light_off()     HAL_GPIO_WritePin(Laser_Light_GPIO_Port,Laser_Light_Pin,GPIO_PIN_SET)
#define tdc_vol_ctl1_Set()        HAL_GPIO_WritePin(TX_Vol_Ctrl1_GPIO_Port,TX_Vol_Ctrl1_Pin,GPIO_PIN_SET)
#define tdc_vol_ctl1_ReSet()      HAL_GPIO_WritePin(TX_Vol_Ctrl1_GPIO_Port,TX_Vol_Ctrl1_Pin,GPIO_PIN_RESET)

#define tdc_vol_ctl2_Set()        HAL_GPIO_WritePin(TX_Vol_Ctrl2_GPIO_Port,TX_Vol_Ctrl2_Pin,GPIO_PIN_SET)
#define tdc_vol_ctl2_ReSet()      HAL_GPIO_WritePin(TX_Vol_Ctrl2_GPIO_Port,TX_Vol_Ctrl2_Pin,GPIO_PIN_RESET)


#define tdc_rx_voltge_high()      HAL_GPIO_WritePin(TDC_Sighal_AngleReles_GPIO_Port,TDC_Sighal_AngleReles_Pin,GPIO_PIN_SET)
#define tdc_rx_voltge_low()       HAL_GPIO_WritePin(TDC_Sighal_AngleReles_GPIO_Port,TDC_Sighal_AngleReles_Pin,GPIO_PIN_RESET)

 void tdc_rx_voltge_relese(void);
/*≤Œ ˝≈‰÷√*/
#define AD603_AGC_DEFAULT   300  
#define AD603_AGC_MIN     200 //0.16V -10DB
#define AD603_AGC_MAX     1200//0.720V 20DB

#define GP21_STATU_CH1      0x0038U
#define GP21_STATU_TIMEOUT  0x0100U

#define PID_KP      0.1
#define PID_KI      0.01
#define PID_SETPOINT 1200



extern _TDC_typ _TDC_GP21;
 void tdc_board_init(void);
void gp21_distance_cal(uint32_t *dit,uint8_t dislens);



#endif

