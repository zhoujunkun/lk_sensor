#include "z_lk02.h"
#define tdc_power_on()      HAL_GPIO_WritePin(TDC_Power_Ctl_GPIO_Port,TDC_Power_Ctl_Pin,GPIO_PIN_RESET)
#define tdc_power_off()      HAL_GPIO_WritePin(TDC_Power_Ctl_GPIO_Port,TDC_Power_Ctl_Pin,GPIO_PIN_SET)

SemaphoreHandle_t  semaExitphore =NULL;  //创建信号量，用于外部中断触发

_TDC_typ _TDC_GP21;
z_tim_sturct  z_rx_pwm_hvCtl ={&htim2,TIM_CHANNEL_4};


 void tdc_rx_voltge_relese(void)
{

   tdc_rx_voltge_high();
	
 	tdc_delay(500);
	
   tdc_rx_voltge_low();

}


 /*rx pwm high power cotrol*/
 void rx_pwmHv(uint16_t vlue)
 {
 
     z_rx_pwm_hvCtl.z_tim->Instance->CCR4 = vlue;
 
 } 
 
 void tlc_rx_pwmHvStart(void)
 {
     HAL_TIM_PWM_Start(z_rx_pwm_hvCtl.z_tim, z_rx_pwm_hvCtl.tim_channel);	
 }


 void tdc_board_init(void)
 {
	  BaseType_t xResult;
	  tdc_power_on();   /*TDC Power ON*/
		tlc_rx_pwmHvStart();     /*LK RV High Voltage open*/
		rx_pwmHv (260);    /*rx high voltage control*/  
		GP21_Init(); 
		gp21_write(OPC_RESET);		 /*LK  gp21 Init*/	
		gp21_defaultcofg();		
    
    _TDC_GP21.pid.Kp = 0;
    _TDC_GP21.pid.Ki = PID_KI;	 
	  _TDC_GP21.pid.setpoint = PID_SETPOINT;
	 
	 	tlc5618_write(TX_HIGH_VOL_TLC5618,AD603_AGC_DEFAULT); /*LK  AGC DAC Voltage control*/  
	  tlc5618_writeAchannal(TX_HIGH_VOL_TLC5618);	 

    semaExitphore = xSemaphoreCreateBinary();	 
 }

#define GP22_TNS  800       //gp21 ???? 800ns 1.25MHZ
float test_dit=0,test_distf=0;
void gp21_distance_cal(uint32_t *dit,uint8_t dislens)
{
	volatile uint8_t minIndex=0;
  volatile	uint32_t tem=0;
	float dist_av=0,dist_f;
   for( int i=0;i<dislens-1;i++)    // selection sort 
	  {
		     minIndex = i;
			  for( int j=i+1;j<dislens;j++)
		   	{
					 if(dit[j]<dit[minIndex])     //?????С????
					 {
					    minIndex = j;
					 }
				}
				tem= dit[i];
				dit[i] = dit[minIndex];
				dit[minIndex] = tem;
		}
		
		for(int i=0;i<dislens;i++)
		{
		   
		    dist_av += dit[i] ;
		
		}
		dist_av  = dist_av /dislens;
    dist_f = (((float)dist_av)/65536.0) * (GP22_TNS/2) * C_VELOCITY;
		test_dit = dist_av;
		_TDC_GP21.tdc_distance = dist_f;
		test_distf = dist_f;
  // dist_av = 0;
}
