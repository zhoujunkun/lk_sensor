#include "tdc_gp21.h"
#include "z_param.h"
//
#define CONTENT_REG0   (uint32_t)( REG0_ANZ_FIRE(0) | REG0_DIV_FIRE(0) | REG0_ANZ_PER_CALRES(0) |     \
                        REG0_DIV_CLKHS(2) | REG0_START_CLKHS(1) |  REG0_ANZ_PORT(0) | REG0_TCYCLE(0) |  \
												REG0_ANZ_FAKE(0)  | EG0_SEL_ECLK_TEMP(1)| REG0_CALIBRATE(1) | REG0_NO_CAL_AUTO(0) | REG0_MESSB2(0) )

#define CONTENT_REG1   (uint32_t)( REG1_HIT2(0) | REG1_HIT1(1) | REG1_HITIN2(0) |       \
												REG1_HITIN1(1) | REG1_EN_FAST_INIT(0) | REG1_CURR32K(0) |    \
												REG1_SEL_START_FIRE(0) | REG1_SEL_TSTO2(0) | REG1_SEL_TSTO1(0) )

#define CONTENT_REG2    (uint32_t)( REG2_EN_INT(0b101) | REG2_RFEDGE2(0) | REG2_RFEDGE1(0))

//variable
GPIO_TypeDef *tx_signalGpio = (GPIO_TypeDef *)TDC_Signal_GPIO_Port;

void gp21_defaultcofg(void);
SPI_HandleTypeDef  *gp21_spi = &DRIVER_SPI_TDC;
uint8_t rxbuf[5] = {0};
uint8_t id[7] ={0};
void GP21_Init(void)
{
  gp21_rstn_idle();
	HAL_Delay(10);
	gp21_rstn_rsow();
  HAL_Delay(10);
	gp21_rstn_idle();
	HAL_Delay(10);
	gp21_en_startSignal();
	gp21_close_stop1Signal();
}


void gp21_hard_rst(void)
{
  gp21_rstn_idle();
	HAL_Delay(1);
	gp21_rstn_rsow();
  HAL_Delay(1);
	gp21_rstn_idle();
	HAL_Delay(1);
}

uint8_t  gp21_get_reg1Highbyte(void)
{
  gp21_write_cfg(OP_CODE_WR(0x01), 0x35410025);
	uint8_t gp21_id[2]={0};
	uint8_t txcmd[2]= {0xB5,0xff};
  gp21_select();
  HAL_SPI_TransmitReceive(gp21_spi,txcmd,gp21_id,2,0xff);
  gp21_release();
	if(gp21_id[1] == 0x35)
	{
		return 1;     //connet succeed
	}
	else
	  return 0;
}
#define GP21_END_HIT_INT  0x40000000U   

void gp21_defaultcofg(void)
{
	  gp21_write_cfg(OP_CODE_WR(0x00), 0x00242012);   //4��Ƶ
    gp21_write_cfg(OP_CODE_WR(0x01), 0x01410025);//0x01420023 //STOPͨ��1�����壬stopͨ��2�رգ����ٳ�ʼ����������,ALU��ǰ���ݴ���ļ��� stop ch1 -start
		//bit29 = 1 ALU ok
		//bit30 = 1 the received pulse counter is ready
		//bit31 = 1 TDC timeout overflow
		//gp21_write_cfg(OP_CODE_WR(0x02), 0xE0000011); //Timeout End Hits ALU�жϴ���, �������½���
		gp21_write_cfg(OP_CODE_WR(0x02), 0x40000011);
		gp21_write_cfg(OP_CODE_WR(0x03), 0x00000012); //����timeout ǿ��ALUд��0XFFFFFFFF������Ĵ������ر�		
		gp21_write_cfg(OP_CODE_WR(0x04), 0x20000013);  //Ĭ������
		gp21_write_cfg(OP_CODE_WR(0x05), 0x00000014);  //���崥�����رգ�������Ԫ�ر�
	  HAL_Delay(5);
		gp21_write_cfg(OP_CODE_WR(0x06), 0x00000015);  //������..�ر�			
/*
		gp21_write_cfg(OP_CODE_WR(0x00), 0x00242012);   //4��Ƶ
		gp21_write_cfg(OP_CODE_WR(0x01), 0x01410025);//0x01420023 //STOPͨ��1�����壬stopͨ��2�رգ����ٳ�ʼ����������,ALU��ǰ���ݴ���ļ��� stop ch1 -start
		//bit29 = 1 ALU ok
		//bit30 = 1 the received pulse counter is ready
		//bit31 = 1 TDC timeout overflow
		//gp21_write_cfg(OP_CODE_WR(0x02), 0xE0000011); //Timeout End Hits ALU�жϴ���, �������½���
		gp21_write_cfg(OP_CODE_WR(0x02), 0x40000011);
	 // gp21_write_cfg(OP_CODE_WR(0x02), 0x80000011);
		gp21_write_cfg(OP_CODE_WR(0x03), 0x00000012); //����timeout ǿ��ALUд��0XFFFFFFFF������Ĵ������ر�
		
		
		gp21_write_cfg(OP_CODE_WR(0x04), 0x20000013);  //Ĭ������
		gp21_write_cfg(OP_CODE_WR(0x05), 0x00000014);  //���崥�����رգ�������Ԫ�ر�
	  HAL_Delay(5);
		gp21_write_cfg(OP_CODE_WR(0x06), 0x00000015);  //������..�ر�		
	*/  
}

HAL_StatusTypeDef gp21_statu;
void gp21_write(uint8_t reg)
{
	uint8_t rg=reg;
	
  gp21_select();
	
	gp21_statu =  HAL_SPI_Transmit(gp21_spi,&rg,1,0xff); 
	if((gp21_statu == HAL_ERROR) ||(gp21_statu == HAL_TIMEOUT)) 
	{
	   
	
	}
	gp21_release();
}

void gp21_write_cfg(uint8_t op_code, uint32_t cfg32)
{
	 uint8_t trant=op_code;
	 uint8_t cfg[4] = {0};
	 cfg[0] = cfg32>>24;
	 cfg[1] = cfg32>>16;	 
	 cfg[2] = cfg32>>8;	 	
	 cfg[3] = cfg32;	 		 
   gp21_select();
   gp21_statu =  HAL_SPI_Transmit(gp21_spi,&trant,1,0xff); 
	 HAL_SPI_Transmit(gp21_spi,cfg,4,0xff); 
	 gp21_release();
}
  


void gp21_get_id(uint8_t *id_7bytes)
{
	uint8_t gp21_id[8]={0};
	uint8_t txcmd[8]= {OPC_ID,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
  gp21_select();
  HAL_SPI_TransmitReceive(gp21_spi,txcmd,gp21_id,8,0xff);
  gp21_release();
	for(int i=0;i<7;i++)
	  {
		  id[i] =  gp21_id[1+i];		
		}
	
}


uint32_t gp21_read_dword(uint8_t opcode)
{
   uint32_t result=0;
	uint8_t txcmd[5] ={opcode,opcode,opcode,opcode,opcode};
	gp21_select();
	HAL_SPI_TransmitReceive(gp21_spi,txcmd,rxbuf,5,0xffff);
   gp21_release();
	 result  = rxbuf[1]<<24 | rxbuf[2] <<16 | rxbuf[3] <<8 | rxbuf[4];   //���ݴ� �±�1��ʼ��Ч
  return result;
}


uint16_t  get_gp21_statu(void)
{
	uint32_t status_gp2 = 0;
  status_gp2=gp21_read_dword(OP_CODE_RD(0x04));	
  status_gp2>>=16;
  return  status_gp2;
	
}

uint32_t   gp21_read_diatance(void)
{

   return  gp21_read_dword(OP_CODE_RD(0x00));
}


  /*delay ns */
 void tdc_delay(uint32_t cval)
 {
    while(cval--);
 
   
 }

void gp21_startOneSignal(void)
{
		TDC_Signal_high() ;    /*tdc tx signal  high*/
	  tdc_delay(5);
		TDC_Signal_low() ;
}




