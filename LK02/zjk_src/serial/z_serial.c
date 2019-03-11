#include "z_serial.h"
#include "cmsis_os.h"
#include "z_param.h"
#define MAX_LIST_SIZES 10

UART_HandleTypeDef  *z_serial = &DRIVER_SERIAL_HUART;
static void z_serial_dma_start(void);
uint16_t rev_lens = 0;
SemaphoreHandle_t  serialSemaphore =NULL;  //�����ź��������ڴ��ڿ����жϱ�־����֪ͨ����ִ��
uint8_t serial_rxbuf[50] = {0};
_dma_listen dma_listen[MAX_LIST_SIZES]={NULL};
uint16_t GetRevBytes(void);

/*
�������е������
*/
void z_serialDriverTask(void const * argument)
{
   	 BaseType_t xResult;
	   z_serial_init();    //dma start
   for(;;)
	  {
			xResult = xSemaphoreTake(serialSemaphore,portMAX_DELAY);
			if(xResult == pdTRUE)  //���յ����ڽ�������ź�
			{
				rev_lens = GetRevBytes(); //
					for(int i=0;i<MAX_LIST_SIZES;i++)
				{
					 if(dma_listen[i].listFunc != NULL)
					 {
						 dma_listen[i].listFunc(&serial_rxbuf[0]);
					 }
				}	   
				HAL_UART_DMAStop(z_serial);
				z_serial_dma_start(); 			
			}			
		
		}

}


//��ȡ���ո���
void get_revLens(uint16_t *data)
{
   *data = rev_lens;
}

/*
@func:������ݽ�����ɼ�������������������
������ɿ��к��Զ�����

*/
 SERIAL_TYPE_BOOL addUartDmaRevListen(_listenFunc func)
{
	
	   static uint8_t count=0;
	   dma_listen[count++].listFunc =func;		 
		 if(count >MAX_LIST_SIZES)  return SERIAL_FALSE;
		 return SERIAL_TRUE;
}

static void z_serial_dma_start(void)
{
  HAL_UART_Receive_DMA(z_serial,serial_rxbuf,sizeof(serial_rxbuf));

}

void z_serial_write( uint8_t *ch, uint32_t lens)
{

   HAL_UART_Transmit(z_serial,ch,lens,0xff);
}

int fputc(int ch, FILE* stream)
{
     HAL_UART_Transmit(z_serial,(uint8_t*)&ch,1,0xff);
    return ch;
}

//���ڳ�ʼ��
void z_serial_init(void)
{
   __HAL_UART_ENABLE_IT(z_serial, UART_IT_IDLE);//ʹ�ܽ��տ����ж�
    z_serial_dma_start();   //����dma����
	  serialSemaphore = xSemaphoreCreateBinary();
    if(serialSemaphore == NULL) //create fail
		{
		
		
		}		

}

//�ڴ����жϺ����е��ã�����Ƿ���У����DMA���ղ�����������
void usartIdleInt(void)
{
   uint32_t rcflag=0;
	BaseType_t XHigherPriorityTaskWoken = pdTRUE;
	rcflag = __HAL_UART_GET_FLAG(z_serial,UART_FLAG_IDLE);
	if(rcflag !=RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(z_serial);  //��������жϱ�־
		xSemaphoreGiveFromISR(serialSemaphore,&XHigherPriorityTaskWoken);
	}

}


//��ȡdma �����ֽڸ���
uint16_t GetRevBytes(void)
{
	uint16_t bytes=0;
  bytes = __HAL_DMA_GET_COUNTER(z_serial->hdmarx);
  bytes = sizeof(serial_rxbuf)-bytes;
	return bytes;
}

