#include "z_serial.h"
#include "cmsis_os.h"
#include "z_param.h"
#define MAX_LIST_SIZES 10

UART_HandleTypeDef  *z_serial = &DRIVER_SERIAL_HUART;
static void z_serial_dma_start(void);
uint16_t rev_lens = 0;
SemaphoreHandle_t  serialSemaphore =NULL;  //创建信号量，用于串口空闲中断标志检查后通知任务执行
uint8_t serial_rxbuf[50] = {0};
_dma_listen dma_listen[MAX_LIST_SIZES]={NULL};
uint16_t GetRevBytes(void);

/*
在任务中调用添加
*/
void z_serialDriverTask(void const * argument)
{
   	 BaseType_t xResult;
	   z_serial_init();    //dma start
   for(;;)
	  {
			xResult = xSemaphoreTake(serialSemaphore,portMAX_DELAY);
			if(xResult == pdTRUE)  //接收到串口接收完成信号
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


//获取接收个数
void get_revLens(uint16_t *data)
{
   *data = rev_lens;
}

/*
@func:添加数据接收完成监听函数，当串口数据
传输完成空闲后自动调用

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

//串口初始化
void z_serial_init(void)
{
   __HAL_UART_ENABLE_IT(z_serial, UART_IT_IDLE);//使能接收空闲中断
    z_serial_dma_start();   //启动dma接收
	  serialSemaphore = xSemaphoreCreateBinary();
    if(serialSemaphore == NULL) //create fail
		{
		
		
		}		

}

//在串口中断函数中调用，检查是否空闲，配合DMA接收不定长度数据
void usartIdleInt(void)
{
   uint32_t rcflag=0;
	BaseType_t XHigherPriorityTaskWoken = pdTRUE;
	rcflag = __HAL_UART_GET_FLAG(z_serial,UART_FLAG_IDLE);
	if(rcflag !=RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(z_serial);  //清除空闲中断标志
		xSemaphoreGiveFromISR(serialSemaphore,&XHigherPriorityTaskWoken);
	}

}


//获取dma 接收字节个数
uint16_t GetRevBytes(void)
{
	uint16_t bytes=0;
  bytes = __HAL_DMA_GET_COUNTER(z_serial->hdmarx);
  bytes = sizeof(serial_rxbuf)-bytes;
	return bytes;
}

