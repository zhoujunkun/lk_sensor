#include "z_flashParamSave.h"

#define FLASH_SIZE 256 //��ѡMCU��FLASH������С����λK��

#if FLASH_SIZE<256
	#define NUMBER_PAGE 1   //�ֽ�
#else
	#define NUMBER_PAGE 2   //ҳ
#endif
FLASH_EraseInitTypeDef FlashTypeDef;	


void flash_writeMoreData( uint16_t *toWriteData, uint16_t toSize)
{

  HAL_FLASH_Unlock();  //����flash
  uint32_t PageError = 0;   
	HAL_FLASHEx_Erase(&FlashTypeDef,&PageError);     //����Flash
	
	for(int index=0;index<toSize;index++)
	{
	   HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,PARAM_FLASH_ADDRESS+index*2,toWriteData[index]); 
	}
 
  HAL_FLASH_Lock();  //����
}


void flash_SaveInit(void)
{
  
	FlashTypeDef.TypeErase = FLASH_TYPEERASE_PAGES;
	FlashTypeDef.PageAddress = PARAM_FLASH_ADDRESS;
	FlashTypeDef.NbPages = NUMBER_PAGE;
}

void flash_paramRead(uint8_t *buff, uint16_t lens)
{
	uint8_t *poitBuff = (uint8_t *)PARAM_FLASH_ADDRESS;
	  memcpy(buff,poitBuff,lens);
    
}