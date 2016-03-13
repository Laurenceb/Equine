//Dactyl project v1.0

#include "dma.h"

//the USART2 Rx DMA - this connects to the GPS
#define USART2RX_DMA1 DMA1_Channel6

/**
  * @brief  Configures the DMA.
  * @param  Buffer pointer
  * @retval None
  * Note this is coded for USART2 Rx - the GPS, and defined in main.h
  */
void DMA_USART2_Configuration(dma_buff_type* buffer)
{ /* Feed this function a pointer to the circular buffer, and a size integer */
  DMA_InitTypeDef DMA_InitStructure;
  /* USART2 RX DMA1 Channel (triggered by USART2 Rx event) Config */
  DMA_DeInit(USART2RX_DMA1);  
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART2-> DR);
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(buffer->data);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = (uint32_t)buffer->size;
  DMA_Init(USART2RX_DMA1, &DMA_InitStructure);
  buffer->head=&(USART2RX_DMA1->CNDTR);
  DMA_Cmd(USART2RX_DMA1, ENABLE);
}

