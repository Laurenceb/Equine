#include "stm32f10x.h"
#pragma once

#define anything_in_buff(buffer) !((buffer)->head==(buffer)->tail)
#define count_in_buff(buffer) ((((buffer)->head-(buffer)->tail+(buffer)->size)%(buffer)->size)/(buffer->block))

// This buffer has variable 
typedef struct{
	volatile uint16_t head;
	volatile uint16_t tail;
	uint16_t size;
	uint8_t* data;
	uint8_t block;
} buff_type;

typedef struct{
	volatile uint32_t* head;
	uint16_t tail;
	uint16_t size;
	uint8_t* data;
} dma_buff_type;

//Functions
void Add_To_Buffer(void* data, buff_type* buffer);
uint8_t Get_From_Buffer(void* data, buff_type* buffer);
void Pop_From_Buffer(void* data, buff_type* buffer);
uint8_t Pop_From_Dma_Buffer(volatile dma_buff_type* buffer);
void Empty_Buffer(buff_type* buffer);
void Empty_Dma_Buffer(volatile dma_buff_type* buffer);
int16_t Bytes_In_DMA_Buffer(volatile dma_buff_type* buffer);
void Init_Buffer(buff_type* buff, uint16_t size, uint8_t blocksize);
void Init_Dma_Buffer(volatile dma_buff_type* buff, uint16_t size);
