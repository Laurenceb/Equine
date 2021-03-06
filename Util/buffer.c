#include <stdlib.h>
#include <string.h>
#include "buffer.h"

void Add_To_Buffer(void * data, buff_type* buffer) {
	memcpy(&(buffer->data[buffer->head]),data,buffer->block);//Put data in and increment, use memcpy to copy the correct number of bytes
	buffer->head+=buffer->block;
	buffer->head%=buffer->size;
	if(buffer->head==buffer->tail)	//Buffer wraparound due to filling
		buffer->tail=(buffer->tail+buffer->block)%buffer->size;
}

uint8_t Get_From_Buffer(void * data, buff_type* buffer) {
	if(buffer->tail==buffer->head) {
		*(uint8_t*)data=0;	//Data reset if there is nothing to return
		return 1;		//Error - no data in buffer
	}
	else {
		memcpy((uint8_t*)data,&(buffer->data[buffer->tail]),buffer->block);//grab a data sample from the buffer
		buffer->tail+=buffer->block;
		buffer->tail%=buffer->size;
		return 0;		//No error
	}
}

/**
* @brief Returns a sample from the buffer.
* @param Buffer pointer
* @retval byte
*/
void Pop_From_Buffer(void * data, buff_type* buffer) {
	memcpy(data,&(buffer->data)[buffer->tail],buffer->block);//read data at tail
	buffer->tail=(buffer->tail+buffer->block)%buffer->size;
}

uint8_t Pop_From_Dma_Buffer(volatile dma_buff_type* buffer) {//DMA buffers are byte based
	uint8_t d=(buffer->data)[buffer->tail];//read data at tail
	buffer->tail=(buffer->tail+1)%buffer->size;
	return d; //returns the byte
}

void Empty_Buffer( buff_type* buffer) {
	buffer->tail=buffer->head;	//Set the tail to the head, to show there is no usable data present.
}

void Empty_Dma_Buffer(volatile dma_buff_type* buffer) {
	buffer->tail=(uint16_t)(*(volatile uint32_t*)(buffer->head));
}

/**
  * @brief  Returns number of bytes in the DMA buffer.
  * @param  Buffer pointer
  * @retval bytes in buffer
  */
int16_t Bytes_In_DMA_Buffer(volatile dma_buff_type* buffer)
{
	return ((buffer->size-buffer->tail-(uint16_t)(*(volatile uint32_t*)(buffer->head)))%buffer->size);
}

//Size argument is the number of elements
void Init_Buffer( buff_type* buff, uint16_t size, uint8_t blocksize) {
	buff->data=(uint8_t*)malloc(size*blocksize);
	buff->size=size*blocksize;//this is in units of bytes
	buff->block=blocksize;
	buff->head=0;
	buff->tail=0;
}

void Init_Dma_Buffer(volatile dma_buff_type* buff, uint16_t size) {
	buff->data=(uint8_t*)malloc(size);
	buff->size=size;
}
