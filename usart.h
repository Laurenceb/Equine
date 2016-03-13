//Dactyl project v1.0
// Send and receive data over the USARTs
#pragma once
#include "stm32f10x.h"
#include <stdio.h>
#include "buffer.h"
#include "ubx.h"
#include "dma.h"
#include "sp1ml_command.h"

extern volatile buff_type Usart1_rx_buff,Usart1_tx_buff;
extern volatile buff_type Usart3_rx_buff,Usart3_tx_buff;
//Defines - USART 1 and 2 used

//This is the DEBUG header
#define USART_GPIO       GPIOA
#define USART1_RCC_GPIO   RCC_APB2Periph_GPIOA
#define USART1_USART      USART1
#define USART1_RCC_USART  RCC_APB2Periph_USART1
#define USART1_TX         GPIO_Pin_9
#define USART1_RX         GPIO_Pin_10
#define USART1_BAUD       115200
//This is the GPS module - must be on same GPIO with this code
#define USART2_RCC_GPIO   RCC_APB2Periph_GPIOA
#define USART2_USART      USART2
#define USART2_RCC_USART  RCC_APB1Periph_USART2
#define USART2_TX         GPIO_Pin_2
#define USART2_RX         GPIO_Pin_3
#define USART2_BAUD       115200
//This is the SP1ML module - runs with 115200 baud comms
#define USART3_GPIO	 GPIOB
#define USART3_RCC_GPIO   RCC_APB2Periph_GPIOB
#define USART3_USART      USART3
#define USART3_RCC_USART  RCC_APB1Periph_USART3
#define USART3_TX         GPIO_Pin_10
#define USART3_RX         GPIO_Pin_11
#define USART3_BAUD       115200

//Public functions
void Usarts_Init();
void Default_Usart_Config(USART_InitTypeDef* init);
void Usart_Send_Str(char* str);
void Usart3_Send_Str(char* str);

/* Private function prototypes -----------------------------------------------*/
#ifdef USE_LIBC_PRINTF	/*define in main.h to set the printf function that is used */
	#ifdef __GNUC__
		/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
		set to 'Yes') calls __io_putchar() */
		#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
	#else
		#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#endif /* __GNUC__ */
#else
	#define RPRINTF_FLOAT
	#define RPRINTF_COMPLEX
	/*reduced printf functionality from Pascal Stang, uncomment as appropriate*/
	#define printf rprintf2RamRom
#endif /*USE_LIBC_PRINTF*/



//Function prototypes
void __usart_send_char(char data);
void __usart3_send_char(char data);
void Gps_Send_Str(char* str);
void Gps_Send_Utf8(char* str);
void __gps_send_char(char data);
void __sp1ml_send_char(char data);

