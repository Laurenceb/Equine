#include "rn42_conf.h"


/**
  * @brief  Configures the USART1 to talk to the RN42, and sets it up
  * @param  None
  * @retval None
  */
void rn42_conf() {
	USART_InitTypeDef   USART_InitStructure;
	//Wait for buffer to empty
	while(Bytes_In_ISR_Buffer(&Usart1_tx_buff))
		__WFI();
	USART_Cmd(USART1_USART, DISABLE);
	USART_DeInit(USART1_USART);
	//Set the config for default RN42 settings
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_BaudRate  = USART1_BAUD;//use the default
	USART_Init(USART1_USART, &USART_InitStructure );
	USART_Cmd(USART1_USART, ENABLE);
	Usart_Send_Str((char*)"$$$");Delay(150000);
	Usart_Send_Str((char*)"$$$");Delay(150000);
	Usart_Send_Str((char*)"\nD\n");Delay(150000);
	Usart_Send_Str((char*)"SH,0000\n");Delay(100000);
	Usart_Send_Str((char*)"S~,0\n");Delay(100000);
	Usart_Send_Str((char*)"SL,E\n");Delay(100000);
	Usart_Send_Str((char*)"R,1\n");	//causes the setting to take effect
	//Wait for buffer to empty
	while(Bytes_In_ISR_Buffer(&Usart1_tx_buff))
		__WFI();
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_Cmd(USART1_USART, DISABLE);
	USART_DeInit(USART1_USART);
	USART_Init(USART1_USART, &USART_InitStructure );//now the module should be configured
	USART_Cmd(USART1_USART, ENABLE);
}


