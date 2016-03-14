//Dactyl project v1.0
// Send and receive data over the USARTs

#include "usart.h"
#include "main.h"

// globals - buffers for use with Usart1 and 3
volatile buff_type Usart1_rx_buff,Usart1_tx_buff;
volatile buff_type Usart3_rx_buff,Usart3_tx_buff;
//Public functions

/**
  * @brief  Configured the USART1 and 2 periferals, including clocks
  * @param  None
  * @retval None
  */
void Usarts_Init() {
    GPIO_InitTypeDef    GPIO_InitStructure;
    USART_InitTypeDef   USART_InitStructure;
    // Initialise the buffers
    Init_Buffer(&Usart1_rx_buff, BUFFER_SIZE, 1);//Initialise the Usart1 Rx and Tx
    Init_Buffer(&Usart1_tx_buff, BUFFER_SIZE, 1);    
    Init_Buffer(&Usart3_rx_buff, (BUFFER_SIZE)/2, 1);//Initialise the Usart3 Rx and Tx. Rx is small as we have simple packets
    Init_Buffer(&Usart3_tx_buff, SP1ML_BUFFER, 1);   

    // Enable DMA clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    // Enable clock to GPIO and USART1, USART2, and USART3 peripherals - on different APBs
    RCC_APB2PeriphClockCmd(USART1_RCC_GPIO | USART1_RCC_USART | USART3_RCC_GPIO, ENABLE);
    RCC_APB1PeriphClockCmd(USART2_RCC_USART | USART3_RCC_USART, ENABLE );

    // Configure Tx pins
    GPIO_InitStructure.GPIO_Pin     = USART1_TX | USART2_TX;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
    GPIO_Init(USART_GPIO, &GPIO_InitStructure);
    
    // Configure Rx pins
    GPIO_InitStructure.GPIO_Pin     = USART1_RX | USART2_RX;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USART_GPIO, &GPIO_InitStructure);

    //Usart3 GPIO config
    GPIO_InitStructure.GPIO_Pin     = USART3_RX;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN_FLOATING;
    GPIO_Init(USART3_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = USART3_TX;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
    GPIO_Init(USART3_GPIO, &GPIO_InitStructure);

    // Configure USART1 peripheral
    USART_InitStructure.USART_BaudRate  = USART1_BAUD;
    Default_Usart_Config(&USART_InitStructure);
    USART_Init(USART1_USART, &USART_InitStructure );
    // Configure USART2 peripheral - only buadrate is changed
    USART_InitStructure.USART_BaudRate = GPS_DEFAULT_BAUD;
    USART_Init(USART2_USART, &USART_InitStructure );
    // Configure Usart3
    USART_InitStructure.USART_BaudRate = USART3_BAUD;
    USART_Init(USART3_USART, &USART_InitStructure );

    Init_Dma_Buffer(&Gps_Buffer, 256);//Initialise the Usart2 Rx for the GPS 
    //Configure the USART2 DMA etc
    DMA_USART2_Configuration(&Gps_Buffer);

    /* Enable USART2 DMA Rx request */
    USART_DMACmd(USART2_USART, USART_DMAReq_Rx , ENABLE);
    /* Enable the RXNE interrupt on USART1 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /* Enable the RXNE interrupt on USART3 */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    /* Enable the USART1 */
    USART_Cmd(USART1_USART, ENABLE);
    /* Enable the USART2 */
    USART_Cmd(USART2_USART, ENABLE);
    /* Enable the USART3 */
    USART_Cmd(USART3_USART, ENABLE);
}

/**
  * @brief  Setup the default USART config stuff
  * @param  Init type pointer
  * @retval None
  */
void Default_Usart_Config(USART_InitTypeDef* init) {
    init->USART_WordLength = USART_WordLength_8b;
    init->USART_StopBits = USART_StopBits_1;
    init->USART_Parity = USART_Parity_No;
    init->USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    init->USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
}

/**
  * @brief  Rebaud the Usart2
  * @param  uint16_t new baudrate
  * @retval None
  */
void USART2_reconf(uint32_t new_baud) {
    USART_InitTypeDef   USART_InitStructure;
    USART_Cmd(USART2_USART, DISABLE);
    USART_DeInit(USART2_USART);
    USART_InitStructure.USART_BaudRate  = new_baud;
    Default_Usart_Config(&USART_InitStructure);
    USART_Init(USART2_USART, &USART_InitStructure );
    /* Enable USART2 DMA Rx request */
    USART_DMACmd(USART2_USART, USART_DMAReq_Rx , ENABLE);
    /* Re-enable the USART2 */
    USART_Cmd(USART2_USART, ENABLE);
}

/**
  * @brief  Writes a string to USART1
  * @param  String pointer - null terminated
  * @retval None
  */
void Usart_Send_Str(char* str) {
	unsigned short int i = 0;
	while(str[i] != 0x00)
		__usart_send_char(str[i++]);
}

/**
  * @brief  Writes a string to USART3
  * @param  String pointer - null terminated
  * @retval None
  */
void Usart3_Send_Str(char* str) {
	unsigned short int i = 0;
	while(str[i] != 0x00)
		__usart3_send_char(str[i++]);
}

/**
  * @brief  Writes config to the GPS
  * @param  Buffer pointer - first byte is number of bytes
  * @retval None
  */
void Gps_Send_Utf8(char* str) {
	unsigned short int i=0;
	while(i<str[0])
		__gps_send_char(str[++i]);
}

/**
  * @brief  Writes config to the GPS
  * @param  String pointer - null terminated
  * @retval None
  */
void Gps_Send_Str(char* str) {
	uint8_t i=0;
	while(str[i])
        	__gps_send_char(str[i++]);
}


#ifdef USE_LIBC_PRINTF
/**
  * @brief  Retargets the C library printf function to the USART1.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1_USART, (uint8_t) ch);
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1_USART, USART_FLAG_TXE) == RESET) {;}
  return ch;
}
#endif

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
 * @brief USART1 Tx/Rx interrupt handler
 * @param None
 * @retval None
 */
__attribute__((externally_visible)) void USART1_IRQHandler(void) {
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
		//Clear pending bit and read the data.
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		uint8_t byte=((uint8_t)(USART_ReceiveData(USART1)&0x00FF));
		Add_To_Buffer(&byte, &Usart1_rx_buff);//Read the data, adding to the rx buffer.
	}
	else if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
		USART_ClearITPendingBit(USART1, USART_IT_TXE);	//Clear pending bit.
		uint8_t tx_data;
		Get_From_Buffer(&tx_data, &Usart1_tx_buff);	//Read the data from the tx buffer.
		USART_SendData(USART1_USART, tx_data);
		if(!anything_in_buff(&Usart1_tx_buff))		/*No more data to send?*/
			USART1->CR1 &=~(1<<7);			//Disable the interrupt here.
	}
	else
		USART_ReceiveData(USART1);			//This might occur in an overrun. Noise and framing errors are read as normal & cleared	
}

/**
 * @brief USART3 Tx/Rx interrupt handler
 * @param None
 * @retval None
 */
__attribute__((externally_visible)) void USART3_IRQHandler(void) {
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
		//Clear pending bit and read the data.
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		uint8_t byte=(uint8_t)(USART_ReceiveData(USART3)&0x00FF);
		Add_To_Buffer(&byte, &Usart3_rx_buff);//Read the data, adding to the rx buffer.
	}
	else if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {
		USART_ClearITPendingBit(USART3, USART_IT_TXE);	//Clear pending bit.
		if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)) {	//CTS is low (active low)
			uint8_t tx_data;
			Get_From_Buffer(&tx_data, &Usart3_tx_buff);//Read the data from the tx buffer.
			USART_SendData(USART3_USART, tx_data);
			SP1ML_tx_bytes++;			//A byte was sent
			if(!anything_in_buff(&Usart3_tx_buff) || (!(SP1ML_tx_bytes%PAYLOAD_BYTES)&&SP1ML_withold))/*No more data, or the transmission is blocked*/
				USART1->CR1 &=~(1<<7);		//Disable the interrupt here.
		}
	}
	else
		USART_ReceiveData(USART1);			//This might occur in an overrun. Noise and framing errors are read as normal & cleared	
}

//Private functions
void __usart_send_char(char data) {
	while(count_in_buff((&Usart1_tx_buff))>=255)		/*Do not overfill the buffer*/
		__WFI();					/*Warning, this can busy wait if the buffer is overfilled*/
	Add_To_Buffer(&data, &Usart1_tx_buff);			/*Add to tx buffer*/
	USART1->CR1 |=(1<<7);					/*Enable the TXE interrupt on USART1*/	
}

void __usart3_send_char(char data) {
    USART_SendData(USART3_USART, data);
    while(USART_GetFlagStatus(USART3_USART, USART_FLAG_TXE) == RESET) {}
}

void __gps_send_char(char data) {
    USART_SendData(USART2_USART, data);
    while(USART_GetFlagStatus(USART2_USART, USART_FLAG_TXE) == RESET) {}
}

void __sp1ml_send_char(char data) {
	Add_To_Buffer(&data, &Usart3_tx_buff);			/*Add to tx buffer*/
	if(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) && !SP1ML_withold)/*Only enable if we are not blocked*/
		USART3->CR1 |=(1<<7);				/*Enable the TXE interrupt on USART3*/	
}
