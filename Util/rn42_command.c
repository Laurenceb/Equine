#include "rn42_command.h"

volatile uint8_t RN42_aligned_data_ready;
volatile SP1ML_tx_rx_state_machine_type RN42_tx_rx_state={ .upper_level_state = ASSIGNED}; // There is no upper level network configuration for BT, so init assigned
volatile uint8_t RN42_tx_sequence_number;

/**
  * @brief Attempt to get the name of a RN42 module (this is usually used for datalogger device name assignment)
  * @param Pointer to serial 'number' string, uint8_t variable set to non zero to enable automatic configuration of RN42 modules (typically for new ones)
  * @reval Returns zero for success, nonzero for failure (1==no name, 2==failed to get module into command mode)
  */
uint8_t RN42_get_name(uint8_t SerialNumber[8], uint8_t allowconfig) {
	if(!RN42_get_command()) {			//If we were able to get Command mode
		uint8_t datavar;
		while(anything_in_buff((&Usart1_rx_buff))) { //Empty CMD from buffer
			Get_From_Buffer(&datavar,&Usart1_rx_buff);//Take from buffer
		}
		Usart_Send_Str((char*)"GN\r\n");	//Get Name command (note that this is busy wait based)
		Watchdog_Reset();
		{
		uint8_t counter=0;
		uint32_t millis=Millis;			//Store this for reference
		while(Millis<(millis+150)) {		//Times out after 150ms
			if(anything_in_buff((&Usart1_rx_buff))) {//Get serial number and place in buffer
				Get_From_Buffer(&(SerialNumber[counter]),&Usart1_rx_buff);//Take from buffer
				if((SerialNumber[counter]=='\n') || (SerialNumber[counter]=='\r') || (counter>=7)) { //Break out if '\r' or reach end
					SerialNumber[counter]=0x00;//Null terminate
					break;
				}
				counter++;
			}
		}
		if(counter>2) {
			//SerialNumber[counter-2]=0;	//Remove the return
			datavar=0;			//Success
		}
		else {					//Something wrong - no name
			strcpy(SerialNumber,"NNM");	//Store an error name
			datavar=1;			//One means no name has been assigned
		}
		}
		Usart_Send_Str((char*)"---\r");		//Return to Data mode
		return datavar;
	}
	else {
		strcpy(SerialNumber,"ERR");		//If RN42 module failure, name goes to "ERR"
		if(allowconfig)				//Usually if time.txt file either doesnt exist or is empty, try to configure the RN-42 (!(FATFS_info.fsize))
			RN42_conf();			//Try to configure the RN-42
		return 2;				//2 means the RN42 command mode failed
	}
}

/**
  * @brief Attempt to force a RN42 bluetooth into command mode
  * @param None
  * @reval Returns zero for success, nonzero for failure
  */
uint8_t RN42_get_command(void)
{
	uint32_t millis_startget=Millis;		//Store entry time
	uint32_t millis_waitreply;
	uint8_t datavar;				//Used to store the read characters
	const char Reply[]="CMD\r\n";
	while(Millis<(millis_startget+1550)) {		//Loop in here until we break out or we get timeout
		uint8_t read_characters=0;
		while(anything_in_buff(&Usart1_rx_buff)) {//Empty any data from buffer
			Get_From_Buffer(&datavar,&Usart1_rx_buff);//Take from buffer
		}
		Usart_Send_Str((char*)"$$$");		//Try to enter Command Mode
		millis_waitreply=Millis;		//Store the time at which we sent the command
		while(Millis<(millis_waitreply+500)) {	//Wait for a timeout or the correct reply
			if(anything_in_buff(&Usart1_rx_buff)) {//Get any data from buffer
				Get_From_Buffer(&datavar,&Usart1_rx_buff);
				if(Reply[read_characters]==datavar) { //Take from buffer and compare
					read_characters++;
				}
				else {
					read_characters=0;
				}
				if(read_characters==(sizeof(Reply)-1)) {
					return 0;
				}
			}
			else {
				__WFI();		//Standby whilst we wait for some data
			}
		}
	}
	return 1;
}

/**
  * @brief  This function generates a device to base station packet, uses a reverse COBS format that requires working backwards from end of packet
  * @param  Pointer to output buffer, pointer to input data, number of data bytes, device ID on the network, pointer to sequence number (iterates)
  * @retval None
  */
void RN42_generate_packet(uint8_t* data_payload, uint8_t number_bytes, uint8_t device_network_id, uint8_t* sequence_number) {
	const uint8_t header=HEAD;			//COBS uses zero as the id byte, with xor over everything with the head byte
	uint8_t tmp;
	Add_To_Buffer(&header,&Usart1_tx_buff);
	uint8_t skip=1;					//The reverse COBS applies to the two header bytes as well
	tmp=device_network_id^header;			//xor all the payload
	if(tmp!=header) {
		Add_To_Buffer(&tmp,&Usart1_tx_buff);	//Packet header is: device network id byte and sequence number (for scrolling graph smoothness)
		skip++;
	}
	else {
		tmp=skip^header;
		Add_To_Buffer(&tmp,&Usart1_tx_buff);
	}
	tmp=(*sequence_number)^header;
	if(tmp!=header) {
		Add_To_Buffer(&tmp,&Usart1_tx_buff);	//This is similar to HDLC packet format
		skip++;
	}
	else {
		tmp=skip^header;
		Add_To_Buffer(&tmp,&Usart1_tx_buff);
		skip=1;
	}
	for(uint8_t n=0; n<number_bytes; n++) {
		if(data_payload[n]==HEAD) {
			__usart_send_char(skip^header);	//The send_char routine is used here as it will kick start the interrupt driven usart comms if needs be
			skip=1;
		}
		else {
			skip++;
			__usart_send_char(data_payload[n]^header);
		}
	}
	__usart_send_char(skip^header);			//Work backwards from the end of the packet, skipping skip bytes and replacing with HEAD until packet header
	(*sequence_number)++;				//Incriment this
}

/**
  * @brief  Configures the USART1 to talk to the RN42, and sets it up. Note that this should only be used to configure new modules!
  * @param  None
  * @retval None
  */
void RN42_conf(void) {
	USART_InitTypeDef   USART_InitStructure;
	//Wait for buffer to empty
	while(anything_in_buff(&Usart1_tx_buff))
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
	while(anything_in_buff(&Usart1_tx_buff))
		__WFI();
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_Cmd(USART1_USART, DISABLE);
	USART_DeInit(USART1_USART);
	USART_Init(USART1_USART, &USART_InitStructure );//now the module should be configured
	USART_Cmd(USART1_USART, ENABLE);
}


