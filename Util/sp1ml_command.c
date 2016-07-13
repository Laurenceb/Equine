#include "sp1ml_command.h"
#include "main.h"
#include "lsm9ds1.h"
#include "dcm_attitude.h"

volatile uint8_t SP1ML_state;				//Used externally to check for correct operation
volatile uint8_t SP1ML_decoder_state;			//Used for the message input format

volatile uint8_t SP1ML_network_address=NETWORK;
volatile uint8_t SP1ML_tx_sequence_number;
volatile uint8_t SP1ML_withold;
volatile uint32_t SP1ML_tx_bytes;			//This is used to chop transmitted data off on packet boundaries 

volatile uint8_t SP1ML_aligned_data_ready;		//Flag use to signal that the 250hz aligned data samples are all ready

volatile SP1ML_tx_rx_state_machine_type SP1ML_tx_rx_state;//This is used for the high speed decoder/encoder

//Input packets consist of two byte header,single byte type (0-f), single byte argument (0-f) or upper nibble, low nibble count (0-f), 4 nibble bytes of mask (0-f)
//Function is designed to take a pointer to state struct, can be used for BT commands
void SP1ML_rx_tx_data_processor(SP1ML_tx_rx_state_machine_type* stat,void (*generate_packet)(uint8_t*,uint8_t,uint8_t,uint8_t*), buff_type* buff, uint8_t* tx_s, uint8_t* flag) {
	const uint8_t request_header[]=REQUEST_HEADER;	//This is the request header, a simple ASCII and HEX argument format
	uint8_t datavar;
	while(anything_in_buff(buff)) {	//Loop through all the data in the incoming buffer
		Get_From_Buffer(&datavar,buff);
		switch(stat->state) {
		case 0:
			if(datavar==request_header[0])	//Packets my be followed by blank data to cause the SP1ML to send packet quickly
				stat->state++;		//Any trailing blank data following a packet will be read here and ignored
			break;
		case 1:					//Two character packet header
			if(datavar==request_header[1])
				stat->state++;
			break;
		case 2:					//The type byte (e.g. address assignment or a data ping request)
			stat->internal_type=hex_to_byte(datavar);//Type is ascii argument in range 0-f (there can be up to 15 message types)			
			stat->state++;
			break;
		case 3:					//First part of the control argument
			stat->argument=hex_to_byte(datavar);
			if(stat->internal_type<=ASSIGNED) {//Init ping or assigned pingback
				stat->main_counter=0;	//This will stop any more data being sent if we have a non request commend during data sample transmission
				stat->signal=stat->internal_type;//This is a global flag indicating that we can act upon the received data
			}
			else if(stat->internal_type>=REQUEST) {//Request command is the other type, followed by argument with number of sample sets, then 2 byte mask
				stat->state++;
				stat->sample_counter=stat->argument<<4;//This will count down and be handled from within this function
			}
			else
				stat->state=0;		//Received an initial ping or an invalid message type, reset
			break;
		case 4:					//Mask of samples to pass back follows
			datavar=hex_to_byte(datavar);	//The argument is an ascii hex digit (lower case) in the range 0-f
			stat->sample_counter+=datavar;	//Add in the lower nibble
			if(stat->sample_counter) {	//Non zero
				SP1ML_withold=0;	//If we get a non zero request, then allow the SP1ML to transmit (so don't try this command from RN42!)
				stat->state++;
			}
			else {
				SP1ML_withold=1;	//Withhold data until we get a new request
				stat->state=0;		//Loop the state back around to zero, we don't expect any nibbles of mask data to be following
			}
			break;
		case 5:					//For the request packet, there are 4 nibbles of mask. This allows up to 16 channels of data to be sent
			stat->internal_mask=hex_to_byte(datavar)<<12;
			stat->state++;
			break;
		case 6:
			stat->internal_mask|=hex_to_byte(datavar)<<8;
			stat->state++;
			break;
		case 7:
			stat->internal_mask|=hex_to_byte(datavar)<<4;
			stat->state++;
			break;
		case 8:
			stat->internal_mask|=hex_to_byte(datavar);
			stat->main_mask=stat->internal_mask;//Start using this
			stat->main_counter=stat->sample_counter;//Use this too
			stat->state=0;			//Reset the state, as this should be the end of the message
			stat->signal=stat->internal_type;//This should be REQUEST, a state that is not actually used externally at present
		}
	}
	if(stat->main_mask&&stat->main_counter&&*flag && stat->upper_level_state==ASSIGNED) {//Next handle the data request, only if we are in correct state and have data to send
		uint8_t numbits=0;
		for(uint8_t n=0; n<16; n++) {
			if(stat->main_mask&(1<<n))
				numbits++;	
		}
		uint8_t data[numbits*2];
		numbits=0;
		for(uint8_t n=0; n<16; n++) {		//This does not use buffers between itself and the sensor reading functions, so must run at >= hz
			if(stat->main_mask&(1<<n)) {	//The mask bit is set
				if(n<8)			//First 8 channels are the ECG, we use the immediate samples, which are added into globals by respective funcs
					*(uint16_t*)&(data[numbits])=Filtered_ECG[n];//Last 8 channels are the IMU, excluding the z compass data
				else if(stat->signal==REQUEST_TWO){
					if(n<11)
						*(uint16_t*)&(data[numbits])=((uint16_t*)&(LSM9DS1_Gyro_Buffer.x))[n-8];//Use sample buffers from I2C driver 
					else if(n<14)
						*(uint16_t*)&(data[numbits])=((uint16_t*)&(LSM9DS1_Acc_Buffer[n-11]));
					else
						*(uint16_t*)&(data[numbits])=((uint16_t*)&(LSM9DS1_Mag_Buffer[n-14]));
				}
				else {			//The complimentary filtered data, followed by alternating heading (0 to -360) or battery v (mv), vel, gps pos
					float magnitude=-1;
					float euler[3];
					if(!(((stat->main_mask&0x700)>>8)&((1<<(n-8))-1)) && (stat->main_mask&0x700)) {//If Euler angles are requested, run the filter
						float acc[3], mag[3], gyro[3];//These are used to pass the data to the filter
						gyro[0]=(float)(*(int16_t*)&(LSM9DS1_Gyro_Buffer.x));
						gyro[1]=(float)(*(int16_t*)&(LSM9DS1_Gyro_Buffer.y));
						gyro[2]=-(float)(*(int16_t*)&(LSM9DS1_Gyro_Buffer.z));
						for(uint8_t m=0; m<3; m++) {
							acc[m]=(float)(*(int16_t*)&(LSM9DS1_Acc_Buffer[m]));
							mag[m]=(float)(*(int16_t*)&(LSM9DS1_Mag_Buffer[m])-LSM9DS1_Mag_Offset[m]);//Take off the magno offset
							gyro[n]*=GYRO_TO_RADIANS;//Convert gyro data to radian units
						}
						acc[2]=-acc[2];//Swap sign of z axes
						mag[2]=-mag[2];
						{float g=mag[0];mag[0]=mag[1];mag[1]=g;}//Fix magno handedness (everything is now in NED space)
						magnitude=main_filter( DCM_glob, mag, acc, euler, gyro, (float)(1.0/250.0));//Run filter, pass output to euler
					}
					if(n<11) {//The Euler angles are sent
						int16_t ang=(euler[n-8]*180.0/M_PI);//Scale to angle in degrees
						*(uint16_t*)&(data[numbits])=*(uint16_t*)&ang;//These are the first three arguments
					}
					if(n==11) {//The fourth extra argument is the acceleration offset in milliG
						if(magnitude<0) {
							float acc[3];
							for(uint8_t m=0; m<3; m++)
								acc[m]=(float)(*(int16_t*)&(LSM9DS1_Acc_Buffer[m]))*LSM9DS1_ACC_SCALE_FACTOR;
							magnitude=sqrtf(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);//mG acc offset from 1G (range is -1000 to +15000)
						}
						int16_t magn=(int16_t)((magnitude-1.0)*1e3);
						*(uint16_t*)&(data[numbits])=*(uint16_t*)&magn;//Load magnitude into the buffer
					}
					if(n>11 && GPS_telem.flag) //Load GPS data if it is ready (heading, velocity, then positions)
						*(uint16_t*)&(data[numbits])=((uint16_t*)&GPS_telem)[n-12];//use pointers to directly load
				}
				numbits+=2;		//Jump 2 bytes
			}
			if(n==15)
				GPS_telem.flag=0;	//At the end of the packetisation, wipe the data ready flag
		}
		(*generate_packet)(data, numbits, SP1ML_network_address, tx_s);//Send a data packet, the SP1ML network address is reused for the bluetooth
		stat->main_counter--;			//Update these
		(*flag)=0;				//Ready for new data to be loaded
	}
	else if(*flag && stat->sequence_is_time && SP1ML_state==ASSIGNED)//If device connection unreliable its better for sequence number to count samples to state
		(*tx_s)++;				//machine, rather than actual transmitted samples. Set this variable to be true for SP1ML, false for RN42 
}

/**
  * @brief Runs the (main loop context, this is IMPORTANT!) SP1ML network address/network name management
  * @param Pointer to the name string
  * @reval None
  */
void SP1ML_manager(uint8_t* SerialNumber, SP1ML_tx_rx_state_machine_type* stat) {//2nd state machine, called from main loop to ctrl SP1ML functionality
	static uint8_t reply_randomiser_delay;		//This is used to prevent a collision if two devices are being assigned with addresses
	static uint8_t inner_delay,seed,timing;		//Used for randomising pingback
	switch(stat->upper_level_state) {
	case INIT:
		if(!seed) {
			srand(Millis);			//Rand gets init using the time, which should be semi random due to start up delays
			seed=1;
		}
		if(stat->signal==PUNG && !timing) {	//This is set to indicate that the device has received a ping enquiry from the base station
			inner_delay=(uint8_t)rand()%255;//Set random timeout
			stat->signal=0;			//Reset the message
			timing=1;
		}
		else if(!(--inner_delay)&&timing) {	//After the timeout, send the packet. But only enter the timeout if we have a set flag
			SP1ML_tx_sequence_number=0;	//Ensure this is reset. A packet with the default address and zero sequence number is processed as a name
			SP1ML_generate_packet(SerialNumber,strlen(SerialNumber),SP1ML_network_address,&SP1ML_tx_sequence_number);
			stat->upper_level_state++;	//With the ping response sent
			timing=0;			//Reset this
		}
		else if(stat->signal==ASSIGNED && !stat->argument) {//Assigning an invalid network address of 0x00 to a device that has not pungback a name configures
			if(!SP1ML_configure())		//This sets up a factory device to Network configuration, writing setting to EEPROM
				stat->signal=0;		//Only wipe this if it worked ok
		}
		break;
	case PUNG:					//Involves entering command mode, to write the device address, this is busy waiting nastyness, so main loop
		if(stat->signal==ASSIGNED) {		//The device address assignment was received, need to enter command mode first 
			stat->signal=0;			//Invalidate this
			if(!SP1ML_assign_addr(stat->argument)) {//Assign the address to the device		
				Usart3_Send_Str((char*)"ATO\n\r");//Exit command mode
				SP1ML_tx_bytes=0;	//Reset this here as we will be in a packet aligned state
				stat->upper_level_state++;
			}
			else
				stat->upper_level_state=INIT;	//In case of failure at any point, return to the init state
		}
	case ASSIGNED:					//Once the device is assigned an address, there are periodic requests for data from the base station
		if(stat->signal==PUNG) {		//These are normally handled inside the 300hz systick ISR, typically approx 25 samples will be requested
			if(!SP1ML_assign_addr(NETWORK)) {//A flag can be set from the ISR to force the state machine back to the INIT state (used to reset devices)
				Usart3_Send_Str((char*)"ATO\n\r");//Exit command mode
				stat->upper_level_state=INIT;	//Go back to the init state
				SP1ML_withold=0;	//Ensure the software tx block is unset
				stat->signal=0;		//Invalidate this
			}
		} 
	}
}

/**
  * @brief Attempt to force a SP1ML into command mode
  * @param None
  * @reval Returns zero for success, nonzero for failure
  */
uint8_t SP1ML_command(void)
{
	uint32_t millis_startget=Millis;		//Store entry time
	uint32_t millis_waitreply;
	uint8_t datavar;				//Used to store the read characters
	const char Reply[]="OK\n\r";
	while(Millis<(millis_startget+1200)) {		//Loop in here until we break out or we get timeout
		uint8_t read_characters=0;
		uint32_t Millis_=Millis;
		while(anything_in_buff(&Usart3_tx_buff) && Millis<(Millis_+100))//Empty any data from buffer (it should empty itself)
			__WFI();
		Usart3_Send_Str((char*)"+++");		//Try to enter Command Mode
		millis_waitreply=Millis;		//Store the time at which we sent the command
		while(Millis<(millis_waitreply+370)) {	//Wait for a timeout or the correct reply
			if(anything_in_buff(&Usart3_rx_buff)) {//Get any data from buffer, this will chuck any preceeding data, but that doesn't matter as entry
				Get_From_Buffer(&datavar,&Usart3_rx_buff);//Is from the disconnected state.
				if(Reply[read_characters]==datavar) { //Take from buffer and compare
					read_characters++;
				}
				else {
					read_characters=0;
				}
				if(read_characters==(sizeof(Reply))-1) {
					return 0;	//Success return point
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
void SP1ML_generate_packet(uint8_t* data_payload, uint8_t number_bytes, uint8_t device_network_id, uint8_t* sequence_number) {
	const uint8_t header=HEAD;			//COBS uses zero as the id byte, with xor over everything with the head byte
	uint8_t tmp;
	Add_To_Buffer(&header,&Usart3_tx_buff);
	uint8_t skip=1;					//The reverse COBS applies to the two header bytes as well
	tmp=device_network_id^header;			//xor all the payload
	if(tmp!=header) {
		Add_To_Buffer(&tmp,&Usart3_tx_buff);	//Packet header is: device network id byte and sequence number (for scrolling graph smoothness)
		skip++;
	}
	else {
		tmp=skip^header;
		Add_To_Buffer(&tmp,&Usart3_tx_buff);
	}
	tmp=(*sequence_number)^header;
	if(tmp!=header) {
		Add_To_Buffer(&tmp,&Usart3_tx_buff);	//This is similar to HDLC packet format
		skip++;
	}
	else {
		tmp=skip^header;
		Add_To_Buffer(&tmp,&Usart3_tx_buff);
		skip=1;
	}
	for(uint8_t n=0; n<number_bytes; n++) {
		if(!data_payload[n]) {
			__sp1ml_send_char(skip^header);	//The send_char routine is used here as it will kick start the interrupt driven usart comms if needs be
			skip=1;
		}
		else {
			skip++;
			__sp1ml_send_char(data_payload[n]^header);
		}
	}
	__sp1ml_send_char(skip^header);			//Work backwards from the end of the packet, skipping skip bytes and replacing with HEAD until packet header
	(*sequence_number)++;				//Incriment this
}


/**
  * @brief  This function configures an SP1ML module, note that the actual device address is asigned at runtime by the base station
  * @param  None
  * @retval None
  */
uint8_t SP1ML_configure(void) {
	uint8_t success;
	//Allow any tx3 data to be sent (there shouldn't really be any if this is first time config)
	if(!(success=SP1ML_command())) {
		Usart3_Send_Str((char*)"ATS01="FREQUENCY"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS02="DATA_RATE"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS04="TX_POWER"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS05="DEVIATION"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS06="RX_FILTER"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS07="CS_MODE"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS08="RSSI_THRESH"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS14="FEC"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS15="SOURCE"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS16="DESTINATION"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS17="FILTER_MULTICAST"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS18="FILTER_BROADCAST"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS19="FILTER_CRC"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS20="FILTER_SOURCE"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"ATS21="FILTER_DESTINATION"\n\r");SP1ML_DELAY//Setting filter source & dest limits us to 1 to 1 comms with the base station
		Usart3_Send_Str((char*)"ATS24="LED"\n\r");SP1ML_DELAY//This is for led to VCC. Use 2 to set for led to GND
		Usart3_Send_Str((char*)"ATS28="PAYLOAD"\n\r");SP1ML_DELAY
		Usart3_Send_Str((char*)"AT/C\n\r");SP1ML_DELAY//this permanently writes to EEPROM, so dont need to call this function again
		Usart3_Send_Str((char*)"ATO\n\r");SP1ML_DELAY
	}
	return success;
}

/**
  * @brief  This function assigns an address to the SP1ML module (the address is temporary and only used for the current session)
  * @param  uint8_t address argument
  * @retval uint8_t success argument 
  */
uint8_t SP1ML_assign_addr(uint8_t addr) {
	uint8_t success;
	if(!(success=SP1ML_command())) {
		uint8_t hexstr[3]={};//Add null terminator
		byte_to_hex(hexstr,addr);
		Usart3_Send_Str((char*)"ATS15=");
		Usart3_Send_Str((char*)hexstr);
		Usart3_Send_Str((char*)"\n\r");
	}
	return success;
}

/**
  * @brief  This function converts a byte into a two digit hex string
  * @param  Pointer to hex array output (no null terminator), and uint8_t argument
  * @retval None
  */
void byte_to_hex(uint8_t hex[2], uint8_t arg) {
	hex[1]=arg&0x0F;
	hex[1]+=0x30;
	if(hex[1]>0x39)
		hex[1]+=0x27;
		hex[1]=arg&0x0F;
	hex[0]=(arg&0xF0)>>4;
	hex[0]+=0x30;
	if(hex[0]>0x39)
		hex[0]+=0x27;
}

/**
  * @brief  This function converts an ascii hex byte (low case, range 0-f) to an integer in the range 0-15
  * @param  Ascii character
  * @retval Hex integer
  */
uint8_t hex_to_byte(uint8_t hex) {
	hex-=0x30;
	if(hex>9)
		hex-=0x27;
	if(hex>15)	//Very crude error handling - invalid characters converted to zero
		hex=0;
	return hex;
}

