#include "ads1298.h"
#include "lsm9ds1.h"

//Global used for data access, note that this requires a filtering routine
volatile int16_t Filtered_ECG[8];
volatile int32_t Raw_ECG[8];

//Globals used here
static uint16_t dummywrite;
static volatile uint8_t raw_data[29];
static volatile uint8_t quality_mask,read_transaction;
static volatile uint8_t rld_quality;	// RLD self test status
static volatile uint8_t channel_wct_conf;
static volatile uint8_t old_quality_mask,wct[2],old_wct[2];
static volatile uint8_t lead_off_mask;	//Mask register setting used for enable/disable of the lead-off excitation (this uses the 10M ohm resistors)
static uint8_t Gain,Enable,Actual_gain,Cap;//Gain setting used for the PGA, and mask of used channels, global copies
static uint16_t ads1298_transaction_queue;//This is used for managing runtime reconfiguration commands, they are prioritised using the queue and run at sample rate
static filter_state_type_c ECG_filter_states[8];//Used for bandpass and notch filtering of the telemetry data
static Int_complex_type qualityfilter_[8];//Used to low pass filter the AC lead-off detect to avoid short upsets
static uint32_t qualityfilter[8];	//Used as a further step of low pass filtering on the I^2+Q^2 output

//Shared Globals
buff_type ECG_buffers[8];		//Only 8 buffers, the lead-off is calculated using demodulation
volatile uint8_t ADS1298_Error_Status;	//Used to signal a small number of major errors in electrode configuration to the mail thread code

/**
  * @brief  This function handles ecg config
  * @param  Pointer to the config data structure (a config type, containing PGA gain settings and stuff), startnow (if this is true the ADS1298 is fired up now)
  * @retval Part number - used for self test
  */
uint8_t ads1298_setup(ADS_config_type* config, uint8_t startnow) {
	uint8_t part=0;
	GPIO_InitTypeDef    GPIO_InitStructure;//Note that the EXTI GPIO config is inside the main interrupt configuration
	USART_InitTypeDef   USART_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	SPI_StructInit(&SPI_InitStructure);
	GPIO_StructInit(&GPIO_InitStructure);

	uint16_t dummyread;
    
	// Enable clock to GPIO and SPI peripherals - on different APBs
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 , ENABLE);

	// Configure NSEL pin
	GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_SET);//Set it high, this deselects the device

	// Configure MOSI,SCLK pins (SPI2)
	GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_13|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// MISO pin (SPI2)
	GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//SPI_NSS_Hard;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // (73.728MHz/2)/16=2.3040MHz, assumes 36.864Mhz PCLK2, i.e. 73.728mhz/2
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	//SPI2->CR2 |= 0x04 ;	//NSS output enable
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_CalculateCRC(SPI2, DISABLE);
	SPI_Cmd(SPI2, ENABLE);

	/* drain SPI */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	dummyread = SPI_I2S_ReceiveData(SPI2);

	/* enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* enable the clock to the ADS1298, using PWM output from the STM32. A startnow code of 1 or 2 causes PWM config */
	if(startnow)
		setup_pwm();

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Lower pre-emption priority. Only use one of the two DMA channels as an interrupt
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;//The DMA complete/half complete triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;	//4th subpriority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//Generate the flipped test signals
	uint8_t flipmask=0,flip=0;
	for(uint8_t n=0; n<8; n++) {
		if(config->enable_mask&(1<<n))
			flip=!flip;
		if(flip)
			flipmask|=(1<<n);
	}

	//Note that config is setup to use the AC lead off detect with 10M resistors (AC Current not supported) and mirroring on every other channel
	uint8_t header[17]={0x46,0x00,0xCC,0x5D,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,config->enable_mask,0x00,config->enable_mask,0x00,flipmask};//Note that some of these settings are reset using the config struct
	Enable=config->enable_mask;// Copy into the global
	for(uint8_t n=0; n<8; n++) {
		Init_Buffer(&(ECG_buffers[n]), ADS1298_BUFFER, 4);//Initialise the data buffers (there are 8, lead-off is calculated later)
		header[n+4]|=((~(config->enable_mask)&(0x01<<n))<<(6-n));//Enable the channel if the enable mask bit is set, bit 6 is a powerdown bit
		uint8_t gain=config->gain;
		gain=(gain==6)?0:gain;				//Possible gain arguments are 1,2,3,4,6(mapped to zero),8(to 5),12(to 6)
		gain=(gain>6)?6:gain;				//Maximum possible range of the gain
		header[n+4]|=gain<<4;				//The gain setting starts at the 4th bit
		if(n==7) {
			Gain=gain;				//Copy over into the global variable for future use
			Actual_gain=config->gain;		//The actual gain value
			Cap=config->cap;
		}
	}
	//Enable with all channels marked as bad and the channel quality filter initialised as above the limit
	quality_mask=0xff;
	for(uint8_t n=0;n<8;n++)
		qualityfilter[n]=ADS1298_LEAD_LIMIT(Cap,Actual_gain)+1;
	//Enable the WCT amplifiers, and connects them to positive inputs 1,2,3 (or the appropriate ones)
	ads1298_wct_config(wct, Enable&0x0F);
	wct[0]|=config->channel_seven_neg?0x00:0x20;		//Connects the channel 7 negative input to (WCTB+WCTC)/2 (note inverted level, normally N7=WCT)
	channel_wct_conf=wct[0]&0xF0;				//Global used for reference if the WCT config is changed
	memcpy(old_wct,wct,2);
	/* Send the SDATAC command, it is unclear if this is essential on POR */
	ads1298_busy_wait_command(ADS1298_SDATAC);
	/* Reset the ADS1298 */
	ads1298_busy_wait_command(ADS1298_RESET);
	Delay(10);
	/* It is essential to set the device to command mode following reset */
	ads1298_busy_wait_command(ADS1298_SDATAC);
	/* Configure the ADS1298 */
	uint8_t gpio_init=0,config3=0xCC;			//Temp config values
	ads1298_busy_wait_write(1, 0x03, &config3);
	uint32_t m=Millis+180;
	while(m>Millis)
		__WFI();					//Wait for the reference startup time (+30ms)
	ads1298_busy_wait_write(17, 0x01, header);		//Configure the device, this uses the config settings
	ads1298_busy_wait_write(2, 0x18, wct);			//Turns on WCT and configures it to use the first three channels
	ads1298_busy_wait_write(1, 0x14, &gpio_init);		//Turn on GPIO (revision 1 PCB this has to be done as floating, future versions poss have LEDs)
	ads1298_busy_wait_read(1, 0x00, &part);			//The first register is the ID register
	if((part&0x0F)==1)					//If we have a reduced functionality ADS version, reduce enabled channels
		Enable&=~0xC0;
	else if((part&0x0F)==0)
		Enable&=~0xF0;
	old_quality_mask=~Enable;				//Globals initialised for reference, assume that all enabled channels are active
	if(startnow==2)						//An argument of two to startnow sets up the PWM and sends the start command, starting transactions
		ads1298_busy_wait_command(ADS1298_START);	//Might be worth calling this later once we have some GPS and other config is complete (so we are ready to run)?
	//ads1298_busy_wait_command(ADS1298_RDATAC);
	return part;						/* Return the part number */
}

/**
  * @brief  This function starts the ADS1298 running conversions
  * @param  None
  * @retval None
  */
void ads1298_start(void) {
	ads1298_busy_wait_command(ADS1298_START);
	NVIC_InitTypeDef   NVIC_InitStructure;
	/* Enable and set EXTI7 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;	//The DRDY triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Lower pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;	//second to lowest group priority - data comes in at only 250hz, so plenty of time
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
}

/**
  * @brief  This is a convenience function to retrieve the current ADS1298 gain setting
  * @param  None
  * @retval Current gain as unsigned character, returns the actual gain, rather than the gain register setting (note that the two are not the same)
  */
uint8_t ads1298_gain(void) {
	if(Gain && Gain<5)
		return Gain;
	else if(!Gain)
		return 6;
	else
		return (Gain*4)-12;
}

/**
  * @brief  This function demodulates the I_sense signal at F_sample/4
  * @param  Pointer to 24 bit singed integer (32bit aligned) buffer of last 4 samples for a channel
  * @retval Unsigned 32 bit integer giving squared demodulator output
  */
Int_complex_type ads1298_electrode_quality(uint32_t buffer[4]) {
	Int_complex_type r;
	r.I=buffer[0]-buffer[1]-buffer[2]+buffer[3];
	r.I>>=11;
	r.Q=buffer[0]+buffer[1]-buffer[2]-buffer[3];
	r.Q>>=11;
	return r;
} 

/**
  * @brief  This function handles simple busy wait one way ads1298 comms
  * @param  Tx number of bytes, register number, pointer to buffer.
  * @retval None
  */
void ads1298_busy_wait_write(uint8_t tx_bytes, uint8_t register_number, uint8_t *tx_data) {
	NSEL_LOW;
	uint8_t tx_payload[2];
	tx_payload[0]=(register_number&0x1F)|0x40;
	tx_payload[1]=(tx_bytes-1)&0x1F;	//This is the 'packet' format command to the ADS
	uint8_t exchanged_bytes=tx_bytes+2;	//There are two extra bytes
	for(uint8_t n=0; n<exchanged_bytes; n++) {
		if(n<2)
			SPI_I2S_SendData(SPI2,tx_payload[n]);
		else
			SPI_I2S_SendData(SPI2,tx_data[n-2]);//Send the data after the header
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	}
	while( SPI2->SR & SPI_I2S_FLAG_BSY );	//Wait until SPI is not busy anymore
	Delay(4);				//There needs to be a delay of at least 2us
	NSEL_HIGH;
	Delay(3);
}

/**
  * @brief  This function handles simple busy wait one way ads1298 comms
  * @param  Rx number of bytes, register number, pointer to buffer.
  * @retval None
  */
void ads1298_busy_wait_read(uint8_t rx_bytes, uint8_t register_number, uint8_t *rx_data) {
	NSEL_LOW;
	uint8_t tx_payload[2];
	uint8_t dummy=SPI2->DR;			//Dummy read to ensure RX is clear
	tx_payload[0]=(register_number&0x1F)|0x20;
	tx_payload[1]=(rx_bytes-1)&0x1F;	//This is the 'packet' format command to the ADS
	uint8_t exchanged_bytes=rx_bytes+2;	//There are two extra bytes
	for(uint8_t n=0; n<exchanged_bytes; n++) {
		if(n<2)
			SPI_I2S_SendData(SPI2,tx_payload[n]);
		else
			SPI_I2S_SendData(SPI2,0x00);//Send padding zeros after the header
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
		if(n<2)
			dummy=SPI2->DR;
		else
			rx_data[n-2]=SPI2->DR;	//Read the data after it arrives
	}
	while( SPI2->SR & SPI_I2S_FLAG_BSY );	//Wait until SPI is not busy anymore
	Delay(4);				//There needs to be a delay of at least 2us
	NSEL_HIGH;
	Delay(3);
}

/**
  * @brief  This function handles simple busy wait ads1298 command
  * @param  The command byte.
  * @retval None
  */
void ads1298_busy_wait_command(uint8_t command) {
	NSEL_LOW;
	SPI_I2S_SendData(SPI2,command);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	while( SPI2->SR & SPI_I2S_FLAG_BSY );	//Wait until SPI is not busy anymore
	Delay(4);				//There needs to be a delay of at least 2us
	NSEL_HIGH;
	Delay(3);
}

/**
  * @brief  This function handles WCT config, attempting to find an optimal configuration given the circumstances
  * @param  Pointer to a buffer holding the two register values, mask bit of active channels
  * @retval None
  */
void ads1298_wct_config(uint8_t wct_regs[2], uint8_t mask) {
	uint8_t amp[3]={},failure=0;	//Holds the channel that each amplifier is assigned to, and the number of amplifier allocation failures
	for(uint8_t n=0;n<3;n++) {
		for(amp[n]=0;(!((1<<amp[n])&mask))&&(amp[n]<4);amp[n]++);
		if(amp[n]==4)
			failure++;	//Amplifier allocation failed
		else
			mask&=~(1<<amp[n]);
		amp[n]<<=1;		//Double amp setting to jump over the negative inputs
	}
	wct_regs[0]=(amp[0]&0x07)|(amp[0]&0x08?0x00:0x08);//Disable/Enable WCTA amp as approriate
	wct_regs[1]=((amp[1]&0x07)<<3)|(amp[2]&0x07)|(amp[1]&0x08?0x00:0x40)|(amp[2]&0x08?0x00:0x80);//Set the amplifiers to the appropriate channels and enable
	wct_regs[0]|=channel_wct_conf;	//Set the channel connection bits using the global variable (this will only be configurable using the config file)
	if(failure)			//Set the WCT warning and failure bits as appropriate dependent on which amplifiers are running
		ADS1298_Error_Status|=(1<<WCT_SUBOPTIMAL);
	else
		ADS1298_Error_Status&=~(1<<WCT_SUBOPTIMAL);
	if(failure>1)			//One or no working amplifiers implies failure
		ADS1298_Error_Status|=(1<<WCT_FAILURE);
	else
		ADS1298_Error_Status&=~(1<<WCT_FAILURE);
}


/**
  * @brief  This function handles the data once it has arrived and packs it into 9 buffers
  * @param  Data pointer to the raw data and to the 9 buffers
  * @retval None
  */
void ads1298_handle_data_arrived(uint8_t* raw_data_, buff_type* buffers) {
	static uint8_t rld_wct_bypass,config_4_reg=0x02,rld_sensep_reg,RLD_replaced=8,RLD_replaced_reg,RLD_replaced_reg_old=8,ADS1298_Error_Status_local,LEDs,rld_sense;
	static int32_t databuffer[8][4];	//Lead-off detect demodulation buffer
	static uint16_t wct_7N_correction,rld_replace_counter;	//This is used to digitally correct channel 7 so it is referenced to WCT rather than (WCTB+WCTC)/2
	for(uint8_t n=0;n<8;n++) {		//Loop through the 8 channels
		uint32_t dat;
		if(RLD_replaced!=n) {		//This is disabled if the RLD is remapped to the current channel TODO check enable mask here
			for(uint8_t m=0; m<3; m++)
				databuffer[n][m]=databuffer[n][m+1];//Update the buffers, copy down from the higher array index
			dat=*((uint32_t*)&(raw_data_[n*3+4]));//five byte offset due to the command byte + three status bytes + (no) periph sync offset?
			dat&=0x00ffffff;
			dat|=(dat<<24);		//Flip the endianess
			dat&=0xffffff00;
			dat|=((dat&0x00ff0000)>>16);
			dat&=0xff00ffff;
			dat|=(dat&0xff000000)>>8;
			dat&=0x00ffffff;
			if(dat&0x800000)
				dat|=0xFF000000;//Sign extend to a 32bit signed integer or int32_t
			if((n==6) && (channel_wct_conf&0x20) && (!rld_wct_bypass)) {//If the channel 7 (i.e. n==6) input is connected to WCTB/C mean
				if(!(ads1298_transaction_queue&(1<<WCT_REMAP)))//There is no WCT reconfiguration job still outstanding, update the status
					memcpy(&wct_7N_correction,wct,2);//We can now use the uint16_t, operating under the assumption it is in effect on ADS1298
				uint16_t flags=wct_7N_correction&0xC080;//Check the amplifier enable flags
				uint8_t chans[3];
				chans[0]=(wct_7N_correction&0x0007)>>1;//The lead inputs that WCT amplifiers a,b, and c are connected to
				chans[1]=(wct_7N_correction&0x3800)>>(4+8);
				chans[2]=(wct_7N_correction&0x0700)>>(1+8);
				if(flags==0xC080) //All amplifiers running
					dat+=-databuffer[n][chans[0]]/3+databuffer[n][chans[1]]/6+databuffer[n][chans[2]]/6;//-a/3+b/6+c/6
				else if(flags==0x4080)//amp c is off
					dat+=-databuffer[n][chans[0]]/12+5*databuffer[n][chans[1]]/12-databuffer[n][chans[2]]/3;
				else if(flags==0x8080)//amp b is off, all other cases do not need to have a correction applied to them
					dat+=-databuffer[n][chans[0]]/12-databuffer[n][chans[1]]/3+5*databuffer[n][chans[2]]/12;
			}
			databuffer[n][3]=(int32_t)dat;//Load the latest data into the history buffer. Don't need to correct the range here as 32 bit
			if(abs(*(int32_t*)&dat)>=((1<<23)-1))//enforce range lim on dat value (so range checking can be used in lead-off & RLD id)
				dat=((*(int32_t*)&dat)<0)?-((int32_t)(1<<23)-2):((int32_t)(1<<23)-2);
		}
		//Disabled channels, and also channels that are masked as disconnected or used as a replacement RLD are zeroed
		if(quality_mask&(1<<n)) {
			dat=(uint32_t)(1<<24)+1;//Second special value for lead-off
			if(!(Enable&(1<<n)))
				dat++;		//A third special value for disabled channels
		}	
		if(RLD_replaced==n)
			dat=(uint32_t)(1<<24);	// The 25th bit is set if the channel has been repurposed for RLD
		Add_To_Buffer(&dat,&(buffers[n]));// Add the data to the buffer
		Raw_ECG[n]=dat;			//Global allowing the raw data to be directly accessed
		if(RLD_replaced!=n) {		// If the RLD is replaced, the low pass is not updated (as we don't really know how good the electrode is)
			Int_complex_type quality=ads1298_electrode_quality(&(databuffer[n][0]));//Calculate the quality
			qualityfilter_[n].I+=(quality.I-(int32_t)qualityfilter_[n].I)>>5;// A low pass, approx 7Hz bandwidth
			qualityfilter_[n].Q+=(quality.Q-(int32_t)qualityfilter_[n].Q)>>5;
			uint32_t q=(qualityfilter_[n].I*qualityfilter_[n].I)+(qualityfilter_[n].Q*qualityfilter_[n].Q);
			qualityfilter[n]+=((int32_t)q-(int32_t)qualityfilter[n])>>5;//Secondary low pass filtering
			if(qualityfilter[n]>ADS1298_LEAD_LIMIT(Cap,Actual_gain))// There is too much AC from the lead off detect
				quality_mask|=1<<n;//RLD replacement also marks electrodes as bad
			else if(qualityfilter[n]<(ADS1298_LEAD_LIMIT(Cap,Actual_gain)-ADS1298_LEAD_HYSTERYSIS(Cap,Actual_gain)))
				quality_mask&=~(1<<n);//Clear or set the mask, set bit implies poor electrode
			quality_mask|=~Enable;	//  Disabled channels added to the mask of inactive channels
		}
	}
	if(old_quality_mask!=quality_mask) {	// Check to see if electrode config changed
		if((old_quality_mask&0x0F) != (quality_mask&0x0F)) {// Something changed with the first 4 electrodes (ones used for WCT generation)
			ads1298_wct_config(wct,((~quality_mask)&Enable)&0x0F);// Generate a new WCT configuration, note that this function uses logic high == usable
			if(*(uint16_t*)wct!=*(uint16_t*)old_wct) {// The WCT config was updated by the config lookup function
				ads1298_transaction_queue|=(1<<WCT_REMAP);// Reconfig tasks are queued
				if(!(*(uint16_t*)wct & 0xFF0F) ) {// Also need to activate RLD_TO_WCT bypass, if all 4 of first 4 electrodes disconnected
					rld_wct_bypass=1;
					config_4_reg|=0x04;
					ads1298_transaction_queue|=(1<<RLD_WCT_REMAP);
				}
				else if(rld_wct_bypass) {// Deactivate the bypass
					rld_wct_bypass=0;
					config_4_reg&=~0x04;// Enable or disable the RLD_TO_WCT as appropriate
					ads1298_transaction_queue|=(1<<RLD_WCT_REMAP);
				}
				memcpy(old_wct,wct,2);// Update the old config
			}
		}
		ads1298_transaction_queue|=(1<<RLD_REMAP);// The RLD will also need to be reconfigured in these cases, to avoid incorporating inactive chans
		rld_sensep_reg=~quality_mask;	// Only use the channels which are active
		old_quality_mask=quality_mask;	// If there was a change to the quality mask, i.e. a change in electrode state
	}
	if(RLD_replaced!=8)
		ADS1298_Error_Status|=(1<<RLD_REMAPPED);// The remap is kind of bad but not the end of the world, mark it as another failure case
	else
		ADS1298_Error_Status&=~(1<<RLD_REMAPPED);
	if(rld_quality) {			// The RLD electrode failed self test, it has to be remapped to the highest numbered working electrode 
		ADS1298_Error_Status|=(1<<RLD_FAILURE);// Status shows the failure
		rld_quality=0;			// Wipe this - only need to deal with each failure once
		if(RLD_replaced!=8) {		// Replacement in operation
			qualityfilter[RLD_replaced]=ADS1298_LEAD_LIMIT(Cap,Actual_gain)+1;// Replacement lead probably faulty if RLD failed, re-initialise the filter in failed state
			quality_mask|=(1<<RLD_replaced);// Mark channel as faulty
		}
		int8_t bestchoice=0;		// Reuse this variable to count spare channels, init as zero 
		for(uint8_t n=0; n<8; n++)
			bestchoice+=(~(quality_mask>>n)&0x01);// Count the number of usable channels (note that these exclude any current RLD remapped channel)
		if(bestchoice>=3) {		// Need at least two remaining electrodes after the RLD remap is applied
			bestchoice=7;		// We start with the highest numbered channel as an option
			for(;(bestchoice>=0)&&(quality_mask&(1<<bestchoice));bestchoice--);//bestchoice is the channel we have to use, exiting with -ive -> impossible
			if(bestchoice>=0) {	// If there is a possible remap option (if there isn't, bestchoice will be negative)
				RLD_replaced_reg_old=RLD_replaced;// An existing remap may need to be cancelled
				if(RLD_replaced_reg_old!=8)
					ads1298_transaction_queue|=(1<<RLD_REMOVE);
				RLD_replaced=bestchoice;// Apply a new replacement
				RLD_replaced_reg=RLD_replaced;// The register to be changed is the current reg
				quality_mask|=(1<<RLD_replaced);// Mark new channel choice as failed, this will allow WCT config and RDT input config to be adjusted
				ads1298_transaction_queue|=(1<<RLD_REPLACE);// Stick the replacement remap task into the queue
				lead_off_mask=Enable&~(1<<RLD_replaced);// Disable the channel
				ads1298_transaction_queue|=(1<<LEAD_OFF_REPLACE);// Lead-off sense register also needs to be updated to avoid current injection to RLD
			}
		}
	}
	else					// RLD passed self-test, remove the error flag
		ADS1298_Error_Status&=~(1<<RLD_FAILURE);
	if(RLD_replaced!=8 && !rld_quality) {	// RLD replacement is in operation on one of the channels, periodically turn it off to allow for RLD lead reconnect (if not testing)
		if(++rld_replace_counter>=ADS1298_RLD_TEST_ITERATIONS && !( ads1298_transaction_queue&((1<<RLD_ON)-1) ) ) {// Incriment the counter
			rld_replace_counter=0;	// Reset the counter to zero (^ note that this can only happen once any RLD measure operations have completed)
			quality_mask&=~(1<<RLD_replaced);// Reset the quality mask bit for this channel to mark it as usable
			qualityfilter[RLD_replaced]=0;// Reset the quality filter too
			RLD_replaced_reg=RLD_replaced;// The old register needs to be fixed
			RLD_replaced_reg_old=8;	// Ensure this is reset also
			RLD_replaced=8;		// Force the RLD back to normal
			ads1298_transaction_queue|=(1<<RLD_REPLACE);// Task into the queue. If the RLD still isnt functional it will be detected 
			lead_off_mask=Enable;	// Enable the channel's lead-off
			ads1298_transaction_queue|=(1<<LEAD_OFF_REPLACE);// The lead-off sense register also needs to be updated to allow lead-off detect
			rld_sense=ADS1298_RLD_ITERATIONS-3;// Run an detection immediatly afterwards
		}
	}
	if((++rld_sense>=ADS1298_RLD_ITERATIONS)&&(!ads1298_transaction_queue)) {// Periodically the RLD electrode configuration is tested, when all jobs completed ok
		rld_sense=0;
		ads1298_transaction_queue|=(1<<RLD_OFF)|(1<<RLD_DISCONNECT)|(1<<RLD_STAT)|(1<<RLD_RECONNECT)|(1<<RLD_ON);//The RLD status sensing jobs
	}
	#ifdef ECG_LEDS
	if(ADS1298_Error_Status!=ADS1298_Error_Status_local) {//Update the LEDs
		ADS1298_Error_Status_local=ADS1298_Error_Status;
		LEDs=ADS1298_Error_Status_local;
		if(ADS1298_Error_Status_local&(1<<WCT_FAILURE))
			LEDs&=~(1<<WCT_SUBOPTIMAL);
		LEDs<<=4;			//GPIOs to output state (bottom nibble all zeros)
		ads1298_transaction_queue|=(1<<GPIO_UPDATE);
	}
	#endif
	if(ads1298_transaction_queue) {		// There are queued tasks to complete
		uint8_t thistask=0;
		uint8_t sendbuffer[4];		//Bytes to send to the ADS1298, (for some queue tasks we send two commands at once DOESNT WORK!!)
		uint8_t register_num,write=0,bytes=1,sentbytes=3;
		for(;!(ads1298_transaction_queue&(1<<thistask));thistask++);// Find the first set bit
		switch(thistask) {
			case RLD_OFF:
				register_num=0x03;	
				sendbuffer[2]=0xDA;//Disable the RLD amplifier and enable the sense comparitor
				write=1;
			break;
			case RLD_DISCONNECT:
				register_num=0x0D;
				sendbuffer[2]=0x00;//Disable the RLD inputs to avoid parasitic currents flowing
				write=1;
			break;
			case RLD_STAT:
				register_num=0x03;
				sendbuffer[2]=0;
			break;
			case RLD_RECONNECT:
				register_num=0x0D;
				sendbuffer[2]=rld_sensep_reg;	//Enable the RLD inputs again
				write=1;
			break;
			case RLD_ON:
				register_num=0x03;
				sendbuffer[2]=0xDC;//Enable RLD amplifier and disable sense
				write=1;
			break;
			case WCT_REMAP:
				register_num=0x18;
				sendbuffer[2]=wct[0];//Load the current WCT settings
				sendbuffer[3]=wct[1];
				write=1;
				bytes=2;
				sentbytes=4;
			break;
			case RLD_WCT_REMAP:	//Connect or disconnect the WCT to RLD 
				register_num=0x17;
				sendbuffer[2]=config_4_reg;
				write=1;
			break;
			case RLD_REMAP:
				register_num=0x0D;
				sendbuffer[2]=rld_sensep_reg;//Copy the sensep register onto the ADS1298
				write=1;
			break;
			case RLD_REPLACE:	//Disable one of the channels, connecting its positive input to RLD (and negative to RLDREF)
				register_num=RLD_replaced_reg+0x05;//The actual reg is offset by 5 from the channel number
				if(RLD_replaced!=8)
					sendbuffer[2]=(0x80|(Gain<<4))|0x06;//Turn on the RLD bypass
				else
					sendbuffer[2]=(Gain<<4);//Turn off RLD bypass
				write=1;
			break;
			case RLD_REMOVE:	//Used to turn off the RLD on a replacement channel when we move to a new replacement channel
				register_num=(RLD_replaced_reg_old+0x05);
				sendbuffer[2]=(Gain<<4);//Turn off RLD bypass
				RLD_replaced_reg_old=8;
				write=1;
			break;
			case LEAD_OFF_REPLACE:	//Turn off the lead-off on any channel that is being used as a RLD replacement, as it will interfere with RLD detect
				register_num=0x0F;
				sendbuffer[2]=lead_off_mask;
				write=1;
			break;
			#ifdef ECG_LEDS
			case GPIO_UPDATE:
				register_num=0x14;
				sendbuffer[2]=LEDs;
				write=1;
			#endif
		}
		if(write)			//ADS1298 write command followed by the data
			sendbuffer[0]=(register_num&0x1F)|0x40;
		else
			sendbuffer[0]=(register_num&0x1F)|0x20;
		sendbuffer[1]=(bytes-1)&0x1F;
		/*ads1298_spi_dma_transaction(sentbytes,sendbuffer,sentbytes);*/
	}
}

/**
  * @brief  This function handles the DRDY signal by reading 27 bytes of status followed by data
  * @param  Data pointer
  * @retval None
  */
void ads1298_handle_data_read(uint8_t* r_data) {
	uint8_t command=ADS1298_RDATA;
	read_transaction=1;			/*State variable to control transaction flow*/
	ads1298_spi_dma_transaction(1, &command, 28);
}

/**
  * @brief  This function handles DMA transfer setup
  * @param  Number of bytes to send, pointer to the transmitted bytes, number of bytes to readback
  * @retval None
  */
void ads1298_spi_dma_transaction(uint8_t bytes_sent, uint8_t* tx_pointer, uint8_t rx_bytes) {
	static uint8_t sentdata[28];		/*This is usually all zero with the exception of the first byte*/
	memcpy(sentdata,tx_pointer,bytes_sent); /*Copy data into the buffer*/
	memset(&(sentdata[bytes_sent]),0,28-bytes_sent);/*Ensure that the trailing end is all zeros*/
	if(rx_bytes<bytes_sent)			/*Ensure sufficient DMA size to send the data*/
		rx_bytes=bytes_sent;
	DMA_InitTypeDef DMA_InitStructure;
	/* Shared DMA configuration values */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI2->DR));
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_Cmd(DMA1_Channel5, DISABLE);
	DMA_DeInit(DMA1_Channel4);
	DMA_DeInit(DMA1_Channel5);
	/* Disable SPI TX/RX request */
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx|SPI_I2S_DMAReq_Tx, DISABLE);
	/* DMA1 channel4 configuration SPI2 RX ---------------------------------------------*/
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)raw_data;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = rx_bytes;		/* There are 27 bytes of data as well as the RDATA command byte*/
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)sentdata;/* Tx buffer used here*/
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	/* Enable the DMA complete callback interrupt here */
	DMA_ClearFlag(DMA1_FLAG_TC4|DMA1_FLAG_HT4|DMA1_FLAG_TE4);  /* Make sure flags are clear */
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);		/* Interrupt on complete */
	NSEL_LOW;						/* Enable the ADS1298*/
	SPI_I2S_ReceiveData(SPI2);				/* Ensure that the Rx registers is empty */
	/* Enable DMA RX Channel */
	DMA_Cmd(DMA1_Channel4, ENABLE);
	/* Enable DMA TX Channel */
	DMA_Cmd(DMA1_Channel5, ENABLE);
	/* Enable SPI TX/RX request */
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx|SPI_I2S_DMAReq_Tx, ENABLE);
}

/**
  * @brief  This function handles all sensors that need to be aligned with the ECG, currently only the IMU
  * @param  None
  * @retval None
  */
void handle_aligned_sensors(void){
	handle_lsm9ds1();//This will look for data and process if it is found, adding to buffers and requesting a new set of reads
	for(uint8_t n=0; n<8; n++) {//Filter the ECG data and load into raw globals
		float out;
		if(!(Raw_ECG[n]&(1<<24)) || (Raw_ECG[n]&(1<<25))) {//Negative or 25th bit not set
			//Weak comb filter, samp[n] - samp[n+2]*7/8 preceeding iir filter, attenuate lead off by factor of 8 to fit into 16bit signed
			out=comb_filter(&((ECG_filter_states[n]).cmb), (float)(Raw_ECG[n]>>3));
			out=iir_filter_50(&((ECG_filter_states[n]).iir),out);//Trim off 3 lowest bits, assume they are noise
			if(fabs(out)>((1<<15)-2))
				Filtered_ECG[n]=(out>0)?(1<<15)-2:-((1<<15)-2);
			else
				Filtered_ECG[n]=(int16_t)out;
		}
		else 
			Filtered_ECG[n]=(Raw_ECG[n]&0x03)?-((1<<15)-(Raw_ECG[n]&0x03))-1:(1<<15)-1;//Lead-off and remap are mapped to the top and bottom of the range
	}//be sure appropriate interrupt pre-eption priority due to runtime
	SP1ML_aligned_data_ready=1;//Indicates that there is new data ready
	RN42_aligned_data_ready=1;//The same data is also available for the RN42 (code reuse)
	SP1ML_tx_rx_state.sequence_is_time=1;//This is true so that the sequence number uses number of datasamples received whilst in connected state, not number sent
	SP1ML_rx_tx_data_processor(&SP1ML_tx_rx_state,&SP1ML_generate_packet,&Usart3_rx_buff,&SP1ML_tx_sequence_number,&SP1ML_aligned_data_ready);
	SP1ML_rx_tx_data_processor(&RN42_tx_rx_state,&RN42_generate_packet,&Usart1_rx_buff,&RN42_tx_sequence_number,&RN42_aligned_data_ready);
}			//Note that the I2C_failure variable should be checked inside the main loop and a reset requested if needed

/**
  * @brief  This function handles DMA channel interrupt request.- DMA SPI2 RX complete ISR - TX is not used as dummy byte is ok
  * @param  None
  * @retval None
  */
__attribute__((externally_visible)) void DMA1_Channel4_IRQHandler(void) {
	if (DMA_GetITStatus(DMA1_IT_TC4)) {
		uint16_t t_entry=SysTick->VAL;
		int16_t t_exit=(t_entry-18);			/* Systick timer value at timeout */
		if(t_exit<0) t_exit+=SysTick->LOAD;		/* Wrap around */
		DMA_ClearFlag(DMA1_FLAG_TC4|DMA1_FLAG_HT4);  	/* make sure all flags are clear */
		DMA_ClearFlag(DMA1_FLAG_TC5|DMA1_FLAG_HT5);  	/* make sure tx flags cleared here too */
		DMA_ClearITPendingBit(DMA1_IT_GL4);
		uint8_t read_transaction_=read_transaction;	/* save this for later reference */
		if(read_transaction) {				/* this callback is the result of a read request */
			read_transaction=0;			/* read no longer in progress */
			ads1298_handle_data_arrived(raw_data,ECG_buffers);/* this function adds the data to the 9 buffers */
			handle_aligned_sensors();		/* handles any sensor actions that need to be aligned with the data */
		}
		else {						/* otherwise it is the result of a queued task */
			uint8_t thistask=0;
			for(;!(ads1298_transaction_queue&(1<<thistask));thistask++);
			ads1298_transaction_queue&=~(1<<thistask);/* wipe the lowest bit as this task is now complete */
			if(thistask==RLD_STAT)			/* we just retreived the RLD_STAT self test result */
				rld_quality=raw_data[2]&0x01;	/* the first read byte (after header) is the RLD register */
		}
		if((!read_transaction_) || (!ads1298_transaction_queue)) {/* nothing ongoing, shut down. Note otherwise we don't reset with NSEL */
			if(t_entry>t_exit)
				while(SysTick->VAL>t_exit && SysTick->VAL<t_entry);/* wait for 4 ADS1298 clocks (assumes ADS1298 is at f_cpu/36) */
			else
				while((SysTick->VAL>t_exit)==(SysTick->VAL>t_entry));/* the wrap around case */
			NSEL_HIGH; 				/* Deselect device */
		}
	}
	else
		DMA_ClearITPendingBit(DMA1_IT_GL4);		/* clear all the interrupts */
}

/**
  * @brief  This function handles External line 7 interrupt request.- DRDY ISR
  * @param  None
  * @retval None
  */
__attribute__((externally_visible)) void EXTI9_5_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line7) != RESET) {
		/* Clear the  EXTI line 7 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line7);
		ads1298_handle_data_read(raw_data);	/* the dma driven read routine gets fired off here */
	}
}

