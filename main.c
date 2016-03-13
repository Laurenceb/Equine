#include <string.h>
#include <math.h>
#include "stm32f10x.h"
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "usart.h"
#include "interrupts.h"
#include "watchdog.h"
#include "Util/rprintf.h"
#include "Util/delay.h"
#include "usb_lib.h"
#include "Util/USB/hw_config.h"
#include "Util/USB/usb_pwr.h"
#include "Util/fat_fs/inc/diskio.h"
#include "Util/fat_fs/inc/ff.h"
#include "Util/fat_fs/inc/integer.h"
#include "Util/fat_fs/inc/rtc.h"


//newlib reent context
struct _reent my_reent;



//Global variables - other files (e.g. hardware interface/drivers) may have their own globals
extern uint16_t MAL_Init (uint8_t lun);			//For the USB filesystem driver
volatile uint8_t file_opened=0;				//So we know to close any opened files before turning off
uint8_t print_string[25];				//For printf data - only used for some file header stuff
UINT a;							//File bytes counter
volatile uint32_t Millis;				//System uptime (rollover after 50 days)
volatile uint32_t Last_WDT;				//For detection of a jammed state
volatile uint8_t System_state_Global;			//Stores the system state, controlled by the button, most significant bit is a flag
volatile uint8_t Shutdown_System;			//Used to order a system shutdown to sleep mode
volatile float Battery_Voltage;

volatile uint16_t* ADCoutbuff;


char SerialNumber[20];
uint32_t flashCode=0;
uint8_t flashCodeEnabled=0;
int8_t rtc_correction=31;			//Too large to be used	
//FatFs filesystem globals go here
FRESULT f_err_code;
static FATFS FATFS_Obj;
FIL FATFS_wavfile,FATFS_wavfile_gps,FATFS_wavfile_imu;
FILINFO FATFS_info;
//volatile int bar[3] __attribute__ ((section (".noinit"))) ;//= 0xaa

int main(void)
{
	uint8_t system_state=0;				//used to track button press functionality
	float sensor_data,battery_voltage;
	uint8_t deadly_flashes=0;
	uint8_t Sensors=0;
	wave_stuffer this_stuffer[3]={};		//used for file stuffing
	ADS_config_type ADS_conf;			//ECG configuration
	RTC_t RTC_time;
	_REENT_INIT_PTR(&my_reent);
	_impure_ptr = &my_reent;
	SystemInit();					//Sets up the clk
	setup_gpio();					//Initialised pins, and detects boot source
	DBGMCU_Config(DBGMCU_IWDG_STOP, ENABLE);	//Watchdog stopped during JTAG halt
	if(RCC->CSR&RCC_CSR_IWDGRSTF) {			//Watchdog reset, turn off
		RCC->CSR|=RCC_CSR_RMVF;			//Reset the reset flags
		shutdown();
	}
	SysTick_Configuration();			//Start up system timer at 100Hz for uSD card functionality
	Watchdog_Config(WATCHDOG_TIMEOUT);		//Set the watchdog
	Watchdog_Reset();				//Reset watchdog as soon as possible incase it is still running at power on
	rtc_init();					//Real time clock initialise - (keeps time unchanged if set)
	Usarts_Init();
	ISR_Config();
	rprintfInit(__usart_send_char);			//Printf over the bluetooth
	if(USB_SOURCE==bootsource) {
		Set_System();				//This actually just inits the storage layer
		Set_USBClock();
		USB_Interrupts_Config();
		USB_Init();
		uint32_t nojack=0x000FFFFF;		//Countdown timer - a few hundered ms of 0v on jack detect forces a shutdown
		while (bDeviceState != CONFIGURED) {	//Wait for USB config - timeout causes shutdown
			if((Millis>10000 && bDeviceState == UNCONNECTED)|| !nojack) { //No USB cable - shutdown (Charger pin will be set to open drain, cant be disabled without usb)
				shutdown();
			}
			if(GET_VBUS_STATE) {		//Jack detect resets the countdown
				nojack=0x0FFFFF;
			}
			if (bDeviceState == SUSPENDED) {
				CHRG_ON;
			}
			nojack--;
			Watchdog_Reset();		//Reset watchdog here, if we are stalled here the Millis timeout should catch us
		}
		USB_Configured_LED();
		EXTI_ONOFF_EN();			//Enable the off interrupt - allow some time for debouncing
		ADC_Configuration();			//Enable ADC for battery voltage measurment
		uint32_t millis_local=0;
		uint16_t flash_cycle_time=1000;
		while(1) {				//If running off USB (mounted as mass storage), stay in this loop - dont turn on anything
			if((Millis-millis_local)%flash_cycle_time<100) {//Only update during the initial LED flash - 100ms leeway to allow other tasks
				if(Battery_Voltage>3.97) {//20% bands derived from test of Farnell 1.3Ah lipo cell
					flash_cycle_time=2000;
				}
				else if(Battery_Voltage>3.85) {
					flash_cycle_time=1750;
				}
				else if(Battery_Voltage>3.77) {
					flash_cycle_time=1500;
				}
				else if(Battery_Voltage>3.72) {
					flash_cycle_time=1250;
				}
				else {
					flash_cycle_time=1000;
				}
				millis_local=Millis-100;//Store this to avoid running again for an entire cycle
			}
			uint16_t time_index=(Millis-millis_local)%flash_cycle_time;//Index into our flash sequence in milliseconds
			if(time_index<500) {		//1Hz on/off flashing at zero charge, with extra trailing flashes as battery charges
				switch_leds_on();    	//Flash the LED(s)
			}
			else if((flash_cycle_time-time_index)>=500 && ((time_index-500)%250)>125) { //The leds are off for the last 500ms of the period
				switch_leds_on();
			}
			else {
				switch_leds_off();
			}
			if(!(Millis%1000)) {
#if BOARD<3
				if (bDeviceState == SUSPENDED) {
					CHRG_ON;	//On early board versions use charge pin
					Delay(100);
				}
#endif
				if(!GET_VBUS_STATE) {
					CHRG_OFF;
					red_flash();
					shutdown();
				}
				if (bDeviceState == SUSPENDED)
					CHRG_OFF;
			}
			Watchdog_Reset();
			__WFI();			//Sleep mode
		}
	}
	if(!GET_PWR_STATE &&  !(CoreDebug->DHCSR&0x00000001)) {//Check here to make sure the power button is still pressed, if not, sleep if no debug
		shutdown();    				//This means a glitch on the supply line, or a power glitch results in sleep
	}
	//Check to see if battery has enough charge to start
	EXTI_ONOFF_EN();				//Enable the off interrupt - allow some time for debouncing
	ADC_Configuration();				//At present this is purely here to detect low battery
	do {
		battery_voltage=Battery_Voltage;	//Have to flush adc for some reason
		Delay(25000);
	} while(fabs(Battery_Voltage-battery_voltage)>0.01 || !battery_voltage);
	I2C_Config();					//Setup the I2C bus
	Sensors=detect_sensors(0);			//Search for connected sensors, probes the I2C for the IMU sensor and sets us and tests GPS and ADS1298
	if(battery_voltage<BATTERY_STARTUP_LIMIT)	//detect_sensors sets up and self tests all the sensors, should have all 3 sensors present
		deadly_flashes=1;
	else if(!(Sensors&(1<<LSM9DS1)))
		deadly_flashes=2;
	else if(!(Sensors&(1<<ADS1298)))
		deadly_flashes=3;
	else if(!(Sensors&(1<<UBLOXGPS)))
		deadly_flashes=4;
	// system has passed battery level check and so file can be opened
	uint8_t br;
	if((f_err_code = f_mount(0, &FATFS_Obj))) {
		Usart_Send_Str((char*)"FatFs mount error\r\n");    //This should only error if internal error
	}
	else if(!deadly_flashes){		//FATFS and the I2C initialised ok, try init the card)
		if(!f_open(&FATFS_wavfile,"time.txt",FA_OPEN_EXISTING | FA_READ | FA_WRITE)) {//Try and open a time file to get the system time
			if(!f_stat((const TCHAR *)"time.txt",&FATFS_info)) {//Get file info
				if(FATFS_info.fsize<5) {	//Empty file
					RTC_time.year=(FATFS_info.fdate>>9)+1980;//populate the time struct (FAT start==1980, RTC.year==0)
					RTC_time.month=(FATFS_info.fdate>>5)&0x000F;
					RTC_time.mday=FATFS_info.fdate&0x001F;
					RTC_time.hour=(FATFS_info.ftime>>11)&0x001F;
					RTC_time.min=(FATFS_info.ftime>>5)&0x003F;
					RTC_time.sec=(FATFS_info.ftime<<1)&0x003E;
					rtc_settime(&RTC_time);
					rprintfInit(__fat_print_char);//printf to the open file
					printf("RTC set to %d/%d/%d %d:%d:%d\n",RTC_time.mday,RTC_time.month,RTC_time.year,\
					       RTC_time.hour,RTC_time.min,RTC_time.sec);
				}
			}
			f_close(&FATFS_wavfile);	//Close the time.txt file
		}
		// Load settings if file exists
		if(!f_open(&FATFS_wavfile,"settings.dat",FA_OPEN_EXISTING | FA_READ)) {
			if(!read_config_file(&FATFS_wavfile, &ADS_conf, &rtc_correction)) {
				if((rtc_correction<30) && (rtc_correction>-92) && rtc_correction ) {
					PWR_BackupAccessCmd(ENABLE);/* Allow access to BKP Domain */
					uint16_t tweaked_prescale = (0x0001<<15)-2;/* Try to run the RTC slightly too fast so it can be corrected either way */
					RTC_WaitForSynchro();	/* Wait for RTC registers synchronization */
					if( RTC->PRLL != tweaked_prescale ) {/*Note that there is a 0.5ppm offset here (correction 0==0.5ppm slow)*/
						RTC_SetPrescaler(tweaked_prescale); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767-2+1) */
						RTC_WaitForLastTask();
					}
					BKP_SetRTCCalibrationValue((uint8_t) ((int16_t)31-(21*(int16_t)rtc_correction)/(int16_t)20) );
                                	BKP_RTCOutputConfig(BKP_RTCOutputSource_None);/* Ensure any output is disabled here */
					/* Lock access to BKP Domain */
					PWR_BackupAccessCmd(DISABLE);
				}
				else if(((uint8_t)rtc_correction==0x91) ) {/* 0x91 magic flag sets the RTC clock output on */
					PWR_BackupAccessCmd(ENABLE);/* Allow access to BKP Domain */
					BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);/* Output a 512Hz reference clock on the TAMPER pin*/
					PWR_BackupAccessCmd(DISABLE);
				}
			}
			else {				//Default is everything enabled
				ADS_conf.enable_mask=0xFF;
				ADS_conf.channel_seven_neg=0;//Normally connected to WCT rather than used
				ADS_conf.gain=4;
			}
			f_close(&FATFS_wavfile);	//Close the settings.dat file
		}
		else
			br=1;
		// Get Bluetooth name from BT Device
		Watchdog_Reset();			//All this config can take a while, periodically reset
		RN42_get_name(SerialNumber, br);	//Allow the module to be configured if there is no settings file present on the drive
		rtc_gettime(&RTC_time);			//Get the RTC time and put a timestamp on the start of the file
		rprintfInit(__str_print_char);		//Print to the string
		//timestamp name
		printf("%d-%02d-%02dT%02d-%02d-%02d",RTC_time.year,RTC_time.month,RTC_time.mday,RTC_time.hour,RTC_time.min,RTC_time.sec);
		rprintfInit(__usart_send_char);	//Printf over the bluetooth
		f_err_code = f_mkdir(print_string); //Try to make a directory where the logfiles will live
		uint8_t repetition_counter=0;
		if(f_err_code) {
			printf("FatFs drive error %d\r\n",f_err_code);
			if(f_err_code==FR_DISK_ERR || f_err_code==FR_NOT_READY)
				Usart_Send_Str((char*)"No uSD card inserted?\r\n");
			repetition_counter=1;
		}
		else
			f_err_code=f_chdir(print_string);//enter our new directory
		if(f_err_code) {
			if(!repetition_counter)
				printf("FatFs drive error entering directory %d\r\n",f_err_code);
			repetition_counter=1;
		}
		else {
			strcat(print_string,".wav");
			f_err_code=f_open(&FATFS_wavfile,print_string,FA_CREATE_ALWAYS | FA_WRITE);//Try to open the main 250sps wav logfile
		}
		if(f_err_code) {
			if(!repetition_counter)
				printf("FatFs drive error creating main wav logfile %d\r\n",f_err_code);
			repetition_counter=1;
		}
		else {	
			print_string[strlen(print_string)-4]=0x00;	//Wipe the .wav off the string
			strcat(print_string,"_gps.wav");
			f_err_code=f_open(&FATFS_wavfile_gps,print_string,FA_CREATE_ALWAYS | FA_WRITE);//Try to open the gps wav logfile
		}
		if(f_err_code) {
			if(!repetition_counter)
				printf("FatFs drive error creating gps wav file %d\r\n",f_err_code);
			repetition_counter=1;
		}
		else {	
			print_string[strlen(print_string)-8]=0x00;	//Wipe the .wav off the string
			strcat(print_string,"_imu.wav");
			f_err_code=f_open(&FATFS_wavfile_imu,print_string,FA_CREATE_ALWAYS | FA_WRITE);//Try to open the gps wav logfile
		}
		if(f_err_code) {
			if(!repetition_counter)
				printf("FatFs drive error creating imu wav file %d\r\n",f_err_code);
			repetition_counter=1;
		}
		else {				//We have a mounted card
			print_string[0]=0x00;	//Wipe the string
			f_err_code=f_lseek(&FATFS_wavfile, PRE_SIZE);// Pre-allocate clusters
			if (f_err_code || f_tell(&FATFS_wavfile) != PRE_SIZE)// Check if the file size has been increased correctly
				Usart_Send_Str((char*)"Pre-Allocation error\r\n");
			else {
				if((f_err_code=f_lseek(&FATFS_wavfile, 0)))//Seek back to start of file to start writing
					Usart_Send_Str((char*)"Seek error\r\n");
				else
					rprintfInit(__str_print_char);//Printf to the logfile
			}
			if(f_err_code)
				f_close(&FATFS_wavfile);//Close the already opened file on error
			else
				file_opened=0x01;//So we know to close the file properly on shutdown - bit mask for the files
			if(!f_err_code) {
				f_err_code=f_lseek(&FATFS_wavfile_gps, PRE_SIZE);// Pre-allocate clusters
				if (f_err_code || f_tell(&FATFS_wavfile_gps) != PRE_SIZE)// Check if the file size has been increased correctly
					Usart_Send_Str((char*)"Pre-Allocation error\r\n");
				else {
					if((f_err_code=f_lseek(&FATFS_wavfile_gps, 0)))//Seek back to start of file to start writing
						Usart_Send_Str((char*)"Seek error\r\n");
				}
			}
			if(f_err_code)
				f_close(&FATFS_wavfile_gps);//Close the already opened file on error
			else
				file_opened|=0x02;//So we know to close the file properly on shutdown - bit mask for the files
			if(!f_err_code) {
				f_err_code=f_lseek(&FATFS_wavfile_imu, PRE_SIZE);// Pre-allocate clusters
				if (f_err_code || f_tell(&FATFS_wavfile_imu) != PRE_SIZE)// Check if the file size has been increased correctly
					Usart_Send_Str((char*)"Pre-Allocation error\r\n");
				else {
					if((f_err_code=f_lseek(&FATFS_wavfile_imu, 0)))//Seek back to start of file to start writing
						Usart_Send_Str((char*)"Seek error\r\n");
				}
			}
			if(f_err_code)
				f_close(&FATFS_wavfile_imu);//Close the already opened file on error
			else
				file_opened|=0x04;//So we know to close the file properly on shutdown - bit mask for the files
		}
	}	
	//We die, but flash out a number of flashes first
	if(f_err_code && !deadly_flashes)
		deadly_flashes=5;			//5 flashes means card error
	if(f_err_code || deadly_flashes) {		//There was an init error
		for(;deadly_flashes;deadly_flashes--) {
			RED_LED_ON;
			Delay(200000);
			RED_LED_OFF;
			Delay(200000);
			Watchdog_Reset();
		}
		RED_LED_ON;
		Delay(400000);
		shutdown();				//Abort after a (further )single red flash
	}
	Watchdog_Reset();				//Card Init can take a second or two
	flashCodeEnabled=1;                             //Enable flashcode handler
	rtc_gettime(&RTC_time);				//Get the RTC time and put a timestamp on the start of the file
	print_string[0]=0x00;				//Set string length to 0
	write_wave_header(&FATFS_wavfile, 8, 250, 24);	//250hz sample rate
	write_wave_header(&FATFS_wavfile_gps, 6, 5, 16);//5hz GPS, using converted 16 bit units, position in meters, alt in cm and velocity in cm/s
	write_wave_header(&FATFS_wavfile_imu, 10, 250, 16);//5hz GPS
	Millis=0;					//Reset system uptime, we have 50 days before overflow
	Gps_DMA_Process();				//Processes all GPS data
	Gps.packetflag=0;				//Reset this to zero
	ads1298_start();				//Fire up the ADS, this will also start up I2C reads
	uint32_t samples[2]={};
	int8_t pad_drop=0;
	int32_t raw_data_ecg[8]={};			//High rate data
	int16_t raw_data_imu[10]={};
	int16_t raw_data_gps[6]={};			//Low rate GPS data (converted into units suitable for wav)
	while(1) {
		while(!anything_in_buff(&IMU_buff[9])) {
			__WFI();			//Await the last part of buffer filling
		}
		samples[1]++;				//High rate sample arrived
		Watchdog_Reset();			//Reset the watchdog each main loop iteration
		Gps_DMA_Process();			//Processes all GPS data
		if(Gps.packetflag==REQUIRED_DATA) {	//GPS packet arrived
			uint8_t gps_code=process_gps_data(raw_data_gps,&Gps,system_state);//GPS formatter puts GPS data into it's wav file buffer as int16_t data
			Gps.packetflag=0;
			samples[0]++;			//Low rate sample arrived
			pad_drop=aligndata(samples, 250/5);//250hz versus 5hz
			write_wave_samples(&FATFS_wavfile_gps, 6, 16, &(this_stuffer[2]), raw_data_gps, 2);//Already converted GPS into correct format
			if(file_opened&0x02)
				f_err_code|=file_preallocation_control(&FATFS_wavfile_gps);
			if(gps_code) {
				if(gps_code>=2)		//No signal
					flashCode=FLASH_NO_GPS;
				else			//Only a 2D fix
					flashCode=FLASH_POOR_GPS;
			}
			else
				flashCode=0;
		}
		if(Battery_Voltage<BATTERY_STARTUP_LIMIT)//Low battery warning flash, this overrules other flash codes
			flashCode=FLASH_BATTERY;
		if(pad_drop<=0) {			//We retrieve the data from the high rate buffers (8 ADS buffers followed by 10 LSM)
			if(pad_drop<0) {		//We run an extra read
				for(uint8_t n=0; n<8; n++)
					Get_From_Buffer(&(raw_data_ecg[n]),&(ECG_buffers[n]));
				for(uint8_t n=0; n<10; n++)
					Get_From_Buffer(&(raw_data_imu[n]),&(IMU_buff[n]));
			}
			for(uint8_t n=0; n<8; n++) {	//Samples that will be used
				Get_From_Buffer(&(raw_data_ecg[n]),&(ECG_buffers[n]));
				if(abs(raw_data_ecg[n])==(1<<24))//There is an error code
					raw_data_ecg[n]=(raw_data_ecg[n]>0)?(1<<23)-1:-((1<<23)-1);//Compress the error codes and saturation handling into 24 bits
				else if(abs(raw_data_ecg[n])>=((1<<23)-1))
					raw_data_ecg[n]=(raw_data_ecg[n]>0)?raw_data_ecg[n]-1:raw_data_ecg[n]+1;
			}
			for(uint8_t n=0; n<10; n++)
				Get_From_Buffer(&(raw_data_imu[n]),&(IMU_buff[n]));
		}					//Otherwise we write the old data
		write_wave_samples(&FATFS_wavfile, 8, 24, &this_stuffer[0], raw_data_ecg, 4);//Go through writing the samples to wav file
		write_wave_samples(&FATFS_wavfile_imu, 10, 16, &this_stuffer[1], raw_data_imu, 2);
		//Manage pre-allocation control of the wav files
		if(file_opened&0x01)
			f_err_code|=file_preallocation_control(&FATFS_wavfile);
		if(file_opened&0x04)
			f_err_code|=file_preallocation_control(&FATFS_wavfile_imu);
		//I2C bus manager
		check_lsm9ds1_functionality();		//It might be worth adding support for the return code and some error handling here in future
		//SP1ML device discovery and network address/name management
		SP1ML_manager(SerialNumber, &SP1ML_tx_rx_state);
		//Other sensors etc can go here
		//Button multipress status
		system_state=0;				//Reset this - it is used earlier
		if(System_state_Global&0x80) {		//A "control" button press
			system_state|=System_state_Global&~0x80;//Copy to local variable
			System_state_Global&=~0x80;	//Wipe the flag bit to show this has been processed
		}
		//Deal with file size - check for card errors due to lack of space
		if(f_err_code)
			Shutdown_System=NO_CARD_SPACE;	//Shutdown to avoid running without recording any data
		if(Millis%1000>500) {			//1Hz on/off flashing
			switch_leds_on();		//Flash the LED(s)
		}
		else {
			switch_leds_off();
		}
		if(Shutdown_System) {			//A system shutdown has been requested
			if(file_opened) {
				shutdown_filesystem(Shutdown_System, file_opened);
			}
			if(Shutdown_System==USB_INSERTED) {
				NVIC_SystemReset();    //Software reset of the system - USB inserted whilst running
			}
			else {
				if( (Shutdown_System==LOW_BATTERY) || (Shutdown_System==NO_CARD_SPACE) ) {
					red_flash();    //Used to indicate an error condition before turnoff, other conditions flash an error code
				}
				shutdown();		//Puts us into sleep mode
			}
		}
	}
}

/**
  * @brief  Detects and sets up all sensors
  * @param  Pointer to the ADS1298 config structure
  * @retval Flag byte containing status flags for the hardware
  */
uint8_t detect_sensors(ADS_config_type* config) {//Detects and sets up all three sensors (ADS1298, LSM9DS1, and Ublox)
	uint8_t res=ads1298_setup(config, 0);	//First setup the ADS1298, but don't start it immediatly (I2C reads and data passing to BT & SP1ML dovetail off this)
	if(res==ADS1298_ID || res==ADS1298R_ID ||res==ADS1296_ID ||res==ADS1296R_ID ||res==ADS1294_ID ||res==ADS1294R_ID )
		res|=(1<<ADS1298);		//Device is present
	SCHEDULE_CONFIG;			//Run the I2C devices config
	uint32_t millis=Millis;
	while(Jobs) {//while((I2C1->CR2)&(I2C_IT_EVT));//Wait for the i2c driver to complete
		if(Millis>(millis+20))
			return 0;
	}
	if((Completed_Jobs&CONFIG_SENSORS)==CONFIG_SENSORS)//Second two config jobs ran ok
		res|=(1<<LSM9DS1);
	if(!Config_Gps())			//Sets up a Ublox module on USART2
		res|=(1<<UBLOXGPS);
	return res;
}

/**
  * @brief  Ensures that we have significant preallocation on a file, useful to avoid significant delays on write
  * @param  Pointer to the file
  * @retval None
  */
FRESULT file_preallocation_control(FIL* file) {
	FRESULT f_err=0;
	if(f_size(file)-f_tell(file)<(PRE_SIZE/2)) {	//More than half way through the pre-allocated area
		f_err=f_sync(file);			//Running a sync here minimizes risk of erranous data loss
		DWORD size=f_tell(file);
		f_err|=f_lseek(file, f_size(file)+PRE_SIZE);//Preallocate another PRE_SIZE
		f_err|=f_lseek(file, size);		//Seek back to where we were before
	}
	return f_err;
}

/**
  * @brief  Processes a gps structure to populate gps logfile buffer
  * @param  Pointers to int16_t buffer and the gps structure, and system state variable for adding to the logfile as the second to bottom nibble of status
  * @retval uint8_t Code giving GPS status
  */
uint8_t process_gps_data(int16_t data_gps[6], Ubx_Gps_Type* Gps_, uint8_t system_state_) {//Returns a GPS status code, 0==ok, 1==2d, >=2 no fix
	static uint8_t gps_state=2;			//Used for finding first lock and storing position
	static uint8_t rtc_set_from_gps;
	static float longitude_factor_from_lat=0.01;
	const float latitude_factor=0.01112;		//Convert from the degrees*10^7 units to meter units (at sea level)
	static int32_t lat_,long_;			//The current fix
	if(Gps_->status>=(UBLOX_3D-1)) {		//A 2 or 3D fix. Each time a fix is obtained, send absolute lat and long as two 16 bit variables 
		if(gps_state==2) {			//We obtained a lock. Check to see if the RTC has ever been set since boot
			if(!rtc_set_from_gps) {
				rtc_set_from_gps=1;	//RTC can only ever be set once, until the whole device reboots
				Set_RTC_From_GPS(Gps_->week,Gps_->time);
			}
			gps_state=1;
			longitude_factor_from_lat=latitude_factor*sinf((float)Gps_->latitude*1.745e-9);//Calculate the mapping factor to convert to meters
			lat_=Gps_->latitude;
			long_=Gps_->longitude;
			data_gps[0]=(int16_t)((uint32_t)lat_&0x0000FFFF);//Lower 16 bits are sent first
			data_gps[1]=(int16_t)((uint32_t)long_&0x0000FFFF);
		}
		else if(gps_state==1) {
			gps_state=0;			//zero implies gps lock and normal data passing
			data_gps[0]=(int16_t)((uint32_t)(lat_>>16)&0x0000FFFF);
			data_gps[1]=(int16_t)((uint32_t)(long_>>16)&0x0000FFFF);
		}
		else {					//convert and send data as normal
			data_gps[0]=(int16_t)((float)Gps_->latitude*latitude_factor);//These are in meter units
			data_gps[1]=(int16_t)((float)Gps_->longitude*longitude_factor_from_lat);
		}
		data_gps[3]=(int16_t)(Gps_->mslaltitude/10);//Altitude is in cm
		data_gps[4]=Gps_->vnorth;		//This happens whenever there is a fix
		data_gps[5]=Gps_->veast;
	}
	else {
		if(!gps_state || (gps_state==1))
			gps_state=0xff;
		gps_state--;
		if(gps_state<2)
			gps_state=2;			//Countdown from 255 to 2 - if we don't have a fix for a while, reinitialise
	}//Stuff in the number of sats (upper byte), the gps status code (lowest nibble), and the system status (second to lowest nibble). This always happens
	data_gps[6]=(int16_t)(((uint16_t)Gps_->nosats)<<8|(uint16_t)((Gps_->status)&0x0f))|((system_state_&0x0f)<<4);
	return (3-Gps_->status);
}

/**
  * @brief  Sets RTC using GPS time
  * @param  GPS Week number and millisecond variables from GPS struct, sets RTC to UTC
  * @retval None
  */
void Set_RTC_From_GPS(uint32_t week, uint32_t milli) {//Note HW time is in seconds of millenium UTC, but existing FatFS converts to DTS
	my_RTC_SetCounter(((week*604800)+(milli/1000))-630720013);//GPS seconds in a week, gps time at millenium UTC
}

/**
  * @brief  Writes a char to logfile
  * @param  Character to write
  * @retval None
  */
void __fat_print_char(char c)
{
	f_write(&FATFS_wavfile,&c,(UINT)1,&a);
}

/**
  * @brief  Writes a char to string - use for better logfile performance
  * @param  Character to write
  * @retval None
  */
void __str_print_char(char c)
{
	uint8_t indx=strlen((const char *)print_string)%255;//Make sure we cant overwrite ram
	print_string[indx]=c;				//Append string
	print_string[indx+1]=0x00;			//Null terminate
	__usart_send_char(c);				//Send to the bluetooth as well
}

