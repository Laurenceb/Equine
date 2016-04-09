#pragma once
#include <stdint.h>
#include <math.h>
#include "stm32f10x.h"
#include "Util/fat_fs/inc/ff.h"
#include "Util/buffer.h"
#include "Util/delay.h"
#include "Util/wave.h"
#include "Sensors/ubx.h"
#include "Sensors/ads1298.h"
#include "Sensors/lsm9ds1.h"
#include "i2c_int.h"
#include "adc.h"
#include "rn42_command.h"
#include "load_setting.h"
#define BUFFER_SIZE 256
//externs for all the globals

extern volatile uint32_t Millis;

extern volatile uint32_t Last_WDT;

extern volatile uint8_t System_state_Global;
extern volatile uint8_t Sensors;
extern uint8_t print_string[25];
extern FIL FATFS_wavfile,FATFS_wavfile_gps,FATFS_wavfile_imu;

extern volatile uint8_t Shutdown_System;
extern volatile float Battery_Voltage;
extern uint32_t flashCode;
extern uint8_t flashCodeEnabled;

#define PRE_SIZE 2000000ul	/*Preallocate size - very large for wav files*/

#define SYSTEM_STATES 4		/*Number of different control states- atm they are all indicators*/

#define WATCHDOG_TIMEOUT 3000	/*4 second timeout - enough for the uSD card to block its max time and a bit*/


#define FLASH_NO_GPS 0xf002
#define FLASH_POOR_GPS 0xf00a
#define	FLASH_OK  0
#define FLASH_BATTERY 0xe738


enum{BUTTON_TURNOFF=1,USB_INSERTED,NO_SENSOR,LOW_BATTERY,OVER_HEATED_SENSOR,PRESSURE_LOSS,SIGNAL_STRENGTH,NO_CARD_SPACE,UNKNOWN};
//Battery specific config goes here
#define BATTERY_STARTUP_LIMIT 3.7 /*Around 25% capacity remaining for lithium polymer at 25C slow discharge*/
#if BOARD<3
	#define MINIMUM_VOLTAGE 3.0	/* A single lithium polymer cell*/
#else
	#define MINIMUM_VOLTAGE 3.42	/* A single lithium polymer cell through LDO regulator - no smps on later boards*/
#endif

//Sensors
enum{LSM9DS1=0,ADS1298,UBLOXGPS};

//GPS telemetry struct
typedef struct {
	int16_t heading;	//In degrees, also 
	int16_t velocity;	//Velocity in kilometers per hour
	int16_t n_pos;		//North and East positions
	int16_t e_pos;		//In meters
	uint8_t flag;		//Used for locking and syncronisation
} GPS_telem_type;

//function prototypes
void __fat_print_char(char c);
void __str_print_char(char c);
uint8_t detect_sensors(void);
uint8_t process_gps_data(int16_t data_gps[6], Ubx_Gps_Type* Gps_, uint8_t system_state_, int8_t rtc_correct);
FRESULT file_preallocation_control(FIL* file);
void Set_RTC_From_GPS(uint32_t week, uint32_t milli, int8_t rtc_correct);

//GPS telemetry data
extern volatile GPS_telem_type GPS_telem;

//fatfs globals
extern volatile uint8_t file_opened;
extern FIL FATFS_logfile;
