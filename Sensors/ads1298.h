#pragma once
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "buffer.h"
#include "filter.h"
#include "timer.h"
#include "delay.h"
#include "gpio.h"
#include "sp1ml_command.h"

typedef struct{
uint8_t gain;
uint8_t enable_mask;
uint8_t channel_seven_neg;
uint16_t cap;		//Holds estimated parasitic capacitance of the cable, used for compensating lead-off AC data
uint8_t updated_flag;	//Used for flagging a pending update to the device
} ADS_config_type;

typedef struct{
int32_t I;
int32_t Q;
} Int_complex_type;

//Globals used for interfacing
extern buff_type ECG_buffers[8];
extern volatile uint8_t ADS1298_Error_Status;

//Lead-off limit, this corresponds to 71% of the ADC range being used by the AC lead off signal
#define LEAD_LIM_(x,y) (4800000UL/((x+200)*y)) /*This isn't a perfect approximation but should be within 10% or so*/ /*0x80000000*/ //TODO 00
#define ADS1298_LEAD_LIMIT(x,y) (LEAD_LIM_(x,y)*LEAD_LIM_(x,y)) /*Actual limit defined relative to measured amplitude squared*/
#define ADS1298_LEAD_HYSTERYSIS(x,y) (ADS1298_LEAD_LIMIT(x,y)>>2) /*0x40000000*/

//Magic numbers used to signal lead-off or lead repurposed as RLD
#define ADS1298_LEAD_OFF (-(1<<25))
#define ADS1298_LEAD_RLD (1<<25)

//These two variables are used for lead-off scheduling control, RLD at 1Hz, RLD turnoff at 0.25Hz (but the first one runs more slowly when common mode is low)
#define ADS1298_RLD_ITERATIONS 250
#define ADS1298_RLD_TEST_ITERATIONS 1000

//Used for marking a saturated signal as faulty (200ms of saturation gives a fault)
#define SATURATION_COUNT_LIMIT 50
#define SATURATION_COUNTDOWN_RATE 5

//Used to automatically schedule RLD sense dependant on common mode signal (mean squared mean common mode). Ignore threshold is 1/4 of range, thresh one is 1/2 etc
#define COMMON_THRESH_IGNORE (1<<10)
#define COMMON_THRESH_ONE (2*COMMON_THRESH_IGNORE)
#define COMMON_THRESH_TWO (3*COMMON_THRESH_IGNORE)

//Buffer size in samples, just under half a second of data and uses 4k of memory
#define ADS1298_BUFFER 120

//This can be defined to activate LEDs on the four GPIO outputs. LED signals are: 1) RLD failed, 2) RLD remapped, 3) WCT poor, 4) WCT failed.
//#define ECG_LEDS /*Now defined as a function of PCB revision (PCB in gpio.h) */ 
#if PCB>1
	#define ECG_LEDS
#endif

//Register and command definitions
#define ADS1298_WAKEUP 0x02
#define ADS1298_STANDBY 0x04
#define ADS1298_RESET 0x06
#define ADS1298_START 0x08
#define ADS1298_STOP 0x0A
#define ADS1298_RDATAC 0x10
#define ADS1298_SDATAC 0x11
#define ADS1298_RDATA 0x12

//Chip ID code definitions
#define ADS1298_ID 0x92
#define ADS1298R_ID 0xD2
#define ADS1296_ID 0x91
#define ADS1296R_ID 0xD1
#define ADS1294_ID 0x90
#define ADS1294R_ID 0xD0

//CS
#define NSEL_LOW GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_RESET)
#define NSEL_HIGH GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_SET)

//List of online reconfiguration tasks, these use a priority queue, highest priority has the lowest number
enum{RLD_OFF=0,RLD_DISCONNECT,RLD_STAT,RLD_RECONNECT,RLD_ON,RLD_REMAP,WCT_REMAP,RLD_WCT_REMAP,RLD_UNMAP,RLD_REPLACE,RLD_REMOVE,LEAD_OFF_REPLACE,GPIO_UPDATE};
//List of failure case enumerations, these can be used to flash an LED from the main thread, but preferably only if the electrode config is really screwed
enum{RLD_FAILURE=0,RLD_REMAPPED,WCT_FAILURE,WCT_SUBOPTIMAL};/*RLD failure means RLD had to be remapped, WCT suboptimal means <3 amps, WCT failure means bypass*/

//Global latest data
extern volatile int16_t Filtered_ECG[8];

//Function prototypes
uint8_t ads1298_setup(ADS_config_type* config, uint8_t startnow);
void ads1298_start(void);
uint8_t ads1298_gain(void);
void ads1298_force_rld_sense(void);
void ads1298_busy_wait_write(uint8_t tx_bytes, uint8_t register_number, uint8_t *tx_data);
void ads1298_busy_wait_read(uint8_t rx_bytes, uint8_t register_number, uint8_t *rx_data);
void ads1298_busy_wait_command(uint8_t command);
void ads1298_handle_data_arrived(uint8_t* raw_data_, buff_type* buffers);
void ads1298_handle_data_read(uint8_t* r_data);
void handle_aligned_sensors(void);
Int_complex_type ads1298_electrode_quality(uint32_t buffer[4]);
void ads1298_wct_config(uint8_t wct_regs[2], uint8_t mask);
void ads1298_spi_dma_transaction(uint8_t bytes_sent, uint8_t* tx_pointer, uint8_t rx_bytes);

