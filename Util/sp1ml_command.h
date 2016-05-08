#pragma once
#include "stm32f10x.h"
#include <stdint.h>
#include <stdlib.h>
#include "gpio.h"
#include "usart.h"

//Radio config definitions, this is the max that will fit into EU band allocations --------------------------------

//This band is licensed for 100% duty cycle operation in EU, 300kHz width centered on 869.85Mhz, 7dBm power allocation
#define FREQUENCY "869850000"
//This should fill the 300khz tx band
#define DATA_RATE "150000"
//Complies with EU regs (+7dBm), but antenna max performance is -1dB when converting to EIRP
#define TX_POWER "+8"
//Gives the best performance
#define DEVIATION "75"
//Should more than encompass the side lobes
#define RX_FILTER "330"
//Enable FEC, halves data rate, gives about 0.5dB gain at 1% PER, considerably more at lower PER. Disable this for time being, gives 150kbps, for 16x16bit, 250hz
#define FEC "0"
//This CS mode allows 6dB dynamic, might be beneficial when used with FEC?
#define CS_MODE "1"
//There is little chance of received signal below this level, 1% PER is at approx -105dBm with FEC and raw DR of 150kbps. Theoretically no decode at -110
#define RSSI_THRESH "-110"
//With FEC+ARQ there will be a performance cliff at approx -106dBm with SMPS enabled and 2FSK.
//SP1ML module uses AM11DG-ST01 antenna, assume horizontal plane at both ends, with the receiver also horizontal on USB stick, gives average 104dB budget
//This results in (rural model), 630 to 950m range on average, and worst case 230m to 340m in null (worst case ~80dB, perhaps 15% of the time). 
#if PCB<2
	#define LED "1"
#else
	#define LED "2"
#endif /*LED is to gnd on version 2 and later pcbs*/ 

//Enable filtering on packets that are received
#define FILTER_CRC "1"
#define FILTER_SOURCE "1"
#define FILTER_DESTINATION "1"
#define FILTER_MULTICAST "1"
#define FILTER_BROADCAST "1"

//Network protocol definitions --------------------------------

//This is the default source address of the module, if there are multiple modules, this could be changed
#define NETWORK 0x01
#define SOURCE "0x01"
#define DESTINATION "0x00"/*This is the base station (at 0x00)*/
#define PAYLOAD "64"
#define PAYLOAD_BYTES 64

//This is used for config feedback from a device COBS type packets with a simple network address assignment and text name system, and (device, sequence_number) header
//HEAD byte is newline (\n)
#define HEAD 0x0A

//Manager state machine and two types of request status states -------------
enum{INIT=0,PUNG,ASSIGNED,REQUEST,REQUEST_TWO};

//Data request structure, this is the data from the host
#define REQUEST_HEADER "MJ"

//Buffer size used by usart buffers
#define SP1ML_BUFFER 3072

//Simple 20ms delay
#define SP1ML_DELAY {uint32_t m=Millis+20; while(Millis<m);};

//Datatype used for the low level request manager state machine. This allows functionality to be shard with bluetooth (RN42)
typedef struct{
	uint8_t state;
	uint8_t internal_type;
	uint8_t sample_counter;
	uint8_t main_counter;
	uint8_t signal;
	uint8_t argument;
	uint16_t main_mask;
	uint16_t internal_mask;
	uint8_t sequence_is_time;//This is used to make the sequence number a function of time rather than transmitted bytes. This makes loss recovery easier
} SP1ML_tx_rx_state_machine_type;

//Globals used for passing data from the sensor read functions
extern volatile uint8_t SP1ML_state;	//This should be in the state ASSIGNED before any data is passed
extern volatile uint8_t SP1ML_aligned_data_ready;
extern volatile SP1ML_tx_rx_state_machine_type SP1ML_tx_rx_state;
extern volatile uint8_t SP1ML_tx_sequence_number;
extern volatile uint8_t SP1ML_withold;
extern volatile uint32_t SP1ML_tx_bytes;

//Functions
void SP1ML_rx_tx_data_processor(SP1ML_tx_rx_state_machine_type* stat,void (*generate_packet)(uint8_t*,uint8_t,uint8_t,uint8_t*), buff_type* buff, uint8_t* tx_s, uint8_t* flag);
uint8_t hex_to_byte(uint8_t hex);
void byte_to_hex(uint8_t hex[2], uint8_t arg);
uint8_t SP1ML_configure(void);
uint8_t SP1ML_assign_addr(uint8_t addr);
void SP1ML_manager(uint8_t* SerialNumber, SP1ML_tx_rx_state_machine_type* stat);
void SP1ML_generate_packet(uint8_t* data_payload, uint8_t number_bytes, uint8_t device_network_id, uint8_t* sequence_number);


