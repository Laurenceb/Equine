#pragma once
#include <stdio.h>
#include "stm32f10x.h"
#include "gpio.h"
#include "buffer.h"
#include "lsm9ds1.h"

//Datatypes
typedef struct{
	uint8_t error;
	uint8_t job;
} I2C_Error_Type;

typedef struct{
	const uint8_t address;	//device address for this job
	const uint8_t direction;//direction (I2C_Direction_Transmitter or I2C_Direction_Receiver)
	const uint8_t bytes;	//number of bytes to read/write
	const uint8_t subaddress;//register subaddress for the device - set to 0xFF if no subaddress used (direct read/write)
	volatile uint8_t* data_pointer;	//points to the data
} I2C_Job_Type;

//Globals
extern volatile uint32_t Jobs,Completed_Jobs;	//used for task control (only ever access this from outside for polling Jobs/Reading Completed_Jobs)
extern volatile I2C_Error_Type I2C1error;	//used to store error state

//Function prototypes
void I2C1_Request_Job(uint8_t job_);//Requests a job
void I2C1_Setup_Job(uint8_t job_, volatile uint8_t* data);//Sets up the data pointer for a job
void I2C_Config(void);//configures the hardware
#define Flipbytes(x) x=(((uint16_t)x>>8)&0x00FF)|((((uint16_t)x&0x00FF)<<8)&0xFF00)
#define Flipedbytes(x) (int16_t)(((x>>8)&0x00FF)|(((x&0x00FF)<<8)&0xFF00))
