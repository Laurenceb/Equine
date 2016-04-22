#include "lsm9ds1.h"


//The data buffers used for the sensor
volatile gyro_data_type LSM9DS1_Gyro_Buffer;
volatile uint16_t LSM9DS1_Acc_Buffer[3];
volatile uint16_t LSM9DS1_Mag_Buffer[3];
volatile int16_t LSM9DS1_Mag_Offset[3];//This is used for simple offset based (i.e. "hard iron") magnetometer correction. Should be good enough for <2 degree error
volatile buff_type IMU_buff[10];

//Failure notification counter
volatile uint8_t I2C_failure;

/**
  * @brief  This function connects the raw data buffers up to the I2C driver
  * @param  None
  * @retval None
  */
void configure_i2c_buffers(void) {
	I2C1_Setup_Job(LSM9DS1_GYRO, (volatile uint8_t*)&LSM9DS1_Gyro_Buffer);
	I2C1_Setup_Job(LSM9DS1_ACC, (volatile uint8_t*)LSM9DS1_Acc_Buffer);
	I2C1_Setup_Job(LSM9DS1_MAGNO, (volatile uint8_t*)LSM9DS1_Mag_Buffer);
	for(uint8_t n=0; n<10; n++)	//Init the buffers to the main loop
		Init_Buffer(&IMU_buff[n], ADS1298_BUFFER, 2);
}


/**
  * @brief  This function handles I2C read completion and packs the data away in the appropriate buffers
  * @param  None
  * @retval None
  */
void handle_lsm9ds1(void) {
	static uint16_t copy_acc[3];
	if(Completed_Jobs&(0x00000001<<LSM9DS1_MAGNO)) {//The last of the read jobs has been completed
		Completed_Jobs&=~((1<<LSM9DS1_GYRO)|(1<<LSM9DS1_ACC)|(1<<LSM9DS1_MAGNO));//wipe the relevant bits
		Flipbytes(LSM9DS1_Gyro_Buffer.x);//endianess bug on LSM9DS1 Gyro x,y,z?
		Flipbytes(LSM9DS1_Gyro_Buffer.y);
		Flipbytes(LSM9DS1_Gyro_Buffer.z);
		Add_To_Buffer(&(LSM9DS1_Gyro_Buffer.x),&(IMU_buff[0]));
		Add_To_Buffer(&(LSM9DS1_Gyro_Buffer.y),&(IMU_buff[1]));
		Add_To_Buffer(&(LSM9DS1_Gyro_Buffer.z),&(IMU_buff[2]));
		for(uint8_t n=0; n<3; n++)
			Add_To_Buffer(&(LSM9DS1_Acc_Buffer[n]),&(IMU_buff[3+n]));
		for(uint8_t n=0; n<3; n++)
			Add_To_Buffer(&(LSM9DS1_Mag_Buffer[n]),&(IMU_buff[6+n]));
		Add_To_Buffer(&(LSM9DS1_Gyro_Buffer.temp),&(IMU_buff[9]));
	}
	if(!GET_LSM9DS1_DTRD) {			//If there is some new data (otherwise we load anyway so it gets padded)
		if((Completed_Jobs&CONFIG_SENSORS)==CONFIG_SENSORS) {//Only schedule new reads if sensors are present and running
			I2C1_Request_Job(LSM9DS1_GYRO);//The Completed_Jobs should first be wiped if the sensors need to be reinitialised
			I2C1_Request_Job(LSM9DS1_ACC);
			I2C1_Request_Job(LSM9DS1_MAGNO);//Read everything again
		}
		if(memcmp(LSM9DS1_Acc_Buffer,copy_acc,sizeof(copy_acc)) && !I2C1error.error) {
			I2C_failure=0;
			memcpy(copy_acc,LSM9DS1_Acc_Buffer,sizeof(copy_acc));//Copy for reference
		}
		else
			I2C_failure++;		//Failure increases if DTRD doesnt go low or we have repeated data or an error. It's reset if all ok
	}
	else
		I2C_failure++;
}

/**
  * @brief  This function handles I2C read completion and packs the data away in the appropriate buffers
  * @param  None
  * @retval uint8_t returned, zero means correctly operating, a non zero argument implies that there is a problem. Upper bit == bus failure, Lower bits == reset count
  */
uint8_t check_lsm9ds1_functionality(void) {
	static uint8_t i2c_resets;		//Stores the number of resets since the device was powered up (max 127)
	if(I2C_failure>LSM9DS1_FAILURE_COUNTOUT) {
		uint32_t repetition_counter;
		uint32_t entry_ms=Millis;
		do {
			if(Millis-entry_ms>I2C_RESET_TIMEOUT)
				return 0x80;	//Upper bit implies failure
			repetition_counter=Millis;//Store time at entry into this loop
			I2C1error.error=0;	//Reset both of these
			I2C_failure=0;
			Completed_Jobs=0;	//Prevent any bus reads from running
			while(Jobs && Millis<(repetition_counter+20))
				__WFI();//Wait for any jobs to finish (with timeout)
			Completed_Jobs=0;	//Ensure this is zeroed
			I2C_Config();		//Setup the I2C bus
			SCHEDULE_CONFIG;	//Setup the sensors on the bus
			while(Jobs && Millis<(repetition_counter+60))//Wait for the initial config jobs to complete, allow 60ms total
				__WFI();
		} while((Completed_Jobs&CONFIG_SENSORS)!=CONFIG_SENSORS);//Loop until the I2C sensors are correctly initialised (or we timeout and return inside loop)
		i2c_resets++;
		if(i2c_resets&0x80)
			i2c_resets&=~0x80;	//Make sure the last bit isnt set if there are a very high number of resets
		return i2c_resets;
	}
	else
		return 0;
}
