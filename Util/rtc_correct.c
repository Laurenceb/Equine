#include "rtc_correct.h"


uint8_t rtc_correct_with_timestamp(uint32_t timestamp) {
	uint32_t f;					//The time from epoch that the RTC was last set
	uint32_t t= RTC_GetCounter();			//The time at entry
	PWR_BackupAccessCmd(ENABLE);			//Allow access to BKP Domain
	((uint16_t*)&f)[0]=BKP_ReadBackupRegister(BKP_DR2); //The pressure PID control I value
	((uint16_t*)&f)[1]=BKP_ReadBackupRegister(BKP_DR3);
	PWR_BackupAccessCmd(DISABLE);
	if(f && (timestamp-f)>500000 && (timestamp-f)<27000000) {//Enough time to get ~2ppm accuracy, and not over ~10months of time, over which accuracy drifts
		f=timestamp-f;				//Time that has elapsed
		int32_t error=t-(timestamp-1000000);
		error-=1000000;				//Allows us to use a signed 32 bit with unsigned 32 bit variables
		if(abs(error)>1000)			//Too much error for the correction to be sensible
			return 2;
		error/=f;				//Error in ppm
		if(error>29 || error<-91)		//This range can be corrected
			return 3;
		set_rtc_correction((int8_t)error);	//Apply the correction to the hardware
	}
	else
		return 1;
}

void set_rtc_correction(int8_t correction) {
	PWR_BackupAccessCmd(ENABLE);			/* Allow access to BKP Domain */
	uint16_t tweaked_prescale = (0x0001<<15)-2;	/* Try to run the RTC slightly too fast so it can be corrected either way */
	RTC_WaitForSynchro();				/* Wait for RTC registers synchronization */
	if( RTC->PRLL != tweaked_prescale ) {		/*Note that there is a 0.5ppm offset here (correction 0==0.5ppm slow)*/
		RTC_SetPrescaler(tweaked_prescale); 	/* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767-2+1) */
		RTC_WaitForLastTask();
	}
	BKP_SetRTCCalibrationValue((uint8_t) ((int16_t)31-(21*(int16_t)correction)/(int16_t)20) );
	BKP_RTCOutputConfig(BKP_RTCOutputSource_None);	/* Ensure any output is disabled here */
	/* Lock access to BKP Domain */
	PWR_BackupAccessCmd(DISABLE);
}

void update_rtc_timestamp(uint32_t timestamp) {
	PWR_BackupAccessCmd(ENABLE);			//Allow access to BKP Domain
	BKP_WriteBackupRegister(BKP_DR2, ((uint16_t*)&timestamp)[0]); //The pressure PID control I value
	BKP_WriteBackupRegister(BKP_DR3, ((uint16_t*)&timestamp)[1]);
	PWR_BackupAccessCmd(DISABLE);
}
