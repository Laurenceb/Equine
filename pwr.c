#include "pwr.h"

/**
  * @brief  Enables the power domain
  * @param  None
  * @retval None
  */
void setuppwr() {
	PWR_DeInit();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);//clk to the pwr control
}

/**
  * @brief  Enables the wakeup pin functionality and places device in low power mode
  * @param  None
  * @retval None
  */
void shutdown() {
	PWR_WakeUpPinCmd(ENABLE);			//enable the pin
	PWR_EnterSTANDBYMode();				//only wakes on RTC signals or WKUP pin
}

/**
  * @brief  Disables the wakeup pin functionality
  * @param  None
  * @retval None
  */
void disable_pin() {
	PWR_WakeUpPinCmd(DISABLE);			//disable the pin
}

/**
  * @brief  This function closes any open files, leaving a file footer in the FATFS_logfile
  * @param  uint8_t reason: reason for shutdown, uses enum definitions in main.h, uint8_t file_flags: flag bits for open files, DIR* dir the current directory
  * @retval None
  */
void shutdown_filesystem(uint8_t reason, uint8_t file_flags) {
	if(file_flags&0x01) {				//The first wav file is open
		if(f_tell(&FATFS_wavfile)>2)		//There is some data in the file
			wave_terminate(&FATFS_wavfile);
	}
	if(file_flags&0x02) {				//The second wav file is open
		if(f_tell(&FATFS_wavfile_gps)>2)	//There is some data in the file
			wave_terminate(&FATFS_wavfile_gps);
	}
	if(file_flags&0x04) {				//The second wav file is open
		if(f_tell(&FATFS_wavfile_imu)>2)	//There is some data in the file
			wave_terminate(&FATFS_wavfile_imu);
	}
	uint8_t path[22]={};
	f_getcwd(path,22);				/* Find the directory name */
	DIR dir;
	FRESULT res = f_opendir(&dir, path);		/* Open the directory */
	FILINFO fno;
	char* fn;
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno);                   /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
			if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
#if _USE_LFN
			fn = *fno.lfname ? fno.lfname : fno.fname;
#else
			fn = fno.fname;
#endif
			if (!(fno.fattrib & AM_DIR)) {                    /* It is not a directory */
				if(!(fno.fsize))
					f_unlink((const TCHAR *)fn);	/* Delete zero sized files */
			}
		}
	}
}

