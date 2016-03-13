#include "load_setting.h"


/**
  * @brief Reads through an ascii format setting file and sets the ECG config. Format: \n sep, "M01010101" mask, "G8" gain, "C1" ch7N reconf on/off, "Rff" hex RTC
  * @param Pointer to the config file, pointer to the config struct
  * @reval Byte value, zero all ok, non zero for error due to malformed file
  */
uint8_t read_config_file(FIL* file, ADS_config_type* set_struct, uint8_t* rtc) {//Note that a special value of 0x91 (or "R91") as the RTC will set reference out on TP
	uint8_t counter,mask=0,gain=0,c=0,state=0,br,byte;
	do {
		f_read(file, (void*)(&byte),1,&br);//Read a character from the file
		if(br || (state&&counter)) {
			switch(state) {
			case 0:
				counter=0;
				switch(byte) {
				case 'M':	//Mask line
					state=1;
					break;
				case 'G':
					state=2;
					break;
				case 'C':
					state=3;
					break;
				case 'R':
					state=4;
					gain=0;//reuse this variable
					break;
				}
				break;
			case 1:	//The start of the mask
				if(byte=='1')
					mask|=1<<counter;
				if((byte=='0') || (byte=='1'))
					counter++;
				else if(byte=='\n' || !br || counter<=8) {
					if(counter)
						set_struct->enable_mask=mask;
					state=0;
				}
				break;
			case 2:	//The gain
				if((byte>=0x30) && (byte<=0x39)) {
					gain*=10;//Move onto the next digit
					gain=(byte-0x30);
					counter++;
				}
				else if(byte=='\n' || !br || counter>=2) {
					if(counter)
						set_struct->gain=gain;
					state=0;	
				}
				break;
			case 3: //The ch7n config state
				if(byte=='1') {
					c=1;
					counter++;
				}
				else if(byte=='0') {
					c=0;
					counter++;
				}
				else if(byte=='\n' || !br || counter>=1) {
					if(counter)
						set_struct->channel_seven_neg=c;
					state=0;
				}
				break;
			case 4: //The hex RTC correction byte (int8_t) as two lower case hex nibbles
				if((byte>=0x30) && (byte<=0x39)) {
					gain+=(byte-0x30);
					counter++;
				}
				if((byte>=0x57) && (byte<=0x5C)) {
					gain+=(byte-0x57+10);
					counter++;
				}
				else if(byte=='\n' || !br || counter>=2) {
					if(counter!=1)//More than one nibble arrived
						*rtc=gain;
					state=0;	
				}
				if(counter==1)
					gain<<=4;//shift the first nibble up
				break;
			}
		}
		else	//end of file
			return 1;//Reached end of the file in the middle of a config	
	} while(br);
	return 0;
}
