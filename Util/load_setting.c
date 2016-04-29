#include "load_setting.h"
#include "lsm9ds1.h"

/**
  * @brief Reads through an ascii format setting file and sets the ECG config. Format: \n sep, "M01010101" mask, "G8" gain, "C1" ch7N reconf on/off, 
	   "B400" set estimated cable parasitic capacitance in pF for compensating lead-off, "Rff" hex RTC, O<x>,<y>,<z> magnetometer offset, e.g. O20,-100,34
  * @param Pointer to the config file, pointer to the config struct
  * @reval Byte value, zero all ok, non zero for error due to malformed file
  */
uint8_t read_config_file(FIL* file, ADS_config_type* set_struct, uint8_t* rtc) {//Note that a special value of 0x91 (or "R91") as the RTC will set reference out on TP
	uint8_t counter,mask=0,c=0,state=0,br,byte,axis_counter=0,axis_sign=0;
	uint16_t gain=0,cap=0;
	do {
		f_read(file, (void*)(&byte),1,&br);//Read a character from the file
		if(br || (state&&counter)) {//Run one more time after getting EOF, so long as we have read something on the current line
			if(!br)
				byte=0x00;	//Null byte on reading nothing
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
				case 'B':
					state=4;
					break;
				case 'R':
					state=5;
					gain=0;//reuse this variable
					break;
				case 'O':
					state=6;
					gain=0;
					axis_counter=0;//First axis
					axis_sign=0;//Positive
				}
				break;
			case 1:	//The start of the mask
				if(byte=='1')
					mask|=1<<counter;
				if((byte=='0') || (byte=='1'))
					counter++;
				else if(byte=='\n' || !br || counter<=8) {
					if(counter && set_struct->enable_mask!=mask) {
						set_struct->enable_mask=mask;
						set_struct->updated_flag|=1;
					}
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
					if(counter && set_struct->gain!=gain && gain<=12) {
						set_struct->gain=gain;
						set_struct->updated_flag|=(1<<1);
					}
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
					if(counter && set_struct->channel_seven_neg!=c) {
						set_struct->channel_seven_neg=c;
						set_struct->updated_flag|=(1<<2);
					}
					state=0;
				}
				break;
			case 4: //The cable capacitance input. This is added to existing structure value
				if((byte>=0x30) && (byte<=0x39)) {
					cap*=10;//Move onto the next digit
					cap=(byte-0x30);
					counter++;
				}
				else if(byte=='\n' || !br || counter>=2) {
					if(counter && set_struct->cap!=cap) {
						set_struct->cap+=cap;
						set_struct->updated_flag|=(1<<3);
					}
					state=0;	
				}
				break;
			case 5: //The hex RTC correction byte (int8_t) as two lower case hex nibbles
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
			case 6: //The axis correction values
				if(counter && (byte==',' || byte=='\n' || !br) && axis_counter<3)//Load the read axis correction
					LSM9DS1_Mag_Offset[axis_counter]=axis_sign?-gain:gain;
				if(byte=='\n' || !br) {
					state=0;
					break;
				}
				counter++;
				if(byte==',') {
					axis_counter++;
					axis_sign=0;
					break;
				}
				if(byte=='-') {
					axis_sign=1;
					break;
				}
				if((byte>=0x30) && (byte<=0x39)) {
					gain*=10;//Move up the previous value
					gain+=(byte-0x30);
				}
			}
		}
		else	//end of file
			return 1;//Reached end of the file in the middle of a config	
	} while(br);
	return 0;
}
