#include "data_alignment.h"

/*
This function is only called when new data at the first sample rate arrives. The first sample rate is the lowest.
return values are: 0=do nothing, 1=pad, -1=drop sample
all sample manipulation takes place in the second stream
With a 5hz GPS stream and 250hz stream for the other sensors, this is always stable and takes less than 10 seconds to pull in within the 50ms deadband
Before calling this, wait for a GPS position to come in then immediatly wipe all buffers and restart to get the best initial alignment
*/
int8_t aligndata(uint32_t samples[2], uint8_t ratio) {
	static int32_t lowpass;
	lowpass+=((samples[1]-samples[0]*ratio)>>4)-(lowpass>>4);//A 1/16 simple rolling mean to take out effects of GPS sample to sample jitter
	if(abs(lowpass)<(ratio>>3))//There is a deadband of one quarter of a sample (from the first i.e. slower stream) total width 
		return 0;
	else if(lowpass>0) {
		samples[1]--;
		return -1;
	}
	samples[1]++;
	return 1;
}
