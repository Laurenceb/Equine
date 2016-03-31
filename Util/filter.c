#include "filter.h"


/**
  * @brief Runs a two stage IIR filter over the data, wide bandpass between 1hz and ~15hz with slow rolloff to 125hz, then tight notch around 50Hz with 48dB attenuation
  * @param Pointer to filter state type, float input data
  * @reval Float data output
  */
float iir_filter_50(filter_state_type *f_state, float input) {
	float in=input+1.322*f_state->z_one[0]-0.339*f_state->z_one[1];
	float out=0.331*in-0.331*f_state->z_one[1];
	f_state->z_one[1]=f_state->z_one[0];
	f_state->z_one[0]=in;
	in=out+0.46*f_state->z_two[0]-0.486*f_state->z_two[1];
	float dataout=0.743*in-0.46*f_state->z_two[0]+0.743*f_state->z_two[1];
	f_state->z_two[1]=f_state->z_two[0];
	f_state->z_two[0]=in;
	return dataout;
}

/**
  * @brief Runs a comb filter over the data, not completely attenuating at the cut frequency
  * @param Pointer to the filter state, float input data
  * @reval Float data output
  */
float comb_filter(comb_state_type *f_state, float input) {
	float f=f_state->z[1];
	f_state->z[1]=f_state->z[0];
	f_state->z[0]=input;
	return (input+(f*7.0/8.0))/1.875;
}
