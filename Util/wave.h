#pragma once
#include <stdint.h>
#include "ff.h"

typedef struct {
	uint8_t index;
	uint8_t lastbyte;
	} wave_stuffer;

FRESULT write_wave_header(FIL* file, uint8_t number_channels, uint16_t sample_rate, uint8_t bits_per_sample);
FRESULT write_wave_samples(FIL* file, uint8_t number_channels, uint8_t bits_per_sample, wave_stuffer* this_stuffer, void* data, uint8_t alignment);
FRESULT wave_terminate(FIL* file);
