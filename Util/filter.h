#pragma once
#include "stm32f10x.h"
#include <stdint.h>

typedef struct {
	float z_one[2];
	float z_two[2];
} filter_state_type;

float iir_filter_50(filter_state_type *f_state, float input);
