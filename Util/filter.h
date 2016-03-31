#pragma once
#include "stm32f10x.h"
#include <stdint.h>

typedef struct {
	float z_one[2];
	float z_two[2];
} filter_state_type;

typedef struct {
	float z[2];
} comb_state_type;

typedef struct {
	filter_state_type iir;
	comb_state_type cmb;
} filter_state_type_c;

float iir_filter_50(filter_state_type *f_state, float input);
float comb_filter(comb_state_type *f_state, float input);
