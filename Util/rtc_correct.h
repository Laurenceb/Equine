#pragma once
#include "stm32f10x.h"

uint8_t rtc_correct_with_timestamp(uint32_t timestamp);
void set_rtc_correction(int8_t correction);
void update_rtc_timestamp(uint32_t timestamp);
