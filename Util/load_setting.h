#include "stm32f10x.h"
#include <stdint.h>

#include "ads1298.h"
#include "ff.h"


uint8_t read_config_file(FIL* file, ADS_config_type* set_struct, uint8_t* rtc);
