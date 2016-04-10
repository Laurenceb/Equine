#include "stm32f10x.h"


#define BATTERY_ADC_CHAN 4/* The battery voltage monitoring */

#define SAMPLING_FACTOR	4096.0/6.6/* 1/2 factor pot divider on the frontend board - integrated on BOARD */

#define GET_BATTERY_VOLTAGE (float)readADC2(BATTERY_ADC_CHAN)/(SAMPLING_FACTOR)/* Macro to return battery voltage as a float using blocking regular conv*/

void ADC_Configuration(void);
void setADC2(uint8_t channel);
uint16_t readADC2(uint8_t channel);
uint16_t getADC2(void);
float getBatteryVoltage(void);
uint8_t getBatteryPercentage(float voltage);
