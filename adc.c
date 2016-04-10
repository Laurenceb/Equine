#include <stdlib.h>
#include "adc.h"
#include "interrupts.h"


void ADC_Configuration(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
  /* PCLK2 is the APB2 clock */
  /* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);

  /* Enable ADC2 clock so that we can talk to it */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);;
  /* Put everything back to power-on defaults */
  ADC_DeInit(ADC2);

  /* ADC2 Configuration ------------------------------------------------------*/

  /* ADC1 and ADC2 operate independently */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  /* Enable the scan conversion so we do three at a time */
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  /* Don't do contimuous conversions - do them on demand */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  /* Start conversin by software, not an external trigger */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  /* Conversions are 12 bit - put them in the lower 12 bits of the result */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  /* Say how many channels would be used by the sequencer */
  ADC_InitStructure.ADC_NbrOfChannel = 1;

  /* Now do the setup */
  ADC_Init(ADC2, &ADC_InitStructure);

  /* ADC2 injected channel configuration */
  ADC_InjectedSequencerLengthConfig(ADC2, 1);//one conversion
  ADC_InjectedChannelConfig(ADC2, 1, 1, ADC_SampleTime_239Cycles5);
  ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None);//set sw injected channels

  /* Set the analogue watchdog on the battery voltage conversion*/
  ADC_AnalogWatchdogCmd(ADC2,ADC_AnalogWatchdog_SingleInjecEnable);
  ADC_AnalogWatchdogThresholdsConfig(ADC2,0x0FFF,(uint16_t)((float)SAMPLING_FACTOR*MINIMUM_VOLTAGE));//watchdog fires on low voltage
  ADC_AnalogWatchdogSingleChannelConfig(ADC2, BATTERY_ADC_CHAN);//set the watchdog to the battery voltage channel
  ADC_ITConfig(ADC2, ADC_IT_AWD, ENABLE);//enable the analogue watchdog interrupt

  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);

  /* Enable ADC2 reset calibaration register */
  ADC_ResetCalibration(ADC2);
  /* Check the end of ADC2 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC2));
  /* Start ADC2 calibaration */
  ADC_StartCalibration(ADC2);
  /* Check the end of ADC2 calibration */
  while(ADC_GetCalibrationStatus(ADC2));
  
  readADC2(BATTERY_ADC_CHAN);//Have to flush this for some reason
  getBatteryVoltage();   //Flush the channel injection
}


/**
* @brief This function returns a conversion from ADC2 (blocking)
* @param Channel number to convert
* @retval unsigned 16 bit integer - adc is 12bit
*/
uint16_t readADC2(uint8_t channel)
{
  ADC_RegularChannelConfig(ADC2, channel, 1, ADC_SampleTime_239Cycles5);
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC2, ENABLE);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET);
  // Reset the flag
  ADC_ClearFlag(ADC2, ADC_FLAG_EOC);
  // Get the conversion value
  return ADC_GetConversionValue(ADC2);
}

/**
* @brief This function sets up a conversion from ADC2 (non blocking)
* @param Channel number to convert
* @retval None
*/
void setADC2(uint8_t channel)
{
  ADC_RegularChannelConfig(ADC2, channel, 1, ADC_SampleTime_239Cycles5);
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC2, ENABLE);
}

/**
* @brief This function gets a conversion from ADC2 (non blocking)
* @param None
* @retval Read value (-1 means adc not ready)
*/
uint16_t getADC2(void)
{
  // Make sure we have conversion completion
  if(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET)
    return -1;
  // Reset the flag
  ADC_ClearFlag(ADC2, ADC_FLAG_EOC);
  // Get the conversion value
  return ADC_GetConversionValue(ADC2);
}

/**
* @brief This function gets a battery level conversion from ADC2 (non blocking)
* @param None
* @retval Read value in volts as a float (-1 means adc not ready)
*/
float getBatteryVoltage(void) {//This starts a new conversion to run as soon as the ADC has been read
	float battery_level;
	if(ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC)) {		//We have adc2 converted data from the injected channels
		ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);		//Clear the flag
		battery_level=((float)ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1))/(SAMPLING_FACTOR);
	}
	else
		return -1;			//Signals an error
	ADC_SoftwareStartInjectedConvCmd(ADC2, ENABLE);//New conversion starts now
	return battery_level;
}

/**
* @brief This function converts a battery level voltage to a percentage as unsigned integer, calibrated to the BAK LP-503759-IS-3 1350mAh cell, but matches most lipo
* @param Float voltage in volts
* @retval Value as unsigned integer in the range 0 to 100
*/
uint8_t getBatteryPercentage(float voltage) {
	if(voltage>4.0) {
		uint8_t charged=(uint8_t)((voltage-4.0)*50.0)+90;
		return charged>100?100:charged;
	}
	else if(voltage>3.75)
		return (uint8_t)((voltage-3.75)*160.0)+50;
	else if(voltage>3.5)
		return (uint8_t)((voltage-3.5)*180.0)+5;
	else
		return (uint8_t)((voltage-MINIMUM_VOLTAGE)*(5.0/(3.5-MINIMUM_VOLTAGE)));
}
