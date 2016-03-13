#include "timer.h"
#include "gpio.h"

/**
  * @brief  Configure the timer channels for PWM clk to ECG front end on ECG board
  * @param  None
  * @retval None
  * This initialiser function assumes the clocks and gpio have been configured
  */
void setup_pwm(void) {
  /* -----------------------------------------------------------------------
    TIM Configuration: generate a 2.048Mhz clock to the ECG front end:
    The TIMxCLK frequency is set to SystemCoreClock (Hz), to get TIMx counter
    clock at 73.728 MHz the Prescaler is computed as following:
     - Prescaler = (TIMxCLK / TIMx counter clock) - 1
    SystemCoreClock is set to 73.728 MHz

    The TIM3 Frequency = TIM4 counter clock/(ARR + 1)
  ----------------------------------------------------------------------- */
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure={};
  TIM_OCInitTypeDef  	TIM_OCInitStructure={};
  GPIO_InitTypeDef	GPIO_InitStructure;
  /*Enable the Tim3 clk*/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  //Setup the GPIO pins
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;	//reduced slew rate to reduce interference on the board
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init( GPIOB, &GPIO_InitStructure );		
  TIM_DeInit(TIM3);
  /*Setup the initstructure*/
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 35;		//PWM to the ADS1298
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 18;
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	//Mode 1 is regular PWM mode
  /* 'PWM1' Mode configuration: Channel4 */
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
  /*We enable the timer*/
  TIM3->CNT=0;
  TIM_Cmd(TIM3, ENABLE);				//We should now have a 2.048Mhz output clock 
}
