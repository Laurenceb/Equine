#include <math.h>
#include <stdlib.h>
#include "interrupts.h"

volatile uint8_t Button_hold_tim;				//Timer for On/Off/Control button functionality
volatile uint16_t Bufferflag;					//Unlock flag

/**
  * @brief  Configure all interrupts accept on/off pin
  * @param  None
  * @retval None
  * This initialiser function assumes the clocks and gpio have been configured
  */
void ISR_Config(void) {
	NVIC_InitTypeDef   NVIC_InitStructure;
	/* Set the Vector Table base location at 0x08000000 */    
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);      
	//First we configure the systick ISR
	/* Configure one bit for preemption priority */   
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* Enable and set SYSTICK Interrupt to the fifth priority */
	NVIC_InitStructure.NVIC_IRQChannel = SysTick_IRQn;	//The 100hz timer triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//High Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;	//6th subpriority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;	//The ADC watchdog triggered interrupt, used for low battery detection	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Low Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x06;	//7th subpriority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//Now we configure the I2C Event ISR
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;	//The I2C1 triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//High Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;	//Second to highest group priority
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//Now we configure the I2C Error ISR
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;	//The I2C1 triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//High Pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;	//Highest group priority
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Enabling interrupt from USART1 - bluetooth commands, e.g. enter bootloader*/
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	//Usart Tx/Rx triggered interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//High pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;	//Third highest group - above the dma
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	/* Enabling interrupt from USART3 - SP1ML command interface*/
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	//Usart Tx/Rx triggered interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//High pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;	//Fourth highest group - above the dma
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	/* Enable the DRDY interrupt pin, used to signal data ready from the ECG front end */
	EXTI_InitTypeDef   EXTI_InitStructure;
	EXTI_DeInit();						//Note that this kills all EXTI settings!
	/* Connect EXTI7 Line to PB.7 pin - DRDY*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);
	/* Configure EXTI7 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	/* Enable and set EXTI7 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;	//The DRDY triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Lower pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;	//second to lowest group priority - data comes in at only 250hz, so plenty of time
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	/* Enable and set EXTI Interrupt to second to lowest, this is the CTS from the SP1ML SPIRIT1 module */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;	//The CTS triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Lower pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x04;
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);//CTS
	EXTI_InitStructure.EXTI_Line = EXTI_Line15;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//Trigger any time that the level changes
	EXTI_Init(&EXTI_InitStructure);
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure on/off pin interrupt
  * @param  None
  * @retval None
  * This initialiser function assumes the clocks and gpio have been configured
  */
void EXTI_ONOFF_EN(void) {
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	//EXTI_DeInit();
	/* Connect EXTI0 Line to PA.0 pin - WKUP*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

	/* Configure EXTI0 line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	if(USB_SOURCE==bootsource)				//If we booted from USB, disconnect gives -ive pulse
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	else
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;	//The WKUP triggered interrupt	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//Lower pre-emption priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x07;	//lowest group priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/**
  * @brief  This function configures the systick timer to 100hz overflow. This 100hz interrupt will only be used for low level housekeeping
  * @param  None
  * @retval None
  */
void SysTick_Configuration(void) {
	RCC_HCLKConfig(RCC_SYSCLK_Div1);			//CLK the periferal - configure the AHB clk
	SysTick_Config(90000);					//SYSTICK at 100Hz - this function also enables the interrupt
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);   //SYSTICK AHB1/8
}

/**
  * @brief  This function handles External line 0 interrupt request.- WKUP/Button press ISR
  * @param  None
  * @retval None
  */
__attribute__((externally_visible)) void EXTI0_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
		if(USB_SOURCE!=bootsource && GET_VBUS_STATE) {	//Interrupt due to USB insertion - reset to usb mode
                        Shutdown_System=USB_INSERTED;		//Request a software reset of the system - USB inserted whilst running
		}
		if(USB_SOURCE==bootsource) {
			if(file_opened) 
				shutdown_filesystem(1,file_opened);//This should not happen
			red_flash();				//Flash red led - provides some debouncing on jack removal
			shutdown();				//Shuts down - only wakes up on power pin i.e. WKUP
		}
		/*Called Code goes here*/
		Button_hold_tim=BUTTON_TURNOFF_TIME;
		flashCodeEnabled=0;
		RED_LED_ON;					//Red LED is used to indicate successful button press to the user
	}
}

/**
  * @brief  This function handles ADC1-2 interrupt requests.- Should only be from the analog watchdog
  * @param  None
  * @retval None
  */
__attribute__((externally_visible)) void ADC1_2_IRQHandler(void) {
	if(ADC_GetITStatus(ADC2, ADC_IT_AWD))			//Analogue watchdog was triggered
	{
		Shutdown_System=LOW_BATTERY;			//Shutdown to save battery
		ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);
		ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC);
		ADC_ClearITPendingBit(ADC2, ADC_IT_AWD);	//make sure flags are clear
	}
	if(ADC_GetITStatus(ADC1, ADC_IT_AWD)) {			//Analogue watchdog ADC1 was triggered, should not happen
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);	//None of these should ever happen, but best to be safe
		ADC_ClearITPendingBit(ADC1, ADC_IT_AWD);	//make sure flags are clear
	}
}

/**
  * @brief  This function handles EXTI15 interrupt requests - the SP1ML CTS
  * @param  None
  * @retval None
  */
__attribute__((externally_visible)) void EXTI15_10_IRQHandler(void) {
	if(EXTI_GetITStatus(EXTI_Line15) != RESET) {		//PA15 is the CTS pin, low == not clear to send, high == clear to send
		EXTI_ClearITPendingBit(EXTI_Line15);
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)) 	//Measure the level, if level is high disable interrupt
			USART3->CR1 &=~(1<<7);			/*Disable the TXE interrupt on USART1*/	
		else {
			if(anything_in_buff(&Usart3_tx_buff) && !SP1ML_withold)	//Enable the interrupt if there is anything in the buffer to be sent, and not blocked 
				USART3->CR1 |=(1<<7);
		}
	}
}


/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler - runs at 100hz.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((externally_visible)) void SysTick_Handler(void)
{
	static uint32_t Last_Button_Press;			//Holds the timestamp for the previous button press
	static uint8_t System_state_counter;			//Holds the system state counter
	//FatFS timer function
	disk_timerproc();
	//Incr the system uptime
	Millis+=10;
	if(ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC)) {		//We have adc2 converted data from the injected channels
		ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);		//Clear the flag
		Battery_Voltage=((float)ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_1)/(SAMPLING_FACTOR));
	}
	ADC_SoftwareStartInjectedConvCmd(ADC2, ENABLE);		//Trigger the injected channel group
	//Now process the control button functions
	if(Button_hold_tim ) {					//If a button press generated timer has been triggered
		if(GET_BUTTON) {				//Button hold turns off the device
			if(Button_hold_tim == 1) {
                                Shutdown_System=BUTTON_TURNOFF;//Request turn off of logger after closing any open files
			}
			else {
				Button_hold_tim--;		//Button_hold_tim decreases to 1, then stays at 1 until device turns off
			}
		}
		else {						//Button released - this can only ever run once per press
			RED_LED_OFF;				//Turn off the red LED - used to indicate button press to user
			flashCodeEnabled=1;
			if(Button_hold_tim<BUTTON_DEBOUNCE) {	//The button has to be held down for longer than the debounce period
				Last_Button_Press=Millis;
				if(++System_state_counter>=SYSTEM_STATES)
					System_state_counter=0;//The system can only have a limited number of states
			}
			Button_hold_tim=0;			//Reset the timer here - button_hold_tim value of zero indicates no press
		}
	}
	if(Last_Button_Press&&(Millis-Last_Button_Press>BUTTON_MULTIPRESS_TIMEOUT)&&!Button_hold_tim) {//Last press timed out and button is not pressed
		if(!(System_state_Global&0x80))			//The main code has unlocked the global using the bit flag - as it has processed
			System_state_Global=0x80|System_state_counter;//The previous state update
		System_state_counter=0;				//Reset state counter here
		Last_Button_Press=0;				//Reset the last button press timestamp, as the is no button press in play
		Button_hold_tim=0;                              //Reset the Button_hold_tim too as we are no longer checking
	}
	// flash status code handler
	if (flashCodeEnabled)
	{
	    int8_t sequencePosn=((Millis/200)%16);
	    GPIO_WriteBit(GPIOB,RED,(flashCode&(1<<sequencePosn))!=0); // set state of RED LED
	}
	//if(Millis-Last_WDT<500)					//Less than 500ms remaining until a reset occurs
	//	prvGetRegistersFromStack( __get_MSP, 5);	//Cause==5 means a hang was detected - gives a stacktrace
}

//Included interrupts from ST um0424 mass storage example
#ifndef STM32F10X_CL
/*******************************************************************************
* Function Name  : USB_HP_CAN1_TX_IRQHandler
* Description    : This function handles USB High Priority or CAN TX interrupts requests
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((externally_visible)) void USB_HP_CAN1_TX_IRQHandler(void)
{
  CTR_HP();
}

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((externally_visible)) void USB_LP_CAN1_RX0_IRQHandler(void)
{
  USB_Istr();
}
#endif /* STM32F10X_CL */

#if defined(STM32F10X_HD) || defined(STM32F10X_XL) 
/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((externally_visible)) void SDIO_IRQHandler(void)
{ 
  /* Process All SDIO Interrupt Sources */
  SD_ProcessIRQSrc();
  
}
#endif /* STM32F10X_HD | STM32F10X_XL*/

#ifdef STM32F10X_CL
/*******************************************************************************
* Function Name  : OTG_FS_IRQHandler
* Description    : This function handles USB-On-The-Go FS global interrupt request.
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
__attribute__((externally_visible)) void OTG_FS_IRQHandler(void)
{
  STM32_PCD_OTG_ISR_Handler(); 
}
#endif /* STM32F10X_CL */


__attribute__((externally_visible)) void NMIException(void) {while(1);}
__attribute__((externally_visible)) void HardFaultException(void) {while(1);}
__attribute__((externally_visible)) void MemManageException(void) {while(1);}
__attribute__((externally_visible)) void BusFaultException(void) {while(1);}
__attribute__((externally_visible)) void UsageFaultException(void) {while(1);}
