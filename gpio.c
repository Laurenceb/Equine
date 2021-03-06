#include "gpio.h"
#include "pwr.h"
#include "main.h"
#include "Util/delay.h"

uint8_t bootsource;

void setup_gpio(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	//enable the clocks 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//GPIO/AFIO clks
	setuppwr();				//configure power control
	disable_pin();				//disable WKUP pin functionality
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//disable JTAG but keep SWD lines
	//Configure and read the Charger_EN/VBus pin - this has a pullup to V_USB, so if it reads 1 we booted off usb so setup USB detatch isr
	GPIO_InitStructure.GPIO_Pin = VBUS_DETECT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init( GPIOB, &GPIO_InitStructure );/* configure pin 2 as input*/
	for(uint16_t n=1;n;n++) {		//USB insertion can be really messy, so loop to detect anything on chrg pin over a few milliseconds
		if(GET_VBUS_STATE) {		//We booted from USB
			bootsource=USB_SOURCE;	//so we know for reference later
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;//reset the pin to an open drain output
			GPIO_InitStructure.GPIO_Pin = CHARGER_EN;
			GPIO_Init( GPIOA, &GPIO_InitStructure );// The TTDM pin on later boards				
			CHRG_ON;		//default to charger enabled
			n=0;
			break;
		}
	}
	//Configure the io pins
	//Pull up the SD CS etc pins
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//pullup
	/* configure SDSEL pin as input pull up until the SD driver is intialised*/
	//Pull up all the SD SPI lines until the bus is intialized - SD spec says MISO, (CLK?), and MOSI should be pulled up at poweron
	GPIO_InitStructure.GPIO_Pin = SD_SEL_PIN | /*GPIO_Pin_5 |*/ GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init( GPIOA, &GPIO_InitStructure );		
	//LEDS
	GPIO_InitStructure.GPIO_Pin = RED|GREEN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOB, &GPIO_InitStructure );/* configure pins 11 and 12 as output*/
	//Power button
	GPIO_InitStructure.GPIO_Pin = WKUP;
	if(USB_SOURCE==bootsource)		//Configure for turnoff on usb removal or pwr button
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//pullup
	else
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//pulldown
	GPIO_Init( GPIOA, &GPIO_InitStructure );/* configure WKUP pin as input pull down/up for button*/
	//Power supply enable
	GPIO_InitStructure.GPIO_Pin = PWREN_;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//pushpull
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	if(!(CoreDebug->DHCSR&0x00000001)) {
		GPIO_WriteBit(GPIOB,PWREN_,Bit_RESET);//Make sure power enabled
		Delay(50000);
	}
	GPIO_WriteBit(GPIOB,PWREN_,Bit_SET);	//Make sure power enabled
	//Configure the ADC input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init( GPIOA, &GPIO_InitStructure );
	//Configure the ADS1298 DRDY pin (B.7) to input pull up, to avoid any noise during device resets
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init( GPIOB, &GPIO_InitStructure );
	//Configure the accel/gyro/magno data ready (C13) to input pull up. Hopefully the pull up is strong enough to work?
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init( GPIOC, &GPIO_InitStructure );
	//Configure anything else here
}

void switch_leds_on(void)
{
	if(USB_SOURCE==bootsource)
		GPIO_WriteBit(GPIOB,RED,Bit_SET);
	else
		GPIO_WriteBit(GPIOB,GREEN,Bit_SET);
}

void switch_leds_off(void)
{
	if(USB_SOURCE==bootsource)
		GPIO_WriteBit(GPIOB,RED,Bit_RESET);
	else
		GPIO_WriteBit(GPIOB,GREEN,Bit_RESET);
}

void red_flash(void)
{
	GPIO_WriteBit(GPIOB,RED,Bit_SET);
	Delay(400000);
	GPIO_WriteBit(GPIOB,RED,Bit_RESET);
}

uint8_t get_wkup()
{
	return GPIO_ReadInputDataBit(GPIOA,WKUP);
}
