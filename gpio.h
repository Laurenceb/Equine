#include "stm32f10x.h"

//PCB revision
#define PCB 1

#define RED		GPIO_Pin_5
#define GREEN		GPIO_Pin_6

#define WKUP		GPIO_Pin_0
#define PWREN		GPIO_Pin_4
#define CHARGER_EN	GPIO_Pin_8
#define SD_SEL_PIN	GPIO_Pin_4

#define GET_BUTTON	GPIO_ReadInputDataBit(GPIOA,WKUP)

#define VBUS_DETECT	GPIO_Pin_2

#define USB_SOURCE	0x01

#define GREEN_LED_ON	GPIO_WriteBit(GPIOB,GREEN,Bit_SET)
#define GREEN_LED_OFF	GPIO_WriteBit(GPIOB,GREEN,Bit_RESET)
#define RED_LED_ON	GPIO_WriteBit(GPIOB,RED,Bit_SET)
#define RED_LED_OFF	GPIO_WriteBit(GPIOB,RED,Bit_RESET)

#define GET_CHRG_STATE  GPIO_ReadInputDataBit(GPIOA,CHARGER_EN)
#define CHRG_ON		GPIO_WriteBit(GPIOA,CHARGER_EN,Bit_SET)
#define CHRG_OFF	GPIO_WriteBit(GPIOA,CHARGER_EN,Bit_RESET)

#define GET_VBUS_STATE	GPIO_ReadInputDataBit(GPIOB,VBUS_DETECT)
#define GET_PWR_STATE	GPIO_ReadInputDataBit(GPIOA,WKUP)

#define GET_LSM9DS1_DTRD GPIO_ReadInputDataBit(GPIOC,15)

//I2C1 on pins 6 and 7 - configured in i2c_int.h
#define I2C1_SCL	GPIO_Pin_8
#define I2C1_SDA	GPIO_Pin_9

extern uint8_t bootsource;

void setup_gpio(void);
void switch_leds_on(void);
void switch_leds_off(void);
void red_flash(void);
uint8_t get_wkup(void);
