/**
  ******************************************************************************
  * @file    LED_toggle.c
  * @author  Ri-Sheng Chen
  * @brief   This file is a GPIO example
  ******************************************************************************
  * @attention
  *    1. Setup PB0 GPIO output  (Green)
  *    2. Setup PB7 GPIO output  (Blue)
  *    3. Setup PB14 GPIO output (Red)
  *    4. The light order ot LED : RED -> GREEN -> BLUE
  **/
 
#include "stm32f303ze_gpio_driver.h"
int main(void)
{
	GPIO_Handle_t RED, GREEN, BLUE;
	/* RED SET */
	RED.pGPIOx = GPIOB;								      /* PORTB */
	RED.GPIO_PINCFG.GPIO_PinNumber   = GPIO_PIN_NO_14;    /* Pin NUmber: 14 */
	RED.GPIO_PINCFG.GPIO_PinMode     = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	RED.GPIO_PINCFG.GPIO_PinOType    = GPIO_OTYPE_PP;     /* Output Type: Push-Pull */
	RED.GPIO_PINCFG.GPIO_PinSpeed    = GPIO_OSPEED_LOW;   /* Speed: LOW */
	RED.GPIO_PINCFG.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;   /* Pull-up Pull-down Control */
	/* GREEN SET */
	GREEN.pGPIOx = GPIOB;								  /* PORTB */
	GREEN.GPIO_PINCFG.GPIO_PinNumber = GPIO_PIN_NO_0;     /* Pin NUmber: 0 */
	GREEN.GPIO_PINCFG.GPIO_PinMode   = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	GREEN.GPIO_PINCFG.GPIO_PinOType  = GPIO_OTYPE_PP;     /* Output Type: Push-Pull */
	GREEN.GPIO_PINCFG.GPIO_PinSpeed  = GPIO_OSPEED_LOW;   /* Speed: LOW */
	GREEN.GPIO_PINCFG.GPIO_PinPuPdCtrl = GPIO_NO_PUPD; /* Pull-up Pull-down Control */
	/* BLUE SET */
	BLUE.pGPIOx = GPIOB;								  /* PORTB */
	BLUE.GPIO_PINCFG.GPIO_PinNumber  = GPIO_PIN_NO_7;     /* Pin NUmber: 7 */
	BLUE.GPIO_PINCFG.GPIO_PinMode    = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	BLUE.GPIO_PINCFG.GPIO_PinOType   = GPIO_OTYPE_PP;     /* Output Type: Push-Pull */
	BLUE.GPIO_PINCFG.GPIO_PinSpeed   = GPIO_OSPEED_LOW;   /* Speed: LOW */
	BLUE.GPIO_PINCFG.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;  /* Pull-up Pull-down Control */
	
	GPIO_PeriClockControl(RED.pGPIOx, ENABLE);
	GPIO_Init(&RED);
	GPIO_PeriClockControl(GREEN.pGPIOx, ENABLE);
	GPIO_Init(&GREEN);
	GPIO_PeriClockControl(BLUE.pGPIOx, ENABLE);
	GPIO_Init(&BLUE);
	while (1) {
		GPIO_ToggleOutputPin(RED.pGPIOx, RED.GPIO_PINCFG.GPIO_PinNumber);     /* Red Open    */
		for(int i = 0; i < 100000; i++);
		GPIO_ToggleOutputPin(RED.pGPIOx, RED.GPIO_PINCFG.GPIO_PinNumber);     /* Red Close   */
		GPIO_ToggleOutputPin(GREEN.pGPIOx, GREEN.GPIO_PINCFG.GPIO_PinNumber); /* Green Open  */
		for(int i = 0; i < 100000; i++);
		GPIO_ToggleOutputPin(GREEN.pGPIOx, GREEN.GPIO_PINCFG.GPIO_PinNumber); /* Green Close */
		GPIO_ToggleOutputPin(BLUE.pGPIOx, BLUE.GPIO_PINCFG.GPIO_PinNumber);   /* Blue Open   */
		for(int i = 0; i < 100000; i++);
		GPIO_ToggleOutputPin(BLUE.pGPIOx, BLUE.GPIO_PINCFG.GPIO_PinNumber);   /* Blue Close  */
	}
	return 0;
}
