/**
  ******************************************************************************
  * @file    LED_button.c
  * @author  Ri-Sheng Chen
  * @brief   This file is a on board LED and Button Example
  ******************************************************************************
  * @attention
  *    1. Setup PB0 GPIO output  (Green)
  *    2. Setup PC13 GPIO input (Button on board)
  *    3. Toggle the on board LED whenever the on board button is pressed
  **/

#include "stm32f303ze_gpio_driver.h"

int main(void){
    GPIO_Handle_t GREEN, Button;
	/* GREEN SET */
	GREEN.pGPIOx = GPIOB;								  /* PORTB */
	GREEN.GPIO_PINCFG.GPIO_PinNumber = GPIO_PIN_NO_0;     /* Pin NUmber: 0 */
	GREEN.GPIO_PINCFG.GPIO_PinMode   = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	GREEN.GPIO_PINCFG.GPIO_PinOType  = GPIO_OTYPE_PP;     /* Output Type: Push-Pull */
	GREEN.GPIO_PINCFG.GPIO_PinSpeed  = GPIO_OSPEED_LOW;   /* Speed: LOW */
	GREEN.GPIO_PINCFG.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;    /* Pull-up Pull-down Control */
    /* Button SET */
    Button.pGPIOx = GPIOC;                                  /* PORTC */
    Button.GPIO_PINCFG.GPIO_PinNumber = GPIO_PIN_NO_13;     /* Pin NUmber: 13 */
    Button.GPIO_PINCFG.GPIO_PinMode     = GPIO_MODE_INPUT;  /* Pin Mode: INPUT */
	Button.GPIO_PINCFG.GPIO_PinOType    = GPIO_OTYPE_PP;    /* Output Type: Push-Pull */
	Button.GPIO_PINCFG.GPIO_PinSpeed    = GPIO_OSPEED_LOW;  /* Speed: LOW */
	Button.GPIO_PINCFG.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;     /* Pull-up Pull-down Control */

    GPIO_PeriClockControl(GREEN.pGPIOx, ENABLE);
	GPIO_Init(&GREEN);
    GPIO_PeriClockControl(Button.pGPIOx, ENABLE);
	GPIO_Init(&Button);
    while(1) {
        if(GPIO_ReadFromInputPin(Button.pGPIOx, Button.GPIO_PINCFG.GPIO_PinNumber)) {
            for(int i = 0; i < 250000; i++); /* Delay */
            GPIO_ToggleOutputPin(GREEN.pGPIOx, GREEN.GPIO_PINCFG.GPIO_PinNumber);
        }
    }
}