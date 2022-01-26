/**
  ******************************************************************************
  * @file    LED_Interrupt.c
  * @author  Ri-Sheng Chen
  * @brief   This file is a on board LED and Button Example
  ******************************************************************************
  * @attention
  *    1. Setup PB0 GPIO output  (Green)
  *    2. Setup PC13 GPIO input (Button on board)
  *    3. Toggle the on board LED whenever the on board button is pressed
  *    4. The button uses the falling edge of the interrupt
  **/

#include "stm32f303ze_gpio_driver.h"
#include <string.h>

int main(void) {
	GPIO_Handle_t GREEN, Button;
	memset(&GREEN, 0, sizeof(GPIO_Handle_t));
	memset(&Button, 0, sizeof(GPIO_Handle_t));

	/* GREEN SET */
	GREEN.pGPIOx = GPIOB;								  /* PORTB */
	GREEN.GPIO_PINCFG.GPIO_PinNumber = GPIO_PIN_NO_0;     /* Pin NUmber: 0 */
	GREEN.GPIO_PINCFG.GPIO_PinMode   = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	GREEN.GPIO_PINCFG.GPIO_PinOType  = GPIO_OTYPE_PP;     /* Output Type: Push-Pull */
	GREEN.GPIO_PINCFG.GPIO_PinSpeed  = GPIO_OSPEED_LOW;   /* Speed: LOW */
	GREEN.GPIO_PINCFG.GPIO_PinPuPdCtrl = GPIO_NO_PUPD; 	  /* Pull-up Pull-down Control */
    GPIO_PeriClockControl(GREEN.pGPIOx, ENABLE);
	GPIO_Init(&GREEN);

	/* Button SET */
    Button.pGPIOx = GPIOC;                                  /* PORTC */
    Button.GPIO_PINCFG.GPIO_PinNumber = GPIO_PIN_NO_13;     /* Pin NUmber: 13 */
    Button.GPIO_PINCFG.GPIO_PinMode   = GPIO_MODE_IT_FT;    /* Pin Mode: INPUT Falling edge */
	Button.GPIO_PINCFG.GPIO_PinOType  = GPIO_OTYPE_PP;      /* Output Type: Push-Pull */
	Button.GPIO_PINCFG.GPIO_PinSpeed  = GPIO_OSPEED_LOW;    /* Speed: LOW */
	Button.GPIO_PINCFG.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;  /* Pull-up Pull-down Control */
    GPIO_PeriClockControl(Button.pGPIOx, ENABLE);
	GPIO_Init(&Button);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI9);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
	while (1);
	return 0;
}

void EXTI15_10_IRQHandler(void) {
	for(int i = 0; i < 80000; i++);
	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_0);
	for(int i = 0; i < 80000; i++); /* Delay */
	GPIO_IRQHandling(GPIO_PIN_NO_13);
}