# GPIO Implementation
###### tags: `ARM Cortex-M`

## 使用 Register 使LED閃爍
- 目的: Red LED (B14) 閃爍

- Register
  1. RCC_AHBENR: 用來開啟 PORTB 的 Timer (Set "IOPB EN" "1")
     ![](https://i.imgur.com/u9Jw7Iz.png)
     需查看 RCC 位址: 0x40021000
     ![](https://i.imgur.com/VlPKK8l.png)

  2. GPIOB: 位址0x48000400
     ![](https://i.imgur.com/GqAstIZ.png)
     ![](https://i.imgur.com/cVRcZ6v.png)

  3. GPIOB_ODR: 位址0x48000414
     ![](https://i.imgur.com/1WTici7.png)

- Code
  ```C=
  #include <stdio.h>
  #include <stdint.h>

  int main(void){
	  uint32_t* RCC_AHBENR = (uint32_t*)0x40021014;
	  uint32_t* GPIOB_MODER = (uint32_t*)0x48000400;
	  uint32_t* GPIOB_ODR = (uint32_t*)0x48000414;
	
   	  *RCC_AHBENR |= (1 << 18);  /* enable GPIOB timer */
	  *GPIOB_MODER |= (1 << 28); /* Output mode */
	
	  while(1){
		  *GPIOB_ODR |= (1 << 14); /* PB14 Output */ 
		  for(int i = 0; i < 1000000; i++);
		  *GPIOB_ODR &= ~(1 << 14); /* PB14 Output */
		  for(int i = 0; i < 1000000; i++);
	  }
	  return 0;
  }
  ```

## LED toggling configuration
- Case1: Use push pull configuration for the output pin
  - 目的: 將三個LED燈分別閃爍(使用自製函式庫)
  - PB0 設為 GPIO output  (綠燈)
  - PB7 設為 GPIO output  (藍燈)
  - PB14 設為 GPIO output (紅燈)
  - Code
  ```c=
  #include "../Drivers/Inc/stm32f303ze_gpio_driver.h"
  int main(void){
    GPIO_Handle_t RED, GREEN, BLUE;
	/* RED SET */
	RED.pGPIOx = GPIOB;								      /* PORTB */
	RED.GPIO_PINCFG.GPIO_PinNumber   = GPIO_PIN_NO_14;    /* Pin NUmber: 14 */
	RED.GPIO_PINCFG.GPIO_PinMode     = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	RED.GPIO_PINCFG.GPIO_PinOType    = GPIO_OTYPE_PP;     /* Output Type: Push-Pull */
	RED.GPIO_PINCFG.GPIO_PinSpeed    = GPIO_OSPEED_LOW;   /* Speed: LOW */
	RED.GPIO_PINCFG.GPIO_PinPuPdControl = GPIO_NO_PUPD;   /* Pull-up Pull-down Control */
	/* GREEN SET */
	GREEN.pGPIOx = GPIOB;								  /* PORTB */
	GREEN.GPIO_PINCFG.GPIO_PinNumber = GPIO_PIN_NO_0;     /* Pin NUmber: 0 */
	GREEN.GPIO_PINCFG.GPIO_PinMode   = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	GREEN.GPIO_PINCFG.GPIO_PinOType  = GPIO_OTYPE_PP;     /* Output Type: Push-Pull */
	GREEN.GPIO_PINCFG.GPIO_PinSpeed  = GPIO_OSPEED_LOW;   /* Speed: LOW */
	GREEN.GPIO_PINCFG.GPIO_PinPuPdControl = GPIO_NO_PUPD; /* Pull-up Pull-down Control */
	/* BLUE SET */
	BLUE.pGPIOx = GPIOB;								  /* PORTB */
	BLUE.GPIO_PINCFG.GPIO_PinNumber  = GPIO_PIN_NO_7;     /* Pin NUmber: 7 */
	BLUE.GPIO_PINCFG.GPIO_PinMode    = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	BLUE.GPIO_PINCFG.GPIO_PinOType   = GPIO_OTYPE_PP;     /* Output Type: Push-Pull */
	BLUE.GPIO_PINCFG.GPIO_PinSpeed   = GPIO_OSPEED_LOW;   /* Speed: LOW */
	BLUE.GPIO_PINCFG.GPIO_PinPuPdControl = GPIO_NO_PUPD;  /* Pull-up Pull-down Control */
	
	GPIO_PeriClockControl(RED.pGPIOx, ENABLE);
	GPIO_Init(&RED);
	GPIO_PeriClockControl(GREEN.pGPIOx, ENABLE);
	GPIO_Init(&GREEN);
	GPIO_PeriClockControl(BLUE.pGPIOx, ENABLE);
	GPIO_Init(&BLUE);
	while (1) {
		GPIO_ToggleOutputPin(RED.pGPIOx, RED.GPIO_PINCFG.GPIO_PinNumber); /* Red Open */
		for(int i = 0; i < 100000; i++);
		GPIO_ToggleOutputPin(RED.pGPIOx, RED.GPIO_PINCFG.GPIO_PinNumber); /* Red Close */
		GPIO_ToggleOutputPin(GREEN.pGPIOx, GREEN.GPIO_PINCFG.GPIO_PinNumber); /* Green Open */
		for(int i = 0; i < 100000; i++);
		GPIO_ToggleOutputPin(GREEN.pGPIOx, GREEN.GPIO_PINCFG.GPIO_PinNumber); /* Green Close */
		GPIO_ToggleOutputPin(BLUE.pGPIOx, BLUE.GPIO_PINCFG.GPIO_PinNumber); /* Blue Open */
		for(int i = 0; i < 100000; i++);
		GPIO_ToggleOutputPin(BLUE.pGPIOx, BLUE.GPIO_PINCFG.GPIO_PinNumber); /* Blue Close */
	}
	return 0;
  }
  ```

- Case2: Use open drain configuration for the output pin
  ```c=
  #include "../Drivers/Inc/stm32f303ze_gpio_driver.h"
    
  int main(void){
    GPIO_Handle_t RED, GREEN, BLUE;
	/* RED SET */
	RED.pGPIOx = GPIOB;								      /* PORTB */
	RED.GPIO_PINCFG.GPIO_PinNumber   = GPIO_PIN_NO_14;    /* Pin NUmber: 14 */
	RED.GPIO_PINCFG.GPIO_PinMode     = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	RED.GPIO_PINCFG.GPIO_PinOType    = GPIO_OTYPE_OD;     /* Output Type: Push-Pull */
	RED.GPIO_PINCFG.GPIO_PinSpeed    = GPIO_OSPEED_LOW;   /* Speed: LOW */
	RED.GPIO_PINCFG.GPIO_PinPuPdControl = GPIO_PU;   /* Pull-up Pull-down Control */
	/* GREEN SET */
	GREEN.pGPIOx = GPIOB;								  /* PORTB */
	GREEN.GPIO_PINCFG.GPIO_PinNumber = GPIO_PIN_NO_0;     /* Pin NUmber: 0 */
	GREEN.GPIO_PINCFG.GPIO_PinMode   = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	GREEN.GPIO_PINCFG.GPIO_PinOType  = GPIO_OTYPE_OD;     /* Output Type: Push-Pull */
	GREEN.GPIO_PINCFG.GPIO_PinSpeed  = GPIO_OSPEED_LOW;   /* Speed: LOW */
	GREEN.GPIO_PINCFG.GPIO_PinPuPdControl = GPIO_PU; /* Pull-up Pull-down Control */
	/* BLUE SET */
	BLUE.pGPIOx = GPIOB;								  /* PORTB */
	BLUE.GPIO_PINCFG.GPIO_PinNumber  = GPIO_PIN_NO_7;     /* Pin NUmber: 7 */
	BLUE.GPIO_PINCFG.GPIO_PinMode    = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	BLUE.GPIO_PINCFG.GPIO_PinOType   = GPIO_OTYPE_OD;     /* Output Type: Push-Pull */
	BLUE.GPIO_PINCFG.GPIO_PinSpeed   = GPIO_OSPEED_LOW;   /* Speed: LOW */
	BLUE.GPIO_PINCFG.GPIO_PinPuPdControl = GPIO_PU;  /* Pull-up Pull-down Control */
	
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
  ```
  
## Handling on board LED and Button
- On Board Button
  1. Button位置: PC13
  2. 不按Button: PC13為LOW
  3. 按下Button: PC13為HIGH
  
  ![](https://i.imgur.com/QKYfSSS.png)
  ![](https://i.imgur.com/1PLwvZ7.png)

- Case: Write a program to toggle the on board LED whenever the on board button is pressed
  ```c=
  #include "../Drivers/Inc/stm32f303ze_gpio_driver.h"

  int main(void){
    GPIO_Handle_t GREEN, Button;
	/* GREEN SET */ㄏ
	GREEN.pGPIOx = GPIOB;								  /* PORTB */
	GREEN.GPIO_PINCFG.GPIO_PinNumber = GPIO_PIN_NO_0;     /* Pin NUmber: 0 */
	GREEN.GPIO_PINCFG.GPIO_PinMode   = GPIO_MODE_OUTPUT;  /* Pin Mode: OUTPUT */
	GREEN.GPIO_PINCFG.GPIO_PinOType  = GPIO_OTYPE_PP;     /* Output Type: Push-Pull */
	GREEN.GPIO_PINCFG.GPIO_PinSpeed  = GPIO_OSPEED_LOW;   /* Speed: LOW */
	GREEN.GPIO_PINCFG.GPIO_PinPuPdControl = GPIO_NO_PUPD; /* Pull-up Pull-down Control */
    /* Button SET */
    Button.pGPIOx = GPIOC;                                  /* PORTC */
    Button.GPIO_PINCFG.GPIO_PinNumber = GPIO_PIN_NO_13;     /* Pin NUmber: 13 */
    Button.GPIO_PINCFG.GPIO_PinMode     = GPIO_MODE_INPUT;  /* Pin Mode: INPUT */
	Button.GPIO_PINCFG.GPIO_PinOType    = GPIO_OTYPE_PP;    /* Output Type: Push-Pull */
	Button.GPIO_PINCFG.GPIO_PinSpeed    = GPIO_OSPEED_LOW;  /* Speed: LOW */
	Button.GPIO_PINCFG.GPIO_PinPuPdControl = GPIO_NO_PUPD;  /* Pull-up Pull-down Control */

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
  ```
  
## GPIO Interrupt
- case: Toggle the on board LED whenever the on board button is pressed, and the button uses the falling edge of the interrupt
  ```c=
  #include "../Drivers/Inc/stm32f303ze_gpio_driver.h"
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
	GREEN.GPIO_PINCFG.GPIO_PinPuPdControl = GPIO_NO_PUPD; /* Pull-up Pull-down Control */
    GPIO_PeriClockControl(GREEN.pGPIOx, ENABLE);
	GPIO_Init(&GREEN);
	
	/* Button SET */
    Button.pGPIOx = GPIOC;                                  /* PORTC */
    Button.GPIO_PINCFG.GPIO_PinNumber = GPIO_PIN_NO_13;     /* Pin NUmber: 13 */
    Button.GPIO_PINCFG.GPIO_PinMode   = GPIO_MODE_IT_FT;    /* Pin Mode: INPUT Falling edge */
	Button.GPIO_PINCFG.GPIO_PinOType  = GPIO_OTYPE_PP;      /* Output Type: Push-Pull */
	Button.GPIO_PINCFG.GPIO_PinSpeed  = GPIO_OSPEED_LOW;    /* Speed: LOW */
	Button.GPIO_PINCFG.GPIO_PinPuPdControl = GPIO_NO_PUPD;       /* Pull-up Pull-down Control */
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
    for(int i = 0; i < 80000; i++);
	GPIO_IRQHandling(GPIO_PIN_NO_13);
  }
  ```


