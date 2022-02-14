# GPIO Implementation
###### tags: `ARM Cortex-M` `ARM Peripheral`

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

## LED toggling configuration
- Case1: Use push pull configuration for the output pin
  - 目的: 將三個LED燈分別閃爍
  - PB0 設為 GPIO output  (綠燈)
  - PB7 設為 GPIO output  (藍燈)
  - PB14 設為 GPIO output (紅燈)

- Case2: Use open drain configuration for the output pin
  
## Handling on board LED and Button
- On Board Button
  1. Button位置: PC13
  2. 不按Button: PC13為LOW
  3. 按下Button: PC13為HIGH
  
  ![](https://i.imgur.com/QKYfSSS.png)
  
  ![](https://i.imgur.com/1PLwvZ7.png)
  
## GPIO Interrupt
- 目標: Toggle the on board LED whenever the on board button is pressed, and the button uses the falling edge of the interrupt


