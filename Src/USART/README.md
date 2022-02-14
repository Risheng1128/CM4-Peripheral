# USART Implementation
###### tags: `ARM Cortex-M` `ARM Peripheral`

## Exercise 1: USART send data to arduino
- 目標: Write a program to send some message over UART from STM32 board to Arduino board. The Arduino board will display the message.
  - Baudrate: 115200 bps
  - Frame format: 1 stop bits, 8bits, no parity


## Exercise 2: Communicating with PC over UART
- 目標: 利用USART完成printf及scanf
  - Baudrate: 38400 bps
  - Frame format: 1 stop bits, 8bits, no parity
- 結果
  ![](https://i.imgur.com/8FUKL75.png)

## Exercise 3: Interrupt
- 目標: Write a program for stm32 board which transmits different messages to the Arduino board over UART communication
  - For every message STM32 board sends, arduino code will change the case of alphabets(lower case to upper case and vice versa) and sends message back to the stm32 board
  - The stm32 board should capture the reply from the arduino board and display using semi hosting





