/**
  ******************************************************************************
  * @file    RAM2UART.c
  * @author  Ri-Sheng Chen
  * @brief   This file is a simple DMA example
  ******************************************************************************
  * @attention
  *    1. Transmit the data stored in SRAM to USART3 by using DMA1
  *       when the user botton (PORTC 13) is pressed
  *    2. The interrupt generated when the user botton is pressed
  *    3. Read data from USART3 by using terminal in PC
  **/

#include <stdio.h>
#include <string.h>
#include "myusart.h"
#include "stm32f303ze_gpio_driver.h"
#include "stm32f303ze_dma_driver.h"

DMA1_Handle_t DMA_Handler;
char *dma_data = "Use DMA success !!\r\n";

void user_botton_set(void)
{
    GPIO_Handle_t botton;
    botton.pGPIOx = GPIOC;                                  /* PORTC */
    botton.GPIO_PINCFG.GPIO_PinNumber   = GPIO_PIN_NO_13;   /* Pin NUmber: 13 */
    botton.GPIO_PINCFG.GPIO_PinMode     = GPIO_MODE_IT_RT;  /* Pin Mode: Interrupt with rising trigger */
	botton.GPIO_PINCFG.GPIO_PinOType    = GPIO_OTYPE_PP;    /* Output Type: Push-Pull */
	botton.GPIO_PINCFG.GPIO_PinSpeed    = GPIO_OSPEED_LOW;  /* Speed: LOW */
	botton.GPIO_PINCFG.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;     /* Pull-up Pull-down Control */
    GPIO_PeriClockControl(botton.pGPIOx, ENABLE);
    GPIO_Init(&botton);
}

void user_botton_interrupt_set(void)
{
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
}

void DMA1_init(void)
{
    DMA_Handler.pDMAx = DMA1;
    DMA_Handler.DMA_CFG.DMA_ChannelNumber = DMA_CHANNEL2;
    DMA_Handler.DMA_CFG.DMA_PriorityLevel = DMA_PRIORITY_LOW;
    DMA_Handler.DMA_CFG.DMA_MemorySize = DMA_MSIZE_BITS_8;
    DMA_Handler.DMA_CFG.DMA_PeripheralSize = DMA_PSIZE_BITS_8;
    DMA_Handler.DMA_CFG.DMA_IncrementMode = DMA_ONLY_MINC;
    DMA_Handler.DMA_CFG.DMA_TransferMode = DMA_M2P;
    DMA_Handler.DMA_CFG.DMA_DataSize = strlen(dma_data);
    DMA_Handler.DMA_CFG.DMA_PeripheralAddr = (uint32_t)&USART3_TDR;
    DMA_Handler.DMA_CFG.DMA_MemoryAddr = (uint32_t)dma_data;

    DMA1_PCLK_EN();
    DMA1_Init(&DMA_Handler);
    DMA1_Data_Size_Set(&DMA_Handler);
    DMA1_Peripheral_Addr_Set(&DMA_Handler);
    DMA1_Memory_Addr_Set(&DMA_Handler);
    DMA1_CHANNEL_EN(DMA_Handler.DMA_CFG.DMA_ChannelNumber);
}

int main(void)
{
    MYUSART_Init();
    user_botton_set();
    user_botton_interrupt_set();
    while (1) {
        DMA1_init();
    }
    return 0;
}

void EXTI15_10_IRQHandler(void)
{
    for (int i = 0; i < 60000; i++);
    // send USART3 TX request to DMA1
    USART3_CR3 |= (1 << 7);

    // wait for transfer complete
    while ((DMA_Handler.pDMAx->ISR >> 5) & 0x1) {
        USART3_CR3 &= ~(1 << 7);
        DMA_Handler.pDMAx->IFCR |= (1 << 4);
        DMA_Handler.pDMAx->IFCR &= ~(1 << 4);
    }

    DMA1_CHANNEL_DI(DMA_Handler.DMA_CFG.DMA_ChannelNumber);
    for (int i = 0; i < 60000; i++);
    GPIO_IRQHandling(GPIO_PIN_NO_13);
}