/**
  ******************************************************************************
  * @file    stm32f303ze_usart_driver.c
  * @author  Ri-Sheng Chen
  * @brief   This file contains the functions for the USART driver.
  ******************************************************************************
  **/

#include "stm32f303ze_usart_driver.h"

/********************************************************
 * @fn              - USART_Init
 * @brief           - This function initializes the USART_Handle_t structure
 * @param[in]       - Handle structure for a USART pin
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle)
{
    /* Enable Peripheral Clock */
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);
    /* UE (USART Enable) Set */
    pUSARTHandle->pUSARTx->CR1 |= 1 << 0;

    /** 
     * USART mode set 
     * Bit 2 RE & Bit 3 TE (CR1)
     */
    if (pUSARTHandle->USART_PINCFG.USART_Mode == USART_MODE_RX) {
        pUSARTHandle->pUSARTx->CR1 &= ~(1 << 3);            /* Disable Tx */
        pUSARTHandle->pUSARTx->CR1 |=   1 << 2;             /* Enable Rx */
    }
    else if (pUSARTHandle->USART_PINCFG.USART_Mode == USART_MODE_TX) {
        pUSARTHandle->pUSARTx->CR1 &= ~(1 << 2);            /* Disable Rx */
        pUSARTHandle->pUSARTx->CR1 |=   1 << 3;             /* Enable Tx */
    }
    else if (pUSARTHandle->USART_PINCFG.USART_Mode == USART_MODE_TXRX) {
        pUSARTHandle->pUSARTx->CR1 |= (1 << 2) | (1 << 3);  /* Enable Tx/Rx */
    }

    /** 
     * USART word length set
     * Bit 12 M0 & Bit 28 M1 (CR1)
     */
    if (pUSARTHandle->USART_PINCFG.USART_WordLength == USART_WORDLEN_7) {
        /* 1 Start bit, 7 data bits, n stop bits */
        pUSARTHandle->pUSARTx->CR1 &= ~(1 << 12);
        pUSARTHandle->pUSARTx->CR1 |=  (1 << 28);
    }
    else if (pUSARTHandle->USART_PINCFG.USART_WordLength == USART_WORDLEN_8) {
        /* 1 Start bit, 8 data bits, n stop bits */
        pUSARTHandle->pUSARTx->CR1 &= ~(1 << 12) & ~(1 << 28); 
    }
    else if (pUSARTHandle->USART_PINCFG.USART_WordLength == USART_WORDLEN_9) {
        /* 1 Start bit, 9 data bits, n stop bits */
        pUSARTHandle->pUSARTx->CR1 |=  (1 << 12);
        pUSARTHandle->pUSARTx->CR1 &= ~(1 << 28);
    }
    
    /** 
     * USART parity control set
     * Bit 9 PS & Bit 10 PCE (CR1)
     */
    pUSARTHandle->pUSARTx->CR1 &= ~(0x3 << 9);      /* clear */
    pUSARTHandle->pUSARTx->CR1 |= pUSARTHandle->USART_PINCFG.USART_ParityCtrl << 9;     /* set */

    /** 
     * USART stop bit set
     * bit 12/13 STOP (CR2)
     */
    pUSARTHandle->pUSARTx->CR2 &= ~(0x3 << 12);     /* clear */
    pUSARTHandle->pUSARTx->CR2 |= pUSARTHandle->USART_PINCFG.USART_StopBits << 12;      /* set */

    /** 
     * USART hardware flow control set
     * bit 8 RTSE & bit 9 CTSE (CR3)
     */
    pUSARTHandle->pUSARTx->CR3 &= ~(0x3 << 8);     /* clear */ 
    pUSARTHandle->pUSARTx->CR3 |= pUSARTHandle->USART_PINCFG.USART_HWFlowCtrl << 8;      /* set */

    /* Set Baud Rate */
    USART_SetBaudRate(pUSARTHandle); 
}

/********************************************************
 * @fn              - USART_Deinit
 * @brief           - This function reset the USART registers
 * @param[in]       - base address of the GPIO peripheral
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void USART_Deinit(USART_RegDef_t *pUSARTx)
{
    if (pUSARTx == USART1)
        USART1_REG_RST();
    else if (pUSARTx == USART2)
        USART2_REG_RST();
    else if (pUSARTx == USART3)
        USART3_REG_RST();
    else if (pUSARTx == UART4)
        UART4_REG_RST();
    else if (pUSARTx == UART5)
        UART5_REG_RST();
}

/********************************************************
 * @fn              - USART_PeriClockControl
 * @brief           - This function enables or disables the clock of the USART
 * @param[in]       - pUSARTx: base address of the GPIO peripheral
 *                  - EnorDi: ENABLE or DISABLE macros
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        if (pUSARTx == USART1)
            USART1_PCLK_EN();
        else if (pUSARTx == USART2)
            USART2_PCLK_EN();
        else if (pUSARTx == USART3)
            USART3_PCLK_EN();
        else if (pUSARTx == UART4)
            UART4_PCLK_EN();
        else if (pUSARTx == UART5)
            UART5_PCLK_EN();
    } else {
        if (pUSARTx == USART1)
            USART1_PCLK_DI();
        else if (pUSARTx == USART2)
            USART2_PCLK_DI();
        else if (pUSARTx == USART3)
            USART3_PCLK_DI();
        else if (pUSARTx == UART4)
            UART4_PCLK_DI();
        else if (pUSARTx == UART5)
            UART5_PCLK_DI();
    }
}

/********************************************************
 * @fn              - USART_SetBaudRate
 * @brief           - This function set the baud rate by USART_BRR
 * @param[in]       - Handle structure for a USART pin
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void USART_SetBaudRate(USART_Handle_t *pUSARTHandle)
{
    /* OVER8 in USART_CR1 bit 15 */
    if (!(pUSARTHandle->pUSARTx->CR1 & 0x8000)) { /* oversampling by 16 */ 
        pUSARTHandle->pUSARTx->BRR = F_CLK / pUSARTHandle->USART_PINCFG.USART_Baud;
    } else { /* oversampling by 16 */
        uint32_t USARTDIV = 2 * F_CLK / pUSARTHandle->USART_PINCFG.USART_Baud;
        uint8_t tmp1 = (USARTDIV & 0xF) >> 1;   /* BRR[2:0]  */
        uint16_t tmp2 = (USARTDIV & 0xFFF0);    /* BRR[15:3] */
        pUSARTHandle->pUSARTx->BRR = tmp1 | tmp2;
    }
}

/********************************************************
 * @fn              - USART_SendData
 * @brief           - USART send data
 * @param[in]       - pUSARTHandle: Handle structure for a USART pin
 *                  - pTxBuffer: Data Buffer
 *                  - Len: Data Length
 * @return          - none
 * @note            - data width 9 not done
 * 
 *********************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    for (int i = 0; i < Len; i++) {
        /* Waiting for the transmit data register empty (bit 7 TXE) */
        while (!(pUSARTHandle->pUSARTx->ISR & 0x80));
        
        if (pUSARTHandle->USART_PINCFG.USART_WordLength == USART_WORDLEN_7) {
            /* Load data to transmit data register */
            pUSARTHandle->pUSARTx->TDR = (*pTxBuffer++ & 0x7F);
        }
        else if(pUSARTHandle->USART_PINCFG.USART_WordLength == USART_WORDLEN_8) {
            /* Load data to transmit data register */
            pUSARTHandle->pUSARTx->TDR = (*pTxBuffer++ & 0xFF);
        }
        // else if(pUSARTHandle->USART_PINCFG.USART_WordLength == USART_WORDLEN_9) {
        //     pUSARTHandle->pUSARTx->TDR = (*pTxBuffer++ & 0x1FF);
            
        //     if(pUSARTHandle->USART_PINCFG.USART_ParityCtrl == USART_PARITY_DI) {
        //         /* No parity bit means that needs to load 2 bytes data to TDR */
        //         pTxBuffer++;
        //     }
        // }
        /* Waiting for the transmit complete (bit 6 TC) */
        while (!(pUSARTHandle->pUSARTx->ISR & 0x40));
    }
}

/********************************************************
 * @fn              - USART_ReceiveData
 * @brief           - USART receive data
 * @param[in]       - Handle structure for a USART pin
 * @return          - received data (uint8_t)
 * @note            - none
 * 
 *********************************************************/
uint8_t USART_ReceiveData(USART_Handle_t *pUSARTHandle)
{
    uint8_t data;

    /* Waiting for the transmit data transmit to USART_RDR register (USART_ISR bit 5 RXNE) */
    while (!(pUSARTHandle->pUSARTx->ISR & 0x20));

    if (pUSARTHandle->USART_PINCFG.USART_WordLength == USART_WORDLEN_7) {
        if (pUSARTHandle->USART_PINCFG.USART_ParityCtrl == USART_PARITY_DI)
            data = pUSARTHandle->pUSARTx->RDR;
        //else data = pUSARTHandle->pUSARTx->RDR;
    }
    else if (pUSARTHandle->USART_PINCFG.USART_WordLength == USART_WORDLEN_8) {
        data = pUSARTHandle->pUSARTx->RDR;
    }
    // else if(pUSARTHandle->USART_PINCFG.USART_WordLength == USART_WORDLEN_9) {
    //     data = pUSARTHandle->pUSARTx->RDR;
    // }

    /* Clear RXNE by USARTx_RQR (bit 3 RXFRQ) */
    pUSARTHandle->pUSARTx->RQR |= 1 << 3;
    return data;
}

/********************************************************
 * @fn              - USART_SendDataIT
 * @brief           - 
 * @param[in]       - 
 * @return          - none
 * @note            - none
 *********************************************************/
void USART_SendDataIT(USART_Handle_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len)
{

}

/********************************************************
 * @fn              - USART_ReceiveDataIT
 * @brief           - 
 * @param[in]       - 
 * @return          - none
 * @note            - none
 *********************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    for (int i = 0; i < Len; i++) {
        pRxBuffer[i] = USART_ReceiveData(pUSARTHandle);
    }
    USART_CR1_Interrupt_Set(pUSARTHandle, TCIE_FLAG, ENABLE);
    return 0;
}

/********************************************************
 * @fn              - USART_CR1_Interrupt_Set
 * @brief           - USART interrupt setting in USART_CR1
 * @param[in]       - pUSARTx: Handle structure for a USART pin
 *                  - flags: interrupt flags in USART_CR1
 *                  - EnorDi: ENABLE or DISABLE macros
 * @return          - none
 * @note            - none
 *********************************************************/
void USART_CR1_Interrupt_Set(USART_Handle_t *pUSARTx, uint32_t flags, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        pUSARTx->pUSARTx->CR1 |= flags;
    } else if (EnorDi == DISABLE) {
        pUSARTx->pUSARTx->CR1 &= ~flags;
    }
}

/********************************************************
 * @fn              - USART_CR2_Interrupt_Set
 * @brief           - USART interrupt setting in USART_CR2
 * @param[in]       - pUSARTx: Handle structure for a USART pin
 *                  - flags: interrupt flags in USART_CR2
 *                  - EnorDi: ENABLE or DISABLE macros
 * @return          - none
 * @note            - none
 *********************************************************/
void USART_CR2_Interrupt_Set(USART_Handle_t *pUSARTx, uint32_t flags, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        pUSARTx->pUSARTx->CR2 |= flags;
    } else if (EnorDi == DISABLE) {
        pUSARTx->pUSARTx->CR2 &= ~flags;
    }
}

/********************************************************
 * @fn              - USART_CR3_Interrupt_Set
 * @brief           - USART interrupt setting in USART_CR3
 * @param[in]       - pUSARTx: Handle structure for a USART pin
 *                  - flags: interrupt flags in USART_CR3
 *                  - EnorDi: ENABLE or DISABLE macros
 * @return          - none
 * @note            - none
 *********************************************************/
void USART_CR3_Interrupt_Set(USART_Handle_t *pUSARTx, uint32_t flags, uint8_t EnorDi)
{
    if (EnorDi == ENABLE) {
        pUSARTx->pUSARTx->CR3 |= flags;
    } else if (EnorDi == DISABLE) {
        pUSARTx->pUSARTx->CR3 &= ~flags;
    }
}

/********************************************************
 * @fn              - USART_IRQPriorityConfig
 * @brief           - 
 * @param[in]       - 
 * @return          - none
 * @note            - none
 *********************************************************/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}