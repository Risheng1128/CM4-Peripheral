/**
  ******************************************************************************
  * @file    stm32f303ze_usart_driver.h
  * @author  Ri-Sheng Chen
  * @brief   This file contains some functions prototypes for the USART driver.
  ******************************************************************************
  **/

#ifndef _STM32F303ZE_USART_DRIVER_H_
#define _STM32F303ZE_USART_DRIVER_H_
#include "stm32f303ze_hal.h"

/**
 * Configuration structure for a USART pin
 */
typedef struct {
    uint8_t USART_Mode;                 /* possible values from @USART_Mode          */
    uint32_t USART_Baud;                /* possible values from @USART_BaudRate      */
    uint8_t USART_StopBits;             /* possible values from @USART_StopBits      */
    uint8_t USART_WordLength;           /* possible values from @USART_WordLength    */
    uint8_t USART_ParityCtrl;           /* possible values from @USART_ParityControl */
    uint8_t USART_HWFlowCtrl;           /* possible values from @USART_HWFlowControl */
} USART_CFG_t;

/**
 * Handle structure for a USART pin
 */
typedef struct {
    USART_RegDef_t *pUSARTx;            /* Pointer holds the base address of the USART peripheral */  
    USART_CFG_t USART_PINCFG;           /* This holds USART configuration setting */    
} USART_Handle_t;

/*
 * @USART_Mode
 * The options for USART_Mode (configure from TE/RE in CR1)
 */
#define USART_MODE_TX                   0        /* Only use Tx */
#define USART_MODE_RX                   1        /* Only use Rx */
#define USART_MODE_TXRX                 2        /* Only use Tx & Rx */

/**
 * @USART_BaudRate
 * Possible options for USART Baudrate
 * We can find it from USART baud rate generation in RM0316
 */
#define USART_STD_BAUD_2400				2400
#define USART_STD_BAUD_9600				9600
#define USART_STD_BAUD_19200 			19200
#define USART_STD_BAUD_38400 			38400
#define USART_STD_BAUD_57600 			57600
#define USART_STD_BAUD_115200 	  115200
#define USART_STD_BAUD_230400 		230400
#define USART_STD_BAUD_460800 		460800
#define USART_STD_BAUD_921600 		921600
#define USART_STD_BAUD_2M 				2000000
#define USART_STD_BAUD_3M 				3000000
#define USART_STD_BAUD_4M 				4000000
#define USART_STD_BAUD_5M 				5000000
#define USART_STD_BAUD_6M 				6000000
#define USART_STD_BAUD_7M 				7000000
#define USART_STD_BAUD_9M 				9000000

/**
 * @USART_StopBits
 * The options for USART_StopBits (configure from STOP in CR2)
 */
#define USART_STOPBITS_1                0   /* 1 stop bit   */
#define USART_STOPBITS_0_5              1   /* 0.5 stop bit */
#define USART_STOPBITS_2                2   /* 2 stop bit   */
#define USART_STOPBITS_1_5              3   /* 1.5 stop bit */

/**
 * @USART_WordLength
 * The options for USART_WordLength (configure from M[1:0] in CR1)
 */
#define USART_WORDLEN_7                 0
#define USART_WORDLEN_8                 1
#define USART_WORDLEN_9                 2

/**
 * @USART_ParityControl
 * The options for USART_ParityControl (configure from PCE & PS in CR1)
 */
#define USART_PARITY_ODD                2
#define USART_PARITY_EVEN               1
#define USART_PARITY_DI                 0

/**
 * @USART_HWFlowControl
 * The options for USART_HWFlowControl (configure from CTSE & RTSE in CR3)
 */
#define USART_HW_FLOWCTRL_NONE    	  0       /* Use neither CTS nor RTS */
#define USART_HW_FLOWCTRL_RTS    	    1       /* Only use CTS */
#define USART_HW_FLOWCTRL_CTS    	    2       /* Only use RTS */
#define USART_HW_FLOWCTRL_CRTS  	    3       /* Use both CTS and RTS */

/**
 * CR1 Interrupt setting macros
 */
#define IDLEIE_FLAG       (1 << 4)      /* Idle line detected */
#define RXNEIE_FLAG       (1 << 5)      /* Receive data register not empty (data ready to be read) and Overrun error detected */
#define TCIE_FLAG         (1 << 6)      /* Transmission Complete */
#define TXEIE_FLAG        (1 << 7)      /* Transmit data register empty */
#define PEIE_FLAG         (1 << 8)      /* Parity error */
#define CMIE_FLAG         (1 << 14)     /* Character match */
#define RTOIE_FLAG        (1 << 26)     /* Receiver timeout */
#define EOBIE_FLAG        (1 << 27)     /* End of Block */ 

/**
 * CR2 Interrupt setting macros
 */
#define LBDIE_FLAG        (1 << 6)      /* LIN break */

/**
 * CR3 Interrupt setting macros
 */
#define EIE_FLAG          (1 << 0)      /* Noise Flag, Overrun error and Framing Error in multibuffer communication */
#define CTSIE_FLAG        (1 << 10)     /* CTS interrupt */
#define WUFIE_FLAG        (1 << 22)     /* Wakeup from Stop mode */

/*************************************************************************
 *                    APIs supported by this driver
 *      For more information about APIs check function definitions
 *************************************************************************/
/**
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_Deinit(USART_RegDef_t *pUSARTx);

/**
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/**
 * USART baud rate setup
 */
void USART_SetBaudRate(USART_Handle_t *pUSARTx);

/**
 * Data read and write
 */
void USART_SendData(USART_Handle_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveData(USART_Handle_t *pUSARTHandle);
void USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/**
 * IRQ Configuration and ISR handling
 */
void USART_CR1_Interrupt_Set(USART_Handle_t *pUSARTx, uint32_t flags, uint8_t EnorDi);
void USART_CR2_Interrupt_Set(USART_Handle_t *pUSARTx, uint32_t flags, uint8_t EnorDi);
void USART_CR3_Interrupt_Set(USART_Handle_t *pUSARTx, uint32_t flags, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv);

#endif /* End of _STM32F303ZE_USART_DRIVER_H_ */