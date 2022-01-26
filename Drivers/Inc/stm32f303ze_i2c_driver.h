/**
  ******************************************************************************
  * @file    stm32f303ze_i2c_driver.h
  * @author  Ri-Sheng Chen
  * @brief   This file contains some functions prototypes for the I2C driver.
  ******************************************************************************
  **/

#ifndef _STM32F303ZE_I2C_DRIVER_H_
#define _STM32F303ZE_I2C_DRIVER_H_
#include "stm32f303ze_hal.h"

/**
 * Configuration structure for a USART pin
 */
typedef struct {
    uint32_t I2C_SCLSpeed;          /* possible values from @I2C_SCLSpeed */            
    uint8_t I2C_DeviceAddress;          
    uint8_t I2C_ACKControl;         /* possible values from @I2C_ACKControl */
    uint16_t I2C_FMDutyCycle;       /* possible values from @I2C_FMDutyCycle */
} I2C_CFG_t;

/**
 * Handle structure for a USART pin
 */
typedef struct {
    I2C_RegDef_t *pI2Cx;            /* Pointer holds the base address of the I2C peripheral */  
    I2C_CFG_t I2C_PINCFG;           /* This holds I2C configuration setting */    
} I2C_Handle_t;

/**
 * @I2C_SCLSpeed
 * I2C SCL Speed macros
 */
#define I2C_SCL_SPEED_SDM      100000  // Standard mode speed
#define I2C_SCL_SPEED_FM       400000  // Fast mode speed

/**
 * @I2C_ACKControl
 * I2C ACK Control macros
 */
#define I2C_ACK_EN      1
#define I2C_ACK_DI      0

/**
 * @I2C_FMDutyCycle
 * I2C Fast mode SCL duty macros
 */
#define I2C_FM_DUTY_


/** 
 * Bit position definition of I2C peripheral
 */
/* I2C_CR1 */
#define I2C_CR1_PE          0  // Peripheral enable
#define I2C_CR1_TXIE        1  // TX Interrupt enable
#define I2C_CR1_RXIE        2  // RX Interrupt enable
#define I2C_CR1_ADDRIE      3  // Address match Interrupt enable
#define I2C_CR1_NACKIE      4  // Not acknowledge received Interrupt enable
#define I2C_CR1_STOPIE      5  // STOP detection Interrupt enable
#define I2C_CR1_TCIE        6  // Transfer Complete interrupt enable
#define I2C_CR1_ERRIE       7  // Error interrupts enable
#define I2C_CR1_DNF         8  // Digital noise filter
#define I2C_CR1_ANFOFF      12 // Analog noise filter OFF
#define I2C_CR1_TXDMAEN     14 // DMA transmission requests enable
#define I2C_CR1_RXDMAEN     15 // DMA reception requests enable
#define I2C_CR1_SBC         16 // Slave byte control
#define I2C_CR1_NOSTRETCH   17 // Clock stretching disable
#define I2C_CR1_WUPEN       18 // Wakeup from Stop mode enable
#define I2C_CR1_GCEN        19 // General call enable
#define I2C_CR1_SMBHEN      20 // SMBus Host address enable
#define I2C_CR1_SMBDEN      21 // SMBus Device Default address enable
#define I2C_CR1_ALERTEN     22 // SMBus alert enable
#define I2C_CR1_PECEN       23 // PEC enable

/* I2C_CR2 */
#define I2C_CR2_SADD0       0  // Slave addess bit 0 (master mode)
#define I2C_CR2_SADD        1  // Slave addess bit 1~9 (master mode)
#define I2C_CR2_RD_WRN      10 // Transfer direction (master mode)
#define I2C_CR2_ADD10       11 // 10-bit addressing mode (master mode)
#define I2C_CR2_HEAD10R     12 // 10-bit address header only read direction (master receiver mode)
#define I2C_CR2_START       13 // Start generation
#define I2C_CR2_STOP        14 // Stop generation (master mode)
#define I2C_CR2_NACK        15 // Nack generation (master mode)
#define I2C_CR2_NBYTES      16 // Number of bytes
#define I2C_CR2_RELOAD      24 // NBYTES reload mode
#define I2C_CR2_AUTOEND     25 // Automatic end mode (master mode)
#define I2C_CR2_PECBYTE     26 // Packet error checking byte

/* I2C_ISR */
#define I2C_ISR_TXE         0  // Transmit data register empty (transmitters)
#define I2C_ISR_TXIS        1  // Transmit interrupt status (transmitters)
#define I2C_ISR_RXNE        2  // Receive data register not empty (receivers)
#define I2C_ISR_ADDR        3  // Address matched (slave mode)
#define I2C_ISR_NACKF       4  // Not Acknowledge received flag
#define I2C_ISR_STOPF       5  // Stop detection flag
#define I2C_ISR_TC          6  // Transfer Complete (master mode)
#define I2C_ISR_TCR         7  // Transfer Complete Reload
#define I2C_ISR_BERR        8  //  Bus error
#define I2C_ISR_ARLO        9  // Arbitration lost
#define I2C_ISR_OVR         10 // Overrun/Underrun (slave mode)
#define I2C_ISR_PECERR      11 // PEC Error in reception
#define I2C_ISR_TIMEOUT     12 //  Timeout or tLOW detection flag
#define I2C_ISR_ALERT       13 // SMBus alert
#define I2C_ISR_BUSY        15 // Bus busy
#define I2C_ISR_DIR         16 // Transfer direction (Slave mode)
#define I2C_ISR_ADDCODE     17 // Address match code (Slave mode)

/*************************************************************************
 *                    APIs supported by this driver
 *      For more information about APIs check function definitions
 *************************************************************************/
/**
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);

/**
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t* pI2CHandle);
void I2C_DeInit(I2C_RegDef_t* pI2Cx);

/**
 * Data Send and Receive
 */

/**
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptCFG(uint8_t IRQnumber, uint8_t EnorDi);
void I2C_IRQPriorityCFG(uint8_t IRQnumber, uint32_t IRQPriority);

/**
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t FlagName);

/**
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);

#endif