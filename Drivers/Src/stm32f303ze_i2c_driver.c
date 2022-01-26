/**
  ******************************************************************************
  * @file    stm32f303ze_i2c_driver.c
  * @author  Ri-Sheng Chen
  * @brief   This file contains the functions for the I2C driver.
  ******************************************************************************
  **/

#include "stm32f303ze_i2c_driver.h"

/********************************************************
 * @fn              - I2C_PeriClockControl
 * @brief           - This function enables or disables the clock of the I2C
 * @param[in]       - base address of the I2C peripheral
 *                  - ENABLE or DISABLE macros
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void I2C_PeriClockControl(I2C_RegDef_t* pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE) {
        if(pI2Cx == I2C1)
            I2C1_PCLK_EN();
        else if(pI2Cx == I2C2)
            I2C2_PCLK_EN();
        else if(pI2Cx == I2C3)
            I2C3_PCLK_EN();
    }
    else if(EnorDi == DISABLE) {
        if(pI2Cx == I2C1)
            I2C1_PCLK_DI();
        else if(pI2Cx == I2C2)
            I2C2_PCLK_DI();
        else if(pI2Cx == I2C3)
            I2C3_PCLK_DI();
    }
}

/********************************************************
 * @fn              - I2C_Init
 * @brief           - This function initializes the GPIO_Handle_t structure
 * @param[in]       - Handle structure for a GPIO pin
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void I2C_Init(I2C_Handle_t* pI2CHandle)
{
    
}

/********************************************************
 * @fn              - I2C_DeInit
 * @brief           - This function initializes the GPIO_Handle_t structure
 * @param[in]       - Handle structure for a GPIO pin
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void I2C_DeInit(I2C_RegDef_t* pI2Cx)
{
    if(pI2Cx == I2C1)
        I2C1_REG_RST();
    else if(pI2Cx == I2C2)
        I2C2_REG_RST();
    else if(pI2Cx == I2C3)
        I2C3_REG_RST();
}