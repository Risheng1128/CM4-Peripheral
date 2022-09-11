/**
  ******************************************************************************
  * @file    stm32f303ze_hal.c
  * @author  Ri-Sheng Chen
  * @brief   This file contains the functions for the core driver.
  ******************************************************************************
  **/

#include "stm32f303ze_hal.h"
#include "stdbool.h"

/********************************************************
 * @fn              - NVIC_Interrupt_Eable
 * @brief           - This function enable the interrupt
 * @param[in]       - IRQ number
 * @return          - True: Enable success
 *                  - False: IRQ number range error
 * @note            - none
 * 
 *********************************************************/
bool NVIC_Interrupt_Eable(uint8_t irq_number)
{
    if (irq_number > IRQ_MAX_NUMBER)
        return false;

    uint8_t offset = irq_number >> 5; // the number of ISER
    uint8_t pos = irq_number & 31; // the bit position of ISER
    NVIC_ISER_BASEADDR[offset] |= 1 << pos;
    return true;
}

/********************************************************
 * @fn              - NVIC_Interrupt_Disable
 * @brief           - This function disable the interrupt
 * @param[in]       - IRQ number
 * @return          - True: Enable success
 *                  - False: IRQ number range error
 * @note            - none
 * 
 *********************************************************/
bool NVIC_Interrupt_Disable(uint8_t irq_number)
{
    if (irq_number > IRQ_MAX_NUMBER)
        return false;

    uint8_t offset = irq_number >> 5; // the number of ICER
    uint8_t pos = irq_number & 31; // the bit position of ICER
    NVIC_ICER_BASEADDR[offset] |= 1 << pos;
    return true;
}