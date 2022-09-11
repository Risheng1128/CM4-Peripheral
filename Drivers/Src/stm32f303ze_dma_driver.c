/**
  ******************************************************************************
  * @file    stm32f303ze_dma_driver.c
  * @author  Ri-Sheng Chen
  * @brief   This file contains the functions for the DMA driver.
  ******************************************************************************
  **/

#include "stm32f303ze_dma_driver.h"

/********************************************************
 * @fn              - Is_Valid_Number
 * @brief           - This function checks the channel number is valid or not
 * @param[in]       - chnl_num: channel number
 *                  - max_chnl_num: max channel number in DMA
 *                    - DMA1_MAX_CHANNEL_NUMBER
 *                    - DMA2_MAX_CHANNEL_NUMBER
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
static bool Is_Valid_Number(uint8_t chnl_num, uint8_t max_chnl_num)
{
    return chnl_num < max_chnl_num ? 1 : 0;
}

/********************************************************
 * @fn              - DMA_Init
 * @brief           - This function initializes the DMA_Handle_t structure
 * @param[in]       - Handle structure for a DMA
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void DMA1_Init(DMA1_Handle_t *pDMA1Handle)
{
    uint8_t chnl_num = pDMA1Handle->DMA_CFG.DMA_ChannelNumber; // channel number

    // Set DMA Priority Level
    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR &= ~(3 << 12);
    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR |= (pDMA1Handle->DMA_CFG.DMA_PriorityLevel << 12);

    // Set Memory Size
    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR &= ~(3 << 10);
    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR |= (pDMA1Handle->DMA_CFG.DMA_MemorySize << 10);

    // Set Peripheral Size
    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR &= ~(3 << 8);
    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR |= (pDMA1Handle->DMA_CFG.DMA_PeripheralSize << 8);

    // Set Increment mode
    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR &= ~(3 << 6);
    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR |= (pDMA1Handle->DMA_CFG.DMA_IncrementMode << 6);

    // Set Transfer mode
    if (pDMA1Handle->DMA_CFG.DMA_TransferMode == DMA_M2M) {
        pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR |= (1 << 14);
    } else if (pDMA1Handle->DMA_CFG.DMA_TransferMode == DMA_M2P) {
        pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR |= (1 << 4);
    } else if (pDMA1Handle->DMA_CFG.DMA_TransferMode == DMA_P2M) {
        pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR &= ~(1 << 4);
    }
}

/********************************************************
 * @fn              - DMA1_Peripheral_Addr_Set
 * @brief           - This function set the DMA peripheral address
 * @param[in]       - Handle structure for a DMA data configuration
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void DMA1_Peripheral_Addr_Set(DMA1_Handle_t *pDMA1Handle)
{
    pDMA1Handle->pDMAx->CHNL_CTRL[pDMA1Handle->DMA_CFG.DMA_ChannelNumber].CPAR = pDMA1Handle->DMA_CFG.DMA_PeripheralAddr;
}

/********************************************************
 * @fn              - DMA1_Memory_Addr_Set
 * @brief           - This function set the DMA memory data address
 * @param[in]       - Handle structure for a DMA data configuration
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void DMA1_Memory_Addr_Set(DMA1_Handle_t *pDMA1Handle)
{
    pDMA1Handle->pDMAx->CHNL_CTRL[pDMA1Handle->DMA_CFG.DMA_ChannelNumber].CMAR = pDMA1Handle->DMA_CFG.DMA_MemoryAddr;
}

/********************************************************
 * @fn              - DMA1_Data_Size_Set
 * @brief           - This function set the DMA data size
 * @param[in]       - Handle structure for a DMA data configuration
 * @return          - none
 * @note            - none
 * 
 *********************************************************/
void DMA1_Data_Size_Set(DMA1_Handle_t *pDMA1Handle)
{
    pDMA1Handle->pDMAx->CHNL_CTRL[pDMA1Handle->DMA_CFG.DMA_ChannelNumber].CNDTR = pDMA1Handle->DMA_CFG.DMA_DataSize;
}

/********************************************************
 * @fn              - DMA1_Interrupt_Enable
 * @brief           - This function set the DMA1 interrupt enable
 * @param[in]       - pDMA1Handle: Handle structure for a DMA data configuration
 *                  - flag: flags of inerrupt
 *                    - TC_FLAG: transfer complete
 *                    - HT_FLAG: half transfer
 *                    - TE_FLAG: transfer error
 * @return          - true: interrupt setting success
 *                  - false: channel number error
 * @note            - only DMA1 has interrupt (DMA2 no interrupt)
 *********************************************************/
bool DMA1_Interrupt_Enable(DMA1_Handle_t *pDMA1Handle, uint8_t flags)
{
    uint8_t chnl_num = pDMA1Handle->DMA_CFG.DMA_ChannelNumber;
    if (!Is_Valid_Number(chnl_num, DMA1_MAX_CHANNEL_NUMBER))
        return false;
    
    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR &= ~(7 << 1);
    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR |= flags;
    return true;
}

/********************************************************
 * @fn              - DMA1_Interrupt_Disable
 * @brief           - This function set the DMA1 interrupt disable
 * @param[in]       - pDMA1Handle: Handle structure for a DMA data configuration
 *                  - flag: flags of inerrupt
 *                    - TC_FLAG: transfer complete
 *                    - HT_FLAG: half transfer
 *                    - TE_FLAG: transfer error
 * @return          - true: interrupt setting success
 *                  - false: channel number error
 * @note            - only DMA1 has interrupt (DMA2 no interrupt)
 *********************************************************/
bool DMA1_Interrupt_Disable(DMA1_Handle_t *pDMA1Handle, uint8_t flags)
{
    uint8_t chnl_num = pDMA1Handle->DMA_CFG.DMA_ChannelNumber;
    if (!Is_Valid_Number(chnl_num, DMA1_MAX_CHANNEL_NUMBER))
        return false;

    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR &= ~flags;
    return true;
}

/********************************************************
 * @fn              - DMA1_Channel_Enable
 * @brief           - This function enable DMA1 channel
 * @param[in]       - pDMA1Handle: Handle structure for a DMA data configuration
 * @return          - true: interrupt setting success
 *                  - false: channel number error
 * @note            - none
 *********************************************************/
bool DMA1_Channel_Enable(DMA1_Handle_t *pDMA1Handle)
{
    uint8_t chnl_num = pDMA1Handle->DMA_CFG.DMA_ChannelNumber;
    if (!Is_Valid_Number(chnl_num, DMA1_MAX_CHANNEL_NUMBER))
        return false;

    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR |= (1 << 0);
    return true;
}

/********************************************************
 * @fn              - DMA1_Channel_Disable
 * @brief           - This function disable DMA1 channel
 * @param[in]       - pDMA1Handle: Handle structure for a DMA data configuration
 * @return          - true: interrupt setting success
 *                  - false: channel number error
 * @note            - none
 *********************************************************/
bool DMA1_Channel_Disable(DMA1_Handle_t *pDMA1Handle)
{
    uint8_t chnl_num = pDMA1Handle->DMA_CFG.DMA_ChannelNumber;
    if (!Is_Valid_Number(chnl_num, DMA1_MAX_CHANNEL_NUMBER))
        return false;

    pDMA1Handle->pDMAx->CHNL_CTRL[chnl_num].CCR &= ~(1 << 0);
    return true;
}

/********************************************************
 * @fn              - DMA1_Interrupt_Clear
 * @brief           - This function clear DMA1 interrupt
 * @param[in]       - pDMA1Handle: Handle structure for a DMA data configuration
 *                  - flags: flags of clear interrupt
 *                    - CGIF: clear global interrupt flag
 *                    - CTCIF: clear transfer complete interrupt flag
 *                    - CHTIF: clear half transfer interrupt flag
 *                    - CTEIF: clear transfer error interrupt flag
 * @return          - true: interrupt setting success
 *                  - false: channel number error
 * @note            - none
 *********************************************************/
bool DMA1_Interrupt_Clear(DMA1_Handle_t *pDMA1Handle, uint8_t flags)
{
    uint8_t chnl_num = pDMA1Handle->DMA_CFG.DMA_ChannelNumber;
    if (!Is_Valid_Number(chnl_num, DMA1_MAX_CHANNEL_NUMBER))
        return false;

    pDMA1Handle->pDMAx->IFCR |= (flags << (chnl_num << 2));
    pDMA1Handle->pDMAx->IFCR &= ~(flags << (chnl_num << 2));
    return true;
}
