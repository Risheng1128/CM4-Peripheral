/**
  ******************************************************************************
  * @file    stm32f303ze_dma_driver.c
  * @author  Ri-Sheng Chen
  * @brief   This file contains the functions for the DMA driver.
  ******************************************************************************
  **/

#include "stm32f303ze_dma_driver.h"

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