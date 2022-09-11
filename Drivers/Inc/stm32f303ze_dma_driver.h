/**
  ******************************************************************************
  * @file    stm32f303ze_dma_driver.h
  * @author  Ri-Sheng Chen
  * @brief   This file contains some functions prototypes for the DMA driver.
  ******************************************************************************
  **/

#ifndef _STM32F303ZE_DMA_DRIVER_H_
#define _STM32F303ZE_DMA_DRIVER_H_
#include "stm32f303ze_hal.h"

/**
 * Configuration structure for a DMA peripheral
 */
typedef struct {
    uint8_t DMA_ChannelNumber;      /* possible values from @DMA_Channel_Number     */
    uint8_t DMA_PriorityLevel;      /* possible values from @DMA_Priority_Level     */
    uint8_t DMA_MemorySize;         /* possible values from @DMA_Memory_Size        */
    uint8_t DMA_PeripheralSize;     /* possible values from @DMA_Peripheral_Size    */
    uint8_t DMA_IncrementMode;      /* possible values from @DMA_Increment_Mode     */
    uint8_t DMA_TransferMode;       /* possible values from @DMA_Transfer_Mode      */
    uint16_t DMA_DataSize;          /* DMA transfer data size in bytes              */
    uint32_t DMA_PeripheralAddr;    /* DMA peripheral data base address             */
    uint32_t DMA_MemoryAddr;        /* DMA memory data base address                 */
} DMA_CFG_t;

/**
 * Handle structure for a DMA peripheral
 */
typedef struct {
    DMA1_RegDef_t *pDMAx;           /* Pointer holds the base address of the DMA peripheral */  
    DMA_CFG_t DMA_CFG;              /* This holds DMA configuration setting */
} DMA1_Handle_t;

/**
 * @DMA_Channel_Number
 * DMA channel number
 * - DMA1: 7 channels
 * - DMA2: 5 channels
 */
#define DMA_CHANNEL1            0
#define DMA_CHANNEL2            1
#define DMA_CHANNEL3            2
#define DMA_CHANNEL4            3
#define DMA_CHANNEL5            4
#define DMA_CHANNEL6            5
#define DMA_CHANNEL7            6

/**
 * @DMA_Priority_Level
 * DMA priority level
 */
#define DMA_PRIORITY_LOW        0
#define DMA_PRIORITY_MEDIUM     1
#define DMA_PRIORITY_HIGH       2
#define DMA_PRIORITY_VERYHIGH   3

/**
 * @DMA_Memory_Size
 * DMA memory size
 */
#define DMA_MSIZE_BITS_8        0
#define DMA_MSIZE_BITS_16       1
#define DMA_MSIZE_BITS_32       2

/**
 * @DMA_Peripheral_Size
 * DMA peripheral size
 */
#define DMA_PSIZE_BITS_8        0
#define DMA_PSIZE_BITS_16       1
#define DMA_PSIZE_BITS_32       2

/**
 * @DMA_Increment_Mode
 * DMA increment mode
 */
#define DMA_NO_MINC_PINC        0   /* Use nor MINC and PINC */
#define DMA_ONLY_PINC           1   /* Only use PINC */
#define DMA_ONLY_MINC           2   /* Only use MINC */
#define DMA_BOTH_MINC_PINC      3   /* Use both PINC and MINC */

/**
 * @DMA_Transfer_Mode
 * DMA transfer mode
 */
#define DMA_M2M                 0   /* Memory to Memory mode */
#define DMA_M2P                 1   /* Memory to Peripheral mode */
#define DMA_P2M                 2   /* Peripheral to Memory mode */
#define DMA_P2P                 3   /* Peripheral to Peripheral mode */

#define DMA1_CHANNEL_EN(chnl_num)    (DMA1->CHNL_CTRL[chnl_num].CCR |= (1 << 0))
#define DMA1_CHANNEL_DI(chnl_num)    (DMA1->CHNL_CTRL[chnl_num].CCR &= ~(1 << 0))

void DMA1_Init(DMA1_Handle_t *pDMA1Handle);
void DMA1_Peripheral_Addr_Set(DMA1_Handle_t *pDMA1Handle);
void DMA1_Memory_Addr_Set(DMA1_Handle_t *pDMA1Handle);
void DMA1_Data_Size_Set(DMA1_Handle_t *pDMA1Handle);

#endif /* end of _STM32F303ZE_DMA_DRIVER_H_ */