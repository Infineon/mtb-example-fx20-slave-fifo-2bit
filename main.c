/***************************************************************************//**
* \file main.c
* \version 1.0
*
* Main source file of the USB Slave FIFO application.
*
*******************************************************************************
* \copyright
* (c) (2021-2024), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cy_pdl.h"
#include <string.h>
#include "cy_usb_common.h"
#include "cy_usb_usbd.h"
#include "cy_usb_app.h"
#include "cy_debug.h"
#include "cy_usbd_version.h"
#include "cy_hbdma_version.h"
#include "cy_lvds.h"
#include "cy_gpif_header.h"
#include "cybsp.h"
#include "../app_version.h"
#include "timers.h"
#include "cy_usb_i2c.h"
#include "cy_usb_qspi.h"

/* Select SCB interface used for UART based logging. */
#define LOGGING_SCB             (SCB1)
#define LOGGING_SCB_IDX         (1)
#define DEBUG_LEVEL             (3u)

#if DEBUG_INFRA_EN
/* Debug log related initilization */
#define LOGBUF_SIZE           (1024U)
uint8_t logBuff[LOGBUF_SIZE];
cy_stc_debug_config_t dbgCfg = {
    .pBuffer         = logBuff,
    .traceLvl        = DEBUG_LEVEL,
    .bufSize         = LOGBUF_SIZE,
#if USBFS_LOGS_ENABLE
    .dbgIntfce       = CY_DEBUG_INTFCE_USBFS_CDC,
#else
    .dbgIntfce       = CY_DEBUG_INTFCE_UART_SCB1,
#endif/* USBFS_LOGS_ENABLE */
    .printNow        = true
};

TaskHandle_t printLogTaskHandle;
#endif /* DEBUG_INFRA_EN */

/* Global variables associated with USB setup. */
cy_stc_usbss_cal_ctxt_t ssCalCtxt;
cy_stc_usb_cal_ctxt_t hsCalCtxt;

/* Global variables associated with High BandWidth DMA setup. */
cy_stc_hbdma_context_t HBW_DrvCtxt;     /* High BandWidth DMA driver context. */
cy_stc_hbdma_dscr_list_t HBW_DscrList;  /* High BandWidth DMA descriptor free list. */
cy_stc_hbdma_buf_mgr_t HBW_BufMgr;      /* High BandWidth DMA buffer manager. */
cy_stc_hbdma_mgr_context_t HBW_MgrCtxt; /* High BandWidth DMA manager context. */

/* Global variables associated with LVDS setup. */
cy_stc_lvds_context_t lvdsContext;

/* CPU DMA register pointers. */
DMAC_Type *pCpuDmacBase;
DW_Type *pCpuDw0Base, *pCpuDw1Base;
uint32_t hfclkFreq = BCLK__BUS_CLK__HZ;

cy_stc_usb_usbd_ctxt_t usbdCtxt;
cy_stc_usb_app_ctxt_t appCtxt;

bool glIsLVDSPhyTrainingDone = false;
bool glIsLVDSLink0TrainingDone = false;
bool glIsLVDSLink1TrainingDone = false;
uint8_t glPhyLinkTrainControl   = 0;
extern void Cy_App_MakeConfigDescriptor(void);

extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[];
extern const uint8_t CyFxUSBStringLangIDDscr[];
extern const uint8_t CyFxUSBManufactureDscr[];
extern const uint8_t CyFxUSBProductDscr[];
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[];

extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);
extern void vPortSVCHandler(void);

void Cy_SysTickIntrWrapper (void)
{
    Cy_USBD_TickIncrement(&usbdCtxt);
    xPortSysTickHandler();
}

void vPortSetupTimerInterrupt(void)
{
    /* Register the exception vectors. */
    Cy_SysInt_SetVector(PendSV_IRQn, xPortPendSVHandler);
    Cy_SysInt_SetVector(SVCall_IRQn, vPortSVCHandler);
    Cy_SysInt_SetVector(SysTick_IRQn, Cy_SysTickIntrWrapper);

    /* Start the SysTick timer with a period of 1 ms. */
    Cy_SysTick_SetClockSource(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU);
    Cy_SysTick_SetReload(hfclkFreq / 1000U);
    Cy_SysTick_Clear();
    Cy_SysTick_Enable();
}

#if DEBUG_INFRA_EN
void Cy_PrintTaskHandler(void *pTaskParam)
{
    while (1)
    {
        /* Print any pending logs to the output console. */
        Cy_Debug_PrintLog();

        /* Put the thread to sleep for 5 ms */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif /* DEBUG_INFRA_EN */

/*****************************************************************************
 * Function Name: Cy_LVDS_GpifEventCb(uint8_t smNo, cy_en_lvds_gpif_event_type_t gpifEvent,
 *                                    void *cntxt)
 *****************************************************************************
 * Description: GPIF error callback function.
 *
 * Parameters:
 * \param smNo
 * state machine number
 *
 * \param gpifEvent
 * GPIF event
 *
 * \param cntxt
 * app context
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_LVDS_GpifEventCb(uint8_t smNo, cy_en_lvds_gpif_event_type_t gpifEvent, void *cntxt)
{
    DBG_APP_INFO("Cy_LVDS_GpifEventCb\r\n");
}


/*****************************************************************************
 * Function Name: Cy_LVDS_PhyEventCb(uint8_t smNo, cy_en_lvds_phy_events_t phyEvent,
 *                                    void *cntxt)
 *****************************************************************************
 * Description: LVDS PHY callback function.
 *
 * Parameters:
 * \param smNo
 * state machine number
 *
 * \param phyEvent
 * LVDS/LVCMOS PHY event
 *
 * \param cntxt
 * app context
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_LVDS_PhyEventCb(uint8_t smNo, cy_en_lvds_phy_events_t phyEvent, void *cntxt)
{
#if FPGA_ENABLE
    cy_en_scb_i2c_status_t status;
#endif
    if(phyEvent == CY_LVDS_PHY_L1_EXIT)
    {
        if(smNo == 0)
        {
           DBG_APP_INFO("P0_L1_Exit\r\n");
        }
        else
        {
           DBG_APP_INFO("P1_L1_Exit\r\n");
        }
    }
    if(phyEvent == CY_LVDS_PHY_L1_ENTRY)
    {
        if(smNo == 0)
        {
            DBG_APP_INFO("P0_L1_Entry\r\n");
        }
        else
        {
            DBG_APP_INFO("P1_L1_Entry\r\n");
        }
    }
    if(phyEvent == CY_LVDS_PHY_L3_ENTRY)
    {
        if(smNo == 0)
        {
            DBG_APP_INFO("P0_L3_Entry\r\n");
        }
        else
        {
            DBG_APP_INFO("P1_L3_Entry\r\n");
        }
    }

#if FPGA_ENABLE
    if (phyEvent == CY_LVDS_PHY_TRAINING_DONE)
    {
        DBG_APP_INFO("Port %d PHY Train Done\r\n",smNo);
        glIsLVDSPhyTrainingDone = true;
    }

    if (phyEvent == CY_LVDS_PHY_LNK_TRAIN_BLK_DET)
    {
        DBG_APP_INFO("Port %d Training Block Detected\r\n",smNo);
        if(smNo)
        {
            glIsLVDSLink1TrainingDone = true;
            SET_BIT (glPhyLinkTrainControl, P1_TRAINING_DONE);
            status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_PHY_LINK_CONTROL_ADDRESS,glPhyLinkTrainControl,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
            ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
        }
        else
        {
            glIsLVDSLink0TrainingDone = true;
            SET_BIT (glPhyLinkTrainControl, P0_TRAINING_DONE);
            status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_PHY_LINK_CONTROL_ADDRESS,glPhyLinkTrainControl,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
            ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
        }
    }
    if (phyEvent == CY_LVDS_PHY_LNK_TRAIN_BLK_DET_FAIL)
    {
        DBG_APP_INFO("Port %d Training Block Detect Failed\r\n",smNo);
    }
#endif /* FPGA_ENABLE */
}

/*****************************************************************************
 * Function Name: Cy_LVDS_LowPowerEventCb(cy_en_lvds_low_power_events_t lowPowerEvent,
 *                                        void *cntxt)
 *****************************************************************************
 * Description: LVDS Low Power callback function.
 *
 * Parameters:
 * \param lowPowerEvent
 * low power event
 *
 * \param cntxt
 * app context
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_LVDS_LowPowerEventCb(cy_en_lvds_low_power_events_t lowPowerEvent, void *cntxt)
{
    if(lowPowerEvent == CY_LVDS_LOW_POWER_LNK0_L3_EXIT)
    {
        DBG_APP_INFO("P0_L3_Exit\r\n");
    }
    if(lowPowerEvent == CY_LVDS_LOW_POWER_LNK1_L3_EXIT)
    {
        DBG_APP_INFO("P1_L3_Exit\r\n");
    }
}

/*****************************************************************************
 * Function Name: Cy_LVDS_GpifErrorCb(uint8_t smNo, cy_en_lvds_gpif_error_t gpifError,
 *                                    void *cntxt)
 *****************************************************************************
 * Description: GPIF error callback function.
 *
 * Parameters:
 * \param smNo
 * state machine number
 *
 * \param gpifError
 * GPIF error
 *
 * \param cntxt
 * app context
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_LVDS_GpifErrorCb(uint8_t smNo, cy_en_lvds_gpif_error_t gpifError, void *cntxt)
{
    switch (gpifError)
    {
        case CY_LVDS_GPIF_ERROR_IN_ADDR_OVER_WRITE:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_IN_ADDR_OVER_WRITE\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_EG_ADDR_NOT_VALID:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_EG_ADDR_NOT_VALID\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_DMA_DATA_RD_ERROR:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_DMA_DATA_RD_ERROR\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_DMA_DATA_WR_ERROR:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_DMA_DATA_WR_ERROR\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_DMA_ADDR_RD_ERROR:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_DMA_DATA_WR_ERROR\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_DMA_ADDR_WR_ERROR:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_DMA_ADDR_WR_ERROR\n\r");
        break;

        case CY_LVDS_GPIF_ERROR_INVALID_STATE_ERROR:
        DBG_APP_ERR("CY_LVDS_GPIF_ERROR_INVALID_STATE_ERROR\n\r");
        break;
    }
}

/*****************************************************************************
 * Function Name: Cy_LVDS_GpifThreadErrorCb(cy_en_lvds_gpif_thread_no_t ThNo,
 *                          cy_en_lvds_gpif_thread_error_t ThError, void *cntxt)
 *****************************************************************************
 * Description: GPIF thread error callback function.
 *
 * Parameters:
 * \param ThNo
 * thread number
 *
 * \param ThError
 * thread error
 *
 * \param cntxt
 * app context
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_LVDS_GpifThreadErrorCb(cy_en_lvds_gpif_thread_no_t ThNo, cy_en_lvds_gpif_thread_error_t ThError, void *cntxt)
{
    switch (ThNo)
    {
        case CY_LVDS_GPIF_THREAD_0:
        case CY_LVDS_GPIF_THREAD_1:
        switch (ThError)
        {
            case CY_LVDS_GPIF_THREAD_DIR_ERROR:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_DIR_ERROR\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_WR_OVERFLOW:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_WR_OVERFLOW\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_RD_UNDERRUN:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_RD_UNDERRUN\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_SCK_ACTIVE:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_SCK_ACTIVE\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_ADAP_OVERFLOW:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_ADAP_OVERFLOW\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_ADAP_UNDERFLOW:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_ADAP_UNDERFLOW\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_READ_FORCE_END:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_READ_FORCE_END\n\r");
            break;

            case CY_LVDS_GPIF_THREAD_READ_BURST_ERR:
            DBG_APP_ERR("CY_LVDS_GPIF_THREAD_READ_BURST_ERR\n\r");
            break;
        }
        break;

        default:
            break;
    }
}

cy_stc_lvds_app_cb_t cb =
{
    .gpif_events = Cy_LVDS_GpifEventCb,
    .gpif_error = Cy_LVDS_GpifErrorCb,
    .gpif_thread_error = Cy_LVDS_GpifThreadErrorCb,
    .gpif_thread_event = NULL,
    .phy_events = Cy_LVDS_PhyEventCb,
    .low_power_events   = Cy_LVDS_LowPowerEventCb
};

cy_stc_lvds_context_t lvdsContext;

/*****************************************************************************
 * Function Name: Cy_LVDS_LVCMOS_Init
 *****************************************************************************
 * Summary
 *  Initialize the LVDS interface. Currently, only the SIP #0 is being initialized
 *  and configured to allow transfers into the HBW SRAM through DMA.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_LVDS_LVCMOS_Init(void)
{
    Cy_LVDS_SetInterruptMask(LVDSSS_LVDS, LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_TRAINING_DONE_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_TRAINING_DONE_Msk|
                                            LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_TRAINING_BLK_DETECTED_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_TRAINING_BLK_DETECTED_Msk |
                                            LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_TRAINING_BLK_DET_FAILD_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_TRAINING_BLK_DET_FAILD_Msk|
                                            LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_L1_ENTRY_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_L1_ENTRY_Msk |
                                            LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_L1_EXIT_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_L1_EXIT_Msk |
                                            LVDSSS_LVDS_LVDS_INTR_WD0_LNK0_L3_ENTRY_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_LNK1_L3_ENTRY_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_PHY_LINK0_INTERRUPT_Msk |
                                            LVDSSS_LVDS_LVDS_INTR_WD0_PHY_LINK1_INTERRUPT_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_THREAD0_ERR_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_THREAD1_ERR_Msk |
                                            LVDSSS_LVDS_LVDS_INTR_WD0_THREAD2_ERR_Msk | LVDSSS_LVDS_LVDS_INTR_WD0_THREAD3_ERR_Msk | LVDSSS_LVDS_LVDS_INTR_MASK_WD0_GPIF0_INTERRUPT_Msk);
    Cy_LVDS_RegisterCallback(LVDSSS_LVDS, &cb, &lvdsContext,&appCtxt);

    Cy_LVDS_Init(LVDSSS_LVDS, 0, &cy_lvds0_config, &lvdsContext);
    DBG_APP_INFO("LVDS_Init done\r\n");

    Cy_LVDS_GpifThreadConfig(LVDSSS_LVDS, 0, 0, 0, 0, 1);
    Cy_LVDS_GpifThreadConfig(LVDSSS_LVDS, 1, 1, 0, 0, 1);
    Cy_LVDS_GpifThreadConfig(LVDSSS_LVDS, 2, 0, 0, 0, 0);
    Cy_LVDS_GpifThreadConfig(LVDSSS_LVDS, 3, 1, 0, 0, 0);

    Cy_LVDS_Enable(LVDSSS_LVDS);
    DBG_APP_INFO("LVDSEn\r\n");

#if LINK_TRAINING_EN
    Cy_LVDS_PhyGpioSet(LVDSSS_LVDS, PORT_0,LINK_READY_CTL_PIN);
#endif /* LINK_TRAINING_EN */

    Cy_LVDS_GpifSMStart(LVDSSS_LVDS, 0, 0, 12);
    DBG_APP_INFO("GpifSMStart\r\n");
}

/*******************************************************************************
 * Function name: Cy_Fx3g2_InitPeripheralClocks
 ****************************************************************************//**
 *
 * Function used to enable clocks to different peripherals on the FX10/FX20 device.
 *
 * \param adcClkEnable
 * Whether to enable clock to the ADC in the USBSS block.
 *
 * \param usbfsClkEnable
 * Whether to enable bus reset detect clock input to the USBFS block.
 *
 *******************************************************************************/
void Cy_Fx3g2_InitPeripheralClocks (
        bool adcClkEnable,
        bool usbfsClkEnable)
{
    if (adcClkEnable) {
        /* Divide PERI clock at 75 MHz by 75 to get 1 MHz clock using 16-bit divider #1. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 1, 74);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 1);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_LVDS2USB32SS_CLOCK_SAR, CY_SYSCLK_DIV_16_BIT, 1);
    }

    if (usbfsClkEnable) {
        /* Divide PERI clock at 75 MHz by 750 to get 100 KHz clock using 16-bit divider #2. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 2, 749);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 2);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_USB_CLOCK_DEV_BRS, CY_SYSCLK_DIV_16_BIT, 2);
    }
}

/*******************************************************************************
 * Function name: Cy_Fx3G2_OnResetInit
 ****************************************************************************//**
 * TODO Ideally, this should be defined in cybsp.c
 * This function performs initialization that is required to enable scatter
 * loading of data into the High BandWidth RAM during device boot-up. The FX10/FX20
 * device comes up with the High BandWidth RAM disabled and hence any attempt
 * to read/write the RAM will cause the processor to hang. The RAM needs to
 * be enabled with default clock settings to allow scatter loading to work.
 * This function needs to be called from Cy_OnResetUser.
 *
 *******************************************************************************/
void
Cy_Fx3G2_OnResetInit (
        void)
{
    /* Enable clk_hf4 with IMO as input. */
    SRSS->CLK_ROOT_SELECT[4] = SRSS_CLK_ROOT_SELECT_ENABLE_Msk;

    /* Enable LVDS2USB32SS IP and select clk_hf[4] as clock input. */
    MAIN_REG->CTRL = (
            MAIN_REG_CTRL_IP_ENABLED_Msk |
            (1UL << MAIN_REG_CTRL_NUM_FAST_AHB_STALL_CYCLES_Pos) |
            (1UL << MAIN_REG_CTRL_NUM_SLOW_AHB_STALL_CYCLES_Pos) |
            (3UL << MAIN_REG_CTRL_DMA_SRC_SEL_Pos));
}

/*****************************************************************************
 * Function Name: Cy_LVDS_ISR
 ******************************************************************************
 * Summary:
 *  Handler for LVDS Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_LVDS_ISR(void)
{
    Cy_LVDS_IrqHandler(LVDSSS_LVDS, &lvdsContext);
}

/*****************************************************************************
 * Function Name: Cy_LVDS_Port0Dma_ISR(void)
 ******************************************************************************
 * Summary:
 *  Handler for LVDS Port0 Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_LVDS_Port0Dma_ISR(void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_LVDS_0);
    portYIELD_FROM_ISR(true);
}

/*****************************************************************************
 * Function Name: Cy_LVDS_Port1Dma_ISR(void)
 ******************************************************************************
 * Summary:
 *  Handler for LVDS Port1 Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_LVDS_Port1Dma_ISR(void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_LVDS_1);
    portYIELD_FROM_ISR(true);
}

/*****************************************************************************
 * Function Name: Cy_USB_SS_ISR(void)
 ******************************************************************************
 * Summary:
 *  Handler for USB-SS Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_SS_ISR(void)
{
    /* Call the USB32DEV interrupt handler. */
    Cy_USBSS_Cal_IntrHandler(&ssCalCtxt);
}

/*****************************************************************************
 * Function Name: Cy_USB_IngressDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB Ingress DMA Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_IngressDma_ISR(void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_USB_IN);
    portYIELD_FROM_ISR(true);
}


/*****************************************************************************
 * Function Name: Cy_USB_EgressDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB Egress DMA Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_EgressDma_ISR(void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_USB_EG);
    portYIELD_FROM_ISR(true);
}

/*****************************************************************************
 * Function Name: Cy_USB_HS_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB-HS Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USB_HS_ISR(void)
{
#if FREERTOS_ENABLE
    if (Cy_USBHS_Cal_IntrHandler(&hsCalCtxt))
    {
        portYIELD_FROM_ISR(true);
    }
#else
    Cy_USBHS_Cal_IntrHandler(&hsCalCtxt);
#endif /* FREERTOS_ENABLE */
}

void InEpDma_ISR(uint8_t endpNum)
{
    /* Clear the interrupt first. */
    Cy_USB_AppClearDmaInterrupt(&appCtxt, endpNum, CY_USB_ENDP_DIR_IN);

    /* The current data buffer has been consumed. Move to another buffer if available. */
    Cy_Slff_AppHandleRxCompletion(&appCtxt,endpNum);
}


/*****************************************************************************
 * Function Name: Cy_PrintVersionInfo
 ******************************************************************************
 * Summary:
 *  Function to print version information to UART console.
 *
 * Parameters:
 *  type: Type of version string.
 *  version: Version number including major, minor, patch and build number.
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_PrintVersionInfo(const char *type, uint32_t version)
{
    char tString[32];
    uint16_t vBuild;
    uint8_t vMajor, vMinor, vPatch;
    uint8_t typeLen = strlen(type);

    vMajor = (version >> 28U);
    vMinor = ((version >> 24U) & 0x0FU);
    vPatch = ((version >> 16U) & 0xFFU);
    vBuild = (uint16_t)(version & 0xFFFFUL);

    memcpy(tString, type, typeLen);
    tString[typeLen++] = '0' + (vMajor / 10);
    tString[typeLen++] = '0' + (vMajor % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vMinor / 10);
    tString[typeLen++] = '0' + (vMinor % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vPatch / 10);
    tString[typeLen++] = '0' + (vPatch % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vBuild / 1000);
    tString[typeLen++] = '0' + ((vBuild % 1000) / 100);
    tString[typeLen++] = '0' + ((vBuild % 100) / 10);
    tString[typeLen++] = '0' + (vBuild % 10);
    tString[typeLen++] = '\r';
    tString[typeLen++] = '\n';
    tString[typeLen] = 0;

    DBG_APP_INFO("%s", tString);
}

/*****************************************************************************
 * Function Name: Cy_VbusDetGpio_ISR
 *****************************************************************************
 * Summary
 *  Interrupt handler for the Vbus detect GPIO transition detection.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
static void Cy_VbusDetGpio_ISR(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    cy_stc_usbd_app_msg_t xMsg;

    /* Send VBus changed message to the task thread. */
    xMsg.type = CY_USB_VBUS_CHANGE_INTR;
    xQueueSendFromISR(appCtxt.usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));

    /* Remember that VBus change has happened and disable the interrupt. */
    appCtxt.vbusChangeIntr = true;
    Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 0);
}

/*****************************************************************************
 * Function Name: Cy_USB_USBSSInit
 *****************************************************************************
 * Summary
 *  Initialize USBSS block and attempt device enumeration.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_USB_USBSSInit(void)
{
    cy_stc_gpio_pin_config_t pinCfg;
    cy_stc_sysint_t intrCfg;
    memset((void *)&pinCfg, 0, sizeof(pinCfg));


    /* Unlock and then disable the watchdog. */
    Cy_WDT_Unlock();
    Cy_WDT_Disable();

    /* Enable interrupts. */
    __enable_irq();

#if FPGA_ENABLE
    Cy_USB_I2CInit ();
#endif /* FPGA_ENABLE */

    memset((uint8_t *)&appCtxt, 0, sizeof(appCtxt));
    memset((uint8_t *)&ssCalCtxt, 0, sizeof(ssCalCtxt));
    memset((uint8_t *)&hsCalCtxt, 0, sizeof(hsCalCtxt));
    memset((uint8_t *)&usbdCtxt, 0, sizeof(usbdCtxt));

    memset((void *)&pinCfg, 0, sizeof(pinCfg));
    /* Configure VBus detect GPIO. */
    pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
    pinCfg.hsiom     = HSIOM_SEL_GPIO;
    pinCfg.intEdge   = CY_GPIO_INTR_BOTH;
    pinCfg.intMask   = 0x01UL;
    (void)Cy_GPIO_Pin_Init(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, &pinCfg);

    /* Register edge detect interrupt for Vbus detect GPIO. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = VBUS_DETECT_GPIO_INTR;
    intrCfg.intrPriority = 7;
#else
    intrCfg.cm0pSrc = VBUS_DETECT_GPIO_INTR;
    intrCfg.intrSrc = NvicMux5_IRQn;
    intrCfg.intrPriority = 3;
#endif
    Cy_SysInt_Init(&intrCfg, Cy_VbusDetGpio_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

#if FPGA_ENABLE
    memset((void *)&pinCfg, 0, sizeof(pinCfg));
    /* Configure c_reset FPGA GPIO. */
    pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pinCfg.hsiom     = TI180_CRESET_GPIO;
    (void)Cy_GPIO_Pin_Init(TI180_CRESET_GPIO_PORT, TI180_CRESET_GPIO_PIN, &pinCfg);
    Cy_GPIO_Clr(TI180_CRESET_GPIO_PORT, TI180_CRESET_GPIO_PIN);
    Cy_SysLib_Delay(20);
    Cy_GPIO_Set(TI180_CRESET_GPIO_PORT, TI180_CRESET_GPIO_PIN);

    /* Configure input GPIO. */
    pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
    pinCfg.hsiom = HSIOM_SEL_GPIO;
    (void)Cy_GPIO_Pin_Init(T120_CDONE_PORT, T120_CDONE_PIN, &pinCfg);
#endif

    /* Register the LVDS ISR and enable the interrupt for LVDS. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = lvds2usb32ss_lvds_int_o_IRQn;
    intrCfg.intrPriority = 6;
#else
    intrCfg.intrSrc = NvicMux0_IRQn;
    intrCfg.intrPriority = 2;
    intrCfg.cm0pSrc = lvds2usb32ss_lvds_int_o_IRQn;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, Cy_LVDS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register the ISR and enable the interrupt for SIP0 DMA adapter. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = lvds2usb32ss_lvds_dma_adap0_int_o_IRQn;
    intrCfg.intrPriority = 3;
#else
    intrCfg.intrSrc = NvicMux4_IRQn;
    intrCfg.intrPriority = 1;
    intrCfg.cm0pSrc = lvds2usb32ss_lvds_dma_adap0_int_o_IRQn;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_LVDS_Port0Dma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register the ISR and enable the interrupt for SIP1 DMA adapter. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = lvds2usb32ss_lvds_dma_adap1_int_o_IRQn;
    intrCfg.intrPriority = 3;
#else
    intrCfg.intrSrc = NvicMux6_IRQn;
    intrCfg.intrPriority = 1;
    intrCfg.cm0pSrc = lvds2usb32ss_lvds_dma_adap1_int_o_IRQn;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_LVDS_Port1Dma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register the USBSS ISR and enable the interrupt. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = lvds2usb32ss_usb32_int_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.intrSrc = NvicMux1_IRQn;
    intrCfg.cm0pSrc = lvds2usb32ss_usb32_int_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_USB_SS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = lvds2usb32ss_usb32_wakeup_int_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.intrSrc = NvicMux1_IRQn;
    intrCfg.cm0pSrc = lvds2usb32ss_usb32_wakeup_int_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_USB_SS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* ISR for the USB Ingress DMA adapter */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 3;
#else
    intrCfg.intrSrc = NvicMux3_IRQn;
    intrCfg.cm0pSrc = lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_USB_IngressDma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

/* ISR for the USB Egress DMA adapter */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = lvds2usb32ss_usb32_egrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 3;
#else
    intrCfg.intrSrc = NvicMux2_IRQn;
    intrCfg.cm0pSrc = lvds2usb32ss_usb32_egrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &Cy_USB_EgressDma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

/* Register ISR for and enable USBHS Interrupt. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = usbhsdev_interrupt_u2d_active_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.cm0pSrc      = usbhsdev_interrupt_u2d_active_o_IRQn;
    intrCfg.intrSrc      = NvicMux5_IRQn;
    intrCfg.intrPriority = 2;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, Cy_USB_HS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);
}

/*****************************************************************************
 * Function Name: Cy_InitHbDma
 *****************************************************************************
 * Summary
 *  Initialize HBDMA block
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
bool Cy_InitHbDma(void)
{
    cy_en_hbdma_status_t drvstat;
    cy_en_hbdma_mgr_status_t mgrstat;

    /* Initialize the HBW DMA driver layer. */
    drvstat = Cy_HBDma_Init(LVDSSS_LVDS, USB32DEV, &HBW_DrvCtxt, 0, 0);
    if (drvstat != CY_HBDMA_SUCCESS)
    {
        return false;
    }

    /* Setup a HBW DMA descriptor list. */
    mgrstat = Cy_HBDma_DscrList_Create(&HBW_DscrList, 256U);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    /* Initialize the DMA buffer manager. We will use 832 KB of space from 0x1C030000 onwards. */
   mgrstat = Cy_HBDma_BufMgr_Create(&HBW_BufMgr, (uint32_t *)0x1C030000UL, 0xD0000UL);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    /* Initialize the HBW DMA channel manager. */
    mgrstat = Cy_HBDma_Mgr_Init(&HBW_MgrCtxt, &HBW_DrvCtxt, &HBW_DscrList, &HBW_BufMgr);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    return true;
}

/*****************************************************************************
 * Function Name: Cy_USBSS_DeInit(cy_stc_usbss_cal_ctxt_t *pCalCtxt)
 ******************************************************************************
 * Summary:
 *  Function to de initialize USBSS device
 *
 * Parameters:
 *  \param pCalCtxt
 * USB CAL layer context pointer
 *
 * Return:
 *  None
 *****************************************************************************/
void Cy_USBSS_DeInit(cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_MAIN_Type *USB32DEV_MAIN = &base->USB32DEV_MAIN;

    /* Disable the clock for USB3.2 function */
    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_CLK_EN_Msk;

    /* Disable PHYSS */
    base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.TOP_CTRL_0 &=
        ~(USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
          USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
          USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk |
          USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk);

    base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.TOP_CTRL_0 &=
                    ~(USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk);

    /* Disable the SuperSpeed Device function */
    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_SSDEV_ENABLE_Msk;
}

/*****************************************************************************
 * Function Name: Cy_USB_SSConnectionEnable(cy_stc_usb_app_ctxt_t *pAppCtxt)
 *****************************************************************************
 * Summary
 *  PSVP specific USB connect function.
 *
 * Parameters:
 *  \param pAppCtxt
 *  Pointer to application context structure.
 *
 * Return:
 *  void
 ****************************************************************************/
bool Cy_USB_SSConnectionEnable(cy_stc_usb_app_ctxt_t *pAppCtxt)
{

	pAppCtxt->desiredSpeed = USB_CONN_TYPE;
    Cy_USBD_ConnectDevice(pAppCtxt->pUsbdCtxt, pAppCtxt->desiredSpeed);
    pAppCtxt->usbConnected = true;
	LOG_COLOR("USB Speed %d \n\r",pAppCtxt->desiredSpeed);

    return true;
}

/*****************************************************************************
 * Function Name: Cy_USB_SSConnectionDisable(cy_stc_usb_app_ctxt_t *pAppCtxt)
 *****************************************************************************
 * Summary
 *  PSVP specific USB disconnect function.
 *
 * Parameters:
 *  \param pAppCtxt
 *  Pointer to application context structure.
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_USB_SSConnectionDisable(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    Cy_USBD_DisconnectDevice(pAppCtxt->pUsbdCtxt);
    Cy_USBSS_DeInit(pAppCtxt->pUsbdCtxt->pSsCalCtxt);
    pAppCtxt->usbConnected = false;
    pAppCtxt->devState     = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
}


/*****************************************************************************
 * Function Name: Cy_OnResetUser(void)
 ******************************************************************************
 * Summary:
 *  Init function which is executed before the load regions in RAM are updated.
 *  The High BandWidth subsystem needs to be enable here to allow variables
 *  placed in the High BandWidth SRAM to be updated.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *****************************************************************************/
void Cy_OnResetUser(void)
{
    Cy_Fx3G2_OnResetInit();
}

/*****************************************************************************
* Function Name: main(void)
******************************************************************************
* Summary:
*  Entry to the application.
*
* Parameters:
*  void
*
* Return:
*  Does not return.
*****************************************************************************/
int main(void)
{

    /* Initialize the PDL driver library and set the clock variables. */
    Cy_PDL_Init (&cy_deviceIpBlockCfgFX3G2);
    cybsp_init();    
    Cy_Fx3g2_InitPeripheralClocks(true, true);

    hfclkFreq = Cy_SysClk_ClkFastGetFrequency();

    pCpuDmacBase = ((DMAC_Type *)DMAC_BASE);
    pCpuDw0Base = ((DW_Type *)DW0_BASE);
    pCpuDw1Base = ((DW_Type *)DW1_BASE);

    /* Initialize the PDL and register ISR for USB block. */
    Cy_USB_USBSSInit();

#if DEBUG_INFRA_EN
#if !USBFS_LOGS_ENABLE
    /* Initialize the UART for logging. */
    InitUart(LOGGING_SCB_IDX);
#endif /* USBFS_LOGS_ENABLE */

    Cy_Debug_LogInit(&dbgCfg);
    /* Create task for printing logs and check status. */
    xTaskCreate(Cy_PrintTaskHandler, "PrintLogTask", 512, NULL, 5, &printLogTaskHandle);
    Cy_SysLib_Delay(500);
    Cy_Debug_AddToLog(1, "***** FX20: Slave FIFO 2-Bit Application *****\r\n");

    /* Print application, USBD stack and HBDMA version information. */
    Cy_PrintVersionInfo("APP_VERSION: ", APP_VERSION_NUM);
    Cy_PrintVersionInfo("USBD_VERSION: ", USBD_VERSION_NUM);
    Cy_PrintVersionInfo("HBDMA_VERSION: ", HBDMA_VERSION_NUM);
#endif /* DEBUG_INFRA_EN */

    /* Store IP base address in CAL context. */
    ssCalCtxt.regBase = USB32DEV;
    hsCalCtxt.pCalBase = MXS40USBHSDEV_USBHSDEV;
    hsCalCtxt.pPhyBase = MXS40USBHSDEV_USBHSPHY;

    /*
     * Make sure any previous USB connection state is cleared. Give some delay to allow the host to process
     * disconnection.
     */
    Cy_USBSS_DeInit(&ssCalCtxt);
    Cy_SysLib_Delay(500);

    /* Initialize the HbDma IP and DMA Manager */
    Cy_InitHbDma();
    DBG_APP_INFO("InitHbDma done\r\n");

    Cy_LVDS_PhyGpioModeEnable(LVDSSS_LVDS, PORT_0, LINK_READY_CTL_PIN, CY_LVDS_PHY_GPIO_OUTPUT, CY_LVDS_PHY_GPIO_NO_INTERRUPT);

#if LINK_TRAINING_EN /*In case of LVDS or LVCMOS DDR */
    Cy_LVDS_PhyGpioClr(LVDSSS_LVDS, PORT_0, LINK_READY_CTL_PIN);
#endif

    /* Initialize the USBD layer */
    Cy_USB_USBD_Init(&appCtxt, &usbdCtxt, pCpuDmacBase, &hsCalCtxt, &ssCalCtxt, &HBW_MgrCtxt);
    DBG_APP_INFO("USBD_Init done\r\n");

    /* Specify that DMA clock should be set to 240 MHz once USB 3.x connection is active. */
    Cy_USBD_SetDmaClkFreq(&usbdCtxt, CY_HBDMA_CLK_240_MHZ);

    /* Enable stall cycles between back-to-back AHB accesses to high bandwidth RAM. */
    MAIN_REG->CTRL = (MAIN_REG->CTRL & 0xF00FFFFFUL) | 0x09900000UL;

    /* Create the configuration descriptor with the required number of endpoints. */
    Cy_App_MakeConfigDescriptor();

    /* Register USB descriptors with the stack. */
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_SS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_SS_BOS_DSCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_DEVICE_QUAL_DSCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_SS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_HS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_FS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_STRING_DSCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_STRING_DSCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    Cy_USBD_SetDscr(&usbdCtxt, CY_USB_SET_STRING_DSCR, 2, (uint8_t *)CyFxUSBProductDscr);

    /* Initialize the application and create echo device thread. */
    Cy_USB_AppInit(&appCtxt, &usbdCtxt, pCpuDmacBase, pCpuDw0Base, pCpuDw1Base, &HBW_MgrCtxt);

    DBG_APP_INFO("Scheduler start done\r\n");
    /* Invokes scheduler: Not expected to return. */
    vTaskStartScheduler();
    while (1);

    return 0;
}

/* [] END OF FILE */
