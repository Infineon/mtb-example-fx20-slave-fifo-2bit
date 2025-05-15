/***************************************************************************//**
* \file cy_usb_app.c
* \version 1.0
*
* Implements the USB data handling part of the Slave FIFO application.
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
#include "cy_device.h"
#include "cy_usbhs_dw_wrapper.h"
#include "cy_usb_common.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_usbd.h"
#include "cy_usb_app.h"
#include "cy_debug.h"
#include "cy_usb_i2c.h"
#include "cy_lvds.h"
#include "cy_usb_qspi.h"

static HBDMA_BUF_ATTRIBUTES uint32_t SetSelDataBuffer[8U];

/* Whether SET_CONFIG is complete or not. */
static volatile bool cy_slff_devconfigured = false;
static volatile bool cy_slff_IsApplnActive = false;
extern cy_stc_hbdma_buf_mgr_t HBW_BufMgr;
bool glIsFPGARegConfigured = false;
extern bool glIsFPGAConfigured;
extern uint8_t glPhyLinkTrainControl;

void Cy_USBSS_DeInit(cy_stc_usbss_cal_ctxt_t *pCalCtxt);

extern cy_israddress GetEPInDmaIsr(uint8_t epNum);

#if FPGA_ENABLE

/*****************************************************************************
* Function Name: Cy_Slaveff_StreamStartStop(uint8_t device_offset, uint8_t IsStreamStart)
******************************************************************************
* Summary:
*  Read the i2c
*
* Parameters:
* 
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
cy_en_scb_i2c_status_t Cy_Slaveff_StreamStartStop(uint8_t device_offset, uint8_t IsStreamStart)
{
	cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;

    cy_slff_IsApplnActive = IsStreamStart?true:false;

    if(IsStreamStart)
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(device_offset+FPGA_DEVICE_STREAM_ENABLE_ADDRESS),CAMERA_APP_ENABLE,
                                            FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    else

        status = Cy_I2C_Write(FPGASLAVE_ADDR,(device_offset+FPGA_DEVICE_STREAM_ENABLE_ADDRESS),CAMERA_APP_DISABLE,
                                            FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);

    DBG_APP_INFO("cy_slff_IsApplnActive = 0x%x\n\r",cy_slff_IsApplnActive);

    if(status == CY_SCB_I2C_SUCCESS)
    {
        DBG_APP_INFO("Cy_Slaveff_StreamStartStop: Stream App started = %d device_offset 0x%x\n\r",IsStreamStart, device_offset);
    }
    else
    {
        DBG_APP_ERR(" Cy_Slaveff_StreamStartStop: Stream App start failed \n\r");
    }

	return status;
}//End of Cy_Slaveff_StreamStartStop()


/*****************************************************************************
* Function Name: Cy_ConfigFpgaRegister()
******************************************************************************
* Summary:
*  Read the i2c
*
* Parameters:
* 
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
static cy_en_scb_i2c_status_t Cy_ConfigFpgaRegister (void)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;
    /* Defalut resolution*/
    uint16_t width = 3840;
    uint16_t hight = 2160;
    uint8_t i = 0;

	/* Disable camera before configuring FPGA register */
    for (i =0 ; i < 4 ; i++)
    {
	    status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+FPGA_DEVICE_STREAM_ENABLE_ADDRESS,CAMERA_APP_DISABLE,
		                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    }

	/* write FPGA register to enable UVC */
	status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_UVC_U3V_SELECTION_ADDRESS,FPGA_UVC_ENABLE,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);


    /* Disable adding UVC header by FPGA. UVC header is added by FX10 */
	status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_UVC_HEADER_CTRL_ADDRESS,FPGA_UVC_HEADER_DISABLE,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);

    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(500);


    /* Number of active device list*/
	status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_ACTIVE_DIVICE_MASK_ADDRESS,0x0F,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);

    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(500);

    /* Configure Four Devices of FPGA; DEVICE0,DEVICE1,DEVICE2 & DEVICE3 */
    for (i = 0 ; i < 4 ; i++)
    {
        /* write FPGA register to Disable format converstion */
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+FPGA_DEVICE_STREAM_MODE_ADDRESS,NO_CONVERSION,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        /* Select colorbar mode by default if DYNAMIC_VIDEOSOURCE is true*/
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_SOURCE_TYPE_ADDRESS,INTERNAL_COLORBAR,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        /* Inform active threads*/
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_ACTIVE_TREAD_INFO_ADDRESS,1,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        /* inform active sockets*/
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_THREAD2_SOCKET_INFO_ADDRESS,0,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        /* Thread 2 information*/
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_THREAD2_INFO_ADDRESS,0,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        /* Thread 1 information*/
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_THREAD1_INFO_ADDRESS, i,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        /* Thread 1 socket information*/
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_THREAD1_SOCKET_INFO_ADDRESS,0,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        /* Clear FPGA register during power up, this will get update when firmware detects HDMI */
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_HDMI_SOURCE_INFO_ADDRESS,HDMI_DISCONECT,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        DBG_APP_INFO("DMA Buffer Size %d \n\r",FPGA_DMA_BUFFER_SIZE);
 
        /* Update DMA buffer size used by Firmware */
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_BUFFER_SIZE_MSB_ADDRESS,CY_GET_MSB(FPGA_DMA_BUFFER_SIZE),
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_BUFFER_SIZE_LSB_ADDRESS,CY_GET_LSB(FPGA_DMA_BUFFER_SIZE),
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        /* Update default resolution width size used by Firmware */
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_IMAGE_WIDTH_MSB_ADDRESS,CY_GET_MSB(width),
		                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_IMAGE_WIDTH_LSB_ADDRESS,CY_GET_LSB(width),
		                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        /* Update default resolution hight size used by Firmware */
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_IMAGE_HEIGHT_MSB_ADDRESS,CY_GET_MSB(hight),
		                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_IMAGE_HEIGHT_LSB_ADDRESS,CY_GET_LSB(hight),
		                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

        /* default fps*/
        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+DEVICE_FPS_ADDRESS,60,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
        Cy_SysLib_DelayUs(500);
    } // for loop

    return status;
} //End of Cy_ConfigFpgaRegister()


/*****************************************************************************
* Function Name: Cy_FPGAPhyLinkTraining()
******************************************************************************
* Summary:
*  Read the i2c
*
* Parameters:
* 
*
* Return:
*  0 for read success, error code for unsuccess.
*****************************************************************************/
cy_en_scb_i2c_status_t Cy_FPGAPhyLinkTraining (void)
{
    cy_en_scb_i2c_status_t  status = CY_SCB_I2C_SUCCESS;
    uint8_t i = 0;
    for (i =0 ; i < 4 ; i++)
    {
        /* write FPGA register to Disable format converstion */

        status = Cy_I2C_Write(FPGASLAVE_ADDR,(DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET)+FPGA_DEVICE_STREAM_ENABLE_ADDRESS,CAMERA_APP_DISABLE,
		                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
        ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    }

    Cy_SysLib_DelayUs(10);
#if LINK_TRAINING_EN
#if LVCMOS_DDR_EN
    SET_BIT (glPhyLinkTrainControl, FPGA_LINK_CONTROL);
    DBG_APP_INFO("Link training \n\r");
#endif

	status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_PHY_LINK_CONTROL_ADDRESS,glPhyLinkTrainControl,
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(10);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_LVDS_PHY_TRAINING_ADDRESS, PHY_TRAINING_PATTERN_BYTE,
                           FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(10);

	status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_LVDS_LINK_TRAINING_BLK_P0_ADDRESS,CY_USB_DWORD_GET_BYTE0(LINK_TRAINING_PATTERN_BYTE),
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs (10);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_LVDS_LINK_TRAINING_BLK_P1_ADDRESS,CY_USB_DWORD_GET_BYTE1(LINK_TRAINING_PATTERN_BYTE),
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs (10);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_LVDS_LINK_TRAINING_BLK_P2_ADDRESS,CY_USB_DWORD_GET_BYTE2(LINK_TRAINING_PATTERN_BYTE),
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs (10);

    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_LVDS_LINK_TRAINING_BLK_P3_ADDRESS,CY_USB_DWORD_GET_BYTE3(LINK_TRAINING_PATTERN_BYTE),
	                                      FPGA_I2C_ADDRESS_WIDTH,FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs (10);
#endif

    return status;
} //End of Cy_FPGAPhyLinkTraining()
#endif /* FPGA_ENABLE */

static void Cy_Slff_AppHandleRxEvent (cy_stc_usb_app_ctxt_t  *pUsbApp, cy_stc_hbdma_channel_t *pChHandle, bool incFlag)
{
    cy_en_hbdma_mgr_status_t   status;
    cy_stc_hbdma_buff_status_t buffStat;

    /* Commit the buffer for transfer */
    if (pUsbApp->devSpeed >= CY_USBD_USB_DEV_SS_GEN1)
    {

        /* Wait for a free buffer. */
        status = Cy_HBDma_Channel_GetBuffer(pChHandle, &buffStat);
        if (status != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("SOC_ERROR\r\n");
            return;
        }

        status = Cy_HBDma_Channel_CommitBuffer(pChHandle, &buffStat);
        if (status != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("HB-DMA CommitBuffer Error\r\n");
        }
    }
    else
    {
        /* Queue USBHS IN transfer */
        if(incFlag == 1)
        {
            pUsbApp->slffPendingRxBufCnt[pChHandle->endpAddr]++;
            if ((pUsbApp->slffPendingRxBufCnt[pChHandle->endpAddr]) > 1)
            {
                return;
            }
        }
        /* Wait for a free buffer. */
        status = Cy_HBDma_Channel_GetBuffer(pChHandle, &buffStat);
        if (status != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("SOC_ERROR\r\n");
            return;
        }
        Cy_USB_AppQueueWrite(pUsbApp, pChHandle->endpAddr, buffStat.pBuffer, buffStat.count);
    }
}

/* Callback function for the channel receiving data into LVDS SPI socket*/
void Cy_Slff_AppHbDmaRxCallback(
        cy_stc_hbdma_channel_t *handle,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t *pbufStat,
        void *userCtx)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

    if (type == CY_HBDMA_CB_PROD_EVENT)
    {
        Cy_Slff_AppHandleRxEvent(pAppCtxt, handle, 1);
    }
}

void Cy_Slff_AppTaskHandler(void *pTaskParam)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;
    cy_stc_usbd_app_msg_t queueMsg;
    BaseType_t xStatus;
    uint8_t i = 0;
    uint8_t index = 0;
    uint8_t epNumber = 0;

    /* If VBus is present, enable the USB connection. */
    pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
    if (pAppCtxt->vbusPresent) {
        Cy_USB_SSConnectionEnable(pAppCtxt);
    }

    DBG_APP_INFO("Configure FPGA\n\r");

#if FPGA_ENABLE
    Cy_FPGAConfigPins(pAppCtxt,FPGA_CONFIG_MODE);
    Cy_QSPI_Start(pAppCtxt,&HBW_BufMgr);
    Cy_SPI_FlashInit(SPI_FLASH_0, true, false);

    Cy_FPGAConfigure(pAppCtxt,FPGA_CONFIG_MODE);
    Cy_FPGAPhyLinkTraining();
    Cy_FPGAGetVersion(pAppCtxt);

    if(!glIsFPGARegConfigured)
    {
        if(0 == Cy_ConfigFpgaRegister())
        {
            glIsFPGARegConfigured = true;
            DBG_APP_INFO("Successfuly configured the FPGA via I2C \n\r");
        }
        else
        {
            LOG_ERROR("Failed to configure FPGA via I2C \r\n");
        }
    }
#endif /* FPGA_ENABLE */

    vTaskDelay(pdMS_TO_TICKS(100)); // Give time to Print Task Handler to empty log buffer
    Cy_LVDS_LVCMOS_Init();
    vTaskDelay(pdMS_TO_TICKS(100)); /// Give time to Print Task Handler to empty log buffer

    DBG_APP_INFO("AppThreadCreated\r\n");
    for (;;)
    {
        /*
         * Wait until some data is received from the queue.
         * Timeout after 100 ms.
         */
        xStatus = xQueueReceive(pAppCtxt->usbMsgQueue, &queueMsg, 100);
        if (xStatus != pdPASS) {
            continue;
        }

        switch (queueMsg.type) {

            case CY_USB_VBUS_CHANGE_INTR:
                /* Start the debounce timer. */
                xTimerStart(pAppCtxt->vbusDebounceTimer, 0);
                break;

            case CY_USB_VBUS_CHANGE_DEBOUNCED:
                /* Check whether VBus state has changed. */
                pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);

                if (pAppCtxt->vbusPresent) {
                    if (!pAppCtxt->usbConnected) {
                        DBG_APP_INFO("Enabling USB connection due to VBus detect\r\n");
                        Cy_USB_SSConnectionEnable(pAppCtxt);
                    }
                } else {
                    if (pAppCtxt->usbConnected) {
                        /* On USB 2.x connections, make sure the DataWire channels are disabled and reset. */

                        for (i = 1; i < CY_USB_NUM_ENDP_CONFIGURED; i++) {
                            if (pAppCtxt->endpInDma[i].valid) {
                                /* DeInit the DMA channel and disconnect the triggers. */
                                Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpInDma[i]));
                            }

                            DBG_APP_TRACE("HBDMA destroy\r\n");
                            if (pAppCtxt->hbBulkInChannel[index] != NULL)
                            {

                                Cy_HBDma_Channel_Disable(pAppCtxt->hbBulkInChannel[index]);
                                Cy_HBDma_Channel_Destroy(pAppCtxt->hbBulkInChannel[index]);
                            }
                        }
                    }
                    DBG_APP_INFO("Disabling USB connection due to VBus removal\r\n");
                    Cy_USB_SSConnectionDisable(pAppCtxt);
                }

                break;

            case CY_USB_STREAMING_START:
                DBG_APP_INFO("CY_USB_STREAMING_START \r\n");
#if FPGA_ENABLE
                    for(i = 0; i < 4 ; i++)
                    {
                        Cy_Slaveff_StreamStartStop((DEVICE0_OFFSET + i * FPGA_DEVICE_OFFSET), START);
                    }
#endif /* FPGA_ENABLE */
            break;

            case CY_USB_STREAMING_STOP:
                DBG_APP_INFO("CY_USB_STREAMING_STOP for EP %x\r\n", (uint8_t)queueMsg.data[0]);
                epNumber = (uint8_t)queueMsg.data[0];
#if FPGA_ENABLE
            Cy_Slaveff_StreamStartStop((DEVICE0_OFFSET + ((epNumber-1) * FPGA_DEVICE_OFFSET)), STOP);

#endif /* FPGA_ENABLE */

            break;

            default:
            break;
        }

    } /* End of for(;;) */
}

/*
 * Function: Cy_USB_VbusDebounceTimerCallback()
 * Description: Timer used to do debounce on VBus changed interrupt notification.
 *
 * Parameter:
 *      xTimer: RTOS timer handle.
 * return: void
 */
void
Cy_USB_VbusDebounceTimerCallback (TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("VbusDebounce_CB\r\n");
    if (pAppCtxt->vbusChangeIntr) {
        /* Notify the VCOM task that VBus debounce is complete. */
        xMsg.type = CY_USB_VBUS_CHANGE_DEBOUNCED;
        xQueueSendFromISR(pAppCtxt->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));

        /* Clear and re-enable the interrupt. */
        pAppCtxt->vbusChangeIntr = false;
        Cy_GPIO_ClearInterrupt(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN);
        Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 1);
    }
}   /* end of function  */

/****************************************************************************************
* This function Initializes application related data structures, register callback
* creates task for device function.
****************************************************************************************/
void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    DMAC_Type *pCpuDmacBase, DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt)
{
    uint32_t index;
    BaseType_t status = pdFALSE;
    cy_stc_app_endp_dma_set_t *pEndpInDma;
    uint8_t *pslffPendingRxBufCnt;

    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pAppCtxt->devAddr = 0x00;
    pAppCtxt->activeCfgNum = 0x00;
    pAppCtxt->prevAltSetting = 0x00;
    pAppCtxt->enumMethod = CY_USB_ENUM_METHOD_FAST;
    pAppCtxt->pHbDmaMgrCtxt = pHbDmaMgrCtxt;
    pAppCtxt->pCpuDmacBase = pCpuDmacBase;
    pAppCtxt->pCpuDw0Base = pCpuDw0Base;
    pAppCtxt->pCpuDw1Base = pCpuDw1Base;
    pAppCtxt->pUsbdCtxt = pUsbdCtxt;

    for (index = 0x01; index < CY_USB_NUM_ENDP_CONFIGURED; index++)
    {
        pEndpInDma = &(pAppCtxt->endpInDma[index]);
        memset((void *)pEndpInDma, 0, sizeof(cy_stc_app_endp_dma_set_t));

        pEndpInDma->channel = index;
        pEndpInDma->firstRqtDone = false;
        pslffPendingRxBufCnt = &(pAppCtxt->slffPendingRxBufCnt[index]);
        *pslffPendingRxBufCnt = 0x00;
        pAppCtxt->hbBulkInChannel[index] = NULL;
    }

    /*
     * Callbacks registered with USBD layer. These callbacks will be called
     * based on appropriate event.
     */
    Cy_USB_AppRegisterCallback(pAppCtxt);

    if (!(pAppCtxt->firstInitDone))
    {

        /* Create the message queue and register it with the kernel. */
        pAppCtxt->usbMsgQueue = xQueueCreate(CY_USB_DEVICE_MSG_QUEUE_SIZE,
                CY_USB_DEVICE_MSG_SIZE);
        if (pAppCtxt->usbMsgQueue == NULL) {
            DBG_APP_ERR("QueuecreateFail\r\n");
            return;
        }

        vQueueAddToRegistry(pAppCtxt->usbMsgQueue, "DeviceMsgQueue");

        /* Create task and check status to confirm task created properly. */
        status = xTaskCreate(Cy_Slff_AppTaskHandler, "SlaveFIFODeviceTask", 2048,
                             (void *)pAppCtxt, 5, &(pAppCtxt->slffTaskHandle));

        if (status != pdPASS)
        {
            DBG_APP_ERR("TaskcreateFail\r\n");
            return;
        }

        pAppCtxt->vbusDebounceTimer = xTimerCreate("VbusDebounceTimer", 200, pdFALSE,
                (void *)pAppCtxt, Cy_USB_VbusDebounceTimerCallback);
        if (pAppCtxt->vbusDebounceTimer == NULL) {
            DBG_APP_ERR("TimerCreateFail\r\n");
            return;
        }
        DBG_APP_INFO("VBus debounce timer created\r\n");
        pAppCtxt->firstInitDone = 0x01;
    }
}

/* This function will register all calback with USBD layer */
void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET, Cy_USB_AppBusResetCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET_DONE, Cy_USB_AppBusResetDoneCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_BUS_SPEED, Cy_USB_AppBusSpeedCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP, Cy_USB_AppSetupCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SUSPEND, Cy_USB_AppSuspendCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESUME, Cy_USB_AppResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_CONFIG, Cy_USB_AppSetCfgCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_INTF, Cy_USB_AppSetIntfCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SLP, Cy_USB_AppSlpCallback);
}

/* Configure and enable HBW DMA channels */
static void Cy_USB_AppSetupEndpDmaParamsSs(cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *pEndpDscr)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t mgrStat;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    uint8_t *pCompDscr;
    uint8_t  burstSize;
    uint16_t maxPktSize;
    uint32_t endpNum, endpDir;

    DBG_APP_TRACE("AppSetupEndpDmaParamsSs >> \r\n");

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNum, &maxPktSize, &endpDir);
    pCompDscr = Cy_USBD_GetSsEndpCompDscr(pUsbApp->pUsbdCtxt, pEndpDscr);
    Cy_USBD_GetEndpCompnMaxburst(pCompDscr, &burstSize);

    DBG_APP_INFO("endpDmaParamsSS endpNum:0x%x maxPktSize:0x%x burstSize:0x%x endpDir:0x%x \r\n",
                  endpNum, maxPktSize, burstSize,endpDir);

    if (endpDir) /* RX: Data Received from LVDS SPI */
    {
        /*
         * Its IN endpoint which means DATA device->Host.
         */
        /* LVDS SPI Interface RX */
        pUsbApp->slffPendingRxBufCnt[endpNum] = 0;

        pEndpDmaSet = &(pUsbApp->endpInDma[endpNum]);
        pEndpDmaSet->maxPktSize = maxPktSize;
        pEndpDmaSet->valid      = 0x01;

        dmaConfig.size          = SLFF_RX_MAX_BUFFER_SIZE;      /* DMA Buffer Size in bytes */
        dmaConfig.count         = SLFF_RX_MAX_BUFFER_COUNT;     /* DMA Buffer Count */
        dmaConfig.bufferMode    = true;                         /* DMA buffer mode disabled */
        dmaConfig.prodHdrSize   = 0;
        dmaConfig.prodBufSize   = SLFF_RX_MAX_BUFFER_SIZE;
        dmaConfig.eventEnable   = 0;                            /* Enable for DMA AUTO */
        dmaConfig.intrEnable    = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk | 
                                    LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
        dmaConfig.prodSckCount  = 1;                            /* No. of producer sockets */
        dmaConfig.consSckCount  = 1;                            /* No. of consumer Sockets */
        dmaConfig.prodSck[1]    = (cy_hbdma_socket_id_t)0;      /* Producer Socket ID: None */
        dmaConfig.consSck[1]    = (cy_hbdma_socket_id_t)0;      /* Consumer Socket ID: None */
        dmaConfig.cb            = Cy_Slff_AppHbDmaRxCallback;   /* HB-DMA callback */
        dmaConfig.userCtx       = (void *)(pUsbApp);            /* Pass the application context as user context. */
        dmaConfig.chType        = CY_HBDMA_TYPE_IP_TO_IP;       /* DMA Channel type: from LVDS SPI to USB EP. */


        /* If channel had already been created, make sure to disable and destroy it. */
        if (pUsbApp->hbBulkInChannel[endpNum] != NULL) {
            DBG_APP_INFO("Destroying previously created streaming channel\r\n");
            Cy_HBDma_Channel_Disable(pUsbApp->hbBulkInChannel[endpNum]);
            Cy_HBDma_Channel_Destroy(pUsbApp->hbBulkInChannel[endpNum]);
            pUsbApp->hbBulkInChannel[endpNum] = NULL;
        }

        pUsbApp->hbBulkInChannel[endpNum] = &(pEndpDmaSet->hbDmaChannel);

        if (endpNum == 1)
        {
            dmaConfig.prodSck[0]    = (cy_hbdma_socket_id_t)(PORT0_LVDS_SOCKET_START_INDEX );
        }
        else if (endpNum == 2)
        {
            dmaConfig.prodSck[0]    = (cy_hbdma_socket_id_t)(PORT0_LVDS_SOCKET_START_INDEX + 1 );
        }
        else if (endpNum == 3)
        {
            dmaConfig.prodSck[0]    = (cy_hbdma_socket_id_t)(PORT1_LVDS_SOCKET_START_INDEX );

        }
        else if (endpNum == 4)
        {
            dmaConfig.prodSck[0]    = (cy_hbdma_socket_id_t)(PORT1_LVDS_SOCKET_START_INDEX + 1 );
        }
        else
        {
            DBG_APP_ERR("Wrong EP \r\n");
        }

        dmaConfig.consSck[0]    = (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + endpNum);
        dmaConfig.endpAddr      = endpNum;

        if (pUsbApp->devSpeed <= CY_USBD_USB_DEV_HS)
        {
            dmaConfig.chType     = CY_HBDMA_TYPE_IP_TO_MEM;
            dmaConfig.consSck[0] = (cy_hbdma_socket_id_t)0;
        }

        mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                                          pUsbApp->hbBulkInChannel[endpNum],
                                          &dmaConfig);

        if (mgrStat != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("BulkIn channel create failed 0x%x\r\n", mgrStat);
            return;
        }
        else
        {
            mgrStat = Cy_HBDma_Channel_Enable((pUsbApp->hbBulkInChannel[endpNum]), 0);
            DBG_APP_INFO("InChnEnable status: %x\r\n", mgrStat);
        }
    }
	else {
        DBG_APP_ERR("OUT endpoint not supported\r\n");
    }

    return;
} /* end of function  */

static void
Cy_USB_AppSetupEndpDmaParamsHs(cy_stc_usb_app_ctxt_t *pUsbApp,
                               uint8_t *pEndpDscr)
{
    cy_stc_app_endp_dma_set_t *pEndpDmaSet = NULL;
    DW_Type *pDW = NULL;
    bool stat;
    uint32_t endpNumber;
    uint16_t maxPktSize = 0x00;
    cy_en_usb_endp_dir_t endpDirection;

    endpNumber = ((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);
    Cy_USBD_GetEndpMaxPktSize(pEndpDscr, &maxPktSize);

    if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80)
    {
        endpDirection = CY_USB_ENDP_DIR_IN;
        pEndpDmaSet = &(pUsbApp->endpInDma[endpNumber]);
        pDW = pUsbApp->pCpuDw1Base;
    }
    else
    {
        DBG_APP_ERR("DIR-OUT not supported\r\n");
        return;
    }

    stat = Cy_USBHS_App_EnableEpDmaSet(pEndpDmaSet, pDW, endpNumber, endpNumber, endpDirection, maxPktSize);
    DBG_APP_INFO("Enable EPDmaSet: endp=%x dir=%x stat=%x\r\n", endpNumber, endpDirection, stat);

    pEndpDmaSet->endpType = (cy_en_usb_endp_type_t)
                    ((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ATTRIBUTE)) & 0x03);

    /* Make the ISR registration and trigger connections from EPM to DMAC. */
    DBG_APP_TRACE("DIR-IN, ChannelNum:0x%x\r\n",pEndpDmaSet->channel);
    Cy_USB_AppInitDmaIntr(endpNumber, CY_USB_ENDP_DIR_IN,
            GetEPInDmaIsr(endpNumber));

    DBG_APP_TRACE("Cy_USB_AppSetupEndpDmaParamsHs << \r\n");
    return;

} /* end of function  */

/*
 * Function: Cy_USB_AppSetupEndpDmaParams()
 * Description: This Function will setup Endpoint and DMA related parameters
 *              before transfer initiated.
 * Parameter: cy_stc_usb_app_ctxt_t, pEndpDscr
 * return: void
 */
void Cy_USB_AppSetupEndpDmaParams(cy_stc_usb_app_ctxt_t *pUsbApp,
                                  uint8_t *pEndpDscr)
{
    /* We always need to do SS setup so that we can get data from the LVDS SPI interface. */
    Cy_USB_AppSetupEndpDmaParamsSs(pUsbApp, pEndpDscr);

    if (pUsbApp->devSpeed <= CY_USBD_USB_DEV_HS)
    {
        Cy_USB_AppSetupEndpDmaParamsHs(pUsbApp, pEndpDscr);
    }
}

/*  Configure all endpoints used by application (except EP0) */
void Cy_USB_AppConfigureEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr)
{
    cy_stc_usb_endp_config_t endpConfig;
    cy_en_usb_endp_dir_t endpDirection;
    bool valid;
    uint32_t endpType;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;
    uint32_t isoPkts = 0x00;
    uint8_t burstSize = 0x00;
    uint8_t maxStream = 0x00;
    uint8_t *pCompDscr = NULL;
    cy_en_usbd_ret_code_t usbdRetCode;

    /* If it is not endpoint descriptor then return */
    if (!Cy_USBD_EndpDscrValid(pEndpDscr))
    {
        return;
    }
    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);

    if (dir)
    {
        endpDirection = CY_USB_ENDP_DIR_IN;
    }
    else
    {
        endpDirection = CY_USB_ENDP_DIR_OUT;
    }
    Cy_USBD_GetEndpType(pEndpDscr, &endpType);

    if ((CY_USB_ENDP_TYPE_ISO == endpType) || (CY_USB_ENDP_TYPE_INTR == endpType))
    {
        /* The ISOINPKS setting in the USBHS register is the actual packets per microframe value. */
        isoPkts = ((*((uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) & CY_USB_ENDP_ADDL_XN_MASK) >> CY_USB_ENDP_ADDL_XN_POS) + 1;
    }

    valid = 0x01;
    if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS)
    {
        /* Get companion descriptor and from there get burstSize. */
        pCompDscr = Cy_USBD_GetSsEndpCompDscr(pUsbdCtxt, pEndpDscr);
        Cy_USBD_GetEndpCompnMaxburst(pCompDscr, &burstSize);
        Cy_USBD_GetEndpCompnMaxStream(pCompDscr, &maxStream);
    }

    /* Prepare endpointConfig parameter. */
    endpConfig.endpType = (cy_en_usb_endp_type_t)endpType;
    endpConfig.endpDirection = endpDirection;
    endpConfig.valid = valid;
    endpConfig.endpNumber = endpNumber;
    endpConfig.maxPktSize = (uint32_t)maxPktSize;
    endpConfig.isoPkts = isoPkts;
    endpConfig.burstSize = burstSize;
    endpConfig.streamID = maxStream;
    endpConfig.interval = 0;
    /*
     * allowNakTillDmaRdy = true means device will send NAK
     * till DMA setup is ready. This field is applicable to only
     * ingress direction ie OUT transfer/OUT endpoint.
     * For Egress ie IN transfer, this field is ignored.
     */
    endpConfig.allowNakTillDmaRdy = TRUE;
    usbdRetCode = Cy_USB_USBD_EndpConfig(pUsbdCtxt, endpConfig);

    /* Print status of the endpoint configuration to help debug. */
    DBG_APP_INFO("#ENDPCFG: %d, %d\r\n", endpNumber, usbdRetCode);
    return;
} /* end of function */

/* Callback function will be invoked by USBD when set configuration is received */
void Cy_USB_AppSetCfgCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{
    cy_slff_devconfigured = true;
    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t *pActiveCfg, *pIntfDscr, *pEndpDscr;
    uint8_t index, numOfIntf, numOfEndp;
    uint16_t maxPktSize;
    uint32_t endpNumber, dir;
    cy_en_usb_speed_t devSpeed;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;


    DBG_APP_INFO("AppSetCfgCbStart\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    devSpeed = pUsbApp->devSpeed;

    /* Disable optional Low Power Mode transitions. */
    Cy_USBD_LpmDisable(pUsbdCtxt);

    /*
    * Based on type of application as well as how data flows,
    * data wire can be used so initialize datawire.
    */
    Cy_DMA_Enable(pUsbApp->pCpuDw0Base);
    Cy_DMA_Enable(pUsbApp->pCpuDw1Base);

    pActiveCfg = Cy_USB_USBD_GetActiveCfgDscr(pUsbdCtxt);
    if (!pActiveCfg)
    {
        /* Set config should be called when active config value > 0x00. */
        return;
    }
    numOfIntf = Cy_USBD_FindNumOfIntf(pActiveCfg);
    if (numOfIntf == 0x00)
    {
        return;
    }

    for (index = 0x00; index < numOfIntf; index++)
    {
        /* During Set Config command always altSetting 0 will be active. */
        pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, index, 0x00);
        if (pIntfDscr == NULL)
        {
            DBG_APP_INFO("pIntfDscrNull\r\n");
            return;
        }

        numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
        if (numOfEndp == 0x00)
        {
            DBG_APP_INFO("numOfEndp 0\r\n");
            continue;
        }

        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00)
        {
            Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParams(pUsbApp, pEndpDscr);

            Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);
            numOfEndp--;
            if (devSpeed > CY_USBD_USB_DEV_HS)
            {
                pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)) + 6);
            }
            else
            {
                pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
            }
        }

    }

    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_CONFIGURED;
    pUsbApp->devState = CY_USB_DEVICE_STATE_CONFIGURED;
	
    /* Enable data from FPGA*/
    DBG_APP_INFO("Enable Device\r\n");
    xMsg.type = CY_USB_STREAMING_START;
    xMsg.data[0] = 0x01;
    xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));

    DBG_APP_INFO("AppSetCfgCbEnd Done\r\n");
    return;
} /* end of function */

/* Callback function will be invoked by USBD when bus detects RESET */
void Cy_USB_AppBusResetCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t index;
    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("AppBusResetCallback\r\n");

    /* Stop and destroy the high bandwidth DMA channel if present. To be done before AppInit is called. */
    for(index = 0x01; index < (CY_USB_NUM_ENDP_CONFIGURED); index++ )
    {
        if(pUsbApp->hbBulkInChannel[index] != NULL)
        {
            Cy_HBDma_Channel_Disable(pUsbApp->hbBulkInChannel[index]);
            Cy_HBDma_Channel_Destroy(pUsbApp->hbBulkInChannel[index]);
        }
    }

    /*
     * USBD layer takes care of reseting its own data structure as well as
     * takes care of calling CAL reset APIs. Application needs to take care
     * of reseting its own data structure as well as "device function".
     */
    Cy_USB_AppInit(pUsbApp, pUsbdCtxt, pUsbApp->pCpuDmacBase, pUsbApp->pCpuDw0Base, pUsbApp->pCpuDw1Base, pUsbApp->pHbDmaMgrCtxt);
    pUsbApp->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_RESET;

    if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1)
    {
        Cy_USBSS_Cal_LPMDisable(pUsbdCtxt->pSsCalCtxt);

        if ((USB32DEV->USB32DEV_LNK.LNK_LTSSM_STATE & USB32DEV_LNK_LTSSM_STATE_LTSSM_STATE_Msk) ==
                CY_USBSS_LNK_STATE_RXDETECT_RES)
        {
            DBG_APP_TRACE("WarmReset received\r\n");
        }
    }
    return;
} /* end of function. */

/*Callback function will be invoked by USBD when RESET is completed */
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    DBG_APP_INFO("ppBusResetDoneCallback\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->prevDevState = pUsbApp->devState;
    return;
} /* end of function. */


/* Callback function will be invoked by USBD when speed is identified or 
  speed change is detected */
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    return;
} /* end of function. */

/* Callback function will be invoked by USBD when SETUP packet is received */
void Cy_USB_AppSetupCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    uint8_t bRequest, bReqType;
    uint8_t bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    bool isReqHandled = false;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    cy_en_usb_endp_dir_t epDir = CY_USB_ENDP_DIR_INVALID;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t status = 0;
    uint32_t epNumber;

    DBG_APP_INFO("AppSetupCallback\r\n");

    /* Decode the fields from the setup request. */
    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bType = ((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS);
    bTarget = (bReqType & CY_USB_CTRL_REQ_RECIPENT_OTHERS);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wValue = pUsbdCtxt->setupReq.wValue;
    wIndex = pUsbdCtxt->setupReq.wIndex;
    wLength = pUsbdCtxt->setupReq.wLength;

    if (bType == CY_USB_CTRL_REQ_STD)
    {
        DBG_APP_INFO("CY_USB_CTRL_REQ_STD\r\n");

        if (bRequest == CY_USB_SC_SET_FEATURE)
        {
            /* Function Suspend: ACK the request if the device is in configured state in an USB 3.x connection. */
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == 0))
            {
                if ((pUsbApp->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) && (cy_slff_devconfigured))
                    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                else
                    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);

                isReqHandled = true;
            }

            /* SET-FEATURE(EP-HALT) is only supported to facilitate Chapter 9 compliance tests. */
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (wValue == CY_USB_FEATURE_ENDP_HALT))
            {
                epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                        epDir, true);

                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }

        if (bRequest == CY_USB_SC_CLEAR_FEATURE)
        {
            /* Function Suspend: ACK the request if the device is in configured state in an USB 3.x connection. */
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == 0))
            {
                if ((pUsbApp->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) && (cy_slff_devconfigured))
                    Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                else
                    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);

                isReqHandled = true;
            }

            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (wValue == CY_USB_FEATURE_ENDP_HALT))
            {
                epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
                epNumber = (uint32_t)(wIndex & 0x7FUL);


				xMsg.type = CY_USB_STREAMING_STOP;
                xMsg.data[0] = epNumber;
                status = xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
                ASSERT_NON_BLOCK(pdTRUE == status,status);
                /* Reset the DMA channel through which data is received from the LVDS SPI side. */
                Cy_HBDma_Channel_Reset(pUsbApp->hbBulkInChannel[epNumber]);

                /* On USB 2.0 connection, reset the DataWire channel used to send data to the EPM. */
                if (pUsbApp->devSpeed <= CY_USBD_USB_DEV_HS)
                {
                    Cy_USBHS_App_ResetEpDma(&(pUsbApp->endpInDma[epNumber]));
                }

                /* Flush and reset the endpoint and clear the STALL bit. */
                Cy_USBD_FlushEndp(pUsbdCtxt, epNumber, CY_USB_ENDP_DIR_IN);
                Cy_USBD_ResetEndp(pUsbdCtxt, epNumber, CY_USB_ENDP_DIR_IN, false);
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL), epDir, false);
                pUsbApp->slffPendingRxBufCnt[epNumber]  = 0;

                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }

#if USE_WINUSB
        /* Handle Microsoft OS String Descriptor request. */
        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                (bRequest == CY_USB_SC_GET_DESCRIPTOR) &&
                (wValue == ((CY_USB_STRING_DSCR << 8) | 0xEE))) {

            /* Make sure we do not send more data than requested. */
            if (wLength > glOsString[0]) {
                wLength = glOsString[0];
            }

            DBG_APP_INFO("OSString\r\n");
            retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsString, wLength);
            if(retStatus == CY_USBD_STATUS_SUCCESS) {
                isReqHandled = true;
            }
        }
#endif /* USE_WINUSB */

        /* SET_SEL request is supposed to have an OUT data phase of 6 bytes. */
        if ((bRequest == CY_USB_SC_SET_SEL) && (wLength == 6))
        {
            DBG_APP_INFO("SET_SEL rqt\r\n");
            retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, (uint8_t *)SetSelDataBuffer, wLength);
            DBG_APP_INFO("EP0 recv stat: %d, Data=%x:%x\r\n", retStatus, SetSelDataBuffer[0], SetSelDataBuffer[1]);
            isReqHandled = true;
        }
    }

#if USE_WINUSB
    if (bType == CY_USB_CTRL_REQ_VENDOR) {
        /* If trying to bind to WinUSB driver, we need to support additional control requests. */
        /* Handle OS Compatibility and OS Feature requests */
        if (bRequest == MS_VENDOR_CODE) {
            if (wIndex == 0x04) {
                if (wLength > *((uint16_t *)glOsCompatibilityId)) {
                    wLength = *((uint16_t *)glOsCompatibilityId);
                }

                DBG_APP_INFO("OSCompat\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsCompatibilityId, wLength);
                if(retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }
            else if (wIndex == 0x05) {
                if (wLength > *((uint16_t *)glOsFeature)) {
                    wLength = *((uint16_t *)glOsFeature);
                }

                DBG_APP_INFO("OSFeature\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsFeature, wLength);
                if(retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }
        }

        if (isReqHandled) {
            return;
        }
    }
#endif /* USE_WINUSB */

    /* If Request is not handled by the callback, Stall the command */
    if (!isReqHandled)
    {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
} /* end of function. */


/* Callback function will be invoked by USBD when Suspend signal/message is detected */
void Cy_USB_AppSuspendCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->prevDevState = pUsbApp->devState;
    pUsbApp->devState = CY_USB_DEVICE_STATE_SUSPEND;
} /* end of function. */

/* Callback function will be invoked by USBD when Resume signal/message is detected */
void Cy_USB_AppResumeCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    cy_en_usb_device_state_t tempState;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    tempState = pUsbApp->devState;
    pUsbApp->devState = pUsbApp->prevDevState;
    pUsbApp->prevDevState = tempState;
    return;
} /* end of function. */

/* Callback function will be invoked by USBD when SET_INTERFACE is  received */
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_setup_req_t *pSetupReq;
    uint8_t intfNum, altSetting;
    int8_t numOfEndp;
    uint8_t *pIntfDscr, *pEndpDscr;
    uint32_t endpNumber;
    cy_en_usb_endp_dir_t endpDirection;
    cy_stc_usb_app_ctxt_t *pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("AppSetIntfCallback Start\r\n");
    pSetupReq = &(pUsbdCtxt->setupReq);
    /*
     * Get interface and alt setting info. If new setting same as previous
     * then return.
     * If new alt setting came then first Unconfigure previous settings
     * and then configure new settings.
     */
    intfNum = pSetupReq->wIndex;
    altSetting = pSetupReq->wValue;

    if (altSetting == pUsbApp->prevAltSetting)
    {
        DBG_APP_INFO("SameAltSetting\r\n");
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
        return;
    }

    /* New altSetting is different than previous one so unconfigure previous. */
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, pUsbApp->prevAltSetting);
    DBG_APP_INFO("unconfigPrevAltSet\r\n");
    if (pIntfDscr == NULL)
    {
        DBG_APP_INFO("pIntfDscrNull\r\n");
        return;
    }
    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp == 0x00)
    {
        DBG_APP_INFO("SetIntf:prevNumEp 0\r\n");
    }
    else
    {
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00)
        {
            if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80)
            {
                endpDirection = CY_USB_ENDP_DIR_IN;
            }
            else
            {
                endpDirection = CY_USB_ENDP_DIR_OUT;
            }
            endpNumber =
                (uint32_t)((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);

            /* with FALSE, unconfgure previous settings. */
            Cy_USBD_EnableEndp(pUsbdCtxt, endpNumber, endpDirection, FALSE);

            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
        }
    }

    /* Now take care of different config with new alt setting. */
    pUsbApp->prevAltSetting = altSetting;
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, altSetting);
    if (pIntfDscr == NULL)
    {
        DBG_APP_INFO("pIntfDscrNull\r\n");
        return;
    }

    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp == 0x00)
    {
        DBG_APP_INFO("SetIntf:numEp 0\r\n");
    }
    else
    {
        pUsbApp->prevAltSetting = altSetting;
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00)
        {
            Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParams(pAppCtxt, pEndpDscr);
            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
        }
    }

    DBG_APP_INFO("AppSetIntfCallback done\r\n");
    return;
}


/*
 * Function: Cy_USB_AppSlpCallback()
 * Description: This Function will be called by USBD layer when
 *              SLP message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    DBG_APP_INFO("AppSlpCb\r\n");
    return;

} /* end of function. */


void Cy_USB_AppQueueWrite(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber,
                          uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t *dmaset_p;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
            (pAppCtxt->pCpuDw1Base == NULL) || (pBuffer == NULL))
    {
        DBG_APP_ERR("QueueWrite Err0\r\n");
        return;
    }

    /*
     * Verify that the selected endpoint is valid and the dataSize
     * is non-zero.
     */
    dmaset_p = &(pAppCtxt->endpInDma[endpNumber]);
    if ((dmaset_p->valid == 0) || (dataSize == 0))
    {
        DBG_APP_ERR("QueueWrite Err1\r\n");
        return;
    }

    Cy_USBHS_App_QueueWrite(dmaset_p, pBuffer, dataSize);
} /* end of function */


void Cy_USB_AppInitDmaIntr(uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection,
                           cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;
    if ((endpNumber > 0) && (endpNumber < CY_USB_NUM_ENDP_CONFIGURED))
    {
#if (!CY_CPU_CORTEX_M4)
        intrCfg.intrPriority = 3;
        intrCfg.intrSrc = NvicMux7_IRQn;
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
            DBG_APP_ERR("Wrong EP DIR \r\n");
            return;
        }
#else
        intrCfg.intrPriority = 5;
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            /* DW1 channels 0 onwards are used for IN endpoints. */
            intrCfg.intrSrc =
                (IRQn_Type)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
            DBG_APP_ERR("Wrong EP DIR \r\n");
            return;
        }
#endif /* (!CY_CPU_CORTEX_M4) */

        if (userIsr != NULL)
        {
            /* If an ISR is provided, register it and enable the interrupt. */
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        }
        else
        {
            /* ISR is NULL. Disable the interrupt. */
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }
    }
}

void Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
                                 uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection)
{
    if ((pAppCtxt != NULL) && (endpNumber > 0) &&
            (endpNumber < CY_USB_NUM_ENDP_CONFIGURED)) {
        if (endpDirection == CY_USB_ENDP_DIR_IN) {
            Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpInDma[endpNumber]));
        } else  {
            DBG_APP_ERR("Wrong EP DIR \r\n");
        }
    }
}


void Cy_Slff_AppHandleRxCompletion (cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t endpNum)
{
    cy_stc_hbdma_buff_status_t buffStat;
    cy_en_hbdma_mgr_status_t   dmaStat;

    /* At least one buffer must be pending. */
    if ((pUsbApp->slffPendingRxBufCnt[endpNum]) == 0)
    {
        DBG_APP_ERR("PendingBufCnt=0 on SendComplete\r\n");
        return;
    }

    /* The buffer which has been sent to the USB host can be discarded. */
    dmaStat = Cy_HBDma_Channel_DiscardBuffer(pUsbApp->hbBulkInChannel[endpNum], &buffStat);
    if (dmaStat != CY_HBDMA_MGR_SUCCESS)
    {
        DBG_APP_ERR("DiscardBuffer failed with status=%x\r\n", dmaStat);
        return;
    }

    /* If another DMA buffer has already been filled by the producer, go
     * on and send it to the host controller.
     */

     pUsbApp->slffPendingRxBufCnt[endpNum]--;
    if ((pUsbApp->slffPendingRxBufCnt[endpNum]) > 0)
    {
        Cy_Slff_AppHandleRxEvent(pUsbApp, pUsbApp->hbBulkInChannel[endpNum], 0);
    }
}

/*******************************************************************************
* Function name: Cy_CheckStatus(const char *function, uint32_t line,
* 								uint8_t condition, uint32_t value,
* 								uint8_t isBlocking)
****************************************************************************//**
*
* Description: Function that handles prints error log
*
* \param function
* Pointer to function
*
* \param line
* Line number where error is seen
*
* \param condition
*  condition of failure
*
* \param value
*  error code
*
* \param isBlocking
*  blocking function
*
* \return
* None
*
********************************************************************************/
void Cy_checkStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking)
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, RED);
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        Cy_Debug_AddToLog(1, COLOR_RESET);
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

/*******************************************************************************
* Function name: Cy_CheckStatusHandleFailure(const char *function, uint32_t line,
* 								uint8_t condition, uint32_t value,
* 								uint8_t isBlocking, void (*failureHandler)(void))
****************************************************************************//**
*
* Description: Function that handles prints error log
*
* \param function
* Pointer to function
*
* \param line
* LineNumber where error is seen
*
* \param condition
* Line number where error is seen
*
* \param value
*  error code
*
* \param isBlocking
*  blocking function
*
* \param failureHandler
*  failure handler function
*
* \return
* None
*
********************************************************************************/
void Cy_checkStatusAndHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)(void))
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, RED);
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        Cy_Debug_AddToLog(1, COLOR_RESET);

        if(failureHandler != NULL)
        {
            (*failureHandler)();
        }
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

/* [] END OF FILE */
