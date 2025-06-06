/***************************************************************************//**
* \file cy_usb_app.h
* \version 1.0
*
* Header file providing interface definitions for the USB Slave FIFO application.
*
*******************************************************************************
* \copyright
* (c) (2021-2023), Cypress Semiconductor Corporation (an Infineon company) or
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

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define RED                             "\033[0;31m"
#define CYAN                            "\033[0;36m"
#define COLOR_RESET                     "\033[0m"

#define LOG_COLOR(...)                  Cy_Debug_AddToLog(1,CYAN);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);

#define LOG_ERROR(...)                  Cy_Debug_AddToLog(1,RED);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);

#define LOG_CLR(CLR, ...)               Cy_Debug_AddToLog(1,CLR);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);


#define LOG_TRACE()                     LOG_COLOR("-->[%s]:%d\r\n",__func__,__LINE__);


#define DELAY_MICRO(us)                 Cy_SysLib_DelayUs(us)
#define DELAY_MILLI(ms)                 Cy_SysLib_Delay(ms)

#define PHY_TRAINING_PATTERN_BYTE      (0x39)
#define LINK_TRAINING_PATTERN_BYTE     (0xAA55AA55) // Working all the time with 100u phy train interval
#define FPS_DEFAULT                     (60)

#define SET_BIT(byte, mask)		       (byte) |= (mask)
#define CLR_BIT(byte, mask)		       (byte) &= ~(mask)
#define CHK_BIT(byte, mask)		       (byte) & (mask)

#define DMA_BUFFER_SIZE                (SLFF_RX_MAX_BUFFER_SIZE)
#define FPGA_DMA_BUFFER_SIZE           (DMA_BUFFER_SIZE)

#if (LVCMOS_EN && (!LVCMOS_DDR_EN))
#define LINK_TRAINING_EN                  (0)
#else
#define LINK_TRAINING_EN                  (1)
#endif

#define LINK_READY_CTL_PIN                (16 + 6)  // 16 Data Lanes + 6th CTRL Lane
#define PORT_0                            (0) // For NL PORT0 & WL case

#define SLFF_TX_MAX_BUFFER_COUNT          (4)
#define SLFF_TX_MAX_BUFFER_SIZE           (16384)
#define ASSERT(condition, value)           Cy_checkStatus(__func__, __LINE__, condition, value, true);
#define ASSERT_NON_BLOCK(condition, value) Cy_checkStatus(__func__, __LINE__, condition, value, false);
#define ASSERT_AND_HANDLE(condition, value, failureHandler) Cy_checkStatusAndHandleFailure(__func__, __LINE__, condition, value, false, failureHandler);
#define SLFF_RX_MAX_BUFFER_COUNT          (2)
#define SLFF_RX_MAX_BUFFER_SIZE           (64512)
#define CY_USB_DEVICE_MSG_QUEUE_SIZE      (16)
#define CY_USB_DEVICE_MSG_SIZE            (sizeof (cy_stc_usbd_app_msg_t))
#define CY_USB_VBUS_CHANGE_INTR           (0x0E)
#define CY_USB_VBUS_CHANGE_DEBOUNCED      (0x0F)
#define CY_USB_STREAMING_START            (0x10)
#define CY_USB_STREAMING_STOP             (0x11)

/* P4.0 is used for VBus detect functionality. */
#define VBUS_DETECT_GPIO_PORT             (P4_0_PORT)
#define VBUS_DETECT_GPIO_PIN              (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR             (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE                 (0u)

/* GPIO port pins*/
#define TI180_CRESET_GPIO		        (P4_3_GPIO)
#define TI180_CRESET_GPIO_PORT          (P4_3_PORT)
#define TI180_CRESET_GPIO_PIN           (P4_3_PIN)

#define USB3_DESC_ATTRIBUTES __attribute__ ((section(".descSection"), used)) __attribute__ ((aligned (32)))
#define HBDMA_BUF_ATTRIBUTES __attribute__ ((section(".hbBufSection"), used)) __attribute__ ((aligned (32)))

/* Vendor command code used to return WinUSB specific descriptors. */
#define MS_VENDOR_CODE        (0xF0)
#define CY_USB_NUM_ENDP_CONFIGURED 5 // 4: 81 to 84
#define CY_SLFF_STREAM_EP_BURST (0x1)
#define PORT0_LVDS_SOCKET_START_INDEX   0x10
#define PORT1_LVDS_SOCKET_START_INDEX   0x20

extern uint8_t glOsString[];
extern uint8_t glOsCompatibilityId[];
extern uint8_t glOsFeature[];

typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;


/* USBD layer return code shared between USBD layer and Application layer. */
typedef enum cy_en_usb_app_ret_code_ {
    CY_USB_APP_STATUS_SUCCESS=0,
    CY_USB_APP_STATUS_FAILURE,
}cy_en_usb_app_ret_code_t;

/*
 * Number of DMA descriptors needed per DMA/DW channel to read/write data is 3.
 * The first one is used to transfer all full packets using 4-byte transfers.
 * The second one is used to transfer the 4-byte multiple part of any short packet at the end of the transfer.
 * The third one is used to transfer the 1 to 3 bytes of remaining data using 1-byte transfers.
 */
#define DSCRS_PER_CHN   (3u)

/* Get the LS byte from a 16-bit number */
#define CY_GET_LSB(w)                   ((uint8_t)((w) & UINT8_MAX))

/* Get the MS byte from a 16-bit number */
#define CY_GET_MSB(w)                   ((uint8_t)((w) >> 8))

#define LVCMOS_GPIF_CTRLBUS_BITMAP      (0x0000038F)
#define LVCMOS_GPIF_CTRLBUS_BITMAP_WL   (0x000C008F)
#define FPGA_DEVICE_OFFSET              (0x1C)

/* 
 * USB application data structure which is bridge between USB system and device
 * functionality.
 * It maintains some usb system information which comes from USBD and it also
 * maintains info about functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    bool slffReadInQueue[CY_USB_NUM_ENDP_CONFIGURED];
    bool slffSlpRx;
    bool usbConnectDone;
    bool vbusChangeIntr;
    bool vbusPresent;
    bool usbConnected;
    cy_en_usb_speed_t desiredSpeed;
    TimerHandle_t vbusDebounceTimer;
    uint8_t slffPendingRxBufCnt[CY_USB_NUM_ENDP_CONFIGURED];

    uint8_t firstInitDone;
    uint8_t devAddr;
    uint8_t activeCfgNum;
    uint8_t *pSlffTxBuffer [CY_USB_NUM_ENDP_CONFIGURED ][SLFF_TX_MAX_BUFFER_COUNT];
    uint16_t slffTxBufferCount[CY_USB_NUM_ENDP_CONFIGURED][SLFF_TX_MAX_BUFFER_COUNT];
    cy_en_usb_device_state_t devState;
    cy_en_usb_device_state_t prevDevState;
    cy_en_usb_speed_t devSpeed;
    cy_en_usb_enum_method_t enumMethod;
    uint8_t prevAltSetting;
    cy_stc_app_endp_dma_set_t endpInDma[CY_USB_NUM_ENDP_CONFIGURED];
    DMAC_Type *pCpuDmacBase;
    DW_Type *pCpuDw0Base;
    DW_Type *pCpuDw1Base;
    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;
    cy_stc_hbdma_channel_t *hbBulkInChannel[CY_USB_NUM_ENDP_CONFIGURED];
    TaskHandle_t slffTaskHandle;
    QueueHandle_t usbMsgQueue;

    uint8_t fpgaVersion;
    uint8_t glpassiveSerialMode;
    uint8_t *qspiWriteBuffer;
    uint8_t *qspiReadBuffer;
};

/*FGPA Register Map*/
typedef enum FPGAI2CRegMap_t
{
    /*Common Register Info*/
    FPGA_MAJOR_VERSION_ADDRESS             = 0x00,
    FPGA_MINOR_VERSION_ADDRESS             = 0x00,

    FPGA_UVC_U3V_SELECTION_ADDRESS         = 0x01,
    FPGA_UVC_ENABLE                        = 1,
    FPGA_U3V_ENABLE                        = 0,

    FPGA_UVC_HEADER_CTRL_ADDRESS           = 0x02,
    FPGA_UVC_HEADER_ENABLE                 = 1,
    FPGA_UVC_HEADER_DISABLE                = 0, // FX2G3 will add UVC Header

    FPGA_LVDS_PHY_TRAINING_ADDRESS         = 0x03,
    FPGA_LVDS_PHY_TRAINING_DATA            = 0x39,

    FPGA_LVDS_LINK_TRAINING_BLK_P0_ADDRESS = 0x04,
    FPGA_LVDS_LINK_TRAINING_BLK_P1_ADDRESS = 0x05,
    FPGA_LVDS_LINK_TRAINING_BLK_P2_ADDRESS = 0x06,
    FPGA_LVDS_LINK_TRAINING_BLK_P3_ADDRESS = 0x07,

    FPGA_ACTIVE_DIVICE_MASK_ADDRESS        = 0x08,
    FPGA_LOW_PWR_MODE_ADDRESS              = 0x09,

    FPGA_PHY_LINK_CONTROL_ADDRESS          = 0x0A,
    FPGA_TRAINING_DISABLE                  = 0x00,
    FPGA_PHY_CONTROL                       = 0x01, // PHY Training is required
    FPGA_LINK_CONTROL                      = 0x02, //  Link Training is required

    P0_TRAINING_DONE                       = 0X40, // Port 0 training is completed
    P1_TRAINING_DONE                       = 0X80, // Port 1 training is completed

    FPGA_EXT_CONTROLLER_STATUS_ADDRESS     = 0X0B,
    DMA_READY_STATUS                       = 0x01, // DMA Ready flag status
    DDR_CONFIG_STATUS                      = 0x02, // DDR configuration status
    DDR_BUSY_STATUS                        = 0x04, // DDR Controller busy status
    DDR_CMD_QUEUE_FULL_STATUS              = 0x08, // Command queue full status
    DATPATH_IDLE_STATUS                    = 0x10, // Datapath is idle or not


    /*Device related Info*/
    DEVICE0_OFFSET                         = 0x20,
    DEVICE1_OFFSET                         = 0x3C,
    DEVICE2_OFFSET                         = 0x58,
    DEVICE3_OFFSET                         = 0x74,
    DEVICE4_OFFSET                         = 0x90,
    DEVICE5_OFFSET                         = 0xAC,
    DEVICE6_OFFSET                         = 0xC8,
    DEVICE7_OFFSET                         = 0xE4,

    FPGA_DEVICE_STREAM_ENABLE_ADDRESS      = 0x00,
    CAMERA_APP_DISABLE                     = 0x00,
    DMA_CH_RESET                           = 0x01,
    CAMERA_APP_ENABLE                      = 0x02,
    APP_STOP_NOTIFICATION                  = 0x04,

    FPGA_DEVICE_STREAM_MODE_ADDRESS        = 0x01,
    NO_CONVERSION                          = 0,
    INTERLEAVED_MODE                       = 0x01,
    STILL_CAPTURE                          = 0x02,
    MONO_8_CONVERSION                      = 0x04,
    YUV422_420_CONVERSION                  = 0x08,

    DEVICE_IMAGE_HEIGHT_LSB_ADDRESS        = 0x02,
    DEVICE_IMAGE_HEIGHT_MSB_ADDRESS        = 0x03,
    DEVICE_IMAGE_WIDTH_LSB_ADDRESS         = 0x04,
    DEVICE_IMAGE_WIDTH_MSB_ADDRESS         = 0x05,
    
    DEVICE_FPS_ADDRESS                     = 0x06,

    DEVICE_PIXEL_WIDTH_ADDRESS             = 0x07,
    _8_BIT_PIXEL                           = 8,
    _12BIT_PIXEL                           = 12,
    _16BIT_PIXEL                           = 16,
    _24BIT_PIXEL                           = 24,
    _36BIT_PIXEL                           = 36,

    DEVICE_SOURCE_TYPE_ADDRESS             = 0x08,
    INTERNAL_COLORBAR                      = 0x00,
    HDMI_SOURCE                            = 0x01,
    MIPI_SOURCE                            = 0x02,

    DEVICE_FLAG_STATUS_ADDRESS             = 0x09,
    SLAVE_FIFO_ALMOST_EMPTY                = 0x01, // Slave FIFO almost empty status
    INTER_MED_FIFO_EMPTY                   = 0x02, // Intermediate FIFO empty status
    INTER_MED_FIFO_FULL                    = 0x04, // Intermediate FIFO full status
    DDR_FULL_FRAME_WRITE_COMPLETE          = 0x10, // DDR write status (Full frame write complete)
    DDR_FULL_FRAME_READ_COMPLETE           = 0x20, // DDR write status (Full frame read complete)

    DEVICE_MIPI_STATUS_ADDRESS             = 0x0A,

    DEVICE_HDMI_SOURCE_INFO_ADDRESS        = 0x0B,
    HDMI_DISCONECT                         = 0x00, // Source connection status
    THIN_MIPI                              = 0x00,
    HDMI_CONNECT                           = 0x01, // Source connection status
    HDMI_DUAL_CH                           = 0x02, // 0 for single channel and 1 for dual channel
    SONY_CIS                               = 0x10, // Enable CIS ISP IP (Valid for only MIPI source)
    SONY_MIPI                              = 0x20, // Enable Crop Algorithm (Valid for only MIPI source)

    DEVICE_U3V_STREAM_MODE_ADDRESS         = 0x0C,
    DEVICE_U3V_CHUNK_MODE_ADDRESS          = 0x0D,
    DEVICE_U3V_TRIGGER_MODE_ADDRESS        = 0x0E,
    DEVICE_ACTIVE_TREAD_INFO_ADDRESS       = 0x0F,
    DEVICE_THREAD1_INFO_ADDRESS            = 0x10,
    DEVICE_THREAD2_INFO_ADDRESS            = 0x11,
    DEVICE_THREAD1_SOCKET_INFO_ADDRESS     = 0x12,
    DEVICE_THREAD2_SOCKET_INFO_ADDRESS     = 0x13,

    DEVICE_FLAG_INFO_ADDRESS               = 0x14,
    FX2G3_READY_TO_REC_DATA                 = 0x08,
    NEW_UVC_PACKET_START                   = 0x02,
    NEW_FRAME_START                        = 0x01,

    DEVICE_COUNTER_CRC_INFO_ADDRESS        = 0x15,
    DEVICE_BUFFER_SIZE_LSB_ADDRESS         = 0x16,
    DEVICE_BUFFER_SIZE_MSB_ADDRESS         = 0x17,

} FPGAI2CRegMap_t;

/* FPGA Configuration mode selection*/
typedef enum cy_en_fpgaConfigMode_t
{
    ACTIVE_SERIAL_MODE,
    PASSIVE_SERIAL_MODE
}cy_en_fpgaConfigMode_t;

typedef enum cy_en_streamControl_t
{
    STOP,
    START
}cy_en_streamControl_t;

void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    DMAC_Type *pCpuDmacBase, DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);

void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);

void Cy_USB_AppSetCfgCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetupCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSuspendCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppResumeCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1SleepCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1ResumeCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetFeatureCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppClearFeatureCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppQueueRead (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);
uint16_t Cy_USB_AppReadShortPacket(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint16_t pktSize);
void Cy_USB_AppQueueWrite (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);
void Cy_USB_AppInitDmaIntr(uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection, cy_israddress userIsr);
void Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt, uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection);
void Cy_Slff_AppHandleRxCompletion(cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t endpNum);
void Cy_Slff_AppHandleTxCompletion(cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t endpNum);

/* Initialize the LVDS/LVCMOS interface to receive the data through. */
void Cy_LVDS_LVCMOS_Init(void);

/* Enable the USB data connection. */
bool Cy_USB_SSConnectionEnable(cy_stc_usb_app_ctxt_t *pAppCtxt);

/* Disable the USB data connection. */
void Cy_USB_SSConnectionDisable(cy_stc_usb_app_ctxt_t *pAppCtxt);

cy_en_scb_i2c_status_t Cy_FPGAGetVersion(cy_stc_usb_app_ctxt_t *pAppCtxt);

void Cy_checkStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking);

void Cy_checkStatusAndHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)());

/*Send Link Traning info to FPGA*/
cy_en_scb_i2c_status_t Cy_FPGAPhyLinkTraining (void);
#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */

