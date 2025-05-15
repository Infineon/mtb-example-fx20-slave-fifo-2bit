# EZ-USB&trade; FX20: Slave FIFO 2-bit application

This code example demonstrates the implementation of LVCMOS to USB data transfer application using a 2-bit Slave FIFO interface on the EZ-USB&trade; FX20 device. This example illustrates the configuration and usage of Sensor Interface Port (SIP) on the EZ-USB&trade; FX20 device to receive multiple streams of data and forward them on separate USB endpoints.

> **Note:** This code example is also applicable to EZ-USB&trade; FX10, EZ-USB&trade; FX5N, and EZ-USB&trade; FX5 devices.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-fx20-slave-fifo-2bit)

[Provide feedback on this code example.](https://cypress.co1.qualtrics.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyNDEzNTgiLCJTcGVjIE51bWJlciI6IjAwMi00MTM1OCIsIkRvYyBUaXRsZSI6IkVaLVVTQiZ0cmFkZTsgRlgyMDogU2xhdmUgRklGTyAyLWJpdCBhcHBsaWNhdGlvbiIsInJpZCI6InN1a3UiLCJEb2MgdmVyc2lvbiI6IjEuMC4wIiwiRG9jIExhbmd1YWdlIjoiRW5nbGlzaCIsIkRvYyBEaXZpc2lvbiI6Ik1DRCIsIkRvYyBCVSI6IldJUkVEIiwiRG9jIEZhbWlseSI6IlNTX1VTQiJ9)


## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.4 or later (tested with v3.4)
- Board support package (BSP) minimum required version: 4.3.3
- Programming language: C


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v11.3.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.22 (`ARM`)


## Supported kits (make variable 'TARGET')

- [EZ-USB&trade; FX20 DVK](https://www.infineon.com/fx20) (`KIT_FX20_FMC_001`) – Default value of `TARGET`


## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

The [Titanium Ti180 J484 Development Kit](https://www.efinixinc.com/products-devkits-titaniumti180j484.html) is used to drive the LVCMOS interface on the FX20 device based on the Synchronous Slave FIFO protocol. The FMC connector (J8) on the FX20 DVK should be connected to the FMC connector on the J484 board while running the application.


## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.

- Install a terminal emulator if you do not have one. Instructions in this document use [Tera Term](https://teratermproject.github.io/index-en.html)

- Install the [EZ-USB&trade; FX Control Center](https://softwaretools.infineon.com/tools/com.ifx.tb.tool.ezusbfxcontrolcenter) application to help with device programming and testing

- Install the [Efinity Software](https://www.efinixinc.com/products-efinity.html) from EFINIX to get the tools to program the Titanium Ti180 J484 Development Board


## Using the code example


### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool

   There are several ways to do this, including launching it from the dashboard or from inside the Eclipse IDE. For more details, see the [Project Creator user guide](https://www.infineon.com/ModusToolboxProjectCreator) (locally available at *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/docs/project-creator.pdf*)

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-kits-make-variable-target)

   > **Note:** To use this code example for a kit not listed here, you may need to update the source files. If the kit does not have the required resources, the application may not work

3. On the **Select Application** page:

   a. Select the **Applications(s) Root Path** and the **Target IDE**

      > **Note:** Depending on how you open the Project Creator tool, these fields may be pre-selected for you

   b. Select this code example from the list by enabling its check box

      > **Note:** You can narrow the list of displayed examples by typing in the filter box

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**

   d. Click **Create** to complete the application creation process

</details>


<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing "modus-shell" in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The following example clones the "[mtb-example-fx20-slave-fifo-2bit](https://github.com/Infineon/mtb-example-fx20-slave-fifo-2bit)" application with the desired name "Slave_FIFO_2Bit" configured for the *KIT_FX20_FMC_001* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id KIT_FX20_FMC_001 --app-id mtb-example-fx20-slave-fifo-2bit --user-app-name Slave_FIFO_2Bit --target-dir "C:/mtb_projects"
   ```

The 'project-creator-cli' tool has the following arguments:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the <id> field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the <id> field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional

<br>

> **Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For details, see the "Project creator tools" section of the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at {ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf).

</details>


### Open the project

After the project has been created, you can open it in your preferred development environment.


<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

For more details, see the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_ide_user_guide.pdf*).

</details>


<details><summary><b>Visual Studio (VS) Code</b></summary>

Launch VS Code manually, and then open the generated *{project-name}.code-workspace* file located in the project directory.

For more details, see the [Visual Studio Code for ModusToolbox&trade; user guide](https://www.infineon.com/MTBVSCodeUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mt_vscode_user_guide.pdf*).

</details>


<details><summary><b>Command line</b></summary>

If you prefer to use the CLI, open the appropriate terminal, and navigate to the project directory. On Windows, use the command-line 'modus-shell' program; on Linux and macOS, you can use any terminal application. From there, you can run various `make` commands.

For more details, see the [ModusToolbox&trade; tools package user guide](https://www.infineon.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation

1. Connect the board (J2) to your PC using the provided USB cable. Connect the USB FS port (J3) on the board to the PC for debug logs

2. Open a terminal program and select the serial COM port. Set the serial port parameters to 8N1 and 921600 baud

3. Connect the FPGA (Titanium Ti180 J484 Development Kit) DVK to the FMC (J8) connector of *KIT_FX20_FMC_001* board

4. Browse the **\<CE Title>/BitFiles/** folder for Ti180 FPGA binary and program the FPGA. For more details, see the [J484 kit user guide](https://www.efinixinc.com/products-devkits-titaniumti180j484.html)

5. Perform the following steps to program the board using the **EZ-USB&trade; FX Control Center** (Alpha) application

   1. To enter into Bootloader mode:

         a. Press and hold the **PMODE** (**SW2**) switch<br>
         b. Press and release the **RESET** (**SW3**) switch<br>
         c. Finally, release the **PMODE** switch<br>

   2. Open the **EZ-USB&trade; FX Control Center** application <br>
      The EZ-USB&trade; FX20 device enumerates as **EZ-USB&trade; FX Bootloader**

   3. Select the **EZ-USB&trade; FX Bootloader** device in **EZ-USB&trade; FX Control Center**

   4. Click **Program** > **Internal Flash**

   5. Navigate to the **\<CE Title>/build/APP_KIT_FX20_FMC_001/Release** folder within the CE directory and locate the *.hex* file and program<br>
      Confirm if the programming is successful in the log window of the **EZ-USB&trade; FX Control Center** application

6. After programming, the application starts automatically. Confirm that the following title is displayed on the UART terminal

   **Figure 1. Terminal output on program startup**

   ![](images/terminal-fx20-slave-fifo-2bit.png)

7. Using the **EZ-USB&trade; FX Control Center** application, go to the **Performance Measurement** tab and select the Endpoints 0x81, 0x82, 0x83, and 0x84 in round robin fashion. Click **Start** to start the data transfer and click **Stop** when errors are reported. Then, move to the next endpoint in the round robin way and repeat the data transfers

8. The device sends 16.5 MB (16,588,800 bytes) of data on each endpoint before it moves to the next endpoint. Use the Control Center application to read the data available on each endpoint before moving forward to the next one


## Debugging


### Using Arm&reg; debug port

If you have access to a MiniProg or KitProg3 device, you can debug the example to step through the code.


<details><summary><b>In Eclipse IDE</b></summary>

Use the **\<Application Name> Debug (KitProg3_MiniProg4)** configuration in the **Quick Panel**. For details, see the "Program and debug" section in the [Eclipse IDE for ModusToolbox&trade; user guide](https://www.infineon.com/MTBEclipseIDEUserGuide).

</details>


<details><summary><b>In other IDEs</b></summary>

Follow the instructions in your preferred IDE.

</details>


### Log messages

The code example and the EZ-USB&trade; FX20 stack output debug log messages indicates any unexpected conditions and highlights the operations performed.

By default, the USB FS port is enabled for debug logs. To enable debug logs on UART, set the `USBFS_LOGS_ENABLE` compiler flag to '0u' in the *Makefile*. SCB1 of the EZ-USB&trade; FX20 device is used as UART with a baud rate of 921600 to send out log messages through the P8.1 pin.

The verbosity of the debug log output can be modified by setting the `DEBUG_LEVEL` macro in *main.c* file with the values shown in **Table 1**.

**Table 1. Debug values**

Macro value | Description
:-------- | :------------
1u | Enable only error messages
2u | Enable error and warning messages
3u | Enable info messages
4u | Enable all message types

<br>


## Design and implementation

This code example demonstrates the implementation of the Synchronous Slave FIFO protocol with 2-bit addressing on the LVCMOS interface of the EZ-USB&trade; FX20 device. As 2-bit addressing is used, the FPGA master can access four different FIFOs on the EZ-USB&trade; FX20 device and write data into them. The data received on each of these FIFO is forwarded by the EZ-USB&trade; FX20 to the USB host over a different USB Bulk-IN endpoint.

> **Note:** Though the Slave FIFO protocol allows bidirectional data transfers, this application only supports data transfers from the FPGA into the EZ-USB&trade; FX20 device and then to the USB interface.

In addition to the LVCMOS and USB interfaces, this application uses the following low-performance peripherals to interface with the system:
- Serial Communication Block (SCB0) is used in I2C Master mode to configure and enable/disable the data stream from the FPGA
- Serial Memory Interface (SMIF) is used in x4 or Quad mode to configure the FPGA device in Passive Serial mode on every boot-up
- Debug logs from the application are output using either the Full-Speed USB debug port (J3) or using SCB1 in UART mode


### Features

This code example demonstrates the following capabilities of the EZ-USB&trade; FX20 device:

- The usage of the EZ-USB&trade; FX20 APIs to implement a vendor-specific USB device capable of operating up to 20 Gbps speeds
- The usage of the EZ-USB&trade; FX20 APIs to implement a Synchronous Slave FIFO interface to receive multiple streams of data from an FPGA master
- FPGA configuration using SMIF Block of the EZ-USB&trade; FX20 device
- I2C accesses to configure the FPGA data generator through a register interface


### Streaming data path

- The application enables four EPs IN (1-IN, 2-IN, 3-IN, and 4-IN) as Bulk endpoints with maximum packet size of 1024 bytes. In case of USB 3.2 operation, the endpoints are configured to support maximum burst of 16 packets each.
- The Sensor Interface Port (SIP) on EZ-USB&trade; FX20 is configured as a WideLink LVCMOS port with 32-bit wide data bus
- Four threads are used to receive separate streams of data from the FPGA. Two bits of address are used to select the thread for data transfer. Each thread is mapped to a single socket in the DMA adapter, which is then connected to a different USB endpoint. The mapping of FIFO address to the thread, socket, and USB endpoint is shown in **Table 2**

   **Table 2. FIFO address to thread, socket, and endpoint mapping**
   
   FIFO address | GPIF thread | SIP Socket | USB endpoint
   :-------- | :------------ | :-------- | :------------
   'b00 | Thread-0 | Socket 0 | 1-IN
   'b01 | Thread-1 | Socket 1 | 2-IN
   'b10 | Thread-2 | Socket 2 | 3-IN
   'b11 | Thread-3 | Socket 3 | 4-IN
    
   <br>
  
4. The SIP sockets are connected to the USB endpoints using high bandWidth DMA channels, which use two RAM buffers of 63 KB (64512 bytes) each. The DMA channels are configured as **auto** channels with no firmware intervention required during the data transfer


### Application workflow

The application flow involves three main steps:
- [Initialization](#initialization)
- [USB device enumeration](#usb-device-enumeration)
- [Data transfer](#data-transfer)


#### Initialization

During initialization, the following steps are performed:

1. All the required data structures are initialized
2. USBD and USB driver (CAL) layers are initialized
3. Application registers all descriptors supported by function/application with the USBD layer
4. Application registers callback functions for different events, such as `RESET`, `SUSPEND`, `RESUME`, `SET_CONFIGURATION`, `SET_INTERFACE`, `SET_FEATURE`, and `CLEAR_FEATURE`. USBD will call the respective callback function when the corresponding events are detected
5. Initialize the data transfer state machines
6. Application registers handlers for all relevant interrupts
7. Application makes the USB device visible to the host by calling the connect API
8. FPGA is configured using the SMIF (in x4 or Quad mode) block to read the bit file stored in the external flash. FPGA sees the data on the bus and gets configured
9. FPGA is initialized using I2C writes to FPGA registers
10. Application initializes the SIP block on the EZ-USB&trade; FX20 as required by the selected LVCMOS operating mode
11. If the LVCMOS interface is used in Dual Data Rate (DDR) mode, the `LINK_READY` pin is asserted HIGH to let the FPGA know that the link training pattern should be driven on the data bus
12. EZ-USB&trade; FX20 device and FPGA are ready for data transfers


#### USB device enumeration

1. During USB device enumeration, the host requests for descriptors, which are already registered with the USBD layer during the initialization phase
2. Host will send `SET_CONFIGURATION` and `SET_INTERFACE` commands to activate the required function in the device
3. As part of the `SET_CONFIGURATION` handler, the application creates and enables all DMA channels that move the data from the LVCMOS interface to the USB endpoints


#### Data transfer

1. After enabling the streaming DMA channels, the DMA ready flag on the SIP interface asserts indicating that the device is ready to receive data from the FPGA.
2. The FPGA data source starts sending data to each of the FIFOs (threads) in a round-robin way. The 16588800 bytes of data (equivalent to one YUY2 video frame with 4K resolution) are sent to each FIFO before the FPGA moves to the next one
3. Data moves from the LVCMOS subsystem to SRAM through high-bandwidth DMA
4. The data is forwarded on the USB 3.2 or USB 2.1 endpoints, based on the active USB connection speed. DataWire DMA channels are used in the case of USB 2.1 transfers and high-bandwidth DMA channels are used in case of USB 3.2 transfers


#### Slave FIFO interface

The code example uses a Synchronous Slave FIFO interface with 2-bit addressing. The Slave FIFO interface connections are as follows:

**Table 3. Control signal usage in LVCMOS Slave FIFO state machine for PORT0 in WideLink mode**

EZ-USB&trade; FX pin | Function | Description
:-------- | :------------ | :--------
P0CLK | PCLK | LVCMOS clock input
P0CTL0 | SLCS# | Active Low Chip Select signal. Assert by FPGA when communicating with EZ-USB&trade; FX20
P0CTL1 | SLWR# | Active Low Write Enable signal. Assert by FPGA when sending data to EZ-USB&trade; FX20
P0CTL2 | SLOE# | Active Low Output Enable signal. Not used in this application and expected to be driven HIGH by FPGA
P0CTL3 | SLRD# | Active Low Read Enable signal. Not used in this application and expected to be driven HIGH by FPGA
P0CTL5 | FlagA | Active Low DMA ready indication for currently addressed (active) thread
P0CTL6 | LINK_READY | Active High Link ready indication from EZ-USB&trade; FX20 to FPGA. Used to trigger link training sequence in DDR mode
P0CTL7 | PKTEND# | Active Low Packet End signal. Assert if FPGA wants to terminate the ongoing data transfer
P1CTL9 | A0 | LS bit of 2-bit address bus used to select thread
P1CTL8 | A1 | MS bit of 2-bit address bus used to select thread

<br>


#### FPGA configuration

Configure the Ti180 FPGA after each power-up using the data stored in the flash on the J484 kit. The configuration of the FPGA is controlled by the EZ-USB&trade; FX20 device. EZ-USB&trade; FX20 uses its Serial Memory Interface (SMIF) in x4 or Quad mode to read the configuration data from the flash memory. As the FPGA is configured in the Passive Serial x4 mode for configuration, it will take the data presented on its pins and configure itself.

Steps to configure FPGA in Passive Serial x4 mode:
- EZ-USB&trade; FX device deasserts INT\_RESET pin
- EZ-USB&trade; FX device starts sending a dummy SMIF (in x4 or Quad mode) clock to read the FPGA bitfile from SPI flash
- FPGA listens to the data on the SMIF lines and configures itself
- FPGA asserts CDONE# pin when configuration is complete

**Table 4. EZ-USB&trade; FX20 GPIOs used during FPGA configuration**

EZ-USB&trade; FX pin | Function | Description
:-------- | :------------ | :--------
P4.3 | INIT_RESET# | Active Low signal used by EZ-USB&trade; FX20 device to reset the Ti180 FPGA
P6.4 | PROG# | Active Low FPGA program signal. Assert by EZ-USB&trade; FX20 before initiating read from QSPI flash memory
P4.4 | CDONE | Active High Configuration Done signal asserted by FPGA when it is fully initialized

<br>


## Compile-time configurations

Application functionality can be customized by setting variables in *Makefile* or by configuring them through `make` CLI arguments.

- Run the `make build` command or build the project in your IDE to compile the application and generate a USB bootloader-compatible binary. This binary can be programmed onto the EZ-USB&trade; FX20 device using the EZ-USB&trade; Control Center application

- Run the `make build BLENABLE=no` command or set the variable in *Makefile* to compile the application and generate the standalone binary. This binary can be programmed onto the EZ-USB&trade; FX20 device through the SWD interface using the OpenOCD tool. For more details, see the [EZ-USB&trade; FX20 SDK user guide](https://www.infineon.com/fx20)

- Choose between the **Arm&reg; Compiler** or the **GNU Arm&reg; Embedded Compiler** build toolchains by setting the `TOOLCHAIN` variable in *Makefile* to `ARM` or `GCC_ARM` respectively. If you set it to `ARM`, ensure to set `CY_ARM_COMPILER_DIR` as a make variable or environment variable, pointing to the path of the compiler's root directory

By default, the application is configured to receive data from a 32-bit wide LVCMOS interface in DDR mode and make a USB 3.2 Gen2x2 (20 Gbps) data connection. Additional settings can be configured through macros specified by the `DEFINES` variable in *Makefile*:

**Table 5. Macro description**

Macro name | Description | Allowed values
:-------- | :------------ | :--------
*USB_CONN_TYPE* | Choose USB connection speed from amongst a set of options | '`CY_USBD_USB_DEV_SS_GEN2X2`' for USB 3.2 Gen2x2<br>'`CY_USBD_USB_DEV_SS_GEN2`' for USB 3.2 Gen2x1<br>'`CY_USBD_USB_DEV_SS_GEN1X2`' for USB 3.2 Gen1x2<br>'`CY_USBD_USB_DEV_SS_GEN1`' for USB 3.2 Gen1x1<br>'`CY_USBD_USB_DEV_HS`' for USB 2.0 HS<br>'`CY_USBD_USB_DEV_FS`' for USB 1.1 FS
*LVCMOS_DDR_EN* | Selects the LVCMOS interface data rate | '1u' to select Dual Data Rate operation <br> '0u' to select Single Data Rate operation
*USBFS_LOGS_ENABLE* | Enable debug logs through USB FS port | '1u' to enable logging through debug USBFS port <br> '0u' to enable logging through UART (SCB1)

<br>


### FPGA BitFile information

The bitfiles required to drive the LVCMOS interface on the EZ-USB&trade; FX20 device from the Ti180 FPGA are provided in the **BitFile** folder of this code example. These bitfiles need to be programmed onto the flash devices in the **Titanium Ti180 J484** kit using the Efinity Software.

**Table 6. BitFile description**

BitFile name | Description
:-------- | :------------ 
*fxn_ti180_dvk_multi_cam_ref_des_lvcmos_148m_ddr_u3v_features_working.hex* | Implements FIFO master functionality for LVCMOS DDR mode
*fxn_ti180_dvk_multi_cam_ref_des_lvcmos_sdr_wl.hex* | Implements FIFO master functionality for LVCMOS SDR mode

<br>


## Application files

**Table 7. Application file description**

File | Description
:-------- | :------------
*cy_gpif_header.h* | Generated header file for GPIF state configuration of LVCMOS interface
*cy_usb_app.c* | C source file implementing Slave FIFO application logic
*cy_usb_app.h* | Header file for application data structures and functions declaration
*cy_usb_descriptors.c* | C source file containing the USB descriptors
*main.c* | Source file for device initialization, ISRs, LVCMOS interface initialization, etc.
*cy_usb_i2c.c* | C source file with I2C handlers
*cy_usb_i2c.h* | Header file with I2C application constants and the function definitions
*cy_usb_qspi.c* | C source file with SMIF handlers and FPGA configuration functions
*cy_usb_qspi.h* | Header file with SMIF application constants and the function definitions
*cm0_code.c* | Binary code block to be run by the Cortex&reg;-M0 Plus core to start the Cortex&reg;-M4 core
*Makefile* | GNU make compliant build script for compiling this example

<br>


## Related resources

Resources  | Links
-----------|----------------------------------
Application notes  | [AN237841](https://www.infineon.com/dgdl/Infineon-Getting_started_with_EZ_USB_FX20_FX10_FX5N_FX5-ApplicationNotes-v01_00-EN.pdf?fileId=8ac78c8c956a0a470195a515c54916e1) – Getting started with EZ-USB&trade; FX20/FX10/FX5N/FX5
Code examples  | [Using ModusToolbox&trade;](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [EZ-USB&trade; FX20 datasheets](https://www.infineon.com/fx20)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board)
Libraries on GitHub  | [mtb-pdl-cat1](https://github.com/Infineon/mtb-pdl-cat1) – Peripheral Driver Library (PDL)
Middleware on GitHub  | [usbfxstack](https://github.com/Infineon/usbfxstack) – USBFX Stack middleware library and docs
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSOC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development

<br>


## Other resources

Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.


## Document history

Document title: *CE241358* – *EZ-USB&trade; FX20: Slave FIFO 2-bit application*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | New code example
<br>


All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.

PSOC&trade;, formerly known as PSoC&trade;, is a trademark of Infineon Technologies. Any references to PSoC&trade; in this document or others shall be deemed to refer to PSOC&trade;.

---------------------------------------------------------

© Cypress Semiconductor Corporation, 2025. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress's patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product. CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications. To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document. Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device. You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device. Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress's published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, ModusToolbox, PSoC, CAPSENSE, EZ-USB, F-RAM, and TRAVEO are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries. For a more complete list of Cypress trademarks, visit www.infineon.com. Other names and brands may be claimed as property of their respective owners.
