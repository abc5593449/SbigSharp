using System;
using System.Net;
using System.Runtime.InteropServices;
using System.Text;

namespace SbigSharp
{
    public static class SBIG
    {
        /*!
           *	\file SBIGUDRV.H
           *	\brief Contains the function prototypes and enumerated constants for the Universal Parallel/USB/Ethernet driver.
           *
           *	This supports the following devices:
           *
           *	* ST-5C/237/237A (PixCel255/237)
           *	* ST-7E/8E/9E/10E
           *	* ST-1K, ST-2K, ST-4K
           *	* STL Large Format Cameras
           *	* ST-402 Family of Cameras
           *	* ST-8300 Cameras
           *	* STF-8300, 8050 Cameras
           *	* STT Cameras
           *	* STX/STXL Cameras
           *	* ST-i Cameras
           *	* AO-7, AOL, AO-8
           *	* CFW-8, CFW-9, CFW-10, CFW-L
           *	* FW5-8300, FW8-8300
           *	* ST Focuser
           *	* Differential Guider Accessory (Preliminary)
           */

        /*

            Enumerated Constants

            Note that the various constants are declared here as enums
            for ease of declaration but in the structures that use the
            enums unsigned shorts are used to force the various
            16 and 32 bit compilers to use 16 bits.

        */

        /*!
            \defgroup BASE_STRUCTURES
            Supported Camera Commands

            These are the commands supported by the driver.
            They are prefixed by CC_ to designate them as
            camera commands and avoid conflicts with other
            enums.

            Some of the commands are marked as SBIG use only
            and have been included to enhance testability
            of the driver for SBIG.

        */
        /*! \enum PAR_COMMAND
         * \ingroup BASE_STRUCTURES
         * Command ID enum 
         */
        public enum PAR_COMMAND : ushort

        {
            /*

                General Use Commands

            */
            CC_NULL,                        /*!< Null Command													*/

            /* 1 - 10 */
            CC_START_EXPOSURE = 1,          /*!< Start exposure command											*/
            CC_END_EXPOSURE,                /*!< End exposure command											*/
            CC_READOUT_LINE,                /*!< Readout line command											*/
            CC_DUMP_LINES,                  /*!< Dump lines command												*/
            CC_SET_TEMPERATURE_REGULATION,  /*!< Set Temperature regulation command								*/
            CC_QUERY_TEMPERATURE_STATUS,    /*!< Query temperature status command								*/
            CC_ACTIVATE_RELAY,              /*!< Activate Relay command											*/
            CC_PULSE_OUT,                   /*!< Pulse out command												*/
            CC_ESTABLISH_LINK,              /*!< Establish link command											*/
            CC_GET_DRIVER_INFO,             /*!< Get driver info command										*/

            /* 11 - 20 */
            CC_GET_CCD_INFO,                /*!< Get CCD info command											*/
            CC_QUERY_COMMAND_STATUS,        /*!< Query command status command									*/
            CC_MISCELLANEOUS_CONTROL,       /*!< Miscellaneous control command									*/
            CC_READ_SUBTRACT_LINE,          /*!< Read subtract line command										*/
            CC_UPDATE_CLOCK,                /*!< Update clock command											*/
            CC_READ_OFFSET,                 /*!< Read offset command											*/
            CC_OPEN_DRIVER,                 /*!< Open driver command											*/
            CC_CLOSE_DRIVER,                /*!< Close driver command											*/
            CC_TX_SERIAL_BYTES,             /*!< TX Serial bytes command										*/
            CC_GET_SERIAL_STATUS,           /*!< Get serial status command										*/

            /* 21 - 30 */
            CC_AO_TIP_TILT,                 /*!< AO tip/tilt command											*/
            CC_AO_SET_FOCUS,                /*!< AO set focus command											*/
            CC_AO_DELAY,                    /*!< AO delay command												*/
            CC_GET_TURBO_STATUS,            /*!< Get turbo status command										*/
            CC_END_READOUT,                 /*!< End readout command											*/
            CC_GET_US_TIMER,                /*!< Get US timer command											*/
            CC_OPEN_DEVICE,                 /*!< Open device command											*/
            CC_CLOSE_DEVICE,                /*!< Close device command											*/
            CC_SET_IRQL,                    /*!< Set IRQL command												*/
            CC_GET_IRQL,                    /*!< Get IRQL command												*/

            /* 31 - 40 */
            CC_GET_LINE,                    /*!< Get line command												*/
            CC_GET_LINK_STATUS,             /*!< Get link status command										*/
            CC_GET_DRIVER_HANDLE,           /*!< Get driver handle command										*/
            CC_SET_DRIVER_HANDLE,           /*!< Set driver handle command										*/
            CC_START_READOUT,               /*!< Start readout command											*/
            CC_GET_ERROR_STRING,            /*!< Get error string command										*/
            CC_SET_DRIVER_CONTROL,          /*!< Set driver control command										*/
            CC_GET_DRIVER_CONTROL,          /*!< Get driver control command										*/
            CC_USB_AD_CONTROL,              /*!< USB A/D control command										*/
            CC_QUERY_USB,                   /*!< Query USB command												*/

            /* 41 - 50 */
            CC_GET_PENTIUM_CYCLE_COUNT,     /*!< Get Pentium cycle count command								*/
            CC_RW_USB_I2C,                  /*!< Read/Write USB I2C command										*/
            CC_CFW,                         /*!< Control Filter Wheel command									*/
            CC_BIT_IO,                      /*!< Bit I/O command												*/
            CC_USER_EEPROM,                 /*!< User EEPROM command											*/
            CC_AO_CENTER,                   /*!< AO Center command												*/
            CC_BTDI_SETUP,                  /*!< BTDI setup command												*/
            CC_MOTOR_FOCUS,                 /*!< Motor focus command											*/
            CC_QUERY_ETHERNET,              /*!< Query Ethernet command											*/
            CC_START_EXPOSURE2,             /*!< Start Exposure command v2										*/

            /* 51 - 60 */
            CC_SET_TEMPERATURE_REGULATION2, /*!< Set Temperature regulation command								*/
            CC_READ_OFFSET2,                /*!< Read offset command v2											*/
            CC_DIFF_GUIDER,                 /*!< Differential Guider command									*/
            CC_COLUMN_EEPROM,               /*!< Column EEPROM command											*/
            CC_CUSTOMER_OPTIONS,            /*!< Customer Options command										*/
            CC_DEBUG_LOG,                   /*!< Debug log command												*/
            CC_QUERY_USB2,                  /*!< Query USB command v2											*/
            CC_QUERY_ETHERNET2,             /*!< Query Ethernet command v2										*/
            CC_GET_AO_MODEL,                /*!< Get AO model command											*/
            CC_QUERY_USB3,                  /*!< Query up to 24 USB cameras										*/
            CC_QUERY_COMMAND_STATUS2,       /*!< Expanded Query Command Status to include extra information		*/
                                            /*
                                                SBIG Use Only Commands
                                            */

            /* 90 - 99 */
            CC_SEND_BLOCK = 90,             /*!< Send block command												*/
            CC_SEND_BYTE,                   /*!< Send byte command												*/
            CC_GET_BYTE,                    /*!< Get byte command												*/
            CC_SEND_AD,                     /*!< Send A/D command												*/
            CC_GET_AD,                      /*!< Get A/D command												*/
            CC_CLOCK_AD,                    /*!< Clock A/D command												*/
            CC_SYSTEM_TEST,                 /*!< System test command											*/
            CC_GET_DRIVER_OPTIONS,          /*!< Get driver options command										*/
            CC_SET_DRIVER_OPTIONS,          /*!< Set driver options command										*/
            CC_FIRMWARE,                    /*!< Firmware command												*/

            /* 100 -109 */
            CC_BULK_IO,                     /*!< Bulk I/O command												*/
            CC_RIPPLE_CORRECTION,           /*!< Ripple correction command										*/
            CC_EZUSB_RESET,                 /*!< EZUSB Reset command											*/
            CC_BREAKPOINT,                  /*!< Breakpoint command												*/
            CC_QUERY_EXPOSURE_TICKS,        /*!< Query exposure ticks command									*/
            CC_SET_ACTIVE_CCD_AREA,         /*!< Set active CCD area command									*/
            CC_READOUT_IN_PROGRESS,         /*!< Returns TRUE if a readout is in progress on any driver handle  */
            CC_GET_RBI_PARAMETERS,          /*!< Updates the RBI Preflash parameters							*/
            CC_SET_RBI_PARAMETERS,          /*!< Obtains the RBI Preflash parameters from the camera			*/
            CC_QUERY_FEATURE_SUPPORTED,     /*!< Checks to see if a camera's firmware supports a command.		*/
            CC_LAST_COMMAND                 /*!< Last command ID												*/

            /* 110 - 119 */

        };

        /*

            Return Error Codes

            These are the error codes returned by the driver
            function.  They are prefixed with CE_ to designate
            them as camera errors.

        */
        /*!
         * \def CE_ERROR_BASE
         * Base value for all error IDs.
         */
        const ushort CE_ERROR_BASE = 1;

        /*!
         * \enum PAR_ERROR
         * \ingroup BASE_STRUCTURES
         * Error ID enum 
         */
        public enum PAR_ERROR : ushort
        {
            /* 0 - 10 */
            CE_NO_ERROR,                            /*!< No error ID								*/
            CE_CAMERA_NOT_FOUND = CE_ERROR_BASE,    /*!< Camera not found error						*/
            CE_EXPOSURE_IN_PROGRESS,                /*!< Exposure in progress error					*/
            CE_NO_EXPOSURE_IN_PROGRESS,             /*!< No exposure in progress error				*/
            CE_UNKNOWN_COMMAND,                     /*!< Unknown command error						*/
            CE_BAD_CAMERA_COMMAND,                  /*!< Bad camera command error					*/
            CE_BAD_PARAMETER,                       /*!< Bad parameter command						*/
            CE_TX_TIMEOUT,                          /*!< Transfer (Tx) timeout error				*/
            CE_RX_TIMEOUT,                          /*!< Receive (Rx) timeout error					*/
            CE_NAK_RECEIVED,                        /*!< Received Negative Acknowledgement 			*/
            CE_CAN_RECEIVED,                        /*!< Received Cancel							*/

            /* 11 - 20 */
            CE_UNKNOWN_RESPONSE,                    /*!< Unknown response error						*/
            CE_BAD_LENGTH,                          /*!< Bad length error							*/
            CE_AD_TIMEOUT,                          /*!< A/D timeout error							*/
            CE_KBD_ESC,                             /*!< Keyboard error								*/
            CE_CHECKSUM_ERROR,                      /*!< Checksum error								*/
            CE_EEPROM_ERROR,                        /*!< EEPROM error								*/
            CE_SHUTTER_ERROR,                       /*!< Shutter error								*/
            CE_UNKNOWN_CAMERA,                      /*!< Unknown camera error						*/
            CE_DRIVER_NOT_FOUND,                    /*!< Driver not found error						*/
            CE_DRIVER_NOT_OPEN,                     /*!< Driver not open error						*/

            /* 21 - 30 */
            CE_DRIVER_NOT_CLOSED,                   /*!< Driver not closed error					*/
            CE_SHARE_ERROR,                         /*!< Share error								*/
            CE_TCE_NOT_FOUND,                       /*!< TCE not found error						*/
            CE_AO_ERROR,                            /*!< AO error									*/
            CE_ECP_ERROR,                           /*!< ECP error									*/
            CE_MEMORY_ERROR,                        /*!< Memory error								*/
            CE_DEVICE_NOT_FOUND,                    /*!< Device not found error						*/
            CE_DEVICE_NOT_OPEN,                     /*!< Device not open error						*/
            CE_DEVICE_NOT_CLOSED,                   /*!< Device not closed error					*/
            CE_DEVICE_NOT_IMPLEMENTED,              /*!< Device not implemented error				*/

            /* 31 - 40 */
            CE_DEVICE_DISABLED,                     /*!< Device disabled error						*/
            CE_OS_ERROR,                            /*!< OS error									*/
            CE_SOCK_ERROR,                          /*!< Socket error								*/
            CE_SERVER_NOT_FOUND,                    /*!< Server not found error						*/
            CE_CFW_ERROR,                           /*!< Filter wheel error							*/
            CE_MF_ERROR,                            /*!< Motor Focus error							*/
            CE_FIRMWARE_ERROR,                      /*!< Firmware error								*/
            CE_DIFF_GUIDER_ERROR,                   /*!< Differential guider error					*/
            CE_RIPPLE_CORRECTION_ERROR,             /*!< Ripple corrections error					*/
            CE_EZUSB_RESET,                         /*!< EZUSB Reset error							*/

            /* 41 - 50*/
            CE_INCOMPATIBLE_FIRMWARE,               /*!< Firmware needs update to support feature.	*/
            CE_INVALID_HANDLE,                      /*!< An invalid R/W handle was supplied for I/O	*/
            CE_NEXT_ERROR                           /*!< Development purposes: Next Error			*/

        };

        /*
            Camera Command State Codes

            These are the return status codes for the Query
            Command Status command.  They are prefixed with
            CS_ to designate them as camera status.

        */
        /*! 
         * \enum PAR_COMMAND_STATUS
         * \ingroup BASE_STRUCTURES
         * Camera states enum 
         */
        public enum PAR_COMMAND_STATUS : ushort
        {
            CS_IDLE,                    /*!< Camera state: Idle.				*/
            CS_IN_PROGRESS,             /*!< Camera state: Exposure in progress */
            CS_INTEGRATING,             /*!< Camera state: Integrating			*/
            CS_INTEGRATION_COMPLETE     /*!< Camera state: Integration complete */
        };

        public enum FeatureFirmwareRequirement : ushort
        {
            FFR_CTRL_OFFSET_CORRECTION,
            FFR_CTRL_EXT_SHUTTER_ONLY,
            FFR_ASYNC_TRIGGER_IN,
            FFR_LAST
        };

        /*!
         * \def CS_PULSE_IN_ACTIVE
         * \ingroup BASE_STRUCTURES
         * Pulse in is currently active state modifier flag.
         */
        const UInt16 CS_PULSE_IN_ACTIVE = 0x8000;

        /*! 
         * \def CS_WAITING_FOR_TRIGGER
         * \ingroup BASE_STRUCTURES
         * Waiting for trigger state modifier flag
         */
        const UInt16 CS_WAITING_FOR_TRIGGER = 0x8000;

        const UInt16 RBI_PREFLASH_LENGTH_MASK = 0x0FFF;
        const UInt16 RBI_PREFLASH_FLUSH_MASK = 0xF000;
        const Byte RBI_PREFLASH_FLUSH_BIT = 0x0C;
        /*
            Misc. Enumerated Constants
            QUERY_TEMP_STATUS_REQUEST - Used with the Query Temperature Status command.
            ABG_STATE7 - Passed to Start Exposure Command
            MY_LOGICAL - General purpose type
            DRIVER_REQUEST - Used with Get Driver Info command
            CCD_REQUEST - Used with Imaging commands to specify CCD
            CCD_INFO_REQUEST - Used with Get CCD Info Command
            PORT - Used with Establish Link Command
            CAMERA_TYPE - Returned by Establish Link and Get CCD Info commands
            SHUTTER_COMMAND, SHUTTER_STATE7 - Used with Start Exposure and Miscellaneous Control Commands
            TEMPERATURE_REGULATION - Used with Enable Temperature Regulation
            LED_STATE - Used with the Miscellaneous Control Command
            FILTER_COMMAND, FILTER_STATE - Used with the Miscellaneous Control Command
            AD_SIZE, FILTER_TYPE - Used with the GetCCDInfo3 Command
            AO_FOCUS_COMMAND - Used with the AO Set Focus Command
            SBIG_DEVICE_TYPE - Used with Open Device Command
            DRIVER_CONTROL_PARAM - Used with Get/SetDriverControl Command
            USB_AD_CONTROL_COMMAND - Used with UsbADControl Command
            CFW_MODEL_SELECT, CFW_STATUS, CFW_ERROR - Used with CFW command
            CFW_POSITION, CFW_GET_INFO_SELECT - Used with CFW Command
            BIT_IO_OPERATION, BIT_IO_NMAE - Used with BitIO command
            MF_MODEL_SELECT, MF_STATUS, MF_ERROR, MF_GET_INFO_SELECT - Used with Motor Focus Command
            DIFF_GUIDER_COMMAND, DIFF_GUIDER_STATE, DIFF_GUIDER_ERROR -	Used with the Diff Guider Command
        */

        /*!
         * \enum QUERY_TEMP_STATUS_REQUESTS
         * \ingroup BASE_STRUCTURES
         * Query Temperature Status enum 
         */
        public enum QUERY_TEMP_STATUS_REQUEST : ushort
        {
            TEMP_STATUS_STANDARD,   /*!< Temperature status Standard		*/
            TEMP_STATUS_ADVANCED,   /*!< Temperature status Advanced		*/
            TEMP_STATUS_ADVANCED2   /*!< Temperature status Advanced 2		*/
        };

        /*! 
         * \enum ABG_STATE_7
         * \ingroup BASE_STRUCTURES
         * ABG state enum 
         */
        public enum ABG_STATE7 : ushort
        {
            ABG_LOW7,       /*!< ABG Low 7			*/
            ABG_CLK_LOW7,   /*!< ABG Clock Low 7	*/
            ABG_CLK_MED7,   /*!< ABG Clock Medium 7	*/
            ABG_CLK_HI7     /*!< ABG Clock High 7	*/
        };

        /*!
         *	\typedef MY_LOGICAL
         * \ingroup BASE_STRUCTURES
         *	Boolean type definition 
         */
        public struct MY_LOGICAL
        {
            /// <summary>
            /// MY_LOGICAL false definition.
            /// </summary>
            const UInt16 FALSE = 0;
            /// <summary>
            /// MY_LOGICAL true definition.
            /// </summary>
            const UInt16 TRUE = 1;

            public UInt16 value;

            public MY_LOGICAL(ushort value)
            {
                this.value = value;
            }

            public MY_LOGICAL(bool value)
            {
                this.value = (ushort)(value ? 1 : 0);
            }

            /// <summary>
            /// implicit ushort to MY_LOGICAL conversion operator
            /// </summary>
            /// <param name="us">ushort type value.</param>
            public static implicit operator MY_LOGICAL(ushort us)
            {
                return new MY_LOGICAL(us);
            }

            /// <summary>
            /// implicit bool to MY_LOGICAL conversion operator
            /// </summary>
            /// <param name="b">bool type value.</param>
            public static implicit operator MY_LOGICAL(bool b)
            {
                return new MY_LOGICAL(b);
            }

            /// <summary>
            /// implicit MY_LOGICAL to ushort conversion operator
            /// </summary>
            /// <param name="logical">MY_LOGICAL type value.</param>
            public static implicit operator bool(MY_LOGICAL logical)
            {
                // FALSE = 0;
                return (logical.value != 0);
            }
        };

        /*!
         * \enum DRIVER_REQUEST 
         * \ingroup BASE_STRUCTURES
         * Driver request enum 
         */
        public enum DRIVER_REQUEST : ushort
        {
            DRIVER_STD,         /*!< Driver standard	*/
            DRIVER_EXTENDED,    /*!< Driver extended	*/
            DRIVER_USB_LOADER   /*!< Driver USB loader	*/
        };

        /*!
         * \enum CCD_REQUEST
         * \ingroup BASE_STRUCTURES
         * CCD Request enum 
         */
        public enum CCD_REQUEST : ushort
        {
            CCD_IMAGING,        /*!< Request Imaging CCD			*/
            CCD_TRACKING,       /*!< Request Internal Tracking CCD	*/
            CCD_EXT_TRACKING    /*!< Request External Tracking CCD	*/
        };

        /*!
         *	\enum READOUT_BINNING_MODE
         * \ingroup BASE_STRUCTURES
         *	Readout Modes enum 
         */
        public enum READOUT_BINNING_MODE : ushort
        {
            RM_1X1,             /*!< 1x1 binning readout mode			*/
            RM_2X2,             /*!< 2x2 binning readout mode			*/
            RM_3X3,             /*!< 3x3 binning readout mode			*/
            RM_NX1,             /*!< Nx1 binning readout mode			*/
            RM_NX2,             /*!< Nx2 binning readout mode			*/
            RM_NX3,             /*!< Nx3 binning readout mode			*/
            RM_1X1_VOFFCHIP,    /*!< 1x1 Off-chip binning readout mode	*/
            RM_2X2_VOFFCHIP,    /*!< 2x2 Off-chip binning readout mode	*/
            RM_3X3_VOFFCHIP,    /*!< 3x3 Off-chip binning readout mode	*/
            RM_9X9,             /*!< 9x9 binning readout mode			*/
            RM_NXN              /*!< NxN binning readout mode			*/
        };

        /*!
         * \enum CCD_INFO_REQUEST
         * \ingroup BASE_STRUCTURES
         * CCD Information request enum 
         */
        public enum CCD_INFO_REQUEST : ushort
        {
            CCD_INFO_IMAGING,               /*!< Imaging CCD Info				*/
            CCD_INFO_TRACKING,              /*!< Tracking CCD Info				*/
            CCD_INFO_EXTENDED,              /*!< Extended CCD Info				*/
            CCD_INFO_EXTENDED_5C,           /*!< Extended CCD Info 5C			*/
            CCD_INFO_EXTENDED2_IMAGING,     /*!< Extended Imaging CCD Info 2	*/
            CCD_INFO_EXTENDED2_TRACKING,    /*!< Extended Tracking CCD Info 2	*/
            CCD_INFO_EXTENDED3              /*!< Extended Imaging CCD Info 3	*/
        };

        /*! 
         * \enum IMAGING_ABG
         * \ingroup BASE_STRUCTURES
         * Anti-blooming gate capability enum 
         */
        public enum IMAGING_ABG : ushort
        {
            ABG_NOT_PRESENT,    /*!< Anti-blooming gate not Present	*/
            ABG_PRESENT         /*!< Anti-blooming gate present		*/
        };

        /*!
         * \enum PORT_RATE
         * \ingroup BASE_STRUCTURES
         * Port bit-rate enum 
         */
        public enum PORT_RATE : ushort
        {
            BR_AUTO,    /*!< Bit-rate auto	*/
            BR_9600,    /*!< Bit-rate 9600	*/
            BR_19K,     /*!< Bit-rate 19K	*/
            BR_38K,     /*!< Bit-rate 38K	*/
            BR_57K,     /*!< Bit-rate 57K	*/
            BR_115K     /*!< Bit-rate 115K	*/
        };

        /*!
         * \enum CAMERA_TYPE
         * \ingroup BASE_STRUCTURES
         * Camera type enum 
         */
        public enum CAMERA_TYPE : ushort
        {
            ST7_CAMERA = 4,     /*!< ST-7 Camera																																	*/
            ST8_CAMERA,         /*!< ST-8 Camera																																	*/
            ST5C_CAMERA,        /*!< ST-5C Camera																																	*/
            TCE_CONTROLLER,     /*!< TCE-Controller																																	*/
            ST237_CAMERA,       /*!< ST-237 Camera																																	*/
            STK_CAMERA,         /*!< ST-K Camera																																	*/
            ST9_CAMERA,         /*!< ST-9 Camera																																	*/
            STV_CAMERA,         /*!< ST-V Camera																																	*/
            ST10_CAMERA,        /*!< ST-10 Camera																																	*/
            ST1K_CAMERA,        /*!< ST-1000 Camera																																	*/
            ST2K_CAMERA,        /*!< ST-2000 Camera																																	*/
            STL_CAMERA,         /*!< STL Camera																																		*/
            ST402_CAMERA,       /*!< ST-402 Camera																																	*/
            STX_CAMERA,         /*!< STX Camera																																		*/
            ST4K_CAMERA,        /*!< ST-4000 Camera																																	*/
            STT_CAMERA,         /*!< STT Camera																																		*/
            STI_CAMERA,         /*!< ST-i Camera																																	*/
            STF_CAMERA,         /*!< STF Camera, NOTE: STF8, and STF cameras both report this kind, but have *DIFFERENT CAMERA MODEL ID VARIABLES* (stf8CameraID and stfCameraID)	*/
            NEXT_CAMERA,        /*!< Next Camera																																	*/
            NO_CAMERA = 0xFFFF  /*!< No Camera																																		*/
        };

        /*!
         * \enum SHUTTER_COMMAND
         * \ingroup BASE_STRUCTURES
         * Shutter Control enum 
         */
        public enum SHUTTER_COMMAND : ushort
        {
            SC_LEAVE_SHUTTER,       /*!< Shutter Control: Leave shutter in current state.	*/
            SC_OPEN_SHUTTER,        /*!< Shutter Control: Open shutter.						*/
            SC_CLOSE_SHUTTER,       /*!< Shutter Control: Close shutter.					*/
            SC_INITIALIZE_SHUTTER,  /*!< Shutter Control: Initialize shutter.				*/
            SC_OPEN_EXT_SHUTTER,    /*!< Shutter Control: Open external shutter.			*/
            SC_CLOSE_EXT_SHUTTER    /*!< Shutter Control: Close external shutter.			*/
        };

        /*!
         * \enum SHUTTER_STATE7
         * \ingroup BASE_STRUCTURES
         * Shutter State enum 
         */
        public enum SHUTTER_STATE7 : ushort
        {
            SS_OPEN,    /*!< Shuter State: Open		*/
            SS_CLOSED,  /*!< Shuter State: Closed	*/
            SS_OPENING, /*!< Shutter State: Opening	*/
            SS_CLOSING  /*!< Shutter State: Closing	*/
        };

        /*! 
         * \enum TEMPERATURE_REGULATION
         * \ingroup BASE_STRUCTURES
         * Temperature regulation enum 
         */
        public enum TEMPERATURE_REGULATION : ushort
        {
            REGULATION_OFF,                 /*!< Temperature regulation off					*/
            REGULATION_ON,                  /*!< Temperature regulation on					*/
            REGULATION_OVERRIDE,            /*!< Temperature regulation override			*/
            REGULATION_FREEZE,              /*!< Temperature regulation freeze				*/
            REGULATION_UNFREEZE,            /*!< Temperature regulation unfreeze			*/
            REGULATION_ENABLE_AUTOFREEZE,   /*!< Temperature regulation enable autofreeze	*/
            REGULATION_DISABLE_AUTOFREEZE   /*!< Temperature regulation disable autofreeze	*/
        };

        /*!
         * \def REGULATION_FROZEN_MASK
         * Mask for Temperature Regulation frozen state
         */
        const UInt16 REGULATION_FROZEN_MASK = 0x8000;

        /*!
         * \enum LED_STATE 
         * LED State enum 
         */
        public enum LED_STATE : ushort
        {
            LED_OFF,        /*!< LED off		*/
            LED_ON,         /*!< LED on			*/
            LED_BLINK_LOW,  /*!< LED Blink low	*/
            LED_BLINK_HIGH  /*!< LED Blink high	*/
        };

        /*!
         * \enum FILTER_COMMAND 
         * Filter command enum
         */
        public enum FILTER_COMMAND : ushort
        {
            FILTER_LEAVE,   /*!< Filter leave		*/
            FILTER_SET_1,   /*!< Filter slot 1		*/
            FILTER_SET_2,   /*!< Filter slot 2		*/
            FILTER_SET_3,   /*!< Filter slot 3		*/
            FILTER_SET_4,   /*!< Filter slot 4		*/
            FILTER_SET_5,   /*!< Filter slot 5		*/
            FILTER_STOP,    /*!< Stop filter		*/
            FILTER_INIT     /*!< Initialize filter	*/
        };

        /*!
         * \enum FILTER_STATE 
         * Filter State enum 
         */
        public enum FILTER_STATE : ushort
        {
            FS_MOVING,  /*!< Filter wheel moving			*/
            FS_AT_1,    /*!< Filter wheel at slot 1			*/
            FS_AT_2,    /*!< Filter wheel at slot 2			*/
            FS_AT_3,    /*!< Filter wheel at slot 3			*/
            FS_AT_4,    /*!< Filter wheel at slot 4			*/
            FS_AT_5,    /*!< Filter wheel at slot 5			*/
            FS_UNKNOWN  /*!< Filter wheel at slot Unknown	*/
        };

        /*! 
         * \enum AD_SIZE
         * A/D Size enum 
         */
        public enum AD_SIZE : ushort
        {
            AD_UNKNOWN, /*!< Unknown size	*/
            AD_12_BITS, /*!< 12-bits		*/
            AD_16_BITS  /*!< 16-bits		*/
        };

        /*!
         * \enum FILTER_TYPE 
         * Filter Wheel Type enum
         */
        public enum FILTER_TYPE : ushort
        {
            FW_UNKNOWN,     /*!< Unkwown Filter Wheel	*/
            FW_EXTERNAL,    /*!< External Filter Wheel	*/
            FW_VANE,        /*!< Vane Filter Wheel		*/
            FW_FILTER_WHEEL /*!< Standard Filter Wheel	*/
        };

        /*! 
         * \enum AO_FOCUS_COMMAND
         * AO Focus enum 
         */
        public enum AO_FOCUS_COMMAND : ushort
        {
            AOF_HARD_CENTER,    /*!< AO Focus hard center	*/
            AOF_SOFT_CENTER,    /*!< AO Focus soft center	*/
            AOF_STEP_IN,        /*!< AO Focus step in		*/
            AOF_STEP_OUT        /*!< AO Focus step out		*/
        };

        // Ethernet stuff
        /*!
         * \def SRV_SERVICE_PORT
         * Service port for Ethernet access.
         */
        const Int16 SRV_SERVICE_PORT = 5000;

        /*!
         * \def BROADCAST_PORT
         * Broadcast port for SBIG Cameras
         */
        const Int16 BROADCAST_PORT = 5001;

        /*! 
         * \enum SBIG_DEVICE_TYPE
         * SBIG Device types enum 
         */
        public enum SBIG_DEVICE_TYPE : ushort
        {
            DEV_NONE,           /*!< Device type: None			*/
            DEV_LPT1,           /*!< LPT port slot 1			*/
            DEV_LPT2,           /*!< LPT port slot 2			*/
            DEV_LPT3,           /*!< LPT port slot 3			*/
            DEV_USB = 0x7F00,   /*!< USB autodetect				*/
            DEV_ETH,            /*!< Ethernet					*/
            DEV_USB1,           /*!< USB slot 1 CC_QUERY_USB	*/
            DEV_USB2,           /*!< USB slot 2					*/
            DEV_USB3,           /*!< USB slot 3					*/
            DEV_USB4,           /*!< USB slot 4					*/
            DEV_USB5,           /*!< USB slot 5	CC_QUERY_USB2	*/
            DEV_USB6,           /*!< USB slot 6					*/
            DEV_USB7,           /*!< USB slot 7					*/
            DEV_USB8,           /*!< USB slot 8					*/
            DEV_USB9,           /*!< USB slot 9	CC_QUERY_USB3	*/
            DEV_USB10,          /*!< USB slot 10				*/
            DEV_USB11,          /*!< USB slot 11				*/
            DEV_USB12,          /*!< USB slot 12				*/
            DEV_USB13,          /*!< USB slot 13				*/
            DEV_USB14,          /*!< USB slot 14				*/
            DEV_USB15,          /*!< USB slot 15				*/
            DEV_USB16,          /*!< USB slot 16				*/
            DEV_USB17,          /*!< USB slot 17				*/
            DEV_USB18,          /*!< USB slot 18				*/
            DEV_USB19,          /*!< USB slot 19				*/
            DEV_USB20,          /*!< USB slot 20				*/
            DEV_USB21,          /*!< USB slot 21				*/
            DEV_USB22,          /*!< USB slot 22				*/
            DEV_USB23,          /*!< USB slot 23				*/
            DEV_USB24,          /*!< USB slot 24				*/
        };

        /*! 
         * \enum DRIVER_CONTROL_PARAM
         * Driver control parameters enum
         */
        public enum DRIVER_CONTROL_PARAM : ushort
        {
            DCP_USB_FIFO_ENABLE,            /*!< Enable FIFO											*/
            DCP_CALL_JOURNAL_ENABLE,        /*!< Enable Journaling										*/
            DCP_IVTOH_RATIO,                /*!< IV to H Ratio											*/
            DCP_USB_FIFO_SIZE,              /*!< USB FIFO size											*/
            DCP_USB_DRIVER,                 /*!< USB Driver												*/
            DCP_KAI_RELGAIN,                /*!< KAI Relative Gain										*/
            DCP_USB_PIXEL_DL_ENABLE,        /*!< USB Pixel D\L enable									*/
            DCP_HIGH_THROUGHPUT,            /*!< High throughput										*/
            DCP_VDD_OPTIMIZED,              /*!< VDD Optimized											*/
            DCP_AUTO_AD_GAIN,               /*!< Auto A/D Gain											*/
            DCP_NO_HCLKS_FOR_INTEGRATION,   /*!< No H-Clocks for Integration							*/
            DCP_TDI_MODE_ENABLE,            /*!< TDI Mode Enable										*/
            DCP_VERT_FLUSH_CONTROL_ENABLE,  /*!< Vertical Flush control enable							*/
            DCP_ETHERNET_PIPELINE_ENABLE,   /*!< Ethernet pipeline enable								*/
            DCP_FAST_LINK,                  /*!< Fast link												*/
            DCP_OVERSCAN_ROWSCOLS,          /*!< Overscan Rows/Columns									*/
            DCP_PIXEL_PIPELINE_ENABLE,      /*!< Enable Pixel Pipeline									*/
            DCP_COLUMN_REPAIR_ENABLE,       /*!< Enable column repair									*/
            DCP_WARM_PIXEL_REPAIR_ENABLE,   /*!< Enable warm pixel repair								*/
            DCP_WARM_PIXEL_REPAIR_COUNT,    /*!< warm pixel repair count								*/
            DCP_TDI_MODE_DRIFT_RATE,        /*!< TDI Drift rate in [XXX]								*/
            DCP_OVERRIDE_AD_GAIN,           /*!< Override A/D Converter's Gain							*/
            DCP_ENABLE_AUTO_OFFSET,     /*!< Override auto offset adjustments in certain cameras.	*/
            DCP_LAST                        /*!< Last Device control parameter							*/
        };

        /*! 
         * \enum USB_AD_CONTROL_COMMAND
         * USB A/D Control commands 
         */
        public enum USB_AD_CONTROL_COMMAND : ushort
        {
            USB_AD_IMAGING_GAIN,            /*!< Imaging gain					*/
            USB_AD_IMAGING_OFFSET,          /*!< Imaging offset					*/

            USB_AD_TRACKING_GAIN,           /*!< Internal tracking gain			*/
            USB_AD_TRACKING_OFFSET,         /*!< Internal tracking offset		*/

            USB_AD_EXTTRACKING_GAIN,        /*!< External tracking gain			*/
            USB_AD_EXTTRACKING_OFFSET,      /*!< External tracking offset		*/

            USB_AD_IMAGING2_GAIN,           /*!< Imaging gain channel 2			*/
            USB_AD_IMAGING2_OFFSET,         /*!< Imaging offset channel 2		*/

            USB_AD_IMAGING_GAIN_RIGHT,      /*!< Imaging gain right channel		*/
            USB_AD_IMAGING_OFFSET_RIGHT,    /*!< Imaging offset right channel	*/
        };

        /*!
         * \enum ENUM_USB_DRIVER
         * USB Driver enum 
         */
        public enum ENUM_USB_DRIVER : ushort
        {
            USBD_SBIGE, /*!< SBIG E */
            USBD_SBIGI, /*!< SBIG I	*/
            USBD_SBIGM, /*!< SBIG_M	*/
            USBD_NEXT   /*!< Next	*/
        };

        /*!
         * \enum CFW_MODEL_SELECT
         * Filter Weel Model Selection enum 
         */
        public enum CFW_MODEL_SELECT : ushort
        {
            CFWSEL_UNKNOWN,         /*!< Unknown Model	*/
            CFWSEL_CFW2,            /*!< CFW2			*/
            CFWSEL_CFW5,            /*!< CFW5			*/
            CFWSEL_CFW8,            /*!< CFW8			*/
            CFWSEL_CFWL,            /*!< CFWL			*/
            CFWSEL_CFW402,          /*!< CFW-402		*/
            CFWSEL_AUTO,            /*!< Auto			*/
            CFWSEL_CFW6A,           /*!< CFW-6A			*/
            CFWSEL_CFW10,           /*!< CFW10			*/
            CFWSEL_CFW10_SERIAL,    /*!< CFW10-Serial	*/
            CFWSEL_CFW9,            /*!< CFW9			*/
            CFWSEL_CFWL8,           /*!< CFWL8			*/
            CFWSEL_CFWL8G,          /*!< CFWL8-G		*/
            CFWSEL_CFW1603,         /*!< CFW1603		*/
            CFWSEL_FW5_STX,         /*!< FW5-STX		*/
            CFWSEL_FW5_8300,        /*!< FW5-8300		*/
            CFWSEL_FW8_8300,        /*!< FW8-8300		*/
            CFWSEL_FW7_STX,         /*!< FW7-STX		*/
            CFWSEL_FW8_STT,         /*!< FW8-STT		*/
            CFWSEL_FW5_STF_DETENT   /*!< FW5-STF Detent */
        };

        /*!
         * \enum CFW_COMMAND
         * Filter Wheel Command enum 
         */
        public enum CFW_COMMAND : ushort
        {
            CFWC_QUERY,         /*!< Query			*/
            CFWC_GOTO,          /*!< Go-to slot		*/
            CFWC_INIT,          /*!< Initialize		*/
            CFWC_GET_INFO,      /*!< Get Info		*/
            CFWC_OPEN_DEVICE,   /*!< Open device	*/
            CFWC_CLOSE_DEVICE   /*!< Close device	*/
        };

        /*!
         * \enum SFW_STATUS
         * Filter Wheel Status enum 
         */
        public enum CFW_STATUS : ushort
        {
            CFWS_UNKNOWN,   /*!< Unknown state	*/
            CFWS_IDLE,      /*!< Idle state		*/
            CFWS_BUSY       /*!< Busy state		*/
        };

        /*! 
         * \enum CFW_ERROR
         * Filter Wheel errors enum 
         */
        public enum CFW_ERROR : ushort
        {
            CFWE_NONE,              /*!< No error					*/
            CFWE_BUSY,              /*!< Busy error					*/
            CFWE_BAD_COMMAND,       /*!< Bad command error			*/
            CFWE_CAL_ERROR,         /*!< Calibration error			*/
            CFWE_MOTOR_TIMEOUT,     /*!< Motor timeout error		*/
            CFWE_BAD_MODEL,         /*!< Bad model error			*/
            CFWE_DEVICE_NOT_CLOSED, /*!< Device not closed	error	*/
            CFWE_DEVICE_NOT_OPEN,   /*!< Device not open error		*/
            CFWE_I2C_ERROR          /*!< I2C communication error	*/
        };

        /*! 
         * \enum CFW_POSITION
         * Filter Wheel position enum 
         */
        public enum CFW_POSITION : ushort
        {
            CFWP_UNKNOWN,   /*!< Unknown	*/
            CFWP_1,         /*!< Slot 1		*/
            CFWP_2,         /*!< Slot 2		*/
            CFWP_3,         /*!< Slot 3		*/
            CFWP_4,         /*!< Slot 4		*/
            CFWP_5,         /*!< Slot 5		*/
            CFWP_6,         /*!< Slot 6		*/
            CFWP_7,         /*!< Slot 7		*/
            CFWP_8,         /*!< Slot 8		*/
            CFWP_9,         /*!< Slot 9		*/
            CFWP_10         /*!< Slot 10	*/
        };

        /*!
         * \enum CFW_COM_PORT
         * Filter Wheel COM port enum 
         */
        public enum CFW_COM_PORT : ushort
        {
            CFWPORT_COM1 = 1,   /*!< COM1	*/
            CFWPORT_COM2,   /*!< COM2	*/
            CFWPORT_COM3,   /*!< COM3	*/
            CFWPORT_COM4    /*!< COM4	*/
        };

        /*! 
         * \enum CFW_GETINFO_SELECT
         * Filter Wheel Get Info select enum 
         */
        public enum CFW_GETINFO_SELECT : ushort
        {
            CFWG_FIRMWARE_VERSION,  /*!< Firmware version	*/
            CFWG_CAL_DATA,          /*!< Calibration data	*/
            CFWG_DATA_REGISTERS     /*!< Data registers		*/
        };

        /*!
         * \enum BITIO_OPERATION
         * Bit I/O Operation enum 
         */
        public enum BITIO_OPERATION : ushort
        {
            BITIO_WRITE,    /*!< Write	*/
            BITIO_READ      /*!< Read	*/
        };

        /*! 
         *\enum BITIO_NAME
         * Bit I/O Name enum 
         */
        public enum BITIO_NAME : ushort
        {
            BITI_PS_LOW,    /*!< In: PS Low	*/
            BITO_IO1,       /*!< Out: I/O 1	*/
            BITO_IO2,       /*!< Out: I/O 2 */
            BITI_IO3,       /*!< In: I/O 3	*/
            BITO_FPGA_WE    /*!< FPGA WE	*/
        };

        /*! 
         * \enum BTDI_ERROR
         * Biorad TDI Error enum 
         */
        public enum BTDI_ERROR : ushort
        {
            BTDI_SCHEDULE_ERROR = 1,    /*!< BTDI Schedule error	*/
            BTDI_OVERRUN_ERROR = 2      /*!< BTDI Overrun error		*/
        };

        /*!
         * \enum MF_MODEL_SELECT
         * Motor Focus Model Selection enum 
         */
        public enum MF_MODEL_SELECT : ushort
        {
            MFSEL_UNKNOWN,  /*!< Unknown	*/
            MFSEL_AUTO,     /*!< Automatic	*/
            MFSEL_STF       /*!< STF		*/
        };

        /*!
         * \enum MF_COMMAND
         * Motor Focus Command enum 
         */
        public enum MF_COMMAND : ushort
        {
            MFC_QUERY,      /*!< Query		*/
            MFC_GOTO,       /*!< Go-to		*/
            MFC_INIT,       /*!< Initialize	*/
            MFC_GET_INFO,   /*!< Get Info	*/
            MFC_ABORT       /*!< Abort		*/
        };

        /*!
         * \enum MF_STATUS
         * Motor Focus Status 
         */
        public enum MF_STATUS : ushort
        {
            MFS_UNKNOWN,    /*!< Unknown	*/
            MFS_IDLE,       /*!< Idle		*/
            MFS_BUSY        /*!< Busy		*/
        };

        /*!
         * \enum MF_ERROR 
         * Motor Focus Error state enum	
         */
        public enum MF_ERROR : ushort
        {
            MFE_NONE,           /*!< None				*/
            MFE_BUSY,           /*!< Busy				*/
            MFE_BAD_COMMAND,    /*!< Bad command		*/
            MFE_CAL_ERROR,      /*!< Calibration error	*/
            MFE_MOTOR_TIMEOUT,  /*!< Motor timeout		*/
            MFE_BAD_MODEL,      /*!< Bad model			*/
            MFE_I2C_ERROR,      /*!< I2C error			*/
            MFE_NOT_FOUND       /*!< Not found			*/
        };

        /*!
         * \enum MF_GETINFO_SELECT
         * Motor Focus Get Info Select enum 
         */
        public enum MF_GETINFO_SELECT : ushort
        {
            MFG_FIRMWARE_VERSION,   /*!< Firmware Version	*/
            MFG_DATA_REGISTERS      /*!< Data Registers		*/
        };

        /*!
         * \enum DIFF_GUIDER_COMMAND
         * Differential guider commands enum 
         */
        public enum DIFF_GUIDER_COMMAND : ushort
        {
            DGC_DETECT,         /*!< Detect	Differential guider hardware	*/
            DGC_GET_BRIGHTNESS, /*!< Get brightness							*/
            DGC_SET_BRIGHTNESS  /*!< Set brightness							*/
        };

        /*!
         * \enum DIFF_GUIDER_ERROR
         * Differential guider error enum 
         */
        public enum DIFF_GUIDER_ERROR : ushort
        {
            DGE_NO_ERROR,       /*!< No error						*/
            DGE_NOT_FOUND,      /*!< Differential guider not found	*/
            DGE_BAD_COMMAND,    /*!< Bad command					*/
            DGE_BAD_PARAMETER   /*!< Bad parameter					*/
        };

        /*!
         * \enum DIFF_GUIDER_STATUS
         * Differential Guider status enum 
         */
        public enum DIFF_GUIDER_STATUS : ushort
        {
            DGS_UNKNOWN,    /*!< Unknown	*/
            DGS_IDLE,       /*!< Idle		*/
            DGS_BUSY        /*!< Busy		*/
        };

        /*!
         * \enum FAN_STATE
         * Fan state enum 
         */
        public enum FAN_STATE : ushort
        {
            FS_OFF,         /*!< Fan Off	*/
            FS_ON,          /*!< Fan On		*/
            FS_AUTOCONTROL  /*!< Fan Auto	*/
        };

        /*!
         * \enum BULK_IO_COMMAND
         * Bulk IO command enum 
         */
        public enum BULK_IO_COMMAND : ushort
        {
            BIO_READ,   /*!< Read	*/
            BIO_WRITE,  /*!< Write	*/
            BIO_FLUSH   /*!< Flush	*/
        };

        /*!
         * \enum PIXEL_CHANNEL_MODE
         * Pixel channel mode enum 
         */
        public enum PIXEL_CHANNEL_MODE : ushort
        {
            PIXEL_CHANNEL_MODE_A,   /*!< Pixel Channel A	*/
            PIXEL_CHANNEL_MODE_B,   /*!< Pixel Channel B	*/
            PIXEL_CHANNEL_MODE_AB   /*!< Pixel Channel AB	*/
        };

        /*! 
         * \enum ACTIVE_PIXEL_CHANNEL
         * Active Pixel Channel enum 
         */
        public enum ACTIVE_PIXEL_CHANNEL : ushort
        {
            PIXEL_CHANNEL_A,    /*!< Pixel Channel A	*/
            PIXEL_CHANNEL_B     /*!< Pixel Channel B	*/
        };

        public enum EXTRA_EXPOSURE_STATUS : ushort
        {
            XES_IDLE,           //!< CCD is currently idle.
            XES_PRE_EXP,        //!< CCD is in the pre-exposure phase.
            XES_INTEGRATING,    //!< CCD is currently exposing/integrating an image.
            XES_POST_EXP        //!< CCD is in the post-exposure phase.
        };

        /*
         *	General Purpose Flags
         */

        /*!
         *	\def END_SKIP_DELAY
         *	set in EndExposureParams::ccd to skip synchronization delay - Use this to increase the
         *	rep rate when taking darks to later be subtracted from SC_LEAVE_SHUTTER exposures such as when tracking and imaging.
         */
        const UInt16 END_SKIP_DELAY = 0x8000;

        /*!
         *	\def START_SKIP_VDD
         *	Set in StartExposureParams::ccd to skip lowering Imaging CCD Vdd during integration. - Use this to
         *	increase the rep rate when you don't care about glow in the upper-left corner of the imaging CCD.
         */
        const UInt16 START_SKIP_VDD = 0x8000;

        /*!
         * \def START_MOTOR_ALWAYS_ON
         * 	Set in StartExposureParams::ccd and EndExposureParams::ccd to force shutter motor to stay on all the 
         *	time which reduces delays in Start and End Exposure timing and yields higher image throughput.  Don't
         *	do this too often or camera head will heat up.
         */
        const UInt16 START_MOTOR_ALWAYS_ON = 0x4000;

        /*!
         * \def ABORT_DONT_END
         *	Set in EndExposureParams::ccd to abort the exposure completely instead of just ending 
         *	the integration phase for cameras with internal frame buffers like the STX.
         */
        const UInt16 ABORT_DONT_END = 0x2000;

        /*!
         * \defgroup EXPOSURE_FLAGS
         */

        /*!
         * \def EXP_TDI_ENABLE
         * \ingroup EXPOSURE_FLAGS
         * Set in StartExposureParams2::exposureTime enable TDI readout mode [TODO: Add supported cameras].
         */
        const UInt32 EXP_TDI_ENABLE = 0x01000000;   //!< Enable TDI mode flag.

        /*!
         * \def EXP_RIPPLE_CORRECTION
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime ripple correction for STF-8050/4070
         */
        const UInt32 EXP_RIPPLE_CORRECTION = 0x02000000;    //!< Enable Ripple correction flag.

        /*!
         * \def EXP_DUAL_CHANNEL_MODE
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to activate the dual channel CCD readout mode of the STF-8050.
         */
        const UInt32 EXP_DUAL_CHANNEL_MODE = 0x04000000;    //!< Enable dual channel readout mode flag.

        /*!
         * \def EXP_FAST_READOUT
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to activate the fast readout mode of the STF-8300, etc.
         */
        const UInt32 EXP_FAST_READOUT = 0x08000000; //!< Enable fast readout mode flag.

        /*!
         * \def EXP_MS_EXPOSURE
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to interpret exposure time as milliseconds.
         */
        const UInt32 EXP_MS_EXPOSURE = 0x10000000;  //!< Enable millisecond exposure time flag.

        /*!
         * \def EXP_LIGHT_CLEAR
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to do light clear of the CCD.
         */
        const UInt32 EXP_LIGHT_CLEAR = 0x20000000;  //!< Do light clear of CCD flag.

        /*!
         * \def EXP_SEND_TRIGGER_OUT
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to send trigger out Y-.
         */
        const UInt32 EXP_SEND_TRIGGER_OUT = 0x40000000;  //!< Send trigger out flag.

        /*!
         * \def EXP_WAIT_FOR_TRIGGER_IN
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to wait for trigger in pulse.
         */
        const UInt32 EXP_WAIT_FOR_TRIGGER_IN = 0x80000000;  //!< Wait for trigger in flag.

        /*!
         * \def EXP_TIME_MASK
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to mask with exposure time to remove flags.
         */
        const UInt32 EXP_TIME_MASK = 0x00FFFFFF;  //!< Mask for exposure time value.

        /*!
         * 	\defgroup CAPABILITIES_BITS 
         *	Bit Field Definitions for the in the GetCCDInfoResults4 struct.
         */

        /*!
         * \def CB_CCD_TYPE_MASK
         * \ingroup CAPABILITIES_BITS
         * mask for CCD type
         */
        const UInt16 CB_CCD_TYPE_MASK = 0x0001; //!< Mask for CCD type.

        /*! 
         *	\def CB_CCD_TYPE_FULL_FRAME
         * \ingroup CAPABILITIES_BITS
         *	b0=0 is full frame CCD
         */
        const UInt16 CB_CCD_TYPE_FULL_FRAME = 0x0000;

        /*! 
         *	\def CB_CCD_TYPE_FRAME_TRANSFER
         * \ingroup CAPABILITIES_BITS
         *	b0=1 is frame transfer CCD
         */
        const UInt16 CB_CCD_TYPE_FRAME_TRANSFER = 0x0001;

        /*!
         *	\def CB_CCD_ESHUTTER_MASK
         * \ingroup CAPABILITIES_BITS
         *	mask for electronic shutter type
         */
        const UInt16 CB_CCD_ESHUTTER_MASK = 0x0002;

        /*! 
         *	\def CB_CCD_ESHUTTER_NO
         * \ingroup CAPABILITIES_BITS
         *	b1=0 indicates no electronic shutter
         */
        const UInt16 CB_CCD_ESHUTTER_NO = 0x0000;

        /*! 
         *	\def CB_CCD_ESHUTTER_YES
         * \ingroup CAPABILITIES_BITS
         *	b1=1 indicates electronic shutter
         */
        const UInt16 CB_CCD_ESHUTTER_YES = 0x0002;

        /*! 
         *	\def CB_CCD_EXT_TRACKER_MASK
         * \ingroup CAPABILITIES_BITS
         *	mask for external tracker support
         */
        const UInt16 CB_CCD_EXT_TRACKER_MASK = 0x0004;

        /*! 
         *	\def CB_CCD_EXT_TRACKER_NO
         * \ingroup CAPABILITIES_BITS
         *	b2=0 indicates no external tracker support
         */
        const UInt16 CB_CCD_EXT_TRACKER_NO = 0x0000;

        /*! 
         *	\def CB_CCD_EXT_TRACKER_YES
         * \ingroup CAPABILITIES_BITS
         *	b2=1 indicates external tracker support
         */
        const UInt16 CB_CCD_EXT_TRACKER_YES = 0x0004;

        /*! 
         *	\def CB_CCD_BTDI_MASK
         * \ingroup CAPABILITIES_BITS
         *	mask for BTDI support
         */
        const UInt16 CB_CCD_BTDI_MASK = 0x0008;

        /*! 
         *	\def CB_CCD_BTDI_NO
         * \ingroup CAPABILITIES_BITS
         *	b3=0 indicates no BTDI support
         */
        const UInt16 CB_CCD_BTDI_NO = 0x0000;

        /*! 
         *	\def CB_CCD_BTDI_YES
         * \ingroup CAPABILITIES_BITS
         *	b3=1 indicates BTDI support
         */
        const UInt16 CB_CCD_BTDI_YES = 0x0008;

        /*! 
         *	\def CB_AO8_MASK
         * \ingroup CAPABILITIES_BITS
         *	mask for AO-8 detected 
         */
        const UInt16 CB_AO8_MASK = 0x0010;

        /*! 
         *	\def CB_AO8_NO
         * \ingroup CAPABILITIES_BITS
         *	b4=0 indicates no AO-8 detected
         */
        const UInt16 CB_AO8_NO = 0x0000;

        /*! 
         *	\def CB_AO8_YES
         * \ingroup CAPABILITIES_BITS
         *	b4=1 indicates AO-8 detected
         */
        const UInt16 CB_AO8_YES = 0x0010;

        /*!
         *	\def CB_FRAME_BUFFER_MASK
         * \ingroup CAPABILITIES_BITS
         *	mask for camera with frame buffer
         */
        const UInt16 CB_FRAME_BUFFER_MASK = 0x0020;

        /*!
         *	\def CB_FRAME_BUFFER_NO
         * \ingroup CAPABILITIES_BITS
         *	b5=0 indicates camera without Frame Buffer
         */
        const UInt16 CB_FRAME_BUFFER_NO = 0x0000;

        /*!
         *	\def CB_FRAME_BUFFER_YES
         * \ingroup CAPABILITIES_BITS
         *	b5=1 indicates camera with Frame Buffer
         */
        const UInt16 CB_FRAME_BUFFER_YES = 0x0020;

        /*!
         *	\def CB_REQUIRES_STARTEXP2_MASK
         * \ingroup CAPABILITIES_BITS
         *	mask for camera that requires StartExposure2
         */
        const UInt16 CB_REQUIRES_STARTEXP2_MASK = 0x0040;

        /*!
         *	\def CB_REQUIRES_STARTEXP2_NO
         * \ingroup CAPABILITIES_BITS
         *	b6=0 indicates camera works with StartExposure
         */
        const UInt16 CB_REQUIRES_STARTEXP2_NO = 0x0000;

        /*!
         *	\def CB_REQUIRES_STARTEXP2_YES
         * \ingroup CAPABILITIES_BITS
         *	b6=1 indicates camera Requires StartExposure2
         */
        const UInt16 CB_REQUIRES_STARTEXP2_YES = 0x0040;

        /*!
         * \defgroup MINIMUM_DEFINES
         */

        /*!
         *	\def MIN_ST7_EXPOSURE
         * \ingroup MINIMUM_DEFINES
         *	Minimum exposure for ST-7 cameras in 1/100ths second
         */
        const Int16 MIN_ST7_EXPOSURE = 12;

        /*!
         *	\def MIN_ST402_EXPOSURE
         * \ingroup MINIMUM_DEFINES
         *	Minimum exposure for ST-402 cameras in 1/100ths second
         */
        const Int16 MIN_ST402_EXPOSURE = 4;

        /*!
         *	\def MIN_ST3200_EXPOSURE
         * \ingroup MINIMUM_DEFINES
         *	Minimum exposure fpr STF-3200 cameras in 1/100ths second
         */
        const Int16 MIN_ST3200_EXPOSURE = 9;


        /*!
         *	\def MIN_STF8300_EXPOSURE
         * \ingroup MINIMUM_DEFINES
         *	Minimum exposure for STF-8300 cameras in 1/100ths second
         */
        const Int16 MIN_STF8300_EXPOSURE = 9;

        /*!
         *	\def MIN_STF8050_EXPOSURE
         * \ingroup MINIMUM_DEFINES
         *	Minimum exposure for STF-8050 cameras in 1/1000ths second since has E Shutter
         */
        const Int16 MIN_STF8050_EXPOSURE = 1;

        /*!
         *	\def MIN_STF4070_EXPOSURE
         * \ingroup MINIMUM_DEFINES
         *	Minimum exposure for STF-4070 cameras in 1/1000ths second since has E Shutter
         */
        const Int16 MIN_STF4070_EXPOSURE = 1;


        /*!
         *	\def MIN_STF4070_EXPOSURE
         * \ingroup MINIMUM_DEFINES
         *	Minimum exposure for STF-0402 cameras in 1/100ths second.
         */
        const Int16 MIN_STF0402_EXPOSURE = 4;

        /*!
         *	\def MIN_STX_EXPOSURE
         * \ingroup MINIMUM_DEFINES
         *	Minimum exposure for STX cameras in 1/100ths second
         */
        const Int16 MIN_STX_EXPOSURE = 18;

        /*!
         *	\def MIN_STT_EXPOSURE
         * \ingroup MINIMUM_DEFINES
         *	Minimum exposure for STT cameras in 1/100ths second
         */
        const Int16 MIN_STT_EXPOSURE = 12;

        /*!
         *	\def MIN_STU_EXPOSURE
         * \ingroup MINIMUM_DEFINES
         *	Minimum exposure in 1/1000ths second since ST-i has E Shutter
         */
        const Int16 MIN_STU_EXPOSURE = 1;

        /*!
            \defgroup commandParamStructs
            Command Parameter and Results Structs

            Make sure you set your compiler for byte structure alignment
            as that is how the driver was built.

        */

        /*!
         * \struct StartExposureParams
         * \brief Start Exposure command parameters
         *
         * Parameters used to start SBIG camera exposures.
         */
        public struct StartExposureParams
        {
            public UInt16 ccd;         //!< Requested CCD. see also: CCD_REQUEST enum.
            public UInt32 exposureTime; //!< Exposure time in hundredths of a second in least significant 24 bits. Most significant bits are bit-flags described in exposureTime #define block.
            public UInt16 abgState;        //!< see also: ABG_STATE7 enum.
            public UInt16 openShutter; //!< see also: SHUTTER_COMMAND enum.
        };

        /*!
         * \struct StartExposureParams2
         * \brief Start Exposure command parameters Expanded
         *
         * Expanded parameters structure used to start SBIG camera exposures.
         */
        public struct StartExposureParams2
        {
            public UInt16 ccd;         //!< Requested CCD. see also: CCD_REQUEST enum.
            public UInt32 exposureTime; //!< Exposure time in hundredths of a second in least significant 24 bits. Most significant bits are bit-flags described in exposureTime #define block.
            public UInt16 abgState;        //!< Deprecated. See also: ABG_STATE7.
            public UInt16 openShutter; //!< see also: SHUTTER_COMMAND enum.
            public UInt16 readoutMode; //!< readout mode. See also: READOUT_BINNING_MODE enum.
            public UInt16 top;         //!< top-most row to read out. (0 based)
            public UInt16 left;            //!< left-most column to read out. (0 based)
            public UInt16 height;          //!< image height in binned pixels.
            public UInt16 width;           //!< image width in binned pixels.
        };

        /*!
         * \struct EndExposureParams
         * \brief End Exposure command parameters
         *
         * Parameters used to end SBIG camera exposures.
         * Set ABORT_DONT_END flag in ccd to abort exposures in supported cameras.
         */
        public struct EndExposureParams
        {
            public UInt16 ccd;         //!< Requested CCD. see also: CCD_REQUEST enum.
        };

        /*!
         * \struct ReadoutLineParams
         * \brief Readout Line command parameters
         *
         * Parameters used to readout lines of SBIG cameras during readout.
         */
        public struct ReadoutLineParams
        {
            public UInt16 ccd;         //!< Requested CCD. see also: CCD_REQUEST enum.
            public UInt16 readoutMode; //!< readout mode. See also: READOUT_BINNING_MODE enum.
            public UInt16 pixelStart;      //!< left-most pixel to read out.
            public UInt16 pixelLength; //!< number of pixels to digitize.
        };

        /*!
         * \struct DumpLinesParams
         * \brief Dump Lines command parameters
         *
         * Parameters used to dump/flush CCD lines during readout.
         */
        public struct DumpLinesParams
        {
            public UInt16 ccd;         //!< Requested CCD. see also: CCD_REQUEST enum.
            public UInt16 readoutMode; //!< readout mode. See also: READOUT_BINNING_MODE enum.
            public UInt16 lineLength;      //!< number of lines to dump.
        };

        /*!
         * \struct EndReadoutParams
         * \brief End Readout command parameters
         *
         * Parameters used to end SBIG camera readout.
         */
        public struct EndReadoutParams
        {
            public UInt16 ccd;         //!< Requested CCD. see also: CCD_REQUEST enum.
        };

        /*!
         * \struct StartReadoutParams
         * \brief Start Readout command parameters
         *
         * (Optional) Parameters used to start SBIG camera readout.
         * Automatically dumps unused exposure lines.
         */
        public struct StartReadoutParams
        {
            public UInt16 ccd;         //!< Requested CCD. see also: CCD_REQUEST enum.
            public UInt16 readoutMode; //!< readout mode. See also: READOUT_BINNING_MODE enum.
            public UInt16 top;         //!< top-most row to read out. (0 based)
            public UInt16 left;            //!< left-most column to read out. (0 based)
            public UInt16 height;          //!< image height in binned pixels.
            public UInt16 width;           //!< image width in binned pixels.
        };

        /*!
         * \struct SetTemperatureRegulationParams
         * \brief Set Temperature Regulation command parameters
         *
         * The Set Temperature Regulation command is used to enable or disable the CCD's temperature
         * regulation. Uses special units for the CCD temperature. The Set Temperature Regulation 2 
         * command described in the next section is easier to use with temperatures stated in Degrees
         * Celsius.
         * 
         */
        public struct SetTemperatureRegulationParams
        {
            public UInt16 regulation;  //!< see also: TEMPERATURE_REGULATION enum.
            public UInt16 ccdSetpoint; //!< CCD temperature setpoint in A/D units if regulation on or TE drive level (0-255 = 0-100%) if regulation override.
        };

        /*!
         * \struct SetTemperatureRegulationParams2
         * \brief Set Temperature Regulation command parameters Alternate
         *
         * The Set Temperature Regulation 2 command is used to enable or disable the CCD's temperature
         * regulation using temperatures in Degrees C instead of the funny A/D units described above.
         */
        public struct SetTemperatureRegulationParams2
        {
            public UInt16 regulation;  //!< see also: TEMPERATURE_REGULATION enum.
            public double ccdSetpoint;         //!< CCD temperature setpoint in degrees Celsius.
        };

        /*!
         * \struct QueryTemperatureStatusParams
         * \brief Query Temperature Status command parameters
         *
         * The Query Temperature Status command is used to monitor the CCD's temperature regulation. The
         * original version of this command took no Parameters (a NULL pointer) but the command has been
         * expanded to allow a more user friendly result. If you pass a NULL pointer in the Parameters variable
         * youl get the classic result. If you pass a pointer to a QueryTemperatureStatusParams struct youl have
         * access to the expanded results.
         */
        public struct QueryTemperatureStatusParams
        {
            public UInt16 request; //!< see also: TEMP_STATUS_REQUEST enum.
        };

        /*!
         * \struct QueryTemperatureStatusResults
         * \brief Query Temperature Status command results
         *
         * The results struct of a Temperature Status Query, with request set to TEMP_STATUS_STANDARD.
         */
        public struct QueryTemperatureStatusResults
        {
            public MY_LOGICAL enabled;         //!< temperature regulation is enabled when this is TRUE.
            public UInt16 ccdSetpoint;     //!< CCD temperature or thermistor setpoint in A/D units.
            public UInt16 power;               //!< this is the power being applied to the TE cooler to maintain temperature regulation and is in the range 0 thru 255.
            public UInt16 ccdThermistor;       //!< this is the CCD thermistor reading in A/D units.
            public UInt16 ambientThermistor;   //!< this is the ambient thermistor reading in A/D units.
        };

        /*!
         * \struct QueryTemperatureStatusResults2
         * \brief Query Temperature Status command results expanded
         *
         * The results struct of a Temperature Status Query, with request set to TEMP_STATUS_ADVANCED.
         */
        public struct QueryTemperatureStatusResults2
        {
            public MY_LOGICAL coolingEnabled;                  //!< temperature regulation is enabled when this is TRUE. &REGULATION_FROZEN_MASK is TRUE when TE is frozen.
            public MY_LOGICAL fanEnabled;                      //!< fan state and is one of the following: FS_OFF (off), FS_ON (manual control) or FS_AUTOCONTROL (auto speed control).
            public double ccdSetpoint;                     //!< CCD Setpoint temperature in 蚓.
            public double imagingCCDTemperature;           //!< imaging CCD temperature in degrees 蚓.
            public double trackingCCDTemperature;          //!< tracking CCD temperature in degrees 蚓.
            public double externalTrackingCCDTemperature;  //!< external tracking CCD temperature in 蚓.
            public double ambientTemperature;              //!< ambient camera temperature in 蚓.
            public double imagingCCDPower;                 //!< percent power applied to the imaging CCD TE cooler.
            public double trackingCCDPower;                //!< percent power applied to the tracking CCD TE cooler.
            public double externalTrackingCCDPower;        //!< percent power applied to the external tracking TE cooler.
            public double heatsinkTemperature;             //!< imaging CCD heatsink temperature in 蚓.
            public double fanPower;                        //!< percent power applied to the fan.
            public double fanSpeed;                        //!< fan speed in RPM.
            public double trackingCCDSetpoint;             //!< tracking CCD Setpoint temperature in 蚓.
        };

        /*!
         * \struct ActivateRelayParams
         * \brief Activate Relay command parameters
         *
         * The Activate Relay command is used to activate one or more of the telescope control outputs or to cancel an activation in progress.
         *
         * The status for this command (from QueryCommandStatus) consists of four bit fields:
         * 
         * b3 = +X Relay, 0=Off, 1= Active
         * b2 = -X Relay, 0=Off, 1= Active
         * b1 = +Y Relay, 0=Off, 1= Active
         * b0 = -Y Relay, 0=Off, 1= Active
         */
        public struct ActivateRelayParams
        {
            public UInt16 tXPlus;  //!< x plus activation duration in hundredths of a second
            public UInt16 tXMinus; //!< x minus activation duration in hundredths of a second
            public UInt16 tYPlus;  //!< y plus activation duration in hundredths of a second
            public UInt16 tYMinus; //!< y minus activation duration in hundredths of a second
        };

        /*!
         * \struct PulseOutParams
         * \brief Pulse Out command parameters
         *
         * The Pulse Out command is used with the ST-7/8/etc to position the CFW-6A/CFW-8 and with the PixCel255 and PixCel237 to position 
         * the internal vane/filter wheel.
         * 
         * The status for this command is: 
         * 
         * b0 - Normal status, 0 = inactive, 1 = pulse out in progress
         * b1-b3 - PixCel255/237 Filter state, 0=moving, 1-5=at position 1-5, 6=unknown
         *
         */
        public struct PulseOutParams
        {
            public UInt16 numberPulses;    //!< number of pulses to generate (0 thru 255).
            public UInt16 pulseWidth;      //!< width of pulses in units of microseconds with a minimum of 9 microseconds.
            public UInt16 pulsePeriod;     //!< period of pulses in units of microseconds with a minimum of 29 plus the pulseWidth microseconds.
        };

        /*!
         * \struct TXSerialBytesParams
         * \brief Transfer Serial Bytes command parameters
         *
         * The TX Serial Bytes command is for internal use by SBIG. It a very low level version of commands
         * like AO Tip Tilt that are used to send data out the ST-7/8/etc telescope port to accessories like the
         * AO-7. There no reason why you should need to use this command. Just use the dedicated commands
         * like AO Tip Tilt.
         *
         */
        public struct TXSerialBytesParams
        {
            public UInt16 dataLength;  //!< Length of data buffer to send
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
            public Byte[] data;    //!< Buffer of data to send.
        };

        /*!
         *	\struct TXSerialBytesResults.
         *  \brief Transfer Serial Bytes command results.
         * 
         * Results of a TXSerialBytes command.
         */
        public struct TXSerialBytesResults
        {
            public UInt16 bytesSent;   //!< Bytes sent out.
        };

        /*!
         *	\struct GetSerialStatusResults.
         *  \brief Get Serial Status command results.
         * 
         * The Get Serial Status command is for internal use by SBIG. It a very low level version of commands
         * like AO Tip Tilt that are used to send data out the ST-7/8/etc telescope port to accessories like the
         * AO-7. There no reason why you should need to use this command. Just use the dedicated commands
         * like AO Tip Tilt.
         */
        public struct GetSerialStatusResults
        {
            public MY_LOGICAL clearToCOM;
        };

        /*! 
         * \struct EstablishLinkParams
         * \brief Establish Link command parameters
         *
         * The Establish Link command is used by the application to establish a communications link with the
         * camera. It should be used before any other commands are issued to the camera (excluding the Get
         * Driver Info command).
         */
        public struct EstablishLinkParams
        {
            public UInt16 sbigUseOnly; //!< Maintained for historical purposes. Keep set to 0.
        };

        /*!
         * \struct EstablishLinkResults 
         * \brief Establish Link command results
         * 
         * Results from an EstablishLink command.
         */
        public struct EstablishLinkResults
        {
            public UInt16 cameraType; //!< Returns connected camera's type ID. See also: CAMERA_TYPE enum. 
        };

        /*!
         * \struct GetDriverInfoParams
         * \brief Get Driver Info command parameters
         *
         * The Get Driver Info command is used to determine the version and capabilities of the DLL/Driver. For
         * future expandability this command allows you to request several types of information. Initially the
         * standard request and extended requests will be supported but as the driver evolves additional requests
         * will be added.
         */
        public struct GetDriverInfoParams
        {
            public UInt16 request; //!< see also: DRIVER_REQUEST enum.
        };

        /*!
         * \struct GetDriverInfoResults0
         * \brief Get Driver Info command Results 0
         * 
         * Standard, Extended and USB Loader Results Struct.
         */
        public struct GetDriverInfoResults0
        {
            public UInt16 version;     //!< driver version in BCD with the format XX.XX
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string name;              //!< driver name, null terminated string
            public UInt16 maxRequest;  //!< maximum request response available from this driver
        };

        /*!
         * \struct GetCCDInfoParams
         * \brief Get CCD Info command parameters
         *
         * The Get CCD Info command is used by the application to determine the model of camera being
         * controlled and its capabilities. For future expandability this command allows you to request several
         * types of information. Currently 6 standard requests are supported but as the driver evolves additional
         * requests will be added.
         */
        public struct GetCCDInfoParams
        {
            public UInt16 request; /* see also: CCD_INFO_REQUEST. */
        };

        /*! 
         * \struct READOUT_INFO
         * \brief Readout mode property struct.
         * 
         * Internal structure for storing readout modes.
         */
        public struct READOUT_INFO
        {
            public UInt16 mode;            //!< readout mode ID (see also: READOUT_BINNING_MODE)
            public UInt16 width;           //!< width of image in pixels
            public UInt16 height;          //!< height of image in pixels
            public UInt16 gain;            //!< a four digit BCD number specifying the amplifier gain in e-/ADU in XX.XX format
            public UInt32 pixel_width;      //!< an eight digit BCD number specifying the pixel width in microns in the XXXXXX.XX format
            public UInt32 pixel_height; //!< an eight digit BCD number specifying the pixel height in microns in the XXXXXX.XX format
        };

        /*!
         * \struct GetCCDInfoResults0
         * \brief Get CCD Info command results
         *
         * Get CCD Info command results 0 and 1 request.
         */
        public struct GetCCDInfoResults0
        {
            public UInt16 firmwareVersion;     //!< version of the firmware in the resident microcontroller in BCD format (XX.XX, 0x1234 = 12.34).
            public UInt16 cameraType;          //!< Camera type ID. see also: CAMERA_TYPE enum.
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string name;                      //!< null terminated string containing the name of the camera.
            public UInt16 readoutModes;        //!< number of readout modes supported.
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 20)]
            public READOUT_INFO[] readoutInfo;
        };

        /*! 
         * \struct GetCCDInfoResults2
         * \brief Get CCD Info command results pass 2
         *
         * Get CCD Info command results second request.
         */
        public struct GetCCDInfoResults2
        {
            public UInt16 badColumns;  //!< number of bad columns in imaging CCD
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public UInt16[] columns;  //!< bad columns
            public UInt16 imagingABG;  //!< type of Imaging CCD, 0= No ABG Protection, 1 = ABG Present. see also: IMAGING_ABG enum.
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            public string serialNumber;      //!< null terminated serial number string
        };

        /*! 
         * \struct GetCCDInfoResults3
         * \brief Get CCD Info command results pass 3
         *
         * Get CCD Info command results third request. (For the PixCel255/237)
         */
        public struct GetCCDInfoResults3
        {
            public UInt16 adSize;      //!< 0 = Unknown, 1 = 12 bits, 2 = 16 bits. see also: AD_SIZE enum.
            public UInt16 filterType;  //!< 0 = Unknown, 1 = External, 2 = 2 Position, 3 = 5 Position. see also: FILTER_TYPE enum. 
        };

        /*! 
         * \struct GetCCDInfoResults4
         * \brief Get CCD Info command results pass 4 and 5
         *
         * Get CCD Info command results fourth and fifth request. (For all cameras)
         * 
         * Capabilities bits:
         * b0: 0 = CCD is Full Frame Device, 1 = CCD is Frame Transfer Device,
         * b1: 0 = No Electronic Shutter, 1 = Interline Imaging CCD with Electronic Shutter and
         * millisecond exposure capability
         * b2: 0 = No hardware support for external Remote Guide Head, 1 = Detected hardware
         * support for external Remote Guide Head.
         * b3: 1 = Supports the special Biorad TDI acquisition mode.
         * b4: 1 = AO8 detected.
         * b5: 1 = Camera contains an internal frame buffer.
         * b6: 1 = Camera requires the StartExposure2 command instead of the older depricated StartExposure command.
         * Other: See the CB_XXX_XXX definitions in the sbigurdv.h header file.
         */
        public struct GetCCDInfoResults4
        {
            public UInt16 capabilitiesBits;    //!< Camera capabilities. See the CB_XXX_XXX definitions in the sbigurdv.h header file.
            public UInt16 dumpExtra;           //!< Number of unbinned rows to dump to transfer image area to storage area.
        };

        /*!
         * \struct GetCCDInfoResults6
         * \brief Get CCD Info command results pass 6
         *
         * Get CCD Info command results sixth request. (For all cameras)
         * 
         * Camera bits:
         * b0: 0 = STX camera, 1 = STXL camera
         * b1: 0 = Mechanical shutter, 1 = No mechanical shutter (only an electronic shutter)
         * b2 ?b31: reserved for future expansion
         * 
         * CCD Bits:
         * b0: 0 = Imaging Mono CCD, 1 = Imaging Color CCD
         * b1: 0 = Bayer color matrix, 1 = Truesense color matrix
         * b2 ?b31: reserved for future expansion
         */
        public struct GetCCDInfoResults6
        {
            public UInt32 cameraBits;   //!< Set of bits for additional camera capabilities
            public UInt32 ccdBits;      //!< Set of bits for additional CCD capabilities
            public UInt32 extraBits;    //!< Set of bits for additional capabilities
        };

        /*! 
         * \struct QueryCommandStatusParams
         * \brief Query Command Status command parameters
         *
         * The Query Command Status command is used to monitor the progress of a previously requested
         * command. Typically this will be used to monitor the progress of an exposure, relay closure or CFW-6A
         * move command.
         */
        public struct QueryCommandStatusParams
        {
            public UInt16 command; //!< command of which the status is desired
        };

        /*! 
         * \struct QueryCommandStatusResults
         * \brief Query Command Status command results 
         *
         * Results for the Query Command Status command.
         */
        public struct QueryCommandStatusResults
        {
            public UInt16 status;      //!< command status.
        };

        /*! 
         * \struct QueryCommandStatusResults2
         * \brief Query Command Status command results 
         *
         * Results for the Query Command Status command.
         */
        public struct QueryCommandStatusResults2
        {
            public UInt16 status;      //!< command status. 
            public UInt16 info;        //!< expanded information on command status.
        };

        /*! 
         * \struct MiscellaneousControlParams
         * \brief Miscellaneous Control command results 
         *
         * The Miscellaneous Control command is used to control the Fan, LED, and shutter. The camera powers
         * up with the Fan on, the LED on solid, and the shutter closed. The driver flashes the LED at the low rate
         * while the Imaging CCD is integrating, flashes the LED at the high rate while the Tracking CCD is
         * integrating and sets it on solid during the readout.
         * 
         * The status returned for this command from Query Command Status has the following structure:
         * b7-b0 - Shutter edge - This is the position the edge of the shutter was detected at for the last shutter move. Normal values are 7 thru 9. Any other value including 255 indicates a shutter failure and the shutter should be reinitialized.
         * b8 - the Fan is enabled when this bit is 1
         * b10b9 - Shutter state, 0=open, 1=closed, 2=opening, 3=closing
         * b12b11 - LED state, 0=off, 1=on, 2=blink low, 3=blink high
         */
        public struct MiscellaneousControlParams
        {
            public MY_LOGICAL fanEnable;           //!< set TRUE to turn on the Fan.
            public UInt16 shutterCommand;  //!< see also: SHUTTER_COMMAND enum.
            public UInt16 ledState;        //!< see also: LED_STATE enum.
        };

        /*! 
         * \struct ReadOffsetParams
         * \brief Read Offset command parameters
         *
         * The Read Offset command is used to measure the CCD's offset. In the SBIG cameras the offset is 
         * adjusted at the factory and this command is for testing or informational purposes only.
         */
        public struct ReadOffsetParams
        {
            public UInt16 ccd;     //!< see also: CCD_REQUEST enum.
        };

        /*! 
         * \struct ReadOffsetResults
         * \brief Read Offset command results
         *
         * Results structure for the Read Offset command.
         */
        public struct ReadOffsetResults
        {
            public UInt16 offset;  //!< the CCD's offset.
        };

        /*! 
         * \struct ReadOffsetResults2
         * \brief Read Offset command results expanded
         *
         * The Read Offset 2 command is used to measure the CCD's offset and the noise in the readout register.
         * In the SBIG cameras the offset is adjusted at the factory and this command is for testing or informational
         * purposes only.
         */
        public struct ReadOffsetResults2
        {
            public UInt16 offset;  //!< the CCD's offset.
            public double rms;             //!< noise in the ccd readout register in ADUs rms.
        };

        /*! 
         * \struct AOTipTiltParams
         * \brief AO Tip/Tilt command parameters
         *
         * The AO Tip Tilt Command is used to position an AO-7 attached to the telescope port of an ST-7/8/etc.
         */
        public struct AOTipTiltParams
        {
            public UInt16 xDeflection; //!< this is the desired position of the mirror in the X axis.
            public UInt16 yDeflection; //!< this is the desired position of the mirror in the Y axis
        };

        /*! 
         * \struct AOSetFocusParams
         * \brief AO Set Focus command parameters
         *
         * This command is reserved for future use with motorized focus units. Prototypes of the AO-7 had
         * motorized focus but the feature was removed in the production units. This command is a holdover from
         * that.
         */
        public struct AOSetFocusParams
        {
            public UInt16 focusCommand;    //!< see also: AO_FOCUS_COMMAND enum.
        };

        /*! 
         * \struct AODelayParams
         * \brief AO Delay command parameters
         *
         * The AO Delay Command is used to generate millisecond type delays for exposing the Tracking CCD.
         * This sleep command is blocking.
         */
        public struct AODelayParams
        {
            public UInt32 delay;    //!< this is the desired delay in microseconds.
        };

        /*!
         * \struct GetTurboStatusResults
         * \brief Get Turbo Status command results
         *
         * The current driver does not use this command. It was added in a previous version and never removed. It
         * could be reassigned in the future.
         */
        public struct GetTurboStatusResults
        {
            public MY_LOGICAL turboDetected;   //!< TRUE if turbo is detected.
        };

        /*!
         * \struct OpenDeviceParams
         * \brief Open Device command parameters
         *
         * The Open Device command is used to load and initialize the low-level driver. You will typically call
         * this second (after Open Driver).
         */
        public struct OpenDeviceParams
        {
            public UInt16 deviceType;          //!< see also: SBIG_DEVICE_TYPE enum. specifies LPT, Ethernet, etc.
            public UInt16 lptBaseAddress;      //!< for deviceType::DEV_LPTN: Windows 9x Only, Win NT uses deviceSelect.
            public UInt32 ipAddress;            //!< for deviceType::DEV_ETH:  Ethernet address.
        };

        /*!
         * \struct SetIRQLParams
         * \brief Set IRQ Level command parameters
         *
         * This command allows you to control the IRQ priority of the driver under Windows NT/2000/XP. The
         * default settings should work fine for all users and these commands should not need to be used.
         * 
         * We use three settings in our CCDOPS software: High = 27, Medium = 15, Low = 2. Under fast
         * machines Low will work fine. On slower machines the mouse may get sluggish unless you select the
         * Medium or High priority.
         */
        public struct SetIRQLParams
        {
            public UInt16 level;   //!< IRQ Level.
        };

        /*!
         * \struct GetIRQLResults
         * \brief Get IRQ Level command results
         *
         * Results of Get IRQ Level command.
         */
        public struct GetIRQLResults
        {
            public UInt16 level;   //!< IRQ Level.
        };

        /*!
         * \struct GetLinkStatusResults
         * \brief Get Link Status command results
         *
         * This command returns the status of the communications link established with the camera.
         */
        public struct GetLinkStatusResults
        {
            public MY_LOGICAL linkEstablished; //!< TRUE when a link has been established
            public UInt16 baseAddress;     //!< base address of the LPT port.
            public UInt16 cameraType;          //!< see also: CAMERA_TYPE enum.
            public UInt32 comTotal;         //!< total number of communications with camera.
            public UInt32 comFailed;            //!< total number of failed communications with camera.
        };

        /*!
         * \struct GetUSTimerResults
         * \brief Get Microsecond Timer command results
         *
         * This command is of extremely limited (and unknown) use. When you have established a link to a
         * parallel port based camera under Windows NT/2000/XP this command returns a counter with 1
         * microsecond resolution. Under all other circumstances the counter is zero.
         */
        public struct GetUSTimerResults
        {
            public UInt32 count;    //!< counter value in microseconds.
        };

        /*!
         * \struct SendBlockParams
         * \brief Send Block command parameters
         * \internal
         *
         * Intended for SBIG internal use only. Unimplemented.
         */
        public struct SendBlockParams
        {
            public UInt16 port;    //!< Destination port.
            public UInt16 length;  //!< Length of data buffer.
            //TODO: unsigned char* 檢查移植是否正確
            public UIntPtr source;  //!< Buffer of data to send.
        };

        /*!
         * \struct SendByteParams
         * \brief Send Byte command parameters
         * \internal
         *
         * Intended for SBIG internal use only. Unimplemented.
         */
        public struct SendByteParams
        {
            public UInt16 port;    //!< Destination port.
            public UInt16 data;    //!< Buffer of data to send.
        };

        /*!
         * \struct ClockADParams
         * \brief Clock A/D command parameters
         * \internal
         *
         * Intended for SBIG internal use only. Clock the AD the number of times passed.
         */
        public struct ClockADParams
        {
            public UInt16 ccd;         //!< CCD to clock. see also: CCD_REQUEST enum. (Unused)
            public UInt16 readoutMode; //!< Readout mode. see also: READOUT_BINNING_MODE enum. (Unused)
            public UInt16 pixelStart;  //!< Starting pixel. (Unused)
            public UInt16 pixelLength; //!< Count of cycles to pass.
        };

        /*!
         * \struct SystemTestParams
         * \brief System Test command parameters
         * \internal
         *
         * Intended for SBIG internal use only. Pass the SystemTest command to the micro.
         */
        public struct SystemTestParams
        {
            public UInt16 testClocks;      //!< Flag TRUE to test the clocks.
            public UInt16 testMotor;       //!< Flag TRUE to test the motors.
            public UInt16 test5800;        //!< Flag TRUE to test 5800 (???).
            public UInt16 stlAlign;        //!< Flag true to align STL (???).
            public UInt16 motorAlwaysOn;   //!< Flag true for motor always on (???).
        };

        /*!
         * \struct SendSTVBlockParams
         * \brief Send STV Block command parameters
         * \internal
         *
         * Intended for SBIG internal use only. Unused.
         */
        public struct SendSTVBlockParams
        {
            public UInt16 outLength;   //!< Outgoing buffer length.
            //TODO: unsigned char* 檢查移植是否正確
            public UIntPtr outPtr;      //!< Outgoing buffer.
            public UInt16 inLength;    //!< Incoming buffer length.
            //TODO: unsigned char* 檢查移植是否正確
            public UIntPtr inPtr;       //!< Incoming buffer.
        };

        /*!
         * \struct GetErrorStringParams
         * \brief Get Error String command parameters
         *
         * This command returns a null terminated C string in English (not Unicode) corresponding to the passed
         * error number. It handy for reporting driver level errors to the user.
         */
        public struct GetErrorStringParams
        {
            public UInt16 errorNo; //!< Error code. see also: PAR_ERROR enum.
        };

        /*!
         * \struct GetErrorStringResults
         * \brief Get Error String command results
         *
         * This command returns a null terminated C string in English (not Unicode) corresponding to the passed
         * error number. It handy for reporting driver level errors to the user.
         */
        public struct GetErrorStringResults
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string errorString;   //!< Error string in english (not unicode).
        };

        /*!
         * \struct SetDriverHandleParams
         * \brief Set Driver Handle command parameters
         *
         * The Get/Set Driver Handle commands are for use by applications that wish to talk to multiple cameras
         * on various ports at the same time. If your software only wants to talk to one camera at a time you can
         * ignore these commands.
         *
         * The Get Driver Handle command takes a NULL Parameters pointer and a pointer to a
         * GetDriverHandleResults struct for Results. The Set Driver Handle command takes a pointer to a
         * SetDriverHandleParams struct for Parameters and a NULL pointer for Results. To establish links to
         * multiple cameras do the following sequence:
         * 
         * * Call Open Driver for Camera 1
         * * Call Open Device for Camera 1
         * * Call Establish Link for Camera 1
         * * Call Get Driver Handle and save the result as Handle1
         * * Call Set Driver Handle with INVALID_HANDLE_VALUE in the handle parameter
         * * Call Open Driver for Camera 2
         * * Call Open Device for Camera 2
         * * Call Establish Link for Camera 2
         * * Call Get Driver Handle and save the result as Handle2
         *
         * Then, when you want to talk to Camera 1, call Set Driver Handle with Handle1 and when you want to
         * talk to Camera 2, call Set Driver Handle with Handle2. To shut down you must call Set Driver Handle,
         * Close Device and Close Driver in that sequence for each camera.
         * 
         * Each time you call Set Driver Handle with INVALID_HANDLE_VALUE you are allowing access to an
         * additional camera up to a maximum of four cameras. These cameras can be on different LPT ports,
         * multiple USB4 cameras or at different Ethernet addresses. There is a restriction though due to memory
         * considerations. You can only have a single readout in process at a time for all cameras and CCDs within
         * a camera. Readout begins with the Start Readout or Readout Line commands and ends with the End
         * Readout command. If you try to do multiple interleaved readouts the data from the multiple cameras
         * will be commingled. To avoid this, simply readout one camera/CCD at a time in an atomic process.
         */
        public struct SetDriverHandleParams
        {
            public Int16 handle;   //!< Handle to driver.
        };

        /*!
         * \struct GetDriverHandleResults
         * \brief Get Driver Handle command results
         *
         * The Get/Set Driver Handle commands are for use by applications that wish to talk to multiple cameras
         * on various ports at the same time. If your software only wants to talk to one camera at a time you can
         * ignore these commands.
         *
         * The Get Driver Handle command takes a NULL Parameters pointer and a pointer to a
         * GetDriverHandleResults struct for Results. The Set Driver Handle command takes a pointer to a
         * SetDriverHandleParams struct for Parameters and a NULL pointer for Results. To establish links to
         * multiple cameras do the following sequence:
         * 
         * * Call Open Driver for Camera 1
         * * Call Open Device for Camera 1
         * * Call Establish Link for Camera 1
         * * Call Get Driver Handle and save the result as Handle1
         * * Call Set Driver Handle with INVALID_HANDLE_VALUE in the handle parameter
         * * Call Open Driver for Camera 2
         * * Call Open Device for Camera 2
         * * Call Establish Link for Camera 2
         * * Call Get Driver Handle and save the result as Handle2
         *
         * Then, when you want to talk to Camera 1, call Set Driver Handle with Handle1 and when you want to
         * talk to Camera 2, call Set Driver Handle with Handle2. To shut down you must call Set Driver Handle,
         * Close Device and Close Driver in that sequence for each camera.
         * 
         * Each time you call Set Driver Handle with INVALID_HANDLE_VALUE you are allowing access to an
         * additional camera up to a maximum of four cameras. These cameras can be on different LPT ports,
         * multiple USB4 cameras or at different Ethernet addresses. There is a restriction though due to memory
         * considerations. You can only have a single readout in process at a time for all cameras and CCDs within
         * a camera. Readout begins with the Start Readout or Readout Line commands and ends with the End
         * Readout command. If you try to do multiple interleaved readouts the data from the multiple cameras
         * will be commingled. To avoid this, simply readout one camera/CCD at a time in an atomic process.
         */
        public struct GetDriverHandleResults
        {
            public Int16 handle;   //!< Handle to driver.
        };

        /*!
         * \struct SetDriverControlParams
         * \brief Set Driver Control command parameters
         *
         * This command is used to modify the behavior of the driver by changing the settings of one of the driver control parameters. 
         * Driver options can be enabled or disabled with this command. There is one set of parameters for the whole DLL vs. one per handle.
         *
         * * The DCP_USB_FIFO_ENABLE parameter defaults to TRUE and can be set FALSE to disable
         *   the FIFO and associated pipelining in the USB cameras. You would do this for example in
         *   applications using Time Delay Integration (TDI) where you don't want data in the CCD digitized
         *   until the actual call to ReadoutLine is made.
         *   
         * * The DCP_CALL_JOURNAL_ENABLE parameter defaults to FALSE and can be set to TRUE
         *   to have the driver broadcast Driver API calls. These broadcasts are handy as a debug tool for
         *   monitoring the sequence of API calls made to the driver. The broadcasts can be received and
         *   displayed with the Windows based SBIGUDRVJournalRx.exe application.
         *   Only use this for testing purposes and do not enabled this feature in your released version of you
         *   application as the journaling mechanism can introduce minor artifacts in the readout.
         *   
         * * The DCP_IVTOH_RATIO parameter sets the number of Vertical Rows that are dumped (fast)
         *   before the Horizontal Register is dumped (not as fast) in the DumpRows command for Parallel
         *   Port based cameras. This is a very specialized parameter and you should think hard about
         *   changing it if you do. The default of 5 for the IHTOV_RATIO has been determined to offer a
         *   good compromise between the time it takes to clear the CCD or Dump Rows and the ability to
         *   effectively clear the CCD after imaging a bright object. Finally should you find it necessary to
         *   change it read the current setting and restore it when you're done.
         *   
         * * The DCP_USB_FIFO_SIZE parameter sets the size of the FIFO used to receive data from USB
         *   cameras. The default and maximum value of 16384 yields the highest download speeds.
         *   Lowering the value will cause the camera to digitize and download pixels in smaller chunks.
         *   Again this is a specialized parameter that 99.9% of programs out there will have no need for
         *   changing.
         *   
         * * The DCP_USB_PIXEL_DL_ENABLE parameter allows disabling the actual downloading of
         *   pixel data from the camera for testing purposes. This parameter defaults to TRUE.
         *   
         * * The DCP_HIGH_THROUGHPUT parameter allows configuring the driver for the highest
         *   possible imaging throughput at the expense of image noise and or artifacts. This parameter
         *   defaults to FALSE and you should only enable this for short periods of time. You might use this
         *   in Focus mode for example to get higher image throughput but you should never use it when you
         *   are taking keeper images. It does things that avoid timed delays in the camera like leaving the
         *   shutter motor on all the time, etc. At this time this feature is supported in the driver but not all
         *   cameras show a benefit from its use.
         *   
         * * The DCP_VDD_OPTIMIZED parameter defaults to TRUE which lowers the CCD's Vdd (which
         *   reduces amplifier glow) only for images 3 seconds and longer. This was done to increase the
         *   image throughput for short exposures as raising and lowering Vdd takes 100s of milliseconds.
         *   The lowering and subsequent raising of Vdd delays the image readout slightly which causes short
         *   exposures to have a different bias structure than long exposures. Setting this parameter to
         *   FALSE stops the short exposure optimization from occurring.
         *   
         * * The DCP_AUTO_AD_GAIN parameter defaults to TRUE whereby the driver is responsible for
         *   setting the A/D gain in USB cameras. Setting this to FALSE allows overriding the driver
         *   imposed A/D gains.
         *   
         * * The DCP_NO_HCLKS_FOR_INTEGRATION parameter defaults to FALSE and setting it to
         *   TRUE disables the horizontal clocks during exposure integration and is intended for SBIG
         *   testing only.
         *   
         * * The DCP_TDI_MODE_ENABLE parameter defaults to FALSE and setting it to TRUE enables
         *   the special Biorad TDI mode.
         *   
         * * The DCP_VERT_FLUSH_CONTROL_ENABLE parameter defaults to TRUE and setting it to
         *   FALSE it disables the background flushing of the vertical clocks of KAI CCDs during exposure
         *   integration and is intended for SBIG testing only.
         *
         * * The DCP_ETHERNET_PIPELINE_ENABLE parameter defaults to FALSE and setting it to
         *   TRUE can increase the throughput of Ethernet based cameras like the STX & STT but doing so
         *   is not recommended for robust operation.
         * 
         * * The DCP_FAST_LINK parameter defaults to FALSE and setting it to TRUE speeds up the
         *   Establish Link command by not dumping the pixel FIFOs in the camera, It is used internally to
         *   speed up the Query USB and Query Ethernet commands.
         * 
         * * The DCP_COLUMN_REPAIR_ENABLE defaults to FALSE and setting it to TRUE causes the
         *   Universal Driver Library to repair up to 7 columns in the Imaging CCD automatically. This is
         *   done in conjunction with column data stored in nonvolatile memory in the cameras. Under
         *   Windows the setting of this parameter persists in the Registry through the setting of the
         *   HKEY_CURRENT_USER\Software\SBIG\SBIGUDRV\Filter\ColumnRepairEnable setting.
         * 
         * * The DCP_WARM_PIXEL_REPAIR_ENABLE defaults to Zero and setting it to 1 through 8
         *   causes the Universal Driver Library to repair warm pixels in the Imaging CCD automatically. A
         *   setting of 8 replaces approximately 5% of pixels and a setting of 1 replaces approximately 1 in a
         *   million. A decrease of 1 in the setting replaces approximately 1/10th the number of pixels of the
         *   higher setting (7 ~ 0.5%, 6 ~ 0.05%, etc). Under Windows the setting of this parameter persists
         *   in the Registry through the setting of the
         *   HKEY_CURRENT_USER\Software\SBIG\SBIGUDRV\Filter\WarmPixelRepairEnable setting.
         * 
         * * The DCP_WARM_PIXEL_REPAIR_COUNT parameter returns the total number of pixels
         *   replaced in the last image by the Warm Pixel Repair routine described above. You can use this
         *   parameter to tweak the DCP_WARM_PIXEL_REPAIR_ENABLE parameter to filter as many
         *   warm pixels as your application requires.
         */
        public struct SetDriverControlParams
        {
            public UInt16 controlParameter;    //!< the parameter to modify. see also: DRIVER_CONTROL_PARAM enum.
            public UInt32 controlValue;         //!< the value of the control parameter.
        };

        /*!
         * \struct GetDriverControlParams
         * \brief Get Driver Control command parameters
         *
         * Requests the value of a driver control parameter.
         */
        public struct GetDriverControlParams
        {
            public UInt16 controlParameter;    //!< the driver parameter to be retrieved. see also: DRIVER_CONTROL_PARAM enum.
        };

        /*!
         * \struct GetDriverControlResults
         * \brief Get Driver Control command results
         *
         * Returns the value of a driver control parameter.
         */
        public struct GetDriverControlResults
        {
            public UInt32 controlValue; //!< The value of the requested driver parameter. see also: DRIVER_CONTROL_PARAM enum.
        };

        /*!
         * \struct USBADControlParams
         * \brief USB AD Control command parameters
         *
         * This command is used to modify the USB cameras A/D gain and offset registers.
         * This command is intended for OEM use only. The typical application does not need to use this
         * command as the USB cameras initialize the A/D to factory set defaults when the camera powers
         * up.
         * 
         * * For the USB_AD_IMAGING_GAIN and AD_USB_TRACKING_GAIN commands the allowed
         *   setting for the data parameter is 0 through 63. The actual Gain of the A/D (in Volts/Volt) ranges
         *   from 1.0 to 6.0 and is determined by the following formula:
         *   Gain = 6.0 / ( 1.0 + 5.0 * ( (63 - data) / 63 )
         *   Note that the default A/D Gain set by the camera at power up is 1.2 for the Imaging CCD and 2.0
         *   for the Tracking CCD. Furthermore, the gain item reported by the Get CCD Info command will
         *   always report the default factory-set gain and will not change based upon changes made to the
         *   A/D gain by this command.
         * * For the USB_AD_IMAGING_OFFSET and USB_AD_TRACKING_OFFSET commands the
         *   allowed setting for the data parameter is -255 through 255. Positive offsets increase the video
         *   black level in ADUs. The cameras are programmed at the factory to typically have a 900 to 1000
         *   ADU black level offset.
         */
        public struct USBADControlParams
        {
            public UInt16 command; //!< Imaging/Tracking Gain or offset. see also: USB_AD_CONTROL_COMMAND enum.
            public Int16 data; //!< Command specific.
        };

        /*!
         * \struct QUERY_USB_INFO
         * \brief Information from queried USB device.
         * 
         * Results for a single USB query.
         */
        public struct QUERY_USB_INFO
        {
            public MY_LOGICAL cameraFound;     //!< TRUE if a camera was found.
            public UInt16 cameraType;          //!< Camera type found. see also: CAMERA_TYPE enum.
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string name;          //!< null terminated string. Name of found camera.
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            public string serialNumber;  //!< null terminated string. Serial number of found camera.
        };

        /*!
         * \struct QueryUSBResults
         * \brief Query USB command results 
         * 
         * Returns a list of up to four cameras found by the driver via USB.
         */
        public struct QueryUSBResults
        {
            public UInt16 camerasFound;    //!< Number of cameras found. (Max 4)
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 4)]
            public QUERY_USB_INFO[] usbInfo;      //!< Information returned by cameras.
        };

        /*!
         * \struct QueryUSBResults2
         * \brief Query USB command results extended
         * 
         * Returns a list of up to eight cameras found by the driver via USB.
         */
        public struct QueryUSBResults2
        {
            public UInt16 camerasFound;    //!< Number of cameras found. (Max 8)
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 8)]
            public QUERY_USB_INFO[] usbInfo;      //!< Information returned by cameras.
        };

        public struct QueryUSBResults3
        {
            public UInt16 camerasFound;    //!< Number of cameras found. (Max 24)
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 24)]
            public QUERY_USB_INFO[] usbInfo;     //<! Information returned by cameras.
        };

        /*!
         * \struct QUERY_ETHERNET_INFO
         * \brief Query Ethernet device results
         * 
         * Returned information for a single device over Ethernet.
         */
        public struct QUERY_ETHERNET_INFO
        {
            public MY_LOGICAL cameraFound;     //!< TRUE if a camera was found.
            public UInt32 ipAddress;            //!< IP address of camera found.
            public UInt16 cameraType;          //!< Camera type found. see also: CAMERA_TYPE enum.
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string name;          //!< null terminated string. Name of found camera.
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            public string serialNumber;  //!< null terminated string. Serial number of found camera. 
        };

        /*!
         * \struct QueryEthernetResults
         * \brief Query Ethernet command results
         * 
         * Returns a list of up to eight cameras found by the driver via Ethernet.
         */
        public struct QueryEthernetResults
        {
            public UInt16 camerasFound;        //!< Number of cameras found.
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 4)]
            public QUERY_ETHERNET_INFO[] ethernetInfo;    //!< Information of found devices.
        };

        /*!
         * \struct QueryEthernetResults2
         * \brief Query Ethernet command results extended
         * 
         * Returns a list of up to eight cameras found by the driver via Ethernet.
         */
        public struct QueryEthernetResults2
        {
            public UInt16 camerasFound;        //!< Number of cameras found.
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 8)]
            public QUERY_ETHERNET_INFO[] ethernetInfo;    //!< Information of found devices.
        };

        /*!
         * \struct GetPentiumCycleCountParams
         * \brief Get Pentium Cycle Count command parameters
         * 
         * This command is used to read a Pentium processor internal cycle counter. Pentium processors have a
         * 32 or 64 bit register that increments every clock cycle. For example on a 1 GHz Pentium the counter
         * advances 1 billion counts per second. This command can be used to retrieve that counter.
         */
        public struct GetPentiumCycleCountParams
        {
            public UInt16 rightShift;  //!< number of bits to shift the results to the right (dividing by 2)
        };

        /*!
         * \struct GetPentiumCycleCountResults
         * \brief Get Pentium Cycle Count command results
         * 
         * This command is used to read a Pentium processor internal cycle counter. Pentium processors have a
         * 32 or 64 bit register that increments every clock cycle. For example on a 1 GHz Pentium the counter
         * advances 1 billion counts per second. This command can be used to retrieve that counter.
         */
        public struct GetPentiumCycleCountResults
        {
            public UInt32 countLow;     //!< lower 32 bits of the Pentium cycle counter
            public UInt32 countHigh;    //!< upper 32 bits of the Pentium cycle counter
        };

        /*!
         * \struct RWUSBI2CParams
         * \brief RW USB I2C Count command parameters
         * 
         * This command is used read or write data to the USB cameras I2C expansion port. It writes the 
         * supplied data to the I2C port, or reads data from the supplied address.
         * 
         * This command is typically called by SBIG code in the Universal Driver. If you think you have
         * some reason to call this function you should check with SBIG first.
         */
        public struct RWUSBI2CParams
        {
            public Byte address;      //!< Address to read from or write to
            public Byte data;         //!< Data to write to the external I2C device, ignored for read
            public MY_LOGICAL write;           //!< TRUE when write is desired , FALSE when read is desired
            public Byte deviceAddress;    //!< Device Address of the I2C peripheral
        };

        /*!
         * \struct RWUSBI2CResults
         * \brief RW USB I2C Count command results
         * 
         * This command is used read or write data to the USB cameras I2C expansion port. It returns
         * the result of the read request.
         *
         * This command is typically called by SBIG code in the Universal Driver. If you think you have
         * some reason to call this function you should check with SBIG first.
         */
        public struct RWUSBI2CResults
        {
            public Byte data; //!< Data read from the external I2C device
        };

        /*!
         * \struct CFWParams
         * \brief CFW command parameters
         * 
         * The CFW Command is a high-level API for controlling the SBIG color filter wheels. It supports the
         * CFW-2 (two position shutter wheel in the ST-5C/237), the CFW-5 (internal color filter wheel for the
         * ST-5C/237), the CFW-8, the internal filter wheel (CFW-L) in the ST-L Large Format Camera, the
         * internal filter wheel (CFW-402) in the ST-402 camera, the old 6-position CFW-6A, the 10-position
         * CFW-10 in both I2C and RS-232 interface modes, the I2C based CFW-9 and 8-position CFW for the
         * STL (CFW-L8), the five (FW5-STX) and seven (FW7-STX) position CFWs for the STX, the five
         * (FW5-8300) and eight (FW8-8300) position CFWs for the ST-8300 and the eight (FW8-STT) position
         * CFW for the STT cameras.

         * * CFW Command CFWC_QUERY
         *   Use this command to monitor the progress of the Goto sub-command. This command takes no
         *   additional parameters in the CFParams. You would typically do this several times a second after
         *   the issuing the Goto command until it reports CFWS_IDLE in the cfwStatus entry of the
         *   CFWResults. Additionally filter wheels that can report their current position (all filter wheels
         *   except the CFW-6A or CFW-8) have that position reported in cfwPosition entry of the
         *   CFWResults.
         * * CFW Command CFWC_GOTO
         *   Use this command to start moving the color filter wheel towards a given position. Set the desired
         *   position in the cfwParam1 entry with entries defined by the CFW_POSITION enum.
         *   CFW Command CFWC_INIT
         * * Use this command to initialize/self-calibrate the color filter wheel. All SBIG color filter wheels
         *   self calibrate on power-up and should not require further initialization. We offer this option for
         *   users that experience difficulties with their color filter wheels or when changing between the
         *   CFW-2 and CFW-5 in the ST-5C/237. This command takes no additional parameters in the
         *   CFWParams struct.
         * * CFW Command CFWC_GET_INFO
         *   This command supports several sub-commands as determined by the cfwParam1 entry (see the
         *   CFW_GETINFO_SELECT enum). Command CFWG_FIRMWARE_VERSION returns the
         * * CFWC_OPEN_DEVICE and CFWC_CLOSE_DEVICE:
         *   These commands are used to Open and Close any OS based communications port associated
         *   with the CFW and should proceed the first command sent and follow the last command sent to
         *   the CFW. While strictly only required for the RS-232 version of the CFW-10 calling these
         *   commands is a good idea for future compatibility. For the RS-232 based CFW-10 set the 
         *   cfwParam1 entry to one of the settings CFW_COM_PORT enum to indicate which PC COM port is 
         *   used to control the CFW-10. Again, only the RS232 controlled CFW-10 requires these calls.
         */
        public struct CFWParams
        {
            public UInt16 cfwModel;  //!< see also: CFW_MODEL_SELECT enum.
            public UInt16 cfwCommand;  //!< see also: CFW_COMMAND enum.
            public UInt32 cfwParam1;    //!< command specific
            public UInt32 cfwParam2;    //!< command specific
            public UInt16 outLength;   //!< command specific
            //TODO: unsigned char* 檢查移植是否正確
            public UIntPtr outPtr;  //!< command specific
            public UInt16 inLength;    //!< command specific
            //TODO: unsigned char* 檢查移植是否正確
            public UIntPtr inPtr;       //!< command specific
        };

        /*!
         * \struct CFWResults
         * \brief CFW command results
         * 
         * The CFW Command is a high-level API for controlling the SBIG color filter wheels. It supports the
         * CFW-2 (two position shutter wheel in the ST-5C/237), the CFW-5 (internal color filter wheel for the
         * ST-5C/237), the CFW-8, the internal filter wheel (CFW-L) in the ST-L Large Format Camera, the
         * internal filter wheel (CFW-402) in the ST-402 camera, the old 6-position CFW-6A, the 10-position
         * CFW-10 in both I2C and RS-232 interface modes, the I2C based CFW-9 and 8-position CFW for the
         * STL (CFW-L8), the five (FW5-STX) and seven (FW7-STX) position CFWs for the STX, the five
         * (FW5-8300) and eight (FW8-8300) position CFWs for the ST-8300 and the eight (FW8-STT) position
         * CFW for the STT cameras.

         * * CFW Command CFWC_QUERY
         *   Use this command to monitor the progress of the Goto sub-command. This command takes no
         *   additional parameters in the CFParams. You would typically do this several times a second after
         *   the issuing the Goto command until it reports CFWS_IDLE in the cfwStatus entry of the
         *   CFWResults. Additionally filter wheels that can report their current position (all filter wheels
         *   except the CFW-6A or CFW-8) have that position reported in cfwPosition entry of the
         *   CFWResults.
         * * CFW Command CFWC_GOTO
         *   Use this command to start moving the color filter wheel towards a given position. Set the desired
         *   position in the cfwParam1 entry with entries defined by the CFW_POSITION enum.
         *   CFW Command CFWC_INIT
         * * Use this command to initialize/self-calibrate the color filter wheel. All SBIG color filter wheels
         *   self calibrate on power-up and should not require further initialization. We offer this option for
         *   users that experience difficulties with their color filter wheels or when changing between the
         *   CFW-2 and CFW-5 in the ST-5C/237. This command takes no additional parameters in the
         *   CFWParams struct.
         * * CFW Command CFWC_GET_INFO
         *   This command supports several sub-commands as determined by the cfwParam1 entry (see the
         *   CFW_GETINFO_SELECT enum). Command CFWG_FIRMWARE_VERSION returns the
         * * CFWC_OPEN_DEVICE and CFWC_CLOSE_DEVICE:
         *   These commands are used to Open and Close any OS based communications port associated
         *   with the CFW and should proceed the first command sent and follow the last command sent to
         *   the CFW. While strictly only required for the RS-232 version of the CFW-10 calling these
         *   commands is a good idea for future compatibility. For the RS-232 based CFW-10 set the 
         *   cfwParam1 entry to one of the settings CFW_COM_PORT enum to indicate which PC COM port is 
         *   used to control the CFW-10. Again, only the RS232 controlled CFW-10 requires these calls.
         */
        public struct CFWResults
        {
            public UInt16 cfwModel;    //!< see also: CFW_MODEL_SELECT enum.
            public UInt16 cfwPosition; //!< see also: CFW_POSITION enum.
            public UInt16 cfwStatus;   //!< see also: CFW_STATUS enum.
            public UInt16 cfwError;    //!< see also: CFW_ERROR enum.
            public UInt32 cfwResult1;   //!< command specific
            public UInt32 cfwResult2;   //!< command specific
        };

        /*!
         * \struct BitIOParams
         * \brief Bit IO command parameters
         * 
         * This command is used read or write control bits in the USB cameras.
         *
         * On the ST-L camera you can use this command to monitor whether the input power supply has
         * dropped to the point where you ought to warn the user. Do this by issuing a Read operation on
         * bit 0 and if that bit is set the power has dropped below 10 Volts.
         * 
         * bitName values:
         * * 0=Read Power Supply Low Voltage, 
         * * 1=Write Genl. Purp. Bit 1,
         * * 2=Write Genl. Purp. Bit 2, 
         * * 3=Read Genl. Purp. Bit 3
         */
        public struct BitIOParams
        {
            public UInt16 bitOperation;    //!< 0=Write, 1=Read. see also: BITIO_OPERATION enum.
            public UInt16 bitName;         //!< see also: BITIO_NAME enum.
            public MY_LOGICAL setBit;              //!< 1=Set Bit, 0=Clear Bit
        };

        /*!
         * \struct BitIOResults
         * \brief Bit IO command results
         * 
         * This command is used read or write control bits in the USB cameras.
         *
         * On the ST-L camera you can use this command to monitor whether the input power supply has
         * dropped to the point where you ought to warn the user. Do this by issuing a Read operation on
         * bit 0 and if that bit is set the power has dropped below 10 Volts.
         */
        public struct BitIOResults
        {
            public MY_LOGICAL bitIsSet;    //!< 1=Bit is set, 0=Bit is clear
        };

        /*!
         * \struct UserEEPROMParams
         * \brief User EEPROM command parameters
         * 
         * Read or write a block of data the user space in the EEPROM.
         */

        /*!
         * \struct UserEEPROMResults
         * \brief User EEPROM command results
         * 
         * Read or write a block of data the user space in the EEPROM.
         */
        public struct UserEEPROMResults
        {
            public MY_LOGICAL writeData;   //!< TRUE to write data to user EEPROM space, FALSE to read.
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public Byte[] data;     //!< Buffer of data to be written.
        };

        /*!
         * \struct ColumnEEPROMParams
         * \brief Column EEPROM command parameters and results
         *	\internal
         * 
         * Internal SBIG use only. This command is used read or write the STF-8300 Column Repair data stored in the camera for use
         * with that camera Auto Filter Capability.
         *
         * * The left most column is column 1 (not zero). Specifying a column zero doesn filter any
         *   columns.
         * * This command is somewhat unique in that the Parameters and the Results are the same struct.
         * * To enable column filtering you must use this command and also the Set Driver Control command
         *   to set the DCP_COLUMN_REPAIR parameter to 1.
         */
        public struct ColumnEEPROMResults
        {
            public MY_LOGICAL writeData;   //!< TRUE to write data to specified EEPROM column, FALSE to read.
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public UInt16[] columns;  //!< Specify up to 7 columns to repair.
            public UInt16 flags;       //!< not used at this time.
        };

        /*!
         * \struct BTDISetupParams
         * \brief Biorad TDI Setup command parameters
         * 
         * Send the Biorad setup to the camera, returning any error.
         */
        public struct BTDISetupParams
        {
            public Byte rowPeriod;    //!< Row period.
        };

        /*!
         * \struct BTDISetupResults
         * \brief Biorad TDI Setup command results
         * 
         * Results of the Biorad setup, returning any error.
         */
        public struct BTDISetupResults
        {
            public Byte btdiErrors;   //!< Results of the command. see also: BTDI_ERROR enum.
        };

        /*!
         * \struct MFParams
         * \brief Motor Focus command parameters
         * 
         * The Motor Focus Command is a high-level API for controlling SBIG Motor Focus accessories. It
         * supports the new ST Motor Focus unit and will be expanded as required to support new models in the
         * future.
         * 
         * * Motor Focus Command MFC_QUERY
         *   Use this command to monitor the progress of the Goto sub-command. This command takes no
         *   additional parameters in the MFParams. You would typically do this several times a second after
         *   the issuing the Goto command until it reports MFS_IDLE in the mfStatus entry of the
         *   MFResults. Motor Focus accessories report their current position in the mfPosition entry of the
         *   MFResults struct where the position is a signed long with 0 designating the center of motion or
         *   the home position. Also the Temperature in hundredths of a degree-C is reported in the
         *   mfResult1 entry.
         * * Motor Focus Command MFC_GOTO
         *   Use this command to start moving the Motor Focus accessory towards a given position. Set the
         *   desired position in the mfParam1 entry. Again, the position is a signed long with 0 representing
         *   the center or home position.
         * * Motor Focus Command MFC_INIT
         *   Use this command to initialize/self-calibrate the Motor Focus accessory. This causes the Motor
         *   Focus accessory to find the center or Home position. You can not count on SBIG Motor Focus
         *   accessories to self calibrate upon power-up and should issue this command upon first
         *   establishing a link to the Camera. Additionally you should retain the last position of the Motor
         *   Focus accessory in a parameter file and after initializing the Motor Focus accessory, you should
         *   return it to its last position. Finally, note that this command takes no additional parameters in the
         *   MFParams struct.
         * * Motor Focus Command MFC_GET_INFO
         *   This command supports several sub-commands as determined by the mfParam1 entry (see the
         *   MF_GETINFO_SELECT enum). Command MFG_FIRMWARE_VERSION returns the version
         *   of the Motor Focus firmware in the mfResults1 entry of the MFResults and the Maximum
         *   Extension (plus or minus) that the Motor Focus supports is in the mfResults2 entry. The
         *   MFG_DATA_REGISTERS command is internal SBIG use only and all other commands are
         *   undefined.
         * * Motor Focus Command MFC_ABORT
         *   Use this command to abort a move in progress from a previous Goto command. Note that this
         *   will not abort an Init.
         *
         * Notes: 
         * * The Motor Focus Command takes pointers to MFParams as parameters and MFResults as
         *   results.
         * * Set the mfModel entry in the MFParams to the type of Motor Focus accessory you want to
         *   control. The same value is returned in the mfModel entry of the MFResults. If you select the
         *   MFSEL_AUTO option the driver will use the most appropriate model and return the model it
         *   found in the mfModel entry of the MFResults.
         * * The Motor Focus Command is a single API call that supports multiple sub-commands through
         *   the mfCommand entry in the MFParams. Each of the sub-commands requires certain settings of
         *   the MFParams entries and returns varying results in the MFResults. Each of these
         *   sub-commands is discussed in detail above.
         * * As with all API calls the Motor Focus Command returns an error code. If the error code is
         *   CE_MF_ERROR, then in addition the mfError entry in the MFResults further enumerates the
         *   error.
         */
        public struct MFParams
        {
            public UInt16 mfModel; //!< see also: MF_MODEL_SELECT enum.
            public UInt16 mfCommand;   //!< see also: MF_COMMAND enum.
            public Int32 mfParam1;  //!< command specific.
            public Int32 mfParam2;  //!< command specific.
            public UInt16 outLength;   //!< command specific.
            //TODO: unsigned char* 檢查移植是否正確
            public UIntPtr outPtr;  //!< command specific.
            public UInt16 inLength;    //!< command specific.
            //TODO: unsigned char* 檢查移植是否正確
            public UIntPtr inPtr;       //!< command specific.
        };

        /*!
         * \struct MFResults
         * \brief Motor Focus command results
         * 
         * The Motor Focus Command is a high-level API for controlling SBIG Motor Focus accessories. It
         * supports the new ST Motor Focus unit and will be expanded as required to support new models in the
         * future.
         * 
         * * Motor Focus Command MFC_QUERY
         *   Use this command to monitor the progress of the Goto sub-command. This command takes no
         *   additional parameters in the MFParams. You would typically do this several times a second after
         *   the issuing the Goto command until it reports MFS_IDLE in the mfStatus entry of the
         *   MFResults. Motor Focus accessories report their current position in the mfPosition entry of the
         *   MFResults struct where the position is a signed long with 0 designating the center of motion or
         *   the home position. Also the Temperature in hundredths of a degree-C is reported in the
         *   mfResult1 entry.
         * * Motor Focus Command MFC_GOTO
         *   Use this command to start moving the Motor Focus accessory towards a given position. Set the
         *   desired position in the mfParam1 entry. Again, the position is a signed long with 0 representing
         *   the center or home position.
         * * Motor Focus Command MFC_INIT
         *   Use this command to initialize/self-calibrate the Motor Focus accessory. This causes the Motor
         *   Focus accessory to find the center or Home position. You can not count on SBIG Motor Focus
         *   accessories to self calibrate upon power-up and should issue this command upon first
         *   establishing a link to the Camera. Additionally you should retain the last position of the Motor
         *   Focus accessory in a parameter file and after initializing the Motor Focus accessory, you should
         *   return it to its last position. Finally, note that this command takes no additional parameters in the
         *   MFParams struct.
         * * Motor Focus Command MFC_GET_INFO
         *   This command supports several sub-commands as determined by the mfParam1 entry (see the
         *   MF_GETINFO_SELECT enum). Command MFG_FIRMWARE_VERSION returns the version
         *   of the Motor Focus firmware in the mfResults1 entry of the MFResults and the Maximum
         *   Extension (plus or minus) that the Motor Focus supports is in the mfResults2 entry. The
         *   MFG_DATA_REGISTERS command is internal SBIG use only and all other commands are
         *   undefined.
         * * Motor Focus Command MFC_ABORT
         *   Use this command to abort a move in progress from a previous Goto command. Note that this
         *   will not abort an Init.
         *
         * Notes: 
         * * The Motor Focus Command takes pointers to MFParams as parameters and MFResults as
         *   results.
         * * Set the mfModel entry in the MFParams to the type of Motor Focus accessory you want to
         *   control. The same value is returned in the mfModel entry of the MFResults. If you select the
         *   MFSEL_AUTO option the driver will use the most appropriate model and return the model it
         *   found in the mfModel entry of the MFResults.
         * * The Motor Focus Command is a single API call that supports multiple sub-commands through
         *   the mfCommand entry in the MFParams. Each of the sub-commands requires certain settings of
         *   the MFParams entries and returns varying results in the MFResults. Each of these
         *   sub-commands is discussed in detail above.
         * * As with all API calls the Motor Focus Command returns an error code. If the error code is
         *   CE_MF_ERROR, then in addition the mfError entry in the MFResults further enumerates the
         *   error.
         */
        public struct MFResults
        {
            public UInt16 mfModel; //!< see also: MF_MODEL_SELECT enum. 
            public Int32 mfPosition;   //!< position of the Motor Focus, 0=Center, signed.
            public UInt16 mfStatus;    //!< see also: MF_STATUS enum.
            public UInt16 mfError; //!< see also: MF_ERROR  enum.
            public Int32 mfResult1; //!< command specific.
            public Int32 mfResult2; //!< command specific.
        };

        /*!
         * \struct DiffGuiderParams
         * \brief Differential Guider command parameters
         * 
         * Differential Guider Command Guide:
         * * DGC_DETECT detects whether a Differential Guide unit is connected to the camera.
         *   Command takes no arguments.
         * * DGC_GET_BRIGHTNESS obtains the brightness setting of the red and IR LEDs in the differential guide unit.
         *   inPtr should be a pointer to a DGLEDState struct.
         * * DGC_SET_BRIGHTNESS sets the brightness registers of the red and IR LEDs in the differential guide unit.
         *   outPtr should be a pointer to a DGLEDState struct with the desired values register values set.
         */
        public struct DiffGuiderParams
        {
            public UInt16 diffGuiderCommand;   //!< Command for Differential Guider. see also: DIFF_GUIDER_COMMAND enum. 
            public UInt16 spareShort;          //!< Unused.
            public UInt32 diffGuiderParam1; //!< Unused.
            public UInt32 diffGuiderParam2; //!< Unused.
            public UInt16 outLength;           //!< Size of output buffer. Command specific.
            //TODO: unsigned char* 檢查移植是否正確
            public UIntPtr outPtr;              //!< output buffer. Command specific.
            public UInt16 inLength;            //!< Size of input buffer. Command specific.
            //TODO: unsigned char* 檢查移植是否正確
            public UIntPtr inPtr;               //!< input buffer. Command specific.
        };

        /*!
         * \struct DiffGuiderResults
         * \brief Differential Guider command results
         * 
         * Returned results of a Differential Guider Command.
         */
        public struct DiffGuiderResults
        {
            public UInt16 diffGuiderError; //!< see also: DIFF_GUIDER_ERROR enum.
            public UInt16 diffGuiderStatus;    //!< see also: DIFF_GUIDER_STATUS enum.
            public UInt32 diffGuiderResult1;    //!< Unused.
            public UInt32 diffGuiderResult2;    //!< Unused.
        };

        /*!
         * \struct DGLEDState
         * \brief Differential Guider LED state
         * 
         * State of the Differential Guider LEDs.
         */
        public struct DGLEDState
        {
            public UInt16 bRedEnable;      //!< TRUE if Red LED is on, FALSE otherwise.
            public UInt16 bIREnable;       //!< TRUE if IR LED is on, FALSE otherwise.
            public UInt16 nRedBrightness;  //!< brightness setting of Red LED from 0x00 to 0xFF.
            public UInt16 nIRBrightness;   //!< brightness setting of IR LED from 0x00 to 0xFF.
        };

        /*!
         * \struct BulkIOParams
         * \brief Bulk I/O command parameters
         *	\internal
         * 
         * Internal SBIG use only. Implement the Bulk IO command which is used for Bulk Reads/Writes to the camera for diagnostic purposes.
         */
        public struct BulkIOParams
        {
            public UInt16 command;             //!< see also: BULK_IO_COMMAND enum.
            public MY_LOGICAL isPixelData;     //!< TRUE if reading/writing data to/from the Pixel pipe, FALSE to read/write from the com pipe.
            public UInt32 dataLength;           //!< Length of data buffer.
            //TODO: char* 檢查移植是否正確
            public IntPtr dataPtr;          //!< data buffer.
        };

        /*!
         * \struct BulkIOResults
         * \brief Bulk I/O command results
         *	\internal
         * 
         * Internal SBIG use only. Results of a Bulk I/O command.
         */
        public struct BulkIOResults
        {
            public UInt32 dataLength;   //!< Bytes sent/received.
        };

        /*!
         * \struct CustomerOptionsParams
         * \brief Customer Options command parameters
         * 
         * This command is used read or write the STX/STXL/STT customer options.
         */

        /*!
         * \struct CustomerOptionsResults
         * \brief Customer Options command results
         * 
         * This command is used read or write the STX/STXL/STT customer options.
         */
        public struct CustomerOptionsResults
        {
            public MY_LOGICAL bSetCustomerOptions; //!< TRUE/FALSE = set/get options respectively.
            public MY_LOGICAL bOverscanRegions;    //!< TRUE to include Overscan region in images.
            public MY_LOGICAL bWindowHeater;       //!< TRUE to turn on window heater.
            public MY_LOGICAL bPreflashCcd;        //!< TRUE to preflash CCD.
            public MY_LOGICAL bVddNormallyOff;     //!< TRUE to turn VDD off.
        };

        /*!
         * \struct GetI2CAoModelResults
         * \brief Get I2C AO Model command results
         * 
         * Results of a CC_GET_AO_MODEL command.
         */
        public struct GetI2CAoModelResults
        {
            public UInt16 i2cAoModel;  //!< AO model.
        };

        /*!
         * \enum DEBUG_LOG_CC_FLAGS
         * \brief Debug flags for Command IDs.
         *
         * Flags for enabling debug messages of CC_***_*** commands.
         */
        public enum DEBUG_LOG_CC_FLAGS : ushort
        {
            DLF_CC_BASE = 0x0001,   //!< Log MC_SYSTEM, CC_BREAKPOINT, CC_OPEN_*, CC_CLOSE_*, etc.
            DLF_CC_READOUT = 0x0002,    //!< Log readout commands.
            DLF_CC_STATUS = 0x0004, //!< Log status commands.
            DLF_CC_TEMPERATURE = 0x0008,    //!< Log temperature commands.
            DLF_CC_CFW = 0x0010,    //!< Log filter wheel commands.
            DLF_CC_AO = 0x0020, //!< Log AO commands
            DLF_CC_40 = 0x0040, //!< Unused.
            DLF_CC_80 = 0x0080  //!< Unused.
        };

        /*!
         * \enum DEBUG_LOG_MC_FLAGS
         * \brief Debug flags for Microcommand IDs.
         * 
         * Flags for enabling debug messages of MC_***_*** commands.
         */
        public enum DEBUG_LOG_MC_FLAGS : ushort
        {
            DLF_MC_BASE = 0x0001,   //!< Log MC_START_*, MC_END_*, MC_OPEN_*, MC_CLOSE_*, etc...
            DLF_MC_READOUT = 0x0002,    //!< Log readout commands at microcommand level.
            DLF_MC_STATUS = 0x0004, //!< Log status commands at microcommand level.
            DLF_MC_TEMPERATURE = 0x0008,    //!< Log temperature commands at microcommand level.
            DLF_MC_EEPROM = 0x0010, //!< Log EEPROM microcommands.
            DLF_MC_20 = 0x0020, //!< Unused.
            DLF_MC_40 = 0x0040, //!< Unused.
            DLF_MC_80 = 0x0080  //!< Unused.
        };

        /*!
         * \enum DEBUG_LOG_FCE_FLAGS
         * \brief Debug flags for communications.
         *
         * Flags for enabling debug messages of communication methods.
         */
        public enum DEBUG_LOG_FCE_FLAGS : ushort
        {
            DLF_FCE_ETH = 0x0001,   //!< Log Ethernet communication functions.
            DLF_FCE_USB = 0x0002,   //!< Log USB communication functions.
            DLF_FCE_FIFO = 0x0004,  //!< Log FIFO communication functions.
            DLF_FCE_0008 = 0x0008,  //!< Unused.
            DLF_FCE_0010 = 0x0010,  //!< Unused.
            DLF_FCE_0020 = 0x0020,  //!< Unused.
            DLF_FCE_0040 = 0x0040,  //!< Unused.
            DLF_FCE_CAMERA = 0x0080 //!< Log camera communication responses.
        };

        /*!
         * \enum DEBUG_LOG_IO_FLAGS
         * \brief Debug flags for I/O operations.
         *
         * Flags for enabling debug messages of I/O operations.
         */
        public enum DEBUG_LOG_IO_FLAGS : ushort
        {
            DLF_IO_RD_COM_PIPE = 0x0001,    //!< Log reading from com pipe.
            DLF_IO_WR_COM_PIPE = 0x0002,    //!< Log writing to com pipe.
            DLF_IO_RD_PIXEL_PIPE = 0x0004,  //!< Log reading from pixel pipe.
            DLF_IO_RD_ALT_PIPE = 0x0008,    //!< Log reading from alternate pixel pipe.
            DLF_IO_WR_ALT_PIPE = 0x0010,    //!< Log writing to alternate pixel pipe.
            DLF_IO_RD = 0x0020, //!< Log reading from Async I/O.
            DLF_IO_WR = 0x0040, //!< Log writing to Async I/O.
            DLF_IO_0080 = 0x0080    //!< Unused.
        };

        /*!
         * \struct DebugLogParams
         * \brief Debug log command parameters.
         *
         * Change debug logging, and path to log file.
         */
        public struct DebugLogParams
        {
            public UInt16 ccFlags;     //!< Command flags.
            public UInt16 mcFlags;     //!< Microcommand flags.
            public UInt16 fceFlags;    //!< Communication flags.
            public UInt16 ioFlags;     //!< I/O flags.
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
            public string logFilePathName; //!< Path to SBIGUDRV log file.
        };

        public struct GetReadoutInProgressResults
        {
            public MY_LOGICAL RIP; //!< Readout In Progress. TRUE if RIP, FALSE otherwise.
        };

        public struct SetRBIPreflashParams
        {
            public UInt16 darkFrameLength;
            public UInt16 flushCount;
        };

        public struct GetRBIPreflashResults
        {
            public UInt16 darkFrameLength;
            public UInt16 flushCount;
        };

        /*!
         *	\brief QueryFeatureSupportedParams
         */
        public struct QueryFeatureSupportedParams
        {
            public FeatureFirmwareRequirement ffr; //!< Feature to query for firmware support. 
        };

        /*!
         *	\brief QueryFeatureSupportedResult
         */
        public struct QueryFeatureSupportedResults
        {
            public MY_LOGICAL result;  //!< TRUE if feature is supported, FALSE otherwise.
        };

        /*!
         * \struct QueryExposureTicksResults
         * \brief Query Exposure Ticks command results.
         *
         * Internal SBIG use only. Queries Start/End exposure performance tracking.
         */
        public struct QueryExposureTicksResults
        {
            /// <summary>
            /// Union LARGE_INTEGER
            /// </summary>
            [StructLayout(LayoutKind.Explicit, Pack = 8)]
            public struct LARGE_INTEGER
            {
                [FieldOffset(0)]
                public UInt32 LowPart;
                [FieldOffset(4)]
                public Int32 HighPart;

                [FieldOffset(0)]
                public Int64 QuadPart;
            };

            public LARGE_INTEGER startExposureTicks0;  //!< Start exposure tick initial value.
            public LARGE_INTEGER startExposureTicks1;  //!< Start exposure tick final value.
            public LARGE_INTEGER endExposureTicks0;        //!< End exposure tick initial value.
            public LARGE_INTEGER endExposureTicks1;        //!< End exposure tick final value.
        };

        /*!
         *	SBIGUnivDrvCommand()
         *  \brief Command function: Supports Parallel, USB and Ethernet based cameras
         *	\param command PAR_COMMAND integer 
         *	\param Params pointer to a command-specific structure containing the relevant command parameters.
         *	\param pResults pointer to a comand-specific results structure containing the results of the command.
         *
         *	The master API hook for the SBIG Universal Driver dll. The calling program needs to allocate the memory 
         *  for the parameters and results structs and these routines read them and fill them in respectively.
         */
        [DllImport("SBIGUDrv.dll", CallingConvention = CallingConvention.StdCall)]
        private static extern PAR_ERROR SBIGUnivDrvCommand(
            PAR_COMMAND command, IntPtr Params, IntPtr pResults);

        /*!
         *	SBIGLogDebugMsg()
         *  \brief Command function: Supports Parallel, USB and Ethernet based cameras
         *	\param pStr pointer to an array of characters, null-terminated, which should be written to the log file.
         *	\param length unsigned int of buffer's length in bytes.
         *	\internal
         *	
         *	A function used to expose writing to the log file to calling programs. Useful for debugging purposes.
         */
        //TODO: 檢查移植是否正確 extern "C" Int16 __stdcall SBIGLogDebugMsg(char* pStr, UInt16 length);
        [DllImport("SBIGUDrv.dll", CallingConvention = CallingConvention.StdCall)]
        private static extern Int16 SBIGLogDebugMsg(UIntPtr pStr, UInt16 length);

    } // class
} // namespace
