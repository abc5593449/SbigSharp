using System;
using System.Net;
using System.Runtime.InteropServices;
using System.Text;

namespace SbigSharp
{
    public static class SBIG
    {
        /*!
           * \file SBIGUDRV.H
           * \brief Contains the function prototypes and enumerated constants for the Universal Parallel/USB/Ethernet driver.
           *
           * This supports the following devices:
           *
           * * ST-5C/237/237A (PixCel255/237)
           * * ST-7E/8E/9E/10E
           * * ST-1K, ST-2K, ST-4K
           * * STL Large Format Cameras
           * * ST-402 Family of Cameras
           * * ST-8300 Cameras
           * * STF-8300, 8050 Cameras
           * * STT Cameras
           * * STX/STXL Cameras
           * * ST-i Cameras
           * * AO-7, AOL, AO-8
           * * CFW-8, CFW-9, CFW-10, CFW-L
           * * FW5-8300, FW8-8300
           * * ST Focuser
           * * Differential Guider Accessory (Preliminary)
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
        /*!
         * \ingroup BASE_STRUCTURES
         * Command ID enum 
         */
        public enum PAR_COMMAND : ushort

        {
            /*

                General Use Commands

            */
            /// <summary>
            /// Null Command
            /// </summary>
            CC_NULL,

            /* 1 - 10 */
            /// <summary>
            /// Start exposure command
            /// </summary>
            CC_START_EXPOSURE = 1,
            /// <summary>
            /// End exposure command
            /// </summary>
            CC_END_EXPOSURE,
            /// <summary>
            /// Readout line command
            /// </summary>
            CC_READOUT_LINE,
            /// <summary>
            /// Dump lines command
            /// </summary>
            CC_DUMP_LINES,
            /// <summary>
            /// Set Temperature regulation command
            /// </summary>
            CC_SET_TEMPERATURE_REGULATION,
            /// <summary>
            /// Query temperature status command
            /// </summary>
            CC_QUERY_TEMPERATURE_STATUS,
            /// <summary>
            /// Activate Relay command
            /// </summary>
            CC_ACTIVATE_RELAY,
            /// <summary>
            /// Pulse out command
            /// </summary>
            CC_PULSE_OUT,
            /// <summary>
            /// Establish link command
            /// </summary>
            CC_ESTABLISH_LINK,
            /// <summary>
            /// Get driver info command
            /// </summary>
            CC_GET_DRIVER_INFO,

            /* 11 - 20 */
            /// <summary>
            /// Get CCD info command
            /// </summary>
            CC_GET_CCD_INFO,
            /// <summary>
            /// Query command status command
            /// </summary>
            CC_QUERY_COMMAND_STATUS,
            /// <summary>
            /// Miscellaneous control command
            /// </summary>
            CC_MISCELLANEOUS_CONTROL,
            /// <summary>
            /// Read subtract line command
            /// </summary>
            CC_READ_SUBTRACT_LINE,
            /// <summary>
            /// Update clock command
            /// </summary>
            CC_UPDATE_CLOCK,
            /// <summary>
            /// Read offset command
            /// </summary>
            CC_READ_OFFSET,
            /// <summary>
            /// Open driver command
            /// </summary>
            CC_OPEN_DRIVER,
            /// <summary>
            /// Close driver command
            /// </summary>
            CC_CLOSE_DRIVER,
            /// <summary>
            /// TX Serial bytes command
            /// </summary>
            CC_TX_SERIAL_BYTES,
            /// <summary>
            /// Get serial status command
            /// </summary>
            CC_GET_SERIAL_STATUS,

            /* 21 - 30 */
            /// <summary>
            /// AO tip/tilt command
            /// </summary>
            CC_AO_TIP_TILT,

            /// <summary>
            /// AO set focus command
            /// </summary>
            CC_AO_SET_FOCUS,
            /// <summary>
            /// AO delay command
            /// </summary>
            CC_AO_DELAY,
            /// <summary>
            /// Get turbo status command
            /// </summary>
            CC_GET_TURBO_STATUS,
            /// <summary>
            /// End readout command
            /// </summary>
            CC_END_READOUT,
            /// <summary>
            /// Get US timer command
            /// </summary>
            CC_GET_US_TIMER,
            /// <summary>
            /// Open device command
            /// </summary>
            CC_OPEN_DEVICE,
            /// <summary>
            /// Close device command
            /// </summary>
            CC_CLOSE_DEVICE,
            /// <summary>
            /// Set IRQL command
            /// </summary>
            CC_SET_IRQL,
            /// <summary>
            /// Get IRQL command
            /// </summary>
            CC_GET_IRQL,

            /* 31 - 40 */
            /// <summary>
            /// Get line command
            /// </summary>
            CC_GET_LINE,
            /// <summary>
            /// Get link status command
            /// </summary>
            CC_GET_LINK_STATUS,
            /// <summary>
            /// Get driver handle command
            /// </summary>
            CC_GET_DRIVER_HANDLE,
            /// <summary>
            /// Set driver handle command
            /// </summary>
            CC_SET_DRIVER_HANDLE,
            /// <summary>
            /// Start readout command
            /// </summary>
            CC_START_READOUT,
            /// <summary>
            /// Get error string command
            /// </summary>
            CC_GET_ERROR_STRING,
            /// <summary>
            /// Set driver control command
            /// </summary>
            CC_SET_DRIVER_CONTROL,
            /// <summary>
            /// Get driver control command
            /// </summary>
            CC_GET_DRIVER_CONTROL,
            /// <summary>
            /// USB A/D control command
            /// </summary>
            CC_USB_AD_CONTROL,
            /// <summary>
            /// Query USB command
            /// </summary>
            CC_QUERY_USB,

            /* 41 - 50 */
            /// <summary>
            /// Get Pentium cycle count command
            /// </summary>
            CC_GET_PENTIUM_CYCLE_COUNT,
            /// <summary>
            /// Read/Write USB I2C command
            /// </summary>
            CC_RW_USB_I2C,
            /// <summary>
            /// Control Filter Wheel command
            /// </summary>
            CC_CFW,
            /// <summary>
            /// Bit I/O command
            /// </summary>
            CC_BIT_IO,
            /// <summary>
            /// User EEPROM command
            /// </summary>
            CC_USER_EEPROM,
            /// <summary>
            /// AO Center command
            /// </summary>
            CC_AO_CENTER,
            /// <summary>
            /// BTDI setup command
            /// </summary>
            CC_BTDI_SETUP,
            /// <summary>
            /// Motor focus command
            /// </summary>
            CC_MOTOR_FOCUS,
            /// <summary>
            /// Query Ethernet command
            /// </summary>
            CC_QUERY_ETHERNET,
            /// <summary>
            /// Start Exposure command v2
            /// </summary>
            CC_START_EXPOSURE2,

            /* 51 - 60 */
            /// <summary>
            /// Set Temperature regulation command
            /// </summary>
            CC_SET_TEMPERATURE_REGULATION2,
            /// <summary>
            /// Read offset command v2
            /// </summary>
            CC_READ_OFFSET2,
            /// <summary>
            /// Differential Guider command
            /// </summary>
            CC_DIFF_GUIDER,
            /// <summary>
            /// Column EEPROM command
            /// </summary>
            CC_COLUMN_EEPROM,
            /// <summary>
            /// Customer Options command
            /// </summary>
            CC_CUSTOMER_OPTIONS,
            /// <summary>
            /// Debug log command
            /// </summary>
            CC_DEBUG_LOG,
            /// <summary>
            /// Query USB command v2
            /// </summary>
            CC_QUERY_USB2,
            /// <summary>
            /// Query Ethernet command v2
            /// </summary>
            CC_QUERY_ETHERNET2,
            /// <summary>
            /// Get AO model command
            /// </summary>
            CC_GET_AO_MODEL,
            /// <summary>
            /// Query up to 24 USB cameras
            /// </summary>
            CC_QUERY_USB3,
            /// <summary>
            /// Expanded Query Command Status to include extra information
            /// </summary>
            CC_QUERY_COMMAND_STATUS2,
            /*
                SBIG Use Only Commands
            */

            /* 90 - 99 */
            /// <summary>
            /// Send block command
            /// </summary>
            CC_SEND_BLOCK = 90,
            /// <summary>
            /// Send byte command
            /// </summary>
            CC_SEND_BYTE,
            /// <summary>
            /// Get byte command
            /// </summary>
            CC_GET_BYTE,
            /// <summary>
            /// Send A/D command
            /// </summary>
            CC_SEND_AD,
            /// <summary>
            /// Get A/D command
            /// </summary>
            CC_GET_AD,
            /// <summary>
            /// Clock A/D command
            /// </summary>
            CC_CLOCK_AD,
            /// <summary>
            /// System test command
            /// </summary>
            CC_SYSTEM_TEST,
            /// <summary>
            /// Get driver options command
            /// </summary>
            CC_GET_DRIVER_OPTIONS,
            /// <summary>
            /// Set driver options command
            /// </summary>
            CC_SET_DRIVER_OPTIONS,
            /// <summary>
            /// Firmware command
            /// </summary>
            CC_FIRMWARE,

            /* 100 -109 */
            /// <summary>
            /// Bulk I/O command
            /// </summary>
            CC_BULK_IO,
            /// <summary>
            /// Ripple correction command
            /// </summary>
            CC_RIPPLE_CORRECTION,
            /// <summary>
            /// EZUSB Reset command
            /// </summary>
            CC_EZUSB_RESET,
            /// <summary>
            /// Breakpoint command
            /// </summary>
            CC_BREAKPOINT,
            /// <summary>
            /// Query exposure ticks command
            /// </summary>
            CC_QUERY_EXPOSURE_TICKS,
            /// <summary>
            /// Set active CCD area command
            /// </summary>
            CC_SET_ACTIVE_CCD_AREA,
            /// <summary>
            /// Returns TRUE if a readout is in progress on any driver handle
            /// </summary>
            CC_READOUT_IN_PROGRESS,
            /// <summary>
            /// Updates the RBI Preflash parameters
            /// </summary>
            CC_GET_RBI_PARAMETERS,
            /// <summary>
            /// Obtains the RBI Preflash parameters from the camera
            /// </summary>
            CC_SET_RBI_PARAMETERS,
            /// <summary>
            /// Checks to see if a camera's firmware supports a command.
            /// </summary>
            CC_QUERY_FEATURE_SUPPORTED,
            /// <summary>
            /// Last command ID
            /// </summary>
            CC_LAST_COMMAND

            /* 110 - 119 */

        };

        /*

            Return Error Codes

            These are the error codes returned by the driver
            function.  They are prefixed with CE_ to designate
            them as camera errors.

        */
        /*!
         * Base value for all error IDs.
         */
        const ushort CE_ERROR_BASE = 1;

        /*!
         * \ingroup BASE_STRUCTURES
         * Error ID enum 
         */
        public enum PAR_ERROR : ushort
        {
            /* 0 - 10 */
            /// <summary>
            /// No error ID
            /// </summary>
            CE_NO_ERROR,
            /// <summary>
            /// Camera not found error
            /// </summary>
            CE_CAMERA_NOT_FOUND = CE_ERROR_BASE,
            /// <summary>
            /// Exposure in progress error
            /// </summary>
            CE_EXPOSURE_IN_PROGRESS,
            /// <summary>
            /// No exposure in progress error
            /// </summary>
            CE_NO_EXPOSURE_IN_PROGRESS,
            /// <summary>
            /// Unknown command error
            /// </summary>
            CE_UNKNOWN_COMMAND,
            /// <summary>
            /// Bad camera command error
            /// </summary>
            CE_BAD_CAMERA_COMMAND,
            /// <summary>
            /// Bad parameter command
            /// </summary>
            CE_BAD_PARAMETER,
            /// <summary>
            /// Transfer (Tx) timeout error
            /// </summary>
            CE_TX_TIMEOUT,
            /// <summary>
            /// Receive (Rx) timeout error
            /// </summary>
            CE_RX_TIMEOUT,
            /// <summary>
            /// Received Negative Acknowledgement
            /// </summary>
            CE_NAK_RECEIVED,
            /// <summary>
            /// Received Cancel
            /// </summary>
            CE_CAN_RECEIVED,

            /* 11 - 20 */
            /// <summary>
            /// Unknown response error
            /// </summary>
            CE_UNKNOWN_RESPONSE,
            /// <summary>
            /// Bad length error
            /// </summary>
            CE_BAD_LENGTH,
            /// <summary>
            /// A/D timeout error
            /// </summary>
            CE_AD_TIMEOUT,
            /// <summary>
            /// Keyboard error
            /// </summary>
            CE_KBD_ESC,
            /// <summary>
            /// Checksum error
            /// </summary>
            CE_CHECKSUM_ERROR,
            /// <summary>
            /// EEPROM error
            /// </summary>
            CE_EEPROM_ERROR,
            /// <summary>
            /// Shutter error
            /// </summary>
            CE_SHUTTER_ERROR,
            /// <summary>
            /// Unknown camera error
            /// </summary>
            CE_UNKNOWN_CAMERA,
            /// <summary>
            /// Driver not found error
            /// </summary>
            CE_DRIVER_NOT_FOUND,
            /// <summary>
            /// Driver not open error
            /// </summary>
            CE_DRIVER_NOT_OPEN,

            /* 21 - 30 */
            /// <summary>
            /// Driver not closed error
            /// </summary>
            CE_DRIVER_NOT_CLOSED,
            /// <summary>
            /// Share error
            /// </summary>
            CE_SHARE_ERROR,
            /// <summary>
            /// TCE not found error
            /// </summary>
            CE_TCE_NOT_FOUND,
            /// <summary>
            /// AO error
            /// </summary>
            CE_AO_ERROR,
            /// <summary>
            /// ECP error
            /// </summary>
            CE_ECP_ERROR,
            /// <summary>
            /// Memory error
            /// </summary>
            CE_MEMORY_ERROR,
            /// <summary>
            /// Device not found error
            /// </summary>
            CE_DEVICE_NOT_FOUND,
            /// <summary>
            /// Device not open error
            /// </summary>
            CE_DEVICE_NOT_OPEN,
            /// <summary>
            /// Device not closed error
            /// </summary>
            CE_DEVICE_NOT_CLOSED,
            /// <summary>
            /// Device not implemented error
            /// </summary>
            CE_DEVICE_NOT_IMPLEMENTED,

            /* 31 - 40 */
            /// <summary>
            /// Device disabled error
            /// </summary>
            CE_DEVICE_DISABLED,
            /// <summary>
            /// OS error
            /// </summary>
            CE_OS_ERROR,
            /// <summary>
            /// Socket error
            /// </summary>
            CE_SOCK_ERROR,
            /// <summary>
            /// Server not found error
            /// </summary>
            CE_SERVER_NOT_FOUND,
            /// <summary>
            /// Filter wheel error
            /// </summary>
            CE_CFW_ERROR,
            /// <summary>
            /// Motor Focus error
            /// </summary>
            CE_MF_ERROR,
            /// <summary>
            /// Firmware error
            /// </summary>
            CE_FIRMWARE_ERROR,
            /// <summary>
            /// Differential guider error
            /// </summary>
            CE_DIFF_GUIDER_ERROR,
            /// <summary>
            /// Ripple corrections error
            /// </summary>
            CE_RIPPLE_CORRECTION_ERROR,
            /// <summary>
            /// EZUSB Reset error
            /// </summary>
            CE_EZUSB_RESET,

            /* 41 - 50*/
            /// <summary>
            /// Firmware needs update to support feature.
            /// </summary>
            CE_INCOMPATIBLE_FIRMWARE,
            /// <summary>
            /// An invalid R/W handle was supplied for I/O
            /// </summary>
            CE_INVALID_HANDLE,
            /// <summary>
            /// Development purposes: Next Error
            /// </summary>
            CE_NEXT_ERROR

        };

        /*
            Camera Command State Codes

            These are the return status codes for the Query
            Command Status command.  They are prefixed with
            CS_ to designate them as camera status.

        */
        /*!
         * \ingroup BASE_STRUCTURES
         * Camera states enum 
         */
        public enum PAR_COMMAND_STATUS : ushort
        {
            /// <summary>
            /// Camera state: Idle.
            /// </summary>
            CS_IDLE,
            /// <summary>
            /// Camera state: Exposure in progress
            /// </summary>
            CS_IN_PROGRESS,
            /// <summary>
            /// Camera state: Integrating
            /// </summary>
            CS_INTEGRATING,
            /// <summary>
            /// Camera state: Integration complete
            /// </summary>
            CS_INTEGRATION_COMPLETE
        };

        public enum FeatureFirmwareRequirement : ushort
        {
            FFR_CTRL_OFFSET_CORRECTION,
            FFR_CTRL_EXT_SHUTTER_ONLY,
            FFR_ASYNC_TRIGGER_IN,
            FFR_LAST
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * Pulse in is currently active state modifier flag.
         */
        const UInt16 CS_PULSE_IN_ACTIVE = 0x8000;

        /*!
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
            DIFF_GUIDER_COMMAND, DIFF_GUIDER_STATE, DIFF_GUIDER_ERROR - Used with the Diff Guider Command
        */

        /*!
         * \ingroup BASE_STRUCTURES
         * Query Temperature Status enum 
         */
        public enum QUERY_TEMP_STATUS_REQUEST : ushort
        {
            /// <summary>
            /// Temperature status Standard
            /// </summary>
            TEMP_STATUS_STANDARD,
            /// <summary>
            /// Temperature status Advanced
            /// </summary>
            TEMP_STATUS_ADVANCED,
            /// <summary>
            /// Temperature status Advanced 2
            /// </summary>
            TEMP_STATUS_ADVANCED2
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * ABG state enum 
         */
        public enum ABG_STATE7 : ushort
        {
            /// <summary>
            /// ABG Low 7
            /// </summary>
            ABG_LOW7,
            /// <summary>
            /// ABG Clock Low 7
            /// </summary>
            ABG_CLK_LOW7,
            /// <summary>
            /// ABG Clock Medium 7
            /// </summary>
            ABG_CLK_MED7,
            /// <summary>
            /// ABG Clock High 7
            /// </summary>
            ABG_CLK_HI7
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * Boolean type definition 
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
         * \ingroup BASE_STRUCTURES
         * Driver request enum 
         */
        public enum DRIVER_REQUEST : ushort
        {
            /// <summary>
            /// Driver standard
            /// </summary>
            DRIVER_STD,
            /// <summary>
            /// Driver extended
            /// </summary>
            DRIVER_EXTENDED,
            /// <summary>
            /// Driver USB loader
            /// </summary>
            DRIVER_USB_LOADER
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * CCD Request enum 
         */
        public enum CCD_REQUEST : ushort
        {
            /// <summary>
            /// Request Imaging CCD
            /// </summary>
            CCD_IMAGING,
            /// <summary>
            /// Request Internal Tracking CCD
            /// </summary>
            CCD_TRACKING,
            /// <summary>
            /// Request External Tracking CCD
            /// </summary>
            CCD_EXT_TRACKING
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * Readout Modes enum 
         */
        public enum READOUT_BINNING_MODE : ushort
        {
            /// <summary>
            /// 1x1 binning readout mode
            /// </summary>
            RM_1X1,
            /// <summary>
            /// 2x2 binning readout mode
            /// </summary>
            RM_2X2,
            /// <summary>
            /// 3x3 binning readout mode
            /// </summary>
            RM_3X3,
            /// <summary>
            /// Nx1 binning readout mode
            /// </summary>
            RM_NX1,
            /// <summary>
            /// Nx2 binning readout mode
            /// </summary>
            RM_NX2,
            /// <summary>
            /// Nx3 binning readout mode
            /// </summary>
            RM_NX3,
            /// <summary>
            /// 1x1 Off-chip binning readout mode
            /// </summary>
            RM_1X1_VOFFCHIP,
            /// <summary>
            /// 2x2 Off-chip binning readout mode
            /// </summary>
            RM_2X2_VOFFCHIP,
            /// <summary>
            /// 3x3 Off-chip binning readout mode
            /// </summary>
            RM_3X3_VOFFCHIP,
            /// <summary>
            /// 9x9 binning readout mode
            /// </summary>
            RM_9X9,
            /// <summary>
            /// NxN binning readout mode
            /// </summary>
            RM_NXN
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * CCD Information request enum 
         */
        public enum CCD_INFO_REQUEST : ushort
        {
            /// <summary>
            /// Imaging CCD Info
            /// </summary>
            CCD_INFO_IMAGING,
            /// <summary>
            /// Tracking CCD Info
            /// </summary>
            CCD_INFO_TRACKING,
            /// <summary>
            /// Extended CCD Info
            /// </summary>
            CCD_INFO_EXTENDED,
            /// <summary>
            /// Extended CCD Info 5C
            /// </summary>
            CCD_INFO_EXTENDED_5C,
            /// <summary>
            /// Extended Imaging CCD Info 2
            /// </summary>
            CCD_INFO_EXTENDED2_IMAGING,
            /// <summary>
            /// Extended Tracking CCD Info 2
            /// </summary>
            CCD_INFO_EXTENDED2_TRACKING,
            /// <summary>
            /// Extended Imaging CCD Info 3
            /// </summary>
            CCD_INFO_EXTENDED3
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * Anti-blooming gate capability enum 
         */
        public enum IMAGING_ABG : ushort
        {
            /// <summary>
            /// Anti-blooming gate not Present
            /// </summary>
            ABG_NOT_PRESENT,
            /// <summary>
            /// Anti-blooming gate present
            /// </summary>
            ABG_PRESENT
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * Port bit-rate enum 
         */
        public enum PORT_RATE : ushort
        {
            /// <summary>
            /// Bit-rate auto
            /// </summary>
            BR_AUTO,
            /// <summary>
            /// Bit-rate 9600
            /// </summary>
            BR_9600,
            /// <summary>
            /// Bit-rate 19K
            /// </summary>
            BR_19K,
            /// <summary>
            /// Bit-rate 38K
            /// </summary>
            BR_38K,
            /// <summary>
            /// Bit-rate 57K
            /// </summary>
            BR_57K,
            /// <summary>
            /// Bit-rate 115K
            /// </summary>
            BR_115K
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * Camera type enum 
         */
        public enum CAMERA_TYPE : ushort
        {
            /// <summary>
            /// ST-7 Camera
            /// </summary>
            ST7_CAMERA = 4,
            /// <summary>
            /// ST-8 Camera
            /// </summary>
            ST8_CAMERA,
            /// <summary>
            /// ST-5C Camera
            /// </summary>
            ST5C_CAMERA,
            /// <summary>
            /// TCE-Controller
            /// </summary>
            TCE_CONTROLLER,
            /// <summary>
            /// ST-237 Camera
            /// </summary>
            ST237_CAMERA,
            /// <summary>
            /// ST-K Camera
            /// </summary>
            STK_CAMERA,
            /// <summary>
            /// ST-9 Camera
            /// </summary>
            ST9_CAMERA,
            /// <summary>
            /// ST-V Camera
            /// </summary>
            STV_CAMERA,
            /// <summary>
            /// ST-10 Camera
            /// </summary>
            ST10_CAMERA,
            /// <summary>
            /// ST-1000 Camera
            /// </summary>
            ST1K_CAMERA,
            /// <summary>
            /// ST-2000 Camera
            /// </summary>
            ST2K_CAMERA,
            /// <summary>
            /// STL Camera
            /// </summary>
            STL_CAMERA,
            /// <summary>
            /// ST-402 Camera
            /// </summary>
            ST402_CAMERA,
            /// <summary>
            /// STX Camera
            /// </summary>
            STX_CAMERA,
            /// <summary>
            /// ST-4000 Camera
            /// </summary>
            ST4K_CAMERA,
            /// <summary>
            /// STT Camera
            /// </summary>
            STT_CAMERA,
            /// <summary>
            /// ST-i Camera
            /// </summary>
            STI_CAMERA,
            /// <summary>
            /// STF Camera, NOTE: STF8, and STF cameras both report this kind, but have *DIFFERENT CAMERA MODEL ID VARIABLES* (stf8CameraID and stfCameraID)
            /// </summary>
            STF_CAMERA,
            /// <summary>
            /// Next Camera
            /// </summary>
            NEXT_CAMERA,
            /// <summary>
            /// No Camera
            /// </summary>
            NO_CAMERA = 0xFFFF
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * Shutter Control enum 
         */
        public enum SHUTTER_COMMAND : ushort
        {
            /// <summary>
            /// Shutter Control: Leave shutter in current state.
            /// </summary>
            SC_LEAVE_SHUTTER,
            /// <summary>
            /// Shutter Control: Open shutter.
            /// </summary>
            SC_OPEN_SHUTTER,
            /// <summary>
            /// Shutter Control: Close shutter.
            /// </summary>
            SC_CLOSE_SHUTTER,
            /// <summary>
            /// Shutter Control: Initialize shutter.
            /// </summary>
            SC_INITIALIZE_SHUTTER,
            /// <summary>
            /// Shutter Control: Open external shutter.
            /// </summary>
            SC_OPEN_EXT_SHUTTER,
            /// <summary>
            /// Shutter Control: Close external shutter.
            /// </summary>
            SC_CLOSE_EXT_SHUTTER
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * Shutter State enum 
         */
        public enum SHUTTER_STATE7 : ushort
        {
            /// <summary>
            /// Shuter State: Open
            /// </summary>
            SS_OPEN,
            /// <summary>
            /// Shuter State: Closed
            /// </summary>
            SS_CLOSED,
            /// <summary>
            /// Shutter State: Opening
            /// </summary>
            SS_OPENING,
            /// <summary>
            /// Shutter State: Closing
            /// </summary>
            SS_CLOSING
        };

        /*!
         * \ingroup BASE_STRUCTURES
         * Temperature regulation enum 
         */
        public enum TEMPERATURE_REGULATION : ushort
        {
            /// <summary>
            /// Temperature regulation off
            /// </summary>
            REGULATION_OFF,
            /// <summary>
            /// Temperature regulation on
            /// </summary>
            REGULATION_ON,
            /// <summary>
            /// Temperature regulation override
            /// </summary>
            REGULATION_OVERRIDE,
            /// <summary>
            /// Temperature regulation freeze
            /// </summary>
            REGULATION_FREEZE,
            /// <summary>
            /// Temperature regulation unfreeze
            /// </summary>
            REGULATION_UNFREEZE,
            /// <summary>
            /// Temperature regulation enable autofreeze
            /// </summary>
            REGULATION_ENABLE_AUTOFREEZE,
            /// <summary>
            /// Temperature regulation disable autofreeze
            /// </summary>
            REGULATION_DISABLE_AUTOFREEZE
        };

        /*!
         * Mask for Temperature Regulation frozen state
         */
        const UInt16 REGULATION_FROZEN_MASK = 0x8000;

        /*!
         * LED State enum 
         */
        public enum LED_STATE : ushort
        {
            /// <summary>
            /// LED off
            /// </summary>
            LED_OFF,
            /// <summary>
            /// LED on
            /// </summary>
            LED_ON,
            /// <summary>
            /// LED Blink low
            /// </summary>
            LED_BLINK_LOW,
            /// <summary>
            /// LED Blink high
            /// </summary>
            LED_BLINK_HIGH
        };

        /*!
         * Filter command enum
         */
        public enum FILTER_COMMAND : ushort
        {
            /// <summary>
            /// Filter leave
            /// </summary>
            FILTER_LEAVE,
            /// <summary>
            /// Filter slot 1
            /// </summary>
            FILTER_SET_1,
            /// <summary>
            /// Filter slot 2
            /// </summary>
            FILTER_SET_2,
            /// <summary>
            /// Filter slot 3
            /// </summary>
            FILTER_SET_3,
            /// <summary>
            /// Filter slot 4
            /// </summary>
            FILTER_SET_4,
            /// <summary>
            /// Filter slot 5
            /// </summary>
            FILTER_SET_5,
            /// <summary>
            /// Stop filter
            /// </summary>
            FILTER_STOP,
            /// <summary>
            /// Initialize filter
            /// </summary>
            FILTER_INIT
        };

        /*!
         * Filter State enum 
         */
        public enum FILTER_STATE : ushort
        {
            /// <summary>
            /// Filter wheel moving
            /// </summary>
            FS_MOVING,
            /// <summary>
            /// Filter wheel at slot 1
            /// </summary>
            FS_AT_1,
            /// <summary>
            /// Filter wheel at slot 2
            /// </summary>
            FS_AT_2,
            /// <summary>
            /// Filter wheel at slot 3
            /// </summary>
            FS_AT_3,
            /// <summary>
            /// Filter wheel at slot 4
            /// </summary>
            FS_AT_4,
            /// <summary>
            /// Filter wheel at slot 5
            /// </summary>
            FS_AT_5,
            /// <summary>
            /// Filter wheel at slot Unknown
            /// </summary>
            FS_UNKNOWN
        };

        /*!
         * A/D Size enum 
         */
        public enum AD_SIZE : ushort
        {
            /// <summary>
            /// Unknown size
            /// </summary>
            AD_UNKNOWN,
            /// <summary>
            /// 12-bits
            /// </summary>
            AD_12_BITS,
            /// <summary>
            /// 16-bits
            /// </summary>
            AD_16_BITS
        };

        /*!
         * Filter Wheel Type enum
         */
        public enum FILTER_TYPE : ushort
        {
            /// <summary>
            /// Unkwown Filter Wheel
            /// </summary>
            FW_UNKNOWN,
            /// <summary>
            /// External Filter Wheel
            /// </summary>
            FW_EXTERNAL,
            /// <summary>
            /// Vane Filter Wheel
            /// </summary>
            FW_VANE,
            /// <summary>
            /// Standard Filter Wheel
            /// </summary>
            FW_FILTER_WHEEL
        };

        /*!
         * AO Focus enum 
         */
        public enum AO_FOCUS_COMMAND : ushort
        {
            /// <summary>
            /// AO Focus hard center
            /// </summary>
            AOF_HARD_CENTER,
            /// <summary>
            /// AO Focus soft center
            /// </summary>
            AOF_SOFT_CENTER,
            /// <summary>
            /// AO Focus step in
            /// </summary>
            AOF_STEP_IN,
            /// <summary>
            /// AO Focus step out
            /// </summary>
            AOF_STEP_OUT
        };

        // Ethernet stuff
        /*!
         * Service port for Ethernet access.
         */
        const Int16 SRV_SERVICE_PORT = 5000;

        /*!
         * Broadcast port for SBIG Cameras
         */
        const Int16 BROADCAST_PORT = 5001;

        /*!
         * SBIG Device types enum 
         */
        public enum SBIG_DEVICE_TYPE : ushort
        {
            /// <summary>
            /// Device type: None
            /// </summary>
            DEV_NONE,
            /// <summary>
            /// LPT port slot 1
            /// </summary>
            DEV_LPT1,
            /// <summary>
            /// LPT port slot 2
            /// </summary>
            DEV_LPT2,
            /// <summary>
            /// LPT port slot 3
            /// </summary>
            DEV_LPT3,
            /// <summary>
            /// USB autodetect
            /// </summary>
            DEV_USB = 0x7F00,
            /// <summary>
            /// Ethernet
            /// </summary>
            DEV_ETH,
            /// <summary>
            /// USB slot 1 CC_QUERY_USB
            /// </summary>
            DEV_USB1,
            /// <summary>
            /// USB slot 2
            /// </summary>
            DEV_USB2,
            /// <summary>
            /// USB slot 3
            /// </summary>
            DEV_USB3,
            /// <summary>
            /// USB slot 4
            /// </summary>
            DEV_USB4,
            /// <summary>
            /// USB slot 5 CC_QUERY_USB2
            /// </summary>
            DEV_USB5,
            /// <summary>
            /// USB slot 6
            /// </summary>
            DEV_USB6,
            /// <summary>
            /// USB slot 7
            /// </summary>
            DEV_USB7,
            /// <summary>
            /// USB slot 8
            /// </summary>
            DEV_USB8,
            /// <summary>
            /// USB slot 9 CC_QUERY_USB3
            /// </summary>
            DEV_USB9,
            /// <summary>
            /// USB slot 10
            /// </summary>
            DEV_USB10,
            /// <summary>
            /// USB slot 11
            /// </summary>
            DEV_USB11,
            /// <summary>
            /// USB slot 12
            /// </summary>
            DEV_USB12,
            /// <summary>
            /// USB slot 13
            /// </summary>
            DEV_USB13,
            /// <summary>
            /// USB slot 14
            /// </summary>
            DEV_USB14,
            /// <summary>
            /// USB slot 15
            /// </summary>
            DEV_USB15,
            /// <summary>
            /// USB slot 16
            /// </summary>
            DEV_USB16,
            /// <summary>
            /// USB slot 17
            /// </summary>
            DEV_USB17,
            /// <summary>
            /// USB slot 18
            /// </summary>
            DEV_USB18,
            /// <summary>
            /// USB slot 19
            /// </summary>
            DEV_USB19,
            /// <summary>
            /// USB slot 20
            /// </summary>
            DEV_USB20,
            /// <summary>
            /// USB slot 21
            /// </summary>
            DEV_USB21,
            /// <summary>
            /// USB slot 22
            /// </summary>
            DEV_USB22,
            /// <summary>
            /// USB slot 23
            /// </summary>
            DEV_USB23,
            /// <summary>
            /// USB slot 24
            /// </summary>
            DEV_USB24,
        };

        /*!
         * Driver control parameters enum
         */
        public enum DRIVER_CONTROL_PARAM : ushort
        {
            /// <summary>
            /// Enable FIFO
            /// </summary>
            DCP_USB_FIFO_ENABLE,
            /// <summary>
            /// Enable Journaling
            /// </summary>
            DCP_CALL_JOURNAL_ENABLE,
            /// <summary>
            /// IV to H Ratio
            /// </summary>
            DCP_IVTOH_RATIO,
            /// <summary>
            /// USB FIFO size
            /// </summary>
            DCP_USB_FIFO_SIZE,
            /// <summary>
            /// USB Driver
            /// </summary>
            DCP_USB_DRIVER,
            /// <summary>
            /// KAI Relative Gain
            /// </summary>
            DCP_KAI_RELGAIN,
            /// <summary>
            /// USB Pixel D\L enable
            /// </summary>
            DCP_USB_PIXEL_DL_ENABLE,
            /// <summary>
            /// High throughput
            /// </summary>
            DCP_HIGH_THROUGHPUT,
            /// <summary>
            /// VDD Optimized
            /// </summary>
            DCP_VDD_OPTIMIZED,
            /// <summary>
            /// Auto A/D Gain
            /// </summary>
            DCP_AUTO_AD_GAIN,
            /// <summary>
            /// No H-Clocks for Integration
            /// </summary>
            DCP_NO_HCLKS_FOR_INTEGRATION,
            /// <summary>
            /// TDI Mode Enable
            /// </summary>
            DCP_TDI_MODE_ENABLE,
            /// <summary>
            /// Vertical Flush control enable
            /// </summary>
            DCP_VERT_FLUSH_CONTROL_ENABLE,
            /// <summary>
            /// Ethernet pipeline enable
            /// </summary>
            DCP_ETHERNET_PIPELINE_ENABLE,
            /// <summary>
            /// Fast link
            /// </summary>
            DCP_FAST_LINK,
            /// <summary>
            /// Overscan Rows/Columns
            /// </summary>
            DCP_OVERSCAN_ROWSCOLS,
            /// <summary>
            /// Enable Pixel Pipeline
            /// </summary>
            DCP_PIXEL_PIPELINE_ENABLE,
            /// <summary>
            /// Enable column repair
            /// </summary>
            DCP_COLUMN_REPAIR_ENABLE,
            /// <summary>
            /// Enable warm pixel repair
            /// </summary>
            DCP_WARM_PIXEL_REPAIR_ENABLE,
            /// <summary>
            /// warm pixel repair count
            /// </summary>
            DCP_WARM_PIXEL_REPAIR_COUNT,
            /// <summary>
            /// TDI Drift rate in [XXX]
            /// </summary>
            DCP_TDI_MODE_DRIFT_RATE,
            /// <summary>
            /// Override A/D Converter's Gain
            /// </summary>
            DCP_OVERRIDE_AD_GAIN,
            /// <summary>
            /// Override auto offset adjustments in certain cameras.
            /// </summary>
            DCP_ENABLE_AUTO_OFFSET,
            /// <summary>
            /// Last Device control parameter
            /// </summary>
            DCP_LAST
        };

        /*!
         * USB A/D Control commands 
         */
        public enum USB_AD_CONTROL_COMMAND : ushort
        {
            /// <summary>
            /// Imaging gain
            /// </summary>
            USB_AD_IMAGING_GAIN,
            /// <summary>
            /// Imaging offset
            /// </summary>
            USB_AD_IMAGING_OFFSET,
            /// <summary>
            /// Internal tracking gain
            /// </summary>

            USB_AD_TRACKING_GAIN,
            /// <summary>
            /// Internal tracking offset
            /// </summary>
            USB_AD_TRACKING_OFFSET,
            /// <summary>
            /// External tracking gain
            /// </summary>

            USB_AD_EXTTRACKING_GAIN,
            /// <summary>
            /// External tracking offset
            /// </summary>
            USB_AD_EXTTRACKING_OFFSET,
            /// <summary>
            /// Imaging gain channel 2
            /// </summary>

            USB_AD_IMAGING2_GAIN,
            /// <summary>
            /// Imaging offset channel 2
            /// </summary>
            USB_AD_IMAGING2_OFFSET,
            /// <summary>
            /// Imaging gain right channel
            /// </summary>

            USB_AD_IMAGING_GAIN_RIGHT,
            /// <summary>
            /// Imaging offset right channel
            /// </summary>
            USB_AD_IMAGING_OFFSET_RIGHT,
        };

        /*!
         * USB Driver enum 
         */
        public enum ENUM_USB_DRIVER : ushort
        {
            /// <summary>
            /// SBIG E
            /// </summary>
            USBD_SBIGE,
            /// <summary>
            /// SBIG I
            /// </summary>
            USBD_SBIGI,
            /// <summary>
            /// SBIG_M
            /// </summary>
            USBD_SBIGM,
            /// <summary>
            /// Next
            /// </summary>
            USBD_NEXT
        };

        /*!
         * Filter Weel Model Selection enum 
         */
        public enum CFW_MODEL_SELECT : ushort
        {
            /// <summary>
            /// Unknown Model
            /// </summary>
            CFWSEL_UNKNOWN,
            /// <summary>
            /// CFW2
            /// </summary>
            CFWSEL_CFW2,
            /// <summary>
            /// CFW5
            /// </summary>
            CFWSEL_CFW5,
            /// <summary>
            /// CFW8
            /// </summary>
            CFWSEL_CFW8,
            /// <summary>
            /// CFWL
            /// </summary>
            CFWSEL_CFWL,
            /// <summary>
            /// CFW-402
            /// </summary>
            CFWSEL_CFW402,
            /// <summary>
            /// Auto
            /// </summary>
            CFWSEL_AUTO,
            /// <summary>
            /// CFW-6A
            /// </summary>
            CFWSEL_CFW6A,
            /// <summary>
            /// CFW10
            /// </summary>
            CFWSEL_CFW10,
            /// <summary>
            /// CFW10-Serial
            /// </summary>
            CFWSEL_CFW10_SERIAL,
            /// <summary>
            /// CFW9
            /// </summary>
            CFWSEL_CFW9,
            /// <summary>
            /// CFWL8
            /// </summary>
            CFWSEL_CFWL8,
            /// <summary>
            /// CFWL8-G
            /// </summary>
            CFWSEL_CFWL8G,
            /// <summary>
            /// CFW1603
            /// </summary>
            CFWSEL_CFW1603,
            /// <summary>
            /// FW5-STX
            /// </summary>
            CFWSEL_FW5_STX,
            /// <summary>
            /// FW5-8300
            /// </summary>
            CFWSEL_FW5_8300,
            /// <summary>
            /// FW8-8300
            /// </summary>
            CFWSEL_FW8_8300,
            /// <summary>
            /// FW7-STX
            /// </summary>
            CFWSEL_FW7_STX,
            /// <summary>
            /// FW8-STT
            /// </summary>
            CFWSEL_FW8_STT,
            /// <summary>
            /// FW5-STF Detent
            /// </summary>
            CFWSEL_FW5_STF_DETENT
        };

        /*!
         * Filter Wheel Command enum 
         */
        public enum CFW_COMMAND : ushort
        {
            /// <summary>
            /// Query
            /// </summary>
            CFWC_QUERY,
            /// <summary>
            /// Go-to slot
            /// </summary>
            CFWC_GOTO,
            /// <summary>
            /// Initialize
            /// </summary>
            CFWC_INIT,
            /// <summary>
            /// Get Info
            /// </summary>
            CFWC_GET_INFO,
            /// <summary>
            /// Open device
            /// </summary>
            CFWC_OPEN_DEVICE,
            /// <summary>
            /// Close device
            /// </summary>
            CFWC_CLOSE_DEVICE
        };

        /*!
         * Filter Wheel Status enum 
         */
        public enum CFW_STATUS : ushort
        {
            /// <summary>
            /// Unknown state
            /// </summary>
            CFWS_UNKNOWN,
            /// <summary>
            /// Idle state
            /// </summary>
            CFWS_IDLE,
            /// <summary>
            /// Busy state
            /// </summary>
            CFWS_BUSY
        };

        /*!
         * Filter Wheel errors enum 
         */
        public enum CFW_ERROR : ushort
        {
            /// <summary>
            /// No error
            /// </summary>
            CFWE_NONE,
            /// <summary>
            /// Busy error
            /// </summary>
            CFWE_BUSY,
            /// <summary>
            /// Bad command error
            /// </summary>
            CFWE_BAD_COMMAND,
            /// <summary>
            /// Calibration error
            /// </summary>
            CFWE_CAL_ERROR,
            /// <summary>
            /// Motor timeout error
            /// </summary>
            CFWE_MOTOR_TIMEOUT,
            /// <summary>
            /// Bad model error
            /// </summary>
            CFWE_BAD_MODEL,
            /// <summary>
            /// Device not closed error
            /// </summary>
            CFWE_DEVICE_NOT_CLOSED,
            /// <summary>
            /// Device not open error
            /// </summary>
            CFWE_DEVICE_NOT_OPEN,
            /// <summary>
            /// I2C communication error
            /// </summary>
            CFWE_I2C_ERROR
        };

        /*!
         * Filter Wheel position enum 
         */
        public enum CFW_POSITION : ushort
        {
            /// <summary>
            /// Unknown
            /// </summary>
            CFWP_UNKNOWN,
            /// <summary>
            /// Slot 1
            /// </summary>
            CFWP_1,
            /// <summary>
            /// Slot 2
            /// </summary>
            CFWP_2,
            /// <summary>
            /// Slot 3
            /// </summary>
            CFWP_3,
            /// <summary>
            /// Slot 4
            /// </summary>
            CFWP_4,
            /// <summary>
            /// Slot 5
            /// </summary>
            CFWP_5,
            /// <summary>
            /// Slot 6
            /// </summary>
            CFWP_6,
            /// <summary>
            /// Slot 7
            /// </summary>
            CFWP_7,
            /// <summary>
            /// Slot 8
            /// </summary>
            CFWP_8,
            /// <summary>
            /// Slot 9
            /// </summary>
            CFWP_9,
            /// <summary>
            /// Slot 10
            /// </summary>
            CFWP_10
        };

        /*!
         * Filter Wheel COM port enum 
         */
        public enum CFW_COM_PORT : ushort
        {
            /// <summary>
            /// COM1
            /// </summary>
            CFWPORT_COM1 = 1,
            /// <summary>
            /// COM2
            /// </summary>
            CFWPORT_COM2,
            /// <summary>
            /// COM3
            /// </summary>
            CFWPORT_COM3,
            /// <summary>
            /// COM4
            /// </summary>
            CFWPORT_COM4
        };

        /*!
         * Filter Wheel Get Info select enum 
         */
        public enum CFW_GETINFO_SELECT : ushort
        {
            /// <summary>
            /// Firmware version
            /// </summary>
            CFWG_FIRMWARE_VERSION,
            /// <summary>
            /// Calibration data
            /// </summary>
            CFWG_CAL_DATA,
            /// <summary>
            /// Data registers
            /// </summary>
            CFWG_DATA_REGISTERS
        };

        /*!
         * Bit I/O Operation enum 
         */
        public enum BITIO_OPERATION : ushort
        {
            /// <summary>
            /// Write
            /// </summary>
            BITIO_WRITE,
            /// <summary>
            /// Read
            /// </summary>
            BITIO_READ
        };

        /*!
         * Bit I/O Name enum 
         */
        public enum BITIO_NAME : ushort
        {
            /// <summary>
            /// In: PS Low
            /// </summary>
            BITI_PS_LOW,
            /// <summary>
            /// Out: I/O 1
            /// </summary>
            BITO_IO1,
            /// <summary>
            /// Out: I/O 2
            /// </summary>
            BITO_IO2,
            /// <summary>
            /// In: I/O 3
            /// </summary>
            BITI_IO3,
            /// <summary>
            /// FPGA WE
            /// </summary>
            BITO_FPGA_WE
        };

        /*!
         * Biorad TDI Error enum 
         */
        public enum BTDI_ERROR : ushort
        {
            /// <summary>
            /// BTDI Schedule error
            /// </summary>
            BTDI_SCHEDULE_ERROR = 1,
            /// <summary>
            /// BTDI Overrun error
            /// </summary>
            BTDI_OVERRUN_ERROR = 2
        };

        /*!
         * Motor Focus Model Selection enum 
         */
        public enum MF_MODEL_SELECT : ushort
        {
            /// <summary>
            /// Unknown
            /// </summary>
            MFSEL_UNKNOWN,
            /// <summary>
            /// Automatic
            /// </summary>
            MFSEL_AUTO,
            /// <summary>
            /// STF
            /// </summary>
            MFSEL_STF
        };

        /*!
         * Motor Focus Command enum 
         */
        public enum MF_COMMAND : ushort
        {
            /// <summary>
            /// Query
            /// </summary>
            MFC_QUERY,
            /// <summary>
            /// Go-to
            /// </summary>
            MFC_GOTO,
            /// <summary>
            /// Initialize
            /// </summary>
            MFC_INIT,
            /// <summary>
            /// Get Info
            /// </summary>
            MFC_GET_INFO,
            /// <summary>
            /// Abort
            /// </summary>
            MFC_ABORT
        };

        /*!
         * Motor Focus Status 
         */
        public enum MF_STATUS : ushort
        {
            /// <summary>
            /// Unknown
            /// </summary>
            MFS_UNKNOWN,
            /// <summary>
            /// Idle
            /// </summary>
            MFS_IDLE,
            /// <summary>
            /// Busy
            /// </summary>
            MFS_BUSY
        };

        /*!
         * Motor Focus Error state enum 
         */
        public enum MF_ERROR : ushort
        {
            /// <summary>
            /// None
            /// </summary>
            MFE_NONE,
            /// <summary>
            /// Busy
            /// </summary>
            MFE_BUSY,
            /// <summary>
            /// Bad command
            /// </summary>
            MFE_BAD_COMMAND,
            /// <summary>
            /// Calibration error
            /// </summary>
            MFE_CAL_ERROR,
            /// <summary>
            /// Motor timeout
            /// </summary>
            MFE_MOTOR_TIMEOUT,
            /// <summary>
            /// Bad model
            /// </summary>
            MFE_BAD_MODEL,
            /// <summary>
            /// I2C error
            /// </summary>
            MFE_I2C_ERROR,
            /// <summary>
            /// Not found
            /// </summary>
            MFE_NOT_FOUND
        };

        /*!
         * Motor Focus Get Info Select enum 
         */
        public enum MF_GETINFO_SELECT : ushort
        {
            /// <summary>
            /// Firmware Version
            /// </summary>
            MFG_FIRMWARE_VERSION,
            /// <summary>
            /// Data Registers
            /// </summary>
            MFG_DATA_REGISTERS
        };

        /*!
         * Differential guider commands enum 
         */
        public enum DIFF_GUIDER_COMMAND : ushort
        {
            /// <summary>
            /// Detect Differential guider hardware
            /// </summary>
            DGC_DETECT,
            /// <summary>
            /// Get brightness
            /// </summary>
            DGC_GET_BRIGHTNESS,
            /// <summary>
            /// Set brightness
            /// </summary>
            DGC_SET_BRIGHTNESS
        };

        /*!
         * Differential guider error enum 
         */
        public enum DIFF_GUIDER_ERROR : ushort
        {
            /// <summary>
            /// No error
            /// </summary>
            DGE_NO_ERROR,
            /// <summary>
            /// Differential guider not found
            /// </summary>
            DGE_NOT_FOUND,
            /// <summary>
            /// Bad command
            /// </summary>
            DGE_BAD_COMMAND,
            /// <summary>
            /// Bad parameter
            /// </summary>
            DGE_BAD_PARAMETER
        };

        /*!
         * Differential Guider status enum 
         */
        public enum DIFF_GUIDER_STATUS : ushort
        {
            /// <summary>
            /// Unknown
            /// </summary>
            DGS_UNKNOWN,
            /// <summary>
            /// Idle
            /// </summary>
            DGS_IDLE,
            /// <summary>
            /// Busy
            /// </summary>
            DGS_BUSY
        };

        /*!
         * Fan state enum 
         */
        public enum FAN_STATE : ushort
        {
            /// <summary>
            /// Fan Off
            /// </summary>
            FS_OFF,
            /// <summary>
            /// Fan On
            /// </summary>
            FS_ON,
            /// <summary>
            /// Fan Auto
            /// </summary>
            FS_AUTOCONTROL
        };

        /*!
         * Bulk IO command enum 
         */
        public enum BULK_IO_COMMAND : ushort
        {
            /// <summary>
            /// Read
            /// </summary>
            BIO_READ,
            /// <summary>
            /// Write
            /// </summary>
            BIO_WRITE,
            /// <summary>
            /// Flush
            /// </summary>
            BIO_FLUSH
        };

        /*!
         * Pixel channel mode enum 
         */
        public enum PIXEL_CHANNEL_MODE : ushort
        {
            /// <summary>
            /// Pixel Channel A
            /// </summary>
            PIXEL_CHANNEL_MODE_A,
            /// <summary>
            /// Pixel Channel B
            /// </summary>
            PIXEL_CHANNEL_MODE_B,
            /// <summary>
            /// Pixel Channel AB
            /// </summary>
            PIXEL_CHANNEL_MODE_AB
        };

        /*!
         * Active Pixel Channel enum 
         */
        public enum ACTIVE_PIXEL_CHANNEL : ushort
        {
            /// <summary>
            /// Pixel Channel A
            /// </summary>
            PIXEL_CHANNEL_A,
            /// <summary>
            /// Pixel Channel B
            /// </summary>
            PIXEL_CHANNEL_B
        };

        public enum EXTRA_EXPOSURE_STATUS : ushort
        {
            XES_IDLE,           //!< CCD is currently idle.
            XES_PRE_EXP,        //!< CCD is in the pre-exposure phase.
            XES_INTEGRATING,    //!< CCD is currently exposing/integrating an image.
            XES_POST_EXP        //!< CCD is in the post-exposure phase.
        };

        /*
         * General Purpose Flags
         */

        /*!
         * set in EndExposureParams::ccd to skip synchronization delay - Use this to increase the
         * rep rate when taking darks to later be subtracted from SC_LEAVE_SHUTTER exposures such as when tracking and imaging.
         */
        const UInt16 END_SKIP_DELAY = 0x8000;

        /*!
         * Set in StartExposureParams::ccd to skip lowering Imaging CCD Vdd during integration. - Use this to
         * increase the rep rate when you don't care about glow in the upper-left corner of the imaging CCD.
         */
        const UInt16 START_SKIP_VDD = 0x8000;

        /*!
         *  Set in StartExposureParams::ccd and EndExposureParams::ccd to force shutter motor to stay on all the 
         * time which reduces delays in Start and End Exposure timing and yields higher image throughput.  Don't
         * do this too often or camera head will heat up.
         */
        const UInt16 START_MOTOR_ALWAYS_ON = 0x4000;

        /*!
         * Set in EndExposureParams::ccd to abort the exposure completely instead of just ending 
         * the integration phase for cameras with internal frame buffers like the STX.
         */
        const UInt16 ABORT_DONT_END = 0x2000;

        /*!
         */

        /*!
         * \ingroup EXPOSURE_FLAGS
         * Set in StartExposureParams2::exposureTime enable TDI readout mode [TODO: Add supported cameras].
         */
        const UInt32 EXP_TDI_ENABLE = 0x01000000;   //!< Enable TDI mode flag.

        /*!
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime ripple correction for STF-8050/4070
         */
        const UInt32 EXP_RIPPLE_CORRECTION = 0x02000000;    //!< Enable Ripple correction flag.

        /*!
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to activate the dual channel CCD readout mode of the STF-8050.
         */
        const UInt32 EXP_DUAL_CHANNEL_MODE = 0x04000000;    //!< Enable dual channel readout mode flag.

        /*!
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to activate the fast readout mode of the STF-8300, etc.
         */
        const UInt32 EXP_FAST_READOUT = 0x08000000; //!< Enable fast readout mode flag.

        /*!
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to interpret exposure time as milliseconds.
         */
        const UInt32 EXP_MS_EXPOSURE = 0x10000000;  //!< Enable millisecond exposure time flag.

        /*!
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to do light clear of the CCD.
         */
        const UInt32 EXP_LIGHT_CLEAR = 0x20000000;  //!< Do light clear of CCD flag.

        /*!
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to send trigger out Y-.
         */
        const UInt32 EXP_SEND_TRIGGER_OUT = 0x40000000;  //!< Send trigger out flag.

        /*!
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to wait for trigger in pulse.
         */
        const UInt32 EXP_WAIT_FOR_TRIGGER_IN = 0x80000000;  //!< Wait for trigger in flag.

        /*!
         * \ingroup EXPOSURE_FLAGS
         * Set in StarExposureParams2::exposureTime to mask with exposure time to remove flags.
         */
        const UInt32 EXP_TIME_MASK = 0x00FFFFFF;  //!< Mask for exposure time value.

        /*!
         * Bit Field Definitions for the in the GetCCDInfoResults4 struct.
         */

        /*!
         * \ingroup CAPABILITIES_BITS
         * mask for CCD type
         */
        const UInt16 CB_CCD_TYPE_MASK = 0x0001; //!< Mask for CCD type.

        /*!
         * \ingroup CAPABILITIES_BITS
         * b0=0 is full frame CCD
         */
        const UInt16 CB_CCD_TYPE_FULL_FRAME = 0x0000;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b0=1 is frame transfer CCD
         */
        const UInt16 CB_CCD_TYPE_FRAME_TRANSFER = 0x0001;

        /*!
         * \ingroup CAPABILITIES_BITS
         * mask for electronic shutter type
         */
        const UInt16 CB_CCD_ESHUTTER_MASK = 0x0002;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b1=0 indicates no electronic shutter
         */
        const UInt16 CB_CCD_ESHUTTER_NO = 0x0000;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b1=1 indicates electronic shutter
         */
        const UInt16 CB_CCD_ESHUTTER_YES = 0x0002;

        /*!
         * \ingroup CAPABILITIES_BITS
         * mask for external tracker support
         */
        const UInt16 CB_CCD_EXT_TRACKER_MASK = 0x0004;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b2=0 indicates no external tracker support
         */
        const UInt16 CB_CCD_EXT_TRACKER_NO = 0x0000;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b2=1 indicates external tracker support
         */
        const UInt16 CB_CCD_EXT_TRACKER_YES = 0x0004;

        /*!
         * \ingroup CAPABILITIES_BITS
         * mask for BTDI support
         */
        const UInt16 CB_CCD_BTDI_MASK = 0x0008;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b3=0 indicates no BTDI support
         */
        const UInt16 CB_CCD_BTDI_NO = 0x0000;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b3=1 indicates BTDI support
         */
        const UInt16 CB_CCD_BTDI_YES = 0x0008;

        /*!
         * \ingroup CAPABILITIES_BITS
         * mask for AO-8 detected 
         */
        const UInt16 CB_AO8_MASK = 0x0010;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b4=0 indicates no AO-8 detected
         */
        const UInt16 CB_AO8_NO = 0x0000;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b4=1 indicates AO-8 detected
         */
        const UInt16 CB_AO8_YES = 0x0010;

        /*!
         * \ingroup CAPABILITIES_BITS
         * mask for camera with frame buffer
         */
        const UInt16 CB_FRAME_BUFFER_MASK = 0x0020;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b5=0 indicates camera without Frame Buffer
         */
        const UInt16 CB_FRAME_BUFFER_NO = 0x0000;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b5=1 indicates camera with Frame Buffer
         */
        const UInt16 CB_FRAME_BUFFER_YES = 0x0020;

        /*!
         * \ingroup CAPABILITIES_BITS
         * mask for camera that requires StartExposure2
         */
        const UInt16 CB_REQUIRES_STARTEXP2_MASK = 0x0040;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b6=0 indicates camera works with StartExposure
         */
        const UInt16 CB_REQUIRES_STARTEXP2_NO = 0x0000;

        /*!
         * \ingroup CAPABILITIES_BITS
         * b6=1 indicates camera Requires StartExposure2
         */
        const UInt16 CB_REQUIRES_STARTEXP2_YES = 0x0040;

        /*!
         */

        /*!
         * \ingroup MINIMUM_DEFINES
         * Minimum exposure for ST-7 cameras in 1/100ths second
         */
        const Int16 MIN_ST7_EXPOSURE = 12;

        /*!
         * \ingroup MINIMUM_DEFINES
         * Minimum exposure for ST-402 cameras in 1/100ths second
         */
        const Int16 MIN_ST402_EXPOSURE = 4;

        /*!
         * \ingroup MINIMUM_DEFINES
         * Minimum exposure fpr STF-3200 cameras in 1/100ths second
         */
        const Int16 MIN_ST3200_EXPOSURE = 9;


        /*!
         * \ingroup MINIMUM_DEFINES
         * Minimum exposure for STF-8300 cameras in 1/100ths second
         */
        const Int16 MIN_STF8300_EXPOSURE = 9;

        /*!
         * \ingroup MINIMUM_DEFINES
         * Minimum exposure for STF-8050 cameras in 1/1000ths second since has E Shutter
         */
        const Int16 MIN_STF8050_EXPOSURE = 1;

        /*!
         * \ingroup MINIMUM_DEFINES
         * Minimum exposure for STF-4070 cameras in 1/1000ths second since has E Shutter
         */
        const Int16 MIN_STF4070_EXPOSURE = 1;


        /*!
         * \ingroup MINIMUM_DEFINES
         * Minimum exposure for STF-0402 cameras in 1/100ths second.
         */
        const Int16 MIN_STF0402_EXPOSURE = 4;

        /*!
         * \ingroup MINIMUM_DEFINES
         * Minimum exposure for STX cameras in 1/100ths second
         */
        const Int16 MIN_STX_EXPOSURE = 18;

        /*!
         * \ingroup MINIMUM_DEFINES
         * Minimum exposure for STT cameras in 1/100ths second
         */
        const Int16 MIN_STT_EXPOSURE = 12;

        /*!
         * \ingroup MINIMUM_DEFINES
         * Minimum exposure in 1/1000ths second since ST-i has E Shutter
         */
        const Int16 MIN_STU_EXPOSURE = 1;

        /*!
            \defgroup commandParamStructs
            Command Parameter and Results Structs

            Make sure you set your compiler for byte structure alignment
            as that is how the driver was built.

        */

        /*!
         * \brief Start Exposure command parameters
         *
         * Parameters used to start SBIG camera exposures.
         */
        public struct StartExposureParams
        {
            /// <summary>
            /// Requested CCD. see also: CCD_REQUEST enum.
            /// </summary>
            public UInt16 ccd;
            /// <summary>
            /// Exposure time in hundredths of a second in least significant 24 bits. Most significant bits are bit-flags described in exposureTime #define block.
            /// </summary>
            public UInt32 exposureTime;
            /// <summary>
            /// see also: ABG_STATE7 enum.
            /// </summary>
            public UInt16 abgState;
            /// <summary>
            /// see also: SHUTTER_COMMAND enum.
            /// </summary>
            public UInt16 openShutter;
        };

        /*!
         * \brief Start Exposure command parameters Expanded
         *
         * Expanded parameters structure used to start SBIG camera exposures.
         */
        public struct StartExposureParams2
        {
            /// <summary>
            /// Requested CCD. see also: CCD_REQUEST enum.
            /// </summary>
            public UInt16 ccd;
            /// <summary>
            /// Exposure time in hundredths of a second in least significant 24 bits. Most significant bits are bit-flags described in exposureTime #define block.
            /// </summary>
            public UInt32 exposureTime;
            /// <summary>
            /// Deprecated. See also: ABG_STATE7.
            /// </summary>
            public UInt16 abgState;
            /// <summary>
            /// see also: SHUTTER_COMMAND enum.
            /// </summary>
            public UInt16 openShutter;
            /// <summary>
            /// readout mode. See also: READOUT_BINNING_MODE enum.
            /// </summary>
            public UInt16 readoutMode;
            /// <summary>
            /// top-most row to read out. (0 based)
            /// </summary>
            public UInt16 top;
            /// <summary>
            /// left-most column to read out. (0 based)
            /// </summary>
            public UInt16 left;
            /// <summary>
            /// image height in binned pixels.
            /// </summary>
            public UInt16 height;
            /// <summary>
            /// image width in binned pixels.
            /// </summary>
            public UInt16 width;
        };

        /*!
         * \brief End Exposure command parameters
         *
         * Parameters used to end SBIG camera exposures.
         * Set ABORT_DONT_END flag in ccd to abort exposures in supported cameras.
         */
        public struct EndExposureParams
        {
            /// <summary>
            /// Requested CCD. see also: CCD_REQUEST enum.
            /// </summary>
            public UInt16 ccd;
        };

        /*!
         * \brief Readout Line command parameters
         *
         * Parameters used to readout lines of SBIG cameras during readout.
         */
        public struct ReadoutLineParams
        {
            /// <summary>
            /// Requested CCD. see also: CCD_REQUEST enum.
            /// </summary>
            public UInt16 ccd;
            /// <summary>
            /// readout mode. See also: READOUT_BINNING_MODE enum.
            /// </summary>
            public UInt16 readoutMode;
            /// <summary>
            /// left-most pixel to read out.
            /// </summary>
            public UInt16 pixelStart;
            /// <summary>
            /// number of pixels to digitize.
            /// </summary>
            public UInt16 pixelLength;
        };

        /*!
         * \brief Dump Lines command parameters
         *
         * Parameters used to dump/flush CCD lines during readout.
         */
        public struct DumpLinesParams
        {
            /// <summary>
            /// Requested CCD. see also: CCD_REQUEST enum.
            /// </summary>
            public UInt16 ccd;
            /// <summary>
            /// readout mode. See also: READOUT_BINNING_MODE enum.
            /// </summary>
            public UInt16 readoutMode;
            /// <summary>
            /// number of lines to dump.
            /// </summary>
            public UInt16 lineLength;
        };

        /*!
         * \brief End Readout command parameters
         *
         * Parameters used to end SBIG camera readout.
         */
        public struct EndReadoutParams
        {
            /// <summary>
            /// Requested CCD. see also: CCD_REQUEST enum.
            /// </summary>
            public UInt16 ccd;
        };

        /*!
         * \brief Start Readout command parameters
         *
         * (Optional) Parameters used to start SBIG camera readout.
         * Automatically dumps unused exposure lines.
         */
        public struct StartReadoutParams
        {
            /// <summary>
            /// Requested CCD. see also: CCD_REQUEST enum.
            /// </summary>
            public UInt16 ccd;
            /// <summary>
            /// readout mode. See also: READOUT_BINNING_MODE enum.
            /// </summary>
            public UInt16 readoutMode;
            /// <summary>
            /// top-most row to read out. (0 based)
            /// </summary>
            public UInt16 top;
            /// <summary>
            /// left-most column to read out. (0 based)
            /// </summary>
            public UInt16 left;
            /// <summary>
            /// image height in binned pixels.
            /// </summary>
            public UInt16 height;
            /// <summary>
            /// image width in binned pixels.
            /// </summary>
            public UInt16 width;
        };

        /*!
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
            /// <summary>
            /// see also: TEMPERATURE_REGULATION enum.
            /// </summary>
            public UInt16 regulation;
            /// <summary>
            /// CCD temperature setpoint in A/D units if regulation on or TE drive level (0-255 = 0-100%) if regulation override.
            /// </summary>
            public UInt16 ccdSetpoint;
        };

        /*!
         * \brief Set Temperature Regulation command parameters Alternate
         *
         * The Set Temperature Regulation 2 command is used to enable or disable the CCD's temperature
         * regulation using temperatures in Degrees C instead of the funny A/D units described above.
         */
        public struct SetTemperatureRegulationParams2
        {
            /// <summary>
            /// see also: TEMPERATURE_REGULATION enum.
            /// </summary>
            public UInt16 regulation;
            /// <summary>
            /// CCD temperature setpoint in degrees Celsius.
            /// </summary>
            public double ccdSetpoint;
        };

        /*!
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
            /// <summary>
            /// see also: TEMP_STATUS_REQUEST enum.
            /// </summary>
            public UInt16 request;
        };

        /*!
         * \brief Query Temperature Status command results
         *
         * The results struct of a Temperature Status Query, with request set to TEMP_STATUS_STANDARD.
         */
        public struct QueryTemperatureStatusResults
        {
            /// <summary>
            /// temperature regulation is enabled when this is TRUE.
            /// </summary>
            public MY_LOGICAL enabled;
            /// <summary>
            /// CCD temperature or thermistor setpoint in A/D units.
            /// </summary>
            public UInt16 ccdSetpoint;
            /// <summary>
            /// this is the power being applied to the TE cooler to maintain temperature regulation and is in the range 0 thru 255.
            /// </summary>
            public UInt16 power;
            /// <summary>
            /// this is the CCD thermistor reading in A/D units.
            /// </summary>
            public UInt16 ccdThermistor;
            /// <summary>
            /// this is the ambient thermistor reading in A/D units.
            /// </summary>
            public UInt16 ambientThermistor;
        };

        /*!
         * \brief Query Temperature Status command results expanded
         *
         * The results struct of a Temperature Status Query, with request set to TEMP_STATUS_ADVANCED.
         */
        public struct QueryTemperatureStatusResults2
        {
            /// <summary>
            /// temperature regulation is enabled when this is TRUE. &REGULATION_FROZEN_MASK is TRUE when TE is frozen.
            /// </summary>
            public MY_LOGICAL coolingEnabled;
            /// <summary>
            /// fan state and is one of the following: FS_OFF (off), FS_ON (manual control) or FS_AUTOCONTROL (auto speed control).
            /// </summary>
            public MY_LOGICAL fanEnabled;
            /// <summary>
            /// CCD Setpoint temperature in °C.
            /// </summary>
            public double ccdSetpoint;
            /// <summary>
            /// imaging CCD temperature in degrees °C.
            /// </summary>
            public double imagingCCDTemperature;
            /// <summary>
            /// tracking CCD temperature in degrees °C.
            /// </summary>
            public double trackingCCDTemperature;
            /// <summary>
            /// external tracking CCD temperature in °C.
            /// </summary>
            public double externalTrackingCCDTemperature;
            /// <summary>
            /// ambient camera temperature in °C.
            /// </summary>
            public double ambientTemperature;
            /// <summary>
            /// percent power applied to the imaging CCD TE cooler.
            /// </summary>
            public double imagingCCDPower;
            /// <summary>
            /// percent power applied to the tracking CCD TE cooler.
            /// </summary>
            public double trackingCCDPower;
            /// <summary>
            /// percent power applied to the external tracking TE cooler.
            /// </summary>
            public double externalTrackingCCDPower;
            /// <summary>
            /// imaging CCD heatsink temperature in °C.
            /// </summary>
            public double heatsinkTemperature;
            /// <summary>
            /// percent power applied to the fan.
            /// </summary>
            public double fanPower;
            /// <summary>
            /// fan speed in RPM.
            /// </summary>
            public double fanSpeed;
            /// <summary>
            /// tracking CCD Setpoint temperature in °C.
            /// </summary>
            public double trackingCCDSetpoint;
        };

        /*!
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
            /// <summary>
            /// x plus activation duration in hundredths of a second
            /// </summary>
            public UInt16 tXPlus;
            /// <summary>
            /// x minus activation duration in hundredths of a second
            /// </summary>
            public UInt16 tXMinus;
            /// <summary>
            /// y plus activation duration in hundredths of a second
            /// </summary>
            public UInt16 tYPlus;
            /// <summary>
            /// y minus activation duration in hundredths of a second
            /// </summary>
            public UInt16 tYMinus;
        };

        /*!
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
            /// <summary>
            /// number of pulses to generate (0 thru 255).
            /// </summary>
            public UInt16 numberPulses;
            /// <summary>
            /// width of pulses in units of microseconds with a minimum of 9 microseconds.
            /// </summary>
            public UInt16 pulseWidth;
            /// <summary>
            /// period of pulses in units of microseconds with a minimum of 29 plus the pulseWidth microseconds.
            /// </summary>
            public UInt16 pulsePeriod;
        };

        /*!
         * \brief Transfer Serial Bytes command parameters
         *
         * The TX Serial Bytes command is for internal use by SBIG. It's a very low level version of commands
         * like AO Tip Tilt that are used to send data out the ST-7/8/etc's telescope port to accessories like the
         * AO-7. There're no reason why you should need to use this command. Just use the dedicated commands
         * like AO Tip Tilt.
         *
         */
        public struct TXSerialBytesParams
        {
            /// <summary>
            /// Length of data buffer to send
            /// </summary>
            public UInt16 dataLength;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
            public Byte[] data;    //!< Buffer of data to send.
        };

        /*!
         *  \brief Transfer Serial Bytes command results.
         * 
         * Results of a TXSerialBytes command.
         */
        public struct TXSerialBytesResults
        {
            /// <summary>
            /// Bytes sent out.
            /// </summary>
            public UInt16 bytesSent;
        };

        /*!
         *  \brief Get Serial Status command results.
         * 
         * The Get Serial Status command is for internal use by SBIG. It's a very low level version of commands
         * like AO Tip Tilt that are used to send data out the ST-7/8/etc's telescope port to accessories like the
         * AO-7. There're no reason why you should need to use this command. Just use the dedicated commands
         * like AO Tip Tilt.
         */
        public struct GetSerialStatusResults
        {
            public MY_LOGICAL clearToCOM;
        };

        /*!
         * \brief Establish Link command parameters
         *
         * The Establish Link command is used by the application to establish a communications link with the
         * camera. It should be used before any other commands are issued to the camera (excluding the Get
         * Driver Info command).
         */
        public struct EstablishLinkParams
        {
            /// <summary>
            /// Maintained for historical purposes. Keep set to 0.
            /// </summary>
            public UInt16 sbigUseOnly;
        };

        /*!
         * \brief Establish Link command results
         * 
         * Results from an EstablishLink command.
         */
        public struct EstablishLinkResults
        {
            /// <summary>
            /// Returns connected camera's type ID. See also: CAMERA_TYPE enum. 
            /// </summary>
            public UInt16 cameraType;
        };

        /*!
         * \brief Get Driver Info command parameters
         *
         * The Get Driver Info command is used to determine the version and capabilities of the DLL/Driver. For
         * future expandability this command allows you to request several types of information. Initially the
         * standard request and extended requests will be supported but as the driver evolves additional requests
         * will be added.
         */
        public struct GetDriverInfoParams
        {
            /// <summary>
            /// see also: DRIVER_REQUEST enum.
            /// </summary>
            public UInt16 request;
        };

        /*!
         * \brief Get Driver Info command Results 0
         * 
         * Standard, Extended and USB Loader Results Struct.
         */
        public struct GetDriverInfoResults0
        {
            /// <summary>
            /// driver version in BCD with the format XX.XX
            /// </summary>
            public UInt16 version;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            /// <summary>
            /// driver name, null terminated string
            /// </summary>
            public string name;
            /// <summary>
            /// maximum request response available from this driver
            /// </summary>
            public UInt16 maxRequest;
        };

        /*!
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
         * \brief Readout mode property struct.
         * 
         * Internal structure for storing readout modes.
         */
        public struct READOUT_INFO
        {
            /// <summary>
            /// readout mode ID (see also: READOUT_BINNING_MODE)
            /// </summary>
            public UInt16 mode;
            /// <summary>
            /// width of image in pixels
            /// </summary>
            public UInt16 width;
            /// <summary>
            /// height of image in pixels
            /// </summary>
            public UInt16 height;
            /// <summary>
            /// a four digit BCD number specifying the amplifier gain in e-/ADU in XX.XX format
            /// </summary>
            public UInt16 gain;
            /// <summary>
            /// an eight digit BCD number specifying the pixel width in microns in the XXXXXX.XX format
            /// </summary>
            public UInt32 pixel_width;
            /// <summary>
            /// an eight digit BCD number specifying the pixel height in microns in the XXXXXX.XX format
            /// </summary>
            public UInt32 pixel_height;
        };

        /*!
         * \brief Get CCD Info command results
         *
         * Get CCD Info command results 0 and 1 request.
         */
        public struct GetCCDInfoResults0
        {
            /// <summary>
            /// version of the firmware in the resident microcontroller in BCD format (XX.XX, 0x1234 = 12.34).
            /// </summary>
            public UInt16 firmwareVersion;
            /// <summary>
            /// Camera type ID. see also: CAMERA_TYPE enum.
            /// </summary>
            public UInt16 cameraType;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            /// <summary>
            /// null terminated string containing the name of the camera.
            /// </summary>
            public string name;
            /// <summary>
            /// number of readout modes supported.
            /// </summary>
            public UInt16 readoutModes;
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 20)]
            public READOUT_INFO[] readoutInfo;
        };

        /*!
         * \brief Get CCD Info command results pass 2
         *
         * Get CCD Info command results second request.
         */
        public struct GetCCDInfoResults2
        {
            /// <summary>
            /// number of bad columns in imaging CCD
            /// </summary>
            public UInt16 badColumns;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public UInt16[] columns;  //!< bad columns
                                      /// <summary>
                                      /// type of Imaging CCD, 0= No ABG Protection, 1 = ABG Present. see also: IMAGING_ABG enum.
                                      /// </summary>
            public UInt16 imagingABG;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            /// <summary>
            /// null terminated serial number string
            /// </summary>
            public string serialNumber;
        };

        /*!
         * \brief Get CCD Info command results pass 3
         *
         * Get CCD Info command results third request. (For the PixCel255/237)
         */
        public struct GetCCDInfoResults3
        {
            /// <summary>
            /// 0 = Unknown, 1 = 12 bits, 2 = 16 bits. see also: AD_SIZE enum.
            /// </summary>
            public UInt16 adSize;
            /// <summary>
            /// 0 = Unknown, 1 = External, 2 = 2 Position, 3 = 5 Position. see also: FILTER_TYPE enum. 
            /// </summary>
            public UInt16 filterType;
        };

        /*!
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
            /// <summary>
            /// Camera capabilities. See the CB_XXX_XXX definitions in the sbigurdv.h header file.
            /// </summary>
            public UInt16 capabilitiesBits;
            /// <summary>
            /// Number of unbinned rows to dump to transfer image area to storage area.
            /// </summary>
            public UInt16 dumpExtra;
        };

        /*!
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
            /// <summary>
            /// Set of bits for additional camera capabilities
            /// </summary>
            public UInt32 cameraBits;
            /// <summary>
            /// Set of bits for additional CCD capabilities
            /// </summary>
            public UInt32 ccdBits;
            /// <summary>
            /// Set of bits for additional capabilities
            /// </summary>
            public UInt32 extraBits;
        };

        /*!
         * \brief Query Command Status command parameters
         *
         * The Query Command Status command is used to monitor the progress of a previously requested
         * command. Typically this will be used to monitor the progress of an exposure, relay closure or CFW-6A
         * move command.
         */
        public struct QueryCommandStatusParams
        {
            /// <summary>
            /// command of which the status is desired
            /// </summary>
            public UInt16 command;
        };

        /*!
         * \brief Query Command Status command results 
         *
         * Results for the Query Command Status command.
         */
        public struct QueryCommandStatusResults
        {
            /// <summary>
            /// command status.
            /// </summary>
            public UInt16 status;
        };

        /*!
         * \brief Query Command Status command results 
         *
         * Results for the Query Command Status command.
         */
        public struct QueryCommandStatusResults2
        {
            /// <summary>
            /// command status. 
            /// </summary>
            public UInt16 status;
            /// <summary>
            /// expanded information on command status.
            /// </summary>
            public UInt16 info;
        };

        /*!
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
            /// <summary>
            /// set TRUE to turn on the Fan.
            /// </summary>
            public MY_LOGICAL fanEnable;
            /// <summary>
            /// see also: SHUTTER_COMMAND enum.
            /// </summary>
            public UInt16 shutterCommand;
            /// <summary>
            /// see also: LED_STATE enum.
            /// </summary>
            public UInt16 ledState;
        };

        /*!
         * \brief Read Offset command parameters
         *
         * The Read Offset command is used to measure the CCD's offset. In the SBIG cameras the offset is 
         * adjusted at the factory and this command is for testing or informational purposes only.
         */
        public struct ReadOffsetParams
        {
            /// <summary>
            /// see also: CCD_REQUEST enum.
            /// </summary>
            public UInt16 ccd;
        };

        /*!
         * \brief Read Offset command results
         *
         * Results structure for the Read Offset command.
         */
        public struct ReadOffsetResults
        {
            /// <summary>
            /// the CCD's offset.
            /// </summary>
            public UInt16 offset;
        };

        /*!
         * \brief Read Offset command results expanded
         *
         * The Read Offset 2 command is used to measure the CCD's offset and the noise in the readout register.
         * In the SBIG cameras the offset is adjusted at the factory and this command is for testing or informational
         * purposes only.
         */
        public struct ReadOffsetResults2
        {
            /// <summary>
            /// the CCD's offset.
            /// </summary>
            public UInt16 offset;
            /// <summary>
            /// noise in the ccd readout register in ADUs rms.
            /// </summary>
            public double rms;
        };

        /*!
         * \brief AO Tip/Tilt command parameters
         *
         * The AO Tip Tilt Command is used to position an AO-7 attached to the telescope port of an ST-7/8/etc.
         */
        public struct AOTipTiltParams
        {
            /// <summary>
            /// this is the desired position of the mirror in the X axis.
            /// </summary>
            public UInt16 xDeflection;
            /// <summary>
            /// this is the desired position of the mirror in the Y axis
            /// </summary>
            public UInt16 yDeflection;
        };

        /*!
         * \brief AO Set Focus command parameters
         *
         * This command is reserved for future use with motorized focus units. Prototypes of the AO-7 had
         * motorized focus but the feature was removed in the production units. This command is a holdover from
         * that.
         */
        public struct AOSetFocusParams
        {
            /// <summary>
            /// see also: AO_FOCUS_COMMAND enum.
            /// </summary>
            public UInt16 focusCommand;
        };

        /*!
         * \brief AO Delay command parameters
         *
         * The AO Delay Command is used to generate millisecond type delays for exposing the Tracking CCD.
         * This sleep command is blocking.
         */
        public struct AODelayParams
        {
            /// <summary>
            /// this is the desired delay in microseconds.
            /// </summary>
            public UInt32 delay;
        };

        /*!
         * \brief Get Turbo Status command results
         *
         * The current driver does not use this command. It was added in a previous version and never removed. It
         * could be reassigned in the future.
         */
        public struct GetTurboStatusResults
        {
            /// <summary>
            /// TRUE if turbo is detected.
            /// </summary>
            public MY_LOGICAL turboDetected;
        };

        /*!
         * \brief Open Device command parameters
         *
         * The Open Device command is used to load and initialize the low-level driver. You will typically call
         * this second (after Open Driver).
         */
        public struct OpenDeviceParams
        {
            /// <summary>
            /// see also: SBIG_DEVICE_TYPE enum. specifies LPT, Ethernet, etc.
            /// </summary>
            public UInt16 deviceType;
            /// <summary>
            /// for deviceType::DEV_LPTN: Windows 9x Only, Win NT uses deviceSelect.
            /// </summary>
            public UInt16 lptBaseAddress;
            /// <summary>
            /// for deviceType::DEV_ETH:  Ethernet address.
            /// </summary>
            public UInt32 ipAddress;
        };

        /*!
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
            /// <summary>
            /// IRQ Level.
            /// </summary>
            public UInt16 level;
        };

        /*!
         * \brief Get IRQ Level command results
         *
         * Results of Get IRQ Level command.
         */
        public struct GetIRQLResults
        {
            /// <summary>
            /// IRQ Level.
            /// </summary>
            public UInt16 level;
        };

        /*!
         * \brief Get Link Status command results
         *
         * This command returns the status of the communications link established with the camera.
         */
        public struct GetLinkStatusResults
        {
            /// <summary>
            /// TRUE when a link has been established
            /// </summary>
            public MY_LOGICAL linkEstablished;
            /// <summary>
            /// base address of the LPT port.
            /// </summary>
            public UInt16 baseAddress;
            /// <summary>
            /// see also: CAMERA_TYPE enum.
            /// </summary>
            public UInt16 cameraType;
            /// <summary>
            /// total number of communications with camera.
            /// </summary>
            public UInt32 comTotal;
            /// <summary>
            /// total number of failed communications with camera.
            /// </summary>
            public UInt32 comFailed;
        };

        /*!
         * \brief Get Microsecond Timer command results
         *
         * This command is of extremely limited (and unknown) use. When you have established a link to a
         * parallel port based camera under Windows NT/2000/XP this command returns a counter with 1
         * microsecond resolution. Under all other circumstances the counter is zero.
         */
        public struct GetUSTimerResults
        {
            /// <summary>
            /// counter value in microseconds.
            /// </summary>
            public UInt32 count;
        };

        /*!
         * \brief Send Block command parameters
         * \internal
         *
         * Intended for SBIG internal use only. Unimplemented.
         */
        public struct SendBlockParams
        {
            /// <summary>
            /// Destination port.
            /// </summary>
            public UInt16 port;
            /// <summary>
            /// Length of data buffer.
            /// </summary>
            public UInt16 length;
            //TODO: unsigned char* 檢查移植是否正確
            /// <summary>
            /// Buffer of data to send.
            /// </summary>
            public UIntPtr source;
        };

        /*!
         * \brief Send Byte command parameters
         * \internal
         *
         * Intended for SBIG internal use only. Unimplemented.
         */
        public struct SendByteParams
        {
            /// <summary>
            /// Destination port.
            /// </summary>
            public UInt16 port;
            /// <summary>
            /// Buffer of data to send.
            /// </summary>
            public UInt16 data;
        };

        /*!
         * \brief Clock A/D command parameters
         * \internal
         *
         * Intended for SBIG internal use only. Clock the AD the number of times passed.
         */
        public struct ClockADParams
        {
            /// <summary>
            /// CCD to clock. see also: CCD_REQUEST enum. (Unused)
            /// </summary>
            public UInt16 ccd;
            /// <summary>
            /// Readout mode. see also: READOUT_BINNING_MODE enum. (Unused)
            /// </summary>
            public UInt16 readoutMode;
            /// <summary>
            /// Starting pixel. (Unused)
            /// </summary>
            public UInt16 pixelStart;
            /// <summary>
            /// Count of cycles to pass.
            /// </summary>
            public UInt16 pixelLength;
        };

        /*!
         * \brief System Test command parameters
         * \internal
         *
         * Intended for SBIG internal use only. Pass the SystemTest command to the micro.
         */
        public struct SystemTestParams
        {
            /// <summary>
            /// Flag TRUE to test the clocks.
            /// </summary>
            public UInt16 testClocks;
            /// <summary>
            /// Flag TRUE to test the motors.
            /// </summary>
            public UInt16 testMotor;
            /// <summary>
            /// Flag TRUE to test 5800 (???).
            /// </summary>
            public UInt16 test5800;
            /// <summary>
            /// Flag true to align STL (???).
            /// </summary>
            public UInt16 stlAlign;
            /// <summary>
            /// Flag true for motor always on (???).
            /// </summary>
            public UInt16 motorAlwaysOn;
        };

        /*!
         * \brief Send STV Block command parameters
         * \internal
         *
         * Intended for SBIG internal use only. Unused.
         */
        public struct SendSTVBlockParams
        {
            /// <summary>
            /// Outgoing buffer length.
            /// </summary>
            public UInt16 outLength;
            //TODO: unsigned char* 檢查移植是否正確
            /// <summary>
            /// Outgoing buffer.
            /// </summary>
            public UIntPtr outPtr;
            /// <summary>
            /// Incoming buffer length.
            /// </summary>
            public UInt16 inLength;
            //TODO: unsigned char* 檢查移植是否正確
            /// <summary>
            /// Incoming buffer.
            /// </summary>
            public UIntPtr inPtr;
        };

        /*!
         * \brief Get Error String command parameters
         *
         * This command returns a null terminated C string in English (not Unicode) corresponding to the passed
         * error number. It's handy for reporting driver level errors to the user.
         */
        public struct GetErrorStringParams
        {
            /// <summary>
            /// Error code. see also: PAR_ERROR enum.
            /// </summary>
            public UInt16 errorNo;
        };

        /*!
         * \brief Get Error String command results
         *
         * This command returns a null terminated C string in English (not Unicode) corresponding to the passed
         * error number. It's handy for reporting driver level errors to the user.
         */
        public struct GetErrorStringResults
        {
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            /// <summary>
            /// Error string in english (not unicode).
            /// </summary>
            public string errorString;
        };

        /*!
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
            /// <summary>
            /// Handle to driver.
            /// </summary>
            public Int16 handle;
        };

        /*!
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
            /// <summary>
            /// Handle to driver.
            /// </summary>
            public Int16 handle;
        };

        /*!
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
            /// <summary>
            /// the parameter to modify. see also: DRIVER_CONTROL_PARAM enum.
            /// </summary>
            public UInt16 controlParameter;
            /// <summary>
            /// the value of the control parameter.
            /// </summary>
            public UInt32 controlValue;
        };

        /*!
         * \brief Get Driver Control command parameters
         *
         * Requests the value of a driver control parameter.
         */
        public struct GetDriverControlParams
        {
            /// <summary>
            /// the driver parameter to be retrieved. see also: DRIVER_CONTROL_PARAM enum.
            /// </summary>
            public UInt16 controlParameter;
        };

        /*!
         * \brief Get Driver Control command results
         *
         * Returns the value of a driver control parameter.
         */
        public struct GetDriverControlResults
        {
            /// <summary>
            /// The value of the requested driver parameter. see also: DRIVER_CONTROL_PARAM enum.
            /// </summary>
            public UInt32 controlValue;
        };

        /*!
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
            /// <summary>
            /// Imaging/Tracking Gain or offset. see also: USB_AD_CONTROL_COMMAND enum.
            /// </summary>
            public UInt16 command;
            /// <summary>
            /// Command specific.
            /// </summary>
            public Int16 data;
        };

        /*!
         * \brief Information from queried USB device.
         * 
         * Results for a single USB query.
         */
        public struct QUERY_USB_INFO
        {
            /// <summary>
            /// TRUE if a camera was found.
            /// </summary>
            public MY_LOGICAL cameraFound;
            /// <summary>
            /// Camera type found. see also: CAMERA_TYPE enum.
            /// </summary>
            public UInt16 cameraType;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            /// <summary>
            /// null terminated string. Name of found camera.
            /// </summary>
            public string name;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            /// <summary>
            /// null terminated string. Serial number of found camera.
            /// </summary>
            public string serialNumber;
        };

        /*!
         * \brief Query USB command results 
         * 
         * Returns a list of up to four cameras found by the driver via USB.
         */
        public struct QueryUSBResults
        {
            /// <summary>
            /// Number of cameras found. (Max 4)
            /// </summary>
            public UInt16 camerasFound;
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 4)]
            public QUERY_USB_INFO[] usbInfo;      //!< Information returned by cameras.
        };

        /*!
         * \brief Query USB command results extended
         * 
         * Returns a list of up to eight cameras found by the driver via USB.
         */
        public struct QueryUSBResults2
        {
            /// <summary>
            /// Number of cameras found. (Max 8)
            /// </summary>
            public UInt16 camerasFound;
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 8)]
            public QUERY_USB_INFO[] usbInfo;      //!< Information returned by cameras.
        };

        public struct QueryUSBResults3
        {
            /// <summary>
            /// Number of cameras found. (Max 24)
            /// </summary>
            public UInt16 camerasFound;
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 24)]
            public QUERY_USB_INFO[] usbInfo;     //<! Information returned by cameras.
        };

        /*!
         * \brief Query Ethernet device results
         * 
         * Returned information for a single device over Ethernet.
         */
        public struct QUERY_ETHERNET_INFO
        {
            /// <summary>
            /// TRUE if a camera was found.
            /// </summary>
            public MY_LOGICAL cameraFound;
            /// <summary>
            /// IP address of camera found.
            /// </summary>
            public UInt32 ipAddress;
            /// <summary>
            /// Camera type found. see also: CAMERA_TYPE enum.
            /// </summary>
            public UInt16 cameraType;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            /// <summary>
            /// null terminated string. Name of found camera.
            /// </summary>
            public string name;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            /// <summary>
            /// null terminated string. Serial number of found camera. 
            /// </summary>
            public string serialNumber;
        };

        /*!
         * \brief Query Ethernet command results
         * 
         * Returns a list of up to eight cameras found by the driver via Ethernet.
         */
        public struct QueryEthernetResults
        {
            /// <summary>
            /// Number of cameras found.
            /// </summary>
            public UInt16 camerasFound;
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 4)]
            public QUERY_ETHERNET_INFO[] ethernetInfo;    //!< Information of found devices.
        };

        /*!
         * \brief Query Ethernet command results extended
         * 
         * Returns a list of up to eight cameras found by the driver via Ethernet.
         */
        public struct QueryEthernetResults2
        {
            /// <summary>
            /// Number of cameras found.
            /// </summary>
            public UInt16 camerasFound;
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 8)]
            public QUERY_ETHERNET_INFO[] ethernetInfo;    //!< Information of found devices.
        };

        /*!
         * \brief Get Pentium Cycle Count command parameters
         * 
         * This command is used to read a Pentium processor's internal cycle counter. Pentium processors have a
         * 32 or 64 bit register that increments every clock cycle. For example on a 1 GHz Pentium the counter
         * advances 1 billion counts per second. This command can be used to retrieve that counter.
         */
        public struct GetPentiumCycleCountParams
        {
            /// <summary>
            /// number of bits to shift the results to the right (dividing by 2)
            /// </summary>
            public UInt16 rightShift;
        };

        /*!
         * \brief Get Pentium Cycle Count command results
         * 
         * This command is used to read a Pentium processor's internal cycle counter. Pentium processors have a
         * 32 or 64 bit register that increments every clock cycle. For example on a 1 GHz Pentium the counter
         * advances 1 billion counts per second. This command can be used to retrieve that counter.
         */
        public struct GetPentiumCycleCountResults
        {
            /// <summary>
            /// lower 32 bits of the Pentium cycle counter
            /// </summary>
            public UInt32 countLow;
            /// <summary>
            /// upper 32 bits of the Pentium cycle counter
            /// </summary>
            public UInt32 countHigh;
        };

        /*!
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
            /// <summary>
            /// Address to read from or write to
            /// </summary>
            public Byte address;
            /// <summary>
            /// Data to write to the external I2C device, ignored for read
            /// </summary>
            public Byte data;
            /// <summary>
            /// TRUE when write is desired , FALSE when read is desired
            /// </summary>
            public MY_LOGICAL write;
            /// <summary>
            /// Device Address of the I2C peripheral
            /// </summary>
            public Byte deviceAddress;
        };

        /*!
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
            /// <summary>
            /// Data read from the external I2C device
            /// </summary>
            public Byte data;
        };

        /*!
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
            /// <summary>
            /// see also: CFW_MODEL_SELECT enum.
            /// </summary>
            public UInt16 cfwModel;
            /// <summary>
            /// see also: CFW_COMMAND enum.
            /// </summary>
            public UInt16 cfwCommand;
            /// <summary>
            /// command specific
            /// </summary>
            public UInt32 cfwParam1;
            /// <summary>
            /// command specific
            /// </summary>
            public UInt32 cfwParam2;
            /// <summary>
            /// command specific
            /// </summary>
            public UInt16 outLength;
            //TODO: unsigned char* 檢查移植是否正確
            /// <summary>
            /// command specific
            /// </summary>
            public UIntPtr outPtr;
            /// <summary>
            /// command specific
            /// </summary>
            public UInt16 inLength;
            //TODO: unsigned char* 檢查移植是否正確
            /// <summary>
            /// command specific
            /// </summary>
            public UIntPtr inPtr;
        };

        /*!
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
            /// <summary>
            /// see also: CFW_MODEL_SELECT enum.
            /// </summary>
            public UInt16 cfwModel;
            /// <summary>
            /// see also: CFW_POSITION enum.
            /// </summary>
            public UInt16 cfwPosition;
            /// <summary>
            /// see also: CFW_STATUS enum.
            /// </summary>
            public UInt16 cfwStatus;
            /// <summary>
            /// see also: CFW_ERROR enum.
            /// </summary>
            public UInt16 cfwError;
            /// <summary>
            /// command specific
            /// </summary>
            public UInt32 cfwResult1;
            /// <summary>
            /// command specific
            /// </summary>
            public UInt32 cfwResult2;
        };

        /*!
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
            /// <summary>
            /// 0=Write, 1=Read. see also: BITIO_OPERATION enum.
            /// </summary>
            public UInt16 bitOperation;
            /// <summary>
            /// see also: BITIO_NAME enum.
            /// </summary>
            public UInt16 bitName;
            /// <summary>
            /// 1=Set Bit, 0=Clear Bit
            /// </summary>
            public MY_LOGICAL setBit;
        };

        /*!
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
            /// <summary>
            /// 1=Bit is set, 0=Bit is clear
            /// </summary>
            public MY_LOGICAL bitIsSet;
        };

        /*!
         * \brief User EEPROM command parameters
         * 
         * Read or write a block of data the user space in the EEPROM.
         */

        /*!
         * \brief User EEPROM command results
         * 
         * Read or write a block of data the user space in the EEPROM.
         */
        public struct UserEEPROMResults
        {
            /// <summary>
            /// TRUE to write data to user EEPROM space, FALSE to read.
            /// </summary>
            public MY_LOGICAL writeData;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
            public Byte[] data;     //!< Buffer of data to be written.
        };

        /*!
         * \brief Column EEPROM command parameters and results
         * \internal
         * 
         * Internal SBIG use only. This command is used read or write the STF-8300's Column Repair data stored in the camera for use
         * with that camera's Auto Filter Capability.
         *
         * * The left most column is column 1 (not zero). Specifying a column zero doesn filter any
         *   columns.
         * * This command is somewhat unique in that the Parameters and the Results are the same struct.
         * * To enable column filtering you must use this command and also the Set Driver Control command
         *   to set the DCP_COLUMN_REPAIR parameter to 1.
         */
        public struct ColumnEEPROMResults
        {
            /// <summary>
            /// TRUE to write data to specified EEPROM column, FALSE to read.
            /// </summary>
            public MY_LOGICAL writeData;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public UInt16[] columns;  //!< Specify up to 7 columns to repair.
                                      /// <summary>
                                      /// not used at this time.
                                      /// </summary>
            public UInt16 flags;
        };

        /*!
         * \brief Biorad TDI Setup command parameters
         * 
         * Send the Biorad setup to the camera, returning any error.
         */
        public struct BTDISetupParams
        {
            /// <summary>
            /// Row period.
            /// </summary>
            public Byte rowPeriod;
        };

        /*!
         * \brief Biorad TDI Setup command results
         * 
         * Results of the Biorad setup, returning any error.
         */
        public struct BTDISetupResults
        {
            /// <summary>
            /// Results of the command. see also: BTDI_ERROR enum.
            /// </summary>
            public Byte btdiErrors;
        };

        /*!
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
            /// <summary>
            /// see also: MF_MODEL_SELECT enum.
            /// </summary>
            public UInt16 mfModel;
            /// <summary>
            /// see also: MF_COMMAND enum.
            /// </summary>
            public UInt16 mfCommand;
            /// <summary>
            /// command specific.
            /// </summary>
            public Int32 mfParam1;
            /// <summary>
            /// command specific.
            /// </summary>
            public Int32 mfParam2;
            /// <summary>
            /// command specific.
            /// </summary>
            public UInt16 outLength;
            //TODO: unsigned char* 檢查移植是否正確
            /// <summary>
            /// command specific.
            /// </summary>
            public UIntPtr outPtr;
            /// <summary>
            /// command specific.
            /// </summary>
            public UInt16 inLength;
            //TODO: unsigned char* 檢查移植是否正確
            /// <summary>
            /// command specific.
            /// </summary>
            public UIntPtr inPtr;
        };

        /*!
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
            /// <summary>
            /// see also: MF_MODEL_SELECT enum. 
            /// </summary>
            public UInt16 mfModel;
            /// <summary>
            /// position of the Motor Focus, 0=Center, signed.
            /// </summary>
            public Int32 mfPosition;
            /// <summary>
            /// see also: MF_STATUS enum.
            /// </summary>
            public UInt16 mfStatus;
            /// <summary>
            /// see also: MF_ERROR  enum.
            /// </summary>
            public UInt16 mfError;
            /// <summary>
            /// command specific.
            /// </summary>
            public Int32 mfResult1;
            /// <summary>
            /// command specific.
            /// </summary>
            public Int32 mfResult2;
        };

        /*!
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
            /// <summary>
            /// Command for Differential Guider. see also: DIFF_GUIDER_COMMAND enum. 
            /// </summary>
            public UInt16 diffGuiderCommand;
            /// <summary>
            /// Unused.
            /// </summary>
            public UInt16 spareShort;
            /// <summary>
            /// Unused.
            /// </summary>
            public UInt32 diffGuiderParam1;
            /// <summary>
            /// Unused.
            /// </summary>
            public UInt32 diffGuiderParam2;
            /// <summary>
            /// Size of output buffer. Command specific.
            /// </summary>
            public UInt16 outLength;
            //TODO: unsigned char* 檢查移植是否正確
            /// <summary>
            /// output buffer. Command specific.
            /// </summary>
            public UIntPtr outPtr;
            /// <summary>
            /// Size of input buffer. Command specific.
            /// </summary>
            public UInt16 inLength;
            //TODO: unsigned char* 檢查移植是否正確
            /// <summary>
            /// input buffer. Command specific.
            /// </summary>
            public UIntPtr inPtr;
        };

        /*!
         * \brief Differential Guider command results
         * 
         * Returned results of a Differential Guider Command.
         */
        public struct DiffGuiderResults
        {
            /// <summary>
            /// see also: DIFF_GUIDER_ERROR enum.
            /// </summary>
            public UInt16 diffGuiderError;
            /// <summary>
            /// see also: DIFF_GUIDER_STATUS enum.
            /// </summary>
            public UInt16 diffGuiderStatus;
            /// <summary>
            /// Unused.
            /// </summary>
            public UInt32 diffGuiderResult1;
            /// <summary>
            /// Unused.
            /// </summary>
            public UInt32 diffGuiderResult2;
        };

        /*!
         * \brief Differential Guider LED state
         * 
         * State of the Differential Guider LEDs.
         */
        public struct DGLEDState
        {
            /// <summary>
            /// TRUE if Red LED is on, FALSE otherwise.
            /// </summary>
            public UInt16 bRedEnable;
            /// <summary>
            /// TRUE if IR LED is on, FALSE otherwise.
            /// </summary>
            public UInt16 bIREnable;
            /// <summary>
            /// brightness setting of Red LED from 0x00 to 0xFF.
            /// </summary>
            public UInt16 nRedBrightness;
            /// <summary>
            /// brightness setting of IR LED from 0x00 to 0xFF.
            /// </summary>
            public UInt16 nIRBrightness;
        };

        /*!
         * \brief Bulk I/O command parameters
         * \internal
         * 
         * Internal SBIG use only. Implement the Bulk IO command which is used for Bulk Reads/Writes to the camera for diagnostic purposes.
         */
        public struct BulkIOParams
        {
            /// <summary>
            /// see also: BULK_IO_COMMAND enum.
            /// </summary>
            public UInt16 command;
            /// <summary>
            /// TRUE if reading/writing data to/from the Pixel pipe, FALSE to read/write from the com pipe.
            /// </summary>
            public MY_LOGICAL isPixelData;
            /// <summary>
            /// Length of data buffer.
            /// </summary>
            public UInt32 dataLength;
            //TODO: char* 檢查移植是否正確
            /// <summary>
            /// data buffer.
            /// </summary>
            public IntPtr dataPtr;
        };

        /*!
         * \brief Bulk I/O command results
         * \internal
         * 
         * Internal SBIG use only. Results of a Bulk I/O command.
         */
        public struct BulkIOResults
        {
            /// <summary>
            /// Bytes sent/received.
            /// </summary>
            public UInt32 dataLength;
        };

        /*!
         * \brief Customer Options command parameters
         * 
         * This command is used read or write the STX/STXL/STT's customer options.
         */

        /*!
         * \brief Customer Options command results
         * 
         * This command is used read or write the STX/STXL/STT's customer options.
         */
        public struct CustomerOptionsResults
        {
            /// <summary>
            /// TRUE/FALSE = set/get options respectively.
            /// </summary>
            public MY_LOGICAL bSetCustomerOptions;
            /// <summary>
            /// TRUE to include Overscan region in images.
            /// </summary>
            public MY_LOGICAL bOverscanRegions;
            /// <summary>
            /// TRUE to turn on window heater.
            /// </summary>
            public MY_LOGICAL bWindowHeater;
            /// <summary>
            /// TRUE to preflash CCD.
            /// </summary>
            public MY_LOGICAL bPreflashCcd;
            /// <summary>
            /// TRUE to turn VDD off.
            /// </summary>
            public MY_LOGICAL bVddNormallyOff;
        };

        /*!
         * \brief Get I2C AO Model command results
         * 
         * Results of a CC_GET_AO_MODEL command.
         */
        public struct GetI2CAoModelResults
        {
            /// <summary>
            /// AO model.
            /// </summary>
            public UInt16 i2cAoModel;
        };

        /*!
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
         * \brief Debug log command parameters.
         *
         * Change debug logging, and path to log file.
         */
        public struct DebugLogParams
        {
            /// <summary>
            /// Command flags.
            /// </summary>
            public UInt16 ccFlags;
            /// <summary>
            /// Microcommand flags.
            /// </summary>
            public UInt16 mcFlags;
            /// <summary>
            /// Communication flags.
            /// </summary>
            public UInt16 fceFlags;
            /// <summary>
            /// I/O flags.
            /// </summary>
            public UInt16 ioFlags;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
            /// <summary>
            /// Path to SBIGUDRV log file.
            /// </summary>
            public string logFilePathName;
        };

        public struct GetReadoutInProgressResults
        {
            /// <summary>
            /// Readout In Progress. TRUE if RIP, FALSE otherwise.
            /// </summary>
            public MY_LOGICAL RIP;
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
         * \brief QueryFeatureSupportedParams
         */
        public struct QueryFeatureSupportedParams
        {
            /// <summary>
            /// Feature to query for firmware support. 
            /// </summary>
            public FeatureFirmwareRequirement ffr;
        };

        /*!
         * \brief QueryFeatureSupportedResult
         */
        public struct QueryFeatureSupportedResults
        {
            /// <summary>
            /// TRUE if feature is supported, FALSE otherwise.
            /// </summary>
            public MY_LOGICAL result;
        };

        /*!
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

            /// <summary>
            /// Start exposure tick initial value.
            /// </summary>
            public LARGE_INTEGER startExposureTicks0;
            /// <summary>
            /// Start exposure tick final value.
            /// </summary>
            public LARGE_INTEGER startExposureTicks1;
            /// <summary>
            /// End exposure tick initial value.
            /// </summary>
            public LARGE_INTEGER endExposureTicks0;
            /// <summary>
            /// End exposure tick final value.
            /// </summary>
            public LARGE_INTEGER endExposureTicks1;
        };

        /*!
         * SBIGUnivDrvCommand()
         *  \brief Command function: Supports Parallel, USB and Ethernet based cameras
         * \param command PAR_COMMAND integer 
         * \param Params pointer to a command-specific structure containing the relevant command parameters.
         * \param pResults pointer to a comand-specific results structure containing the results of the command.
         *
         * The master API hook for the SBIG Universal Driver dll. The calling program needs to allocate the memory 
         *  for the parameters and results structs and these routines read them and fill them in respectively.
         */
        [DllImport("SBIGUDrv.dll", CallingConvention = CallingConvention.StdCall)]
        private static extern PAR_ERROR SBIGUnivDrvCommand(
            PAR_COMMAND command, IntPtr Params, IntPtr pResults);

        /*!
         * SBIGLogDebugMsg()
         *  \brief Command function: Supports Parallel, USB and Ethernet based cameras
         * \param pStr pointer to an array of characters, null-terminated, which should be written to the log file.
         * \param length unsigned int of buffer's length in bytes.
         * \internal
         * 
         * A function used to expose writing to the log file to calling programs. Useful for debugging purposes.
         */
        //TODO: 檢查移植是否正確 extern "C" Int16 __stdcall SBIGLogDebugMsg(char* pStr, UInt16 length);
        [DllImport("SBIGUDrv.dll", CallingConvention = CallingConvention.StdCall)]
        private static extern Int16 SBIGLogDebugMsg(UIntPtr pStr, UInt16 length);

    } // class
} // namespace
