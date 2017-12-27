using System;
using System.Drawing;
using System.IO;
using System.Net;
using System.Runtime.InteropServices;
using System.Text;

namespace SbigSharp
{
    public struct SBIG
    {
        //
        #region Constants
        //

        private static GCHandle NullGch = new GCHandle();

        #endregion

        //
        #region Enums
        //

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

        } // enum PAR_COMMAND

        /// <summary>
        /// Base value for all error IDs.
        /// </summary>
        public const ushort CE_ERROR_BASE = 1;
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
            CE_NEXT_ERROR							/*!< Development purposes: Next Error			*/

        } // enum PAR_ERROR

        public enum CommandStatus : ushort
        {
            CS_IDLE,                    /*!< Camera state: Idle.				*/
            CS_IN_PROGRESS,             /*!< Camera state: Exposure in progress */
            CS_INTEGRATING,             /*!< Camera state: Integrating			*/
            CS_INTEGRATION_COMPLETE     /*!< Camera state: Integration complete */
        }

        public enum FeatureFirmwareRequirement : ushort
        {
            FFR_CTRL_OFFSET_CORRECTION,
            FFR_CTRL_EXT_SHUTTER_ONLY,
            FFR_ASYNC_TRIGGER_IN,
            FFR_LAST
        }

        /// <summary>
        /// Pulse in is currently active state modifier flag.
        /// </summary>
        public const ushort CS_PULSE_IN_ACTIVE = 0x8000;

        /// <summary>
        /// Waiting for trigger state modifier flag
        /// </summary>
        public const ushort CS_WAITING_FOR_TRIGGER = 0x8000;

        public const ushort RBI_PREFLASH_LENGTH_MASK = 0x0FFF;
        public const ushort RBI_PREFLASH_FLUSH_MASK = 0xF000;
        public const ushort RBI_PREFLASH_FLUSH_BIT = 0x0C;

        public enum QueryTemperatureStatusRequest : ushort
        {
            TEMP_STATUS_STANDARD,   /*!< Temperature status Standard		*/
            TEMP_STATUS_ADVANCED,   /*!< Temperature status Advanced		*/
            TEMP_STATUS_ADVANCED2   /*!< Temperature status Advanced 2		*/
        }

        public enum AbgState7 : ushort
        {
            ABG_LOW7,       /*!< ABG Low 7			*/
            ABG_CLK_LOW7,   /*!< ABG Clock Low 7	*/
            ABG_CLK_MED7,   /*!< ABG Clock Medium 7	*/
            ABG_CLK_HI7		/*!< ABG Clock High 7	*/
        }

        public enum DriverRequest : ushort
        {
            DRIVER_STD,         /*!< Driver standard	*/
            DRIVER_EXTENDED,    /*!< Driver extended	*/
            DRIVER_USB_LOADER   /*!< Driver USB loader	*/
        }

        public enum CCD_Request : ushort
        {
            CCD_IMAGING,        /*!< Request Imaging CCD			*/
            CCD_TRACKING,       /*!< Request Internal Tracking CCD	*/
            CCD_EXT_TRACKING	/*!< Request External Tracking CCD	*/
        }

        public enum ReadoutBinningMode : ushort
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
        }

        public enum CcdInfoRequest : ushort
        {
            CCD_INFO_IMAGING,               /*!< Imaging CCD Info				*/
            CCD_INFO_TRACKING,              /*!< Tracking CCD Info				*/
            CCD_INFO_EXTENDED,              /*!< Extended CCD Info				*/
            CCD_INFO_EXTENDED_5C,           /*!< Extended CCD Info 5C			*/
            CCD_INFO_EXTENDED2_IMAGING,     /*!< Extended Imaging CCD Info 2	*/
            CCD_INFO_EXTENDED2_TRACKING,    /*!< Extended Tracking CCD Info 2	*/
            CCD_INFO_EXTENDED3				/*!< Extended Imaging CCD Info 3	*/
        }

        public enum DeviceType : ushort
        {
            LPT1 = 1,
            LPT2 = 2,
            LPT3 = 3,
            Ethernet = 0x7F01,
            USB1 = 0x7F02,
            USB2 = 0x7F03,
            USB3 = 0x7F04,
            USB4 = 0x7F05,
            USB5 = 0x7F06,
            USB6 = 0x7F07,
            USB7 = 0x7F08,
            USB8 = 0x7F09
        }

        /// <summary>
        /// Anti-blooming gate capability enum 
        /// </summary>
        public enum ImagingABG : ushort
        {
            ABG_NOT_PRESENT,    /*!< Anti-blooming gate not Present	*/
            ABG_PRESENT         /*!< Anti-blooming gate present		*/
        }

        public enum PortRate : ushort
        {
            BR_AUTO,    /*!< Bit-rate auto	*/
            BR_9600,    /*!< Bit-rate 9600	*/
            BR_19K,     /*!< Bit-rate 19K	*/
            BR_38K,     /*!< Bit-rate 38K	*/
            BR_57K,     /*!< Bit-rate 57K	*/
            BR_115K     /*!< Bit-rate 115K	*/
        }

        public enum CameraType : ushort
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
        }

        public enum ShutterCommand : ushort
        {
            SC_LEAVE_SHUTTER,       /*!< Shutter Control: Leave shutter in current state.	*/
            SC_OPEN_SHUTTER,        /*!< Shutter Control: Open shutter.						*/
            SC_CLOSE_SHUTTER,       /*!< Shutter Control: Close shutter.					*/
            SC_INITIALIZE_SHUTTER,  /*!< Shutter Control: Initialize shutter.				*/
            SC_OPEN_EXT_SHUTTER,    /*!< Shutter Control: Open external shutter.			*/
            SC_CLOSE_EXT_SHUTTER    /*!< Shutter Control: Close external shutter.			*/
        }

        public enum ShutterState7 : ushort
        {
            SS_OPEN,    /*!< Shuter State: Open		*/
            SS_CLOSED,  /*!< Shuter State: Closed	*/
            SS_OPENING, /*!< Shutter State: Opening	*/
            SS_CLOSING  /*!< Shutter State: Closing	*/
        }

        public enum TemperatureRegulation : ushort
        {
            REGULATION_OFF,                 /*!< Temperature regulation off					*/
            REGULATION_ON,                  /*!< Temperature regulation on					*/
            REGULATION_OVERRIDE,            /*!< Temperature regulation override			*/
            REGULATION_FREEZE,              /*!< Temperature regulation freeze				*/
            REGULATION_UNFREEZE,            /*!< Temperature regulation unfreeze			*/
            REGULATION_ENABLE_AUTOFREEZE,   /*!< Temperature regulation enable autofreeze	*/
            REGULATION_DISABLE_AUTOFREEZE	/*!< Temperature regulation disable autofreeze	*/
        }

        /// <summary>
        /// Mask for Temperature Regulation frozen state.
        /// </summary>
        public const ushort REGULATION_FROZEN_MASK = 0x8000;

        public enum LedState : ushort
        {
            LED_OFF,        /*!< LED off		*/
            LED_ON,         /*!< LED on			*/
            LED_BLINK_LOW,  /*!< LED Blink low	*/
            LED_BLINK_HIGH	/*!< LED Blink high	*/
        }

        public enum FilterCommand : ushort
        {
            FILTER_LEAVE,   /*!< Filter leave		*/
            FILTER_SET_1,   /*!< Filter slot 1		*/
            FILTER_SET_2,   /*!< Filter slot 2		*/
            FILTER_SET_3,   /*!< Filter slot 3		*/
            FILTER_SET_4,   /*!< Filter slot 4		*/
            FILTER_SET_5,   /*!< Filter slot 5		*/
            FILTER_STOP,    /*!< Stop filter		*/
            FILTER_INIT     /*!< Initialize filter	*/
        }

        public enum FilterState : ushort
        {
            FS_MOVING,  /*!< Filter wheel moving			*/
            FS_AT_1,    /*!< Filter wheel at slot 1			*/
            FS_AT_2,    /*!< Filter wheel at slot 2			*/
            FS_AT_3,    /*!< Filter wheel at slot 3			*/
            FS_AT_4,    /*!< Filter wheel at slot 4			*/
            FS_AT_5,    /*!< Filter wheel at slot 5			*/
            FS_UNKNOWN  /*!< Filter wheel at slot Unknown	*/
        }

        /// <summary>
        /// A/D Size enum.
        /// </summary>
        public enum AD_Size : ushort
        {
            AD_UNKNOWN, /*!< Unknown size	*/
            AD_12_BITS, /*!< 12-bits		*/
            AD_16_BITS	/*!< 16-bits		*/
        }

        public enum FilterType : ushort
        {
            FW_UNKNOWN,     /*!< Unkwown Filter Wheel	*/
            FW_EXTERNAL,    /*!< External Filter Wheel	*/
            FW_VANE,        /*!< Vane Filter Wheel		*/
            FW_FILTER_WHEEL	/*!< Standard Filter Wheel	*/
        }

        /// <summary>
        /// AO Focus enum.
        /// </summary>
        public enum AO_FocusCommand : ushort
        {
            AOF_HARD_CENTER,    /*!< AO Focus hard center	*/
            AOF_SOFT_CENTER,    /*!< AO Focus soft center	*/
            AOF_STEP_IN,        /*!< AO Focus step in		*/
            AOF_STEP_OUT        /*!< AO Focus step out		*/
        }

        /// <summary>
        /// Service port for Ethernet access.
        /// </summary>
        public const ushort SRV_SERVICE_PORT = 5000;

        /// <summary>
        /// Broadcast port for SBIG Cameras.
        /// </summary>
        public const ushort BROADCAST_PORT = 5001;

        public enum SbigDeviceType : ushort
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
        }

        public enum DriverControlParam : ushort
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
        }

        public enum USB_AD_ControlCommand : ushort
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
        }

        public enum EnumUSB_Driver : ushort
        {
            USBD_SBIGE, /*!< SBIG E */
            USBD_SBIGI, /*!< SBIG I	*/
            USBD_SBIGM, /*!< SBIG_M	*/
            USBD_NEXT   /*!< Next	*/
        }

        /// <summary>
        /// Filter Weel Model Selection enum.
        /// </summary>
        public enum CFW_ModelSelect : ushort
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
        }

        /// <summary>
        /// Filter Wheel Command enum.
        /// </summary>
        public enum CFW_Command : ushort
        {
            CFWC_QUERY,         /*!< Query			*/
            CFWC_GOTO,          /*!< Go-to slot		*/
            CFWC_INIT,          /*!< Initialize		*/
            CFWC_GET_INFO,      /*!< Get Info		*/
            CFWC_OPEN_DEVICE,   /*!< Open device	*/
            CFWC_CLOSE_DEVICE   /*!< Close device	*/
        }

        /// <summary>
        /// Filter Wheel Status enum.
        /// </summary>
        public enum CFW_Status : ushort
        {
            CFWS_UNKNOWN,   /*!< Unknown state	*/
            CFWS_IDLE,      /*!< Idle state		*/
            CFWS_BUSY       /*!< Busy state		*/
        }

        /// <summary>
        /// Filter Wheel errors enum.
        /// </summary>
        public enum CFW_Error : ushort
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
        }

        /// <summary>
        /// Filter Wheel position enum.
        /// </summary>
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
        }

        public enum TempStatusRequest : ushort
        {
            TEMP_STATUS_STANDARD,
            TEMP_STATUS_ADVANCED,
            TEMP_STATUS_ADVANCED2
        }

        /// <summary>
        /// Filter Wheel COM port enum.
        /// </summary>
        public enum CFW_COM_Port : ushort
        {
            CFWPORT_COM1 = 1,   /*!< COM1	*/
            CFWPORT_COM2,   /*!< COM2	*/
            CFWPORT_COM3,   /*!< COM3	*/
            CFWPORT_COM4    /*!< COM4	*/
        }

        /// <summary>
        /// Filter Wheel Get Info select enum.
        /// </summary>
        public enum CFW_GetInfoSelect : ushort
        {
            CFWG_FIRMWARE_VERSION,  /*!< Firmware version	*/
            CFWG_CAL_DATA,          /*!< Calibration data	*/
            CFWG_DATA_REGISTERS     /*!< Data registers		*/
        }

        /// <summary>
        /// Bit I/O Operation enum.
        /// </summary>
        public enum Bit_IO_Operation : ushort
        {
            BITIO_WRITE,    /*!< Write	*/
            BITIO_READ      /*!< Read	*/
        }

        public enum BIT_IO_Name : ushort
        {
            BITI_PS_LOW,    /*!< In: PS Low	*/
            BITO_IO1,       /*!< Out: I/O 1	*/
            BITO_IO2,       /*!< Out: I/O 2 */
            BITI_IO3,       /*!< In: I/O 3	*/
            BITO_FPGA_WE    /*!< FPGA WE	*/
        }

        /// <summary>
        /// Biorad TDI Error enum.
        /// </summary>
        public enum BTDI_Error : ushort
        {
            BTDI_SCHEDULE_ERROR = 1,    /*!< BTDI Schedule error	*/
            BTDI_OVERRUN_ERROR = 2      /*!< BTDI Overrun error		*/
        }

        /// <summary>
        /// Motor Focus Model Selection enum.
        /// </summary>
        public enum MF_ModelSelect : ushort
        {
            MFSEL_UNKNOWN,  /*!< Unknown	*/
            MFSEL_AUTO,     /*!< Automatic	*/
            MFSEL_STF       /*!< STF		*/
        }

        /// <summary>
        /// Motor Focus Command enum.
        /// </summary>
        public enum MF_Command : ushort
        {
            MFC_QUERY,      /*!< Query		*/
            MFC_GOTO,       /*!< Go-to		*/
            MFC_INIT,       /*!< Initialize	*/
            MFC_GET_INFO,   /*!< Get Info	*/
            MFC_ABORT       /*!< Abort		*/
        }

        /// <summary>
        /// Motor Focus Status.
        /// </summary>
        public enum MF_Status : ushort
        {
            MFS_UNKNOWN,    /*!< Unknown	*/
            MFS_IDLE,       /*!< Idle		*/
            MFS_BUSY        /*!< Busy		*/
        }

        /// <summary>
        /// Motor Focus Error state enum.
        /// </summary>
        public enum MF_Error : ushort
        {
            MFE_NONE,           /*!< None				*/
            MFE_BUSY,           /*!< Busy				*/
            MFE_BAD_COMMAND,    /*!< Bad command		*/
            MFE_CAL_ERROR,      /*!< Calibration error	*/
            MFE_MOTOR_TIMEOUT,  /*!< Motor timeout		*/
            MFE_BAD_MODEL,      /*!< Bad model			*/
            MFE_I2C_ERROR,      /*!< I2C error			*/
            MFE_NOT_FOUND       /*!< Not found			*/
        }

        /// <summary>
        /// Motor Focus Get Info Select enum.
        /// </summary>
        public enum MF_GetInfoSelect : ushort
        {
            MFG_FIRMWARE_VERSION,   /*!< Firmware Version	*/
            MFG_DATA_REGISTERS      /*!< Data Registers		*/
        }

        /// <summary>
        /// Differential guider commands enum.
        /// </summary>
        public enum DiffGuiderCommand : ushort
        {
            DGC_DETECT,         /*!< Detect	Differential guider hardware	*/
            DGC_GET_BRIGHTNESS, /*!< Get brightness							*/
            DGC_SET_BRIGHTNESS  /*!< Set brightness							*/
        }

        /// <summary>
        /// Differential guider error enum.
        /// </summary>
        public enum DiffGuiderError : ushort
        {
            DGE_NO_ERROR,       /*!< No error						*/
            DGE_NOT_FOUND,      /*!< Differential guider not found	*/
            DGE_BAD_COMMAND,    /*!< Bad command					*/
            DGE_BAD_PARAMETER   /*!< Bad parameter					*/
        }

        /// <summary>
        /// Differential Guider status enum.
        /// </summary>
        public enum DiffGuiderStatus : ushort
        {
            DGS_UNKNOWN,    /*!< Unknown	*/
            DGS_IDLE,       /*!< Idle		*/
            DGS_BUSY        /*!< Busy		*/
        }

        /// <summary>
        /// Fan state enum.
        /// </summary>
        public enum FanState : ushort
        {
            FS_OFF,         /*!< Fan Off	*/
            FS_ON,          /*!< Fan On		*/
            FS_AUTOCONTROL  /*!< Fan Auto	*/
        }

        public enum BulkIO_Command : ushort
        {
            BIO_READ,   /*!< Read	*/
            BIO_WRITE,  /*!< Write	*/
            BIO_FLUSH   /*!< Flush	*/
        }

        public enum PixelChannelMode : ushort
        {
            PIXEL_CHANNEL_MODE_A,   /*!< Pixel Channel A	*/
            PIXEL_CHANNEL_MODE_B,   /*!< Pixel Channel B	*/
            PIXEL_CHANNEL_MODE_AB   /*!< Pixel Channel AB	*/
        }

        public enum ActivePixelChannel : ushort
        {
            PIXEL_CHANNEL_A,    /*!< Pixel Channel A	*/
            PIXEL_CHANNEL_B     /*!< Pixel Channel B	*/
        }

        public enum ExtraExposureStatus : ushort
        {
            XES_IDLE,           //!< CCD is currently idle.
            XES_PRE_EXP,        //!< CCD is in the pre-exposure phase.
            XES_INTEGRATING,    //!< CCD is currently exposing/integrating an image.
            XES_POST_EXP        //!< CCD is in the post-exposure phase.
        }

        public const ushort END_SKIP_DELAY = 0x8000;

        /// <summary>
        /// Set in StartExposureParams::ccd to skip lowering Imaging CCD Vdd during integration. - Use this to increase the rep rate when you don't care about glow in the upper-left corner of the imaging CCD.
        /// </summary>
        public const ushort START_SKIP_VDD = 0x8000;

        /// <summary>
        /// Set in StartExposureParams::ccd and EndExposureParams::ccd to force shutter motor to stay on all the time which reduces delays in Start and End Exposure timing and yields higher image throughput.Don't	do this too often or camera head will heat up.
        /// </summary>
        public const ushort START_MOTOR_ALWAYS_ON = 0x4000;

        /// <summary>
        /// the integration phase for cameras with internal frame buffers like the STX.
        /// </summary>
        public const ushort ABORT_DONT_END = 0x2000;

        /// <summary>
        /// Set in StartExposureParams2::exposureTime enable TDI readout mode [TODO: Add supported cameras].
        /// </summary>
        public const uint EXP_TDI_ENABLE = 0x01000000;    //!< Enable TDI mode flag.

        /// <summary>
        /// Set in StarExposureParams2::exposureTime ripple correction for STF-8050/4070.
        /// </summary>
        public const uint EXP_RIPPLE_CORRECTION = 0x02000000;//!< Enable Ripple correction flag.

        /// <summary>
        /// Set in StarExposureParams2::exposureTime to activate the dual channel CCD readout mode of the STF-8050.
        /// </summary>
        public const uint EXP_DUAL_CHANNEL_MODE = 0x04000000;//!< Enable dual channel readout mode flag.

        /// <summary>
        /// Set in StarExposureParams2::exposureTime to activate the fast readout mode of the STF-8300, etc.
        /// </summary>
        public const uint EXP_FAST_READOUT = 0x08000000;//!< Enable fast readout mode flag.

        /// <summary>
        /// Set in StarExposureParams2::exposureTime to interpret exposure time as milliseconds.
        /// </summary>
        public const uint EXP_MS_EXPOSURE = 0x10000000;   //!< Enable millisecond exposure time flag.

        /// <summary>
        /// Set in StarExposureParams2::exposureTime to do light clear of the CCD.
        /// </summary>
        public const uint EXP_LIGHT_CLEAR = 0x20000000;//!< Do light clear of CCD flag.

        /// <summary>
        /// Set in StarExposureParams2::exposureTime to send trigger out Y-.
        /// </summary>
        public const uint EXP_SEND_TRIGGER_OUT = 0x40000000; //!< Send trigger out flag.

        /// <summary>
        /// Set in StarExposureParams2::exposureTime to wait for trigger in pulse.
        /// </summary>
        public const uint EXP_WAIT_FOR_TRIGGER_IN = 0x80000000; //!< Wait for trigger in flag.

        /// <summary>
        /// Set in StarExposureParams2::exposureTime to mask with exposure time to remove flags.
        /// </summary>
        public const uint EXP_TIME_MASK = 0x00FFFFFF;

        /// <summary>
        /// mask for CCD type.
        /// </summary>
        public const ushort CB_CCD_TYPE_MASK = 0x0001;

        /// <summary>
        /// b0=0 is full frame CCD.
        /// </summary>
        public const ushort CB_CCD_TYPE_FULL_FRAME = 0x0000;

        /// <summary>
        /// b0=1 is frame transfer CCD.
        /// </summary>
        public const ushort CB_CCD_TYPE_FRAME_TRANSFER = 0x0001;

        /// <summary>
        /// mask for electronic shutter type.
        /// </summary>
        public const ushort CB_CCD_ESHUTTER_MASK = 0x0002;

        /// <summary>
        /// b1=0 indicates no electronic shutter.
        /// </summary>
        public const ushort CB_CCD_ESHUTTER_NO = 0x0000;

        /// <summary>
        /// b1=1 indicates electronic shutter.
        /// </summary>
        public const ushort CB_CCD_ESHUTTER_YES = 0x0002;

        /// <summary>
        /// mask for external tracker support.
        /// </summary>
        public const ushort CB_CCD_EXT_TRACKER_MASK = 0x0004;

        /// <summary>
        /// b2=0 indicates no external tracker support.
        /// </summary>
        public const ushort CB_CCD_EXT_TRACKER_NO = 0x0000;

        /// <summary>
        /// b2=1 indicates external tracker support.
        /// </summary>
        public const ushort CB_CCD_EXT_TRACKER_YES = 0x0004;

        /// <summary>
        /// mask for BTDI support.
        /// </summary>
        public const ushort CB_CCD_BTDI_MASK = 0x0008;

        /// <summary>
        /// b3=0 indicates no BTDI support.
        /// </summary>
        public const ushort CB_CCD_BTDI_NO = 0x0000;

        /// <summary>
        /// b3=1 indicates BTDI support.
        /// </summary>
        public const ushort CB_CCD_BTDI_YES = 0x0008;

        /// <summary>
        /// mask for AO-8 detected.
        /// </summary>
        public const ushort CB_AO8_MASK = 0x0010;

        /// <summary>
        /// b4=0 indicates no AO-8 detected.
        /// </summary>
        public const ushort CB_AO8_NO = 0x0000;

        /// <summary>
        /// b4=1 indicates AO-8 detected.
        /// </summary>
        public const ushort CB_AO8_YES = 0x0010;

        /// <summary>
        /// mask for camera with frame buffer.
        /// </summary>
        public const ushort CB_FRAME_BUFFER_MASK = 0x0020;

        /// <summary>
        /// b5=0 indicates camera without Frame Buffer.
        /// </summary>
        public const ushort CB_FRAME_BUFFER_NO = 0x0000;

        /// <summary>
        /// b5=1 indicates camera with Frame Buffer.
        /// </summary>
        public const ushort CB_FRAME_BUFFER_YES = 0x0020;

        /// <summary>
        /// mask for camera that requires StartExposure2.
        /// </summary>
        public const ushort CB_REQUIRES_STARTEXP2_MASK = 0x0040;

        /// <summary>
        /// b6=0 indicates camera works with StartExposure.
        /// </summary>
        public const ushort CB_REQUIRES_STARTEXP2_NO = 0x0000;

        /// <summary>
        /// b6=1 indicates camera Requires StartExposure2.
        /// </summary>
        public const ushort CB_REQUIRES_STARTEXP2_YES = 0x0040;

        /// <summary>
        /// Minimum exposure for ST-7 cameras in 1/100ths second.
        /// </summary>
        public const ushort MIN_ST7_EXPOSURE = 12;

        /// <summary>
        /// Minimum exposure for ST-402 cameras in 1/100ths second.
        /// </summary>
        public const ushort MIN_ST402_EXPOSURE = 4;

        /// <summary>
        /// Minimum exposure fpr STF-3200 cameras in 1/100ths second.
        /// </summary>
        public const ushort MIN_ST3200_EXPOSURE = 9;

        /// <summary>
        /// Minimum exposure for STF-8300 cameras in 1/100ths second.
        /// </summary>
        public const ushort MIN_STF8300_EXPOSURE = 9;

        /// <summary>
        /// Minimum exposure for STF-8050 cameras in 1/1000ths second since has E Shutter.
        /// </summary>
        public const ushort MIN_STF8050_EXPOSURE = 1;

        /// <summary>
        /// Minimum exposure for STF-4070 cameras in 1/1000ths second since has E Shutter.
        /// </summary>
        public const ushort MIN_STF4070_EXPOSURE = 1;

        /// <summary>
        /// Minimum exposure for STF-0402 cameras in 1/100ths second.
        /// </summary>
        public const ushort MIN_STF0402_EXPOSURE = 4;

        /// <summary>
        /// Minimum exposure for STX cameras in 1/100ths second.
        /// </summary>
        public const ushort MIN_STX_EXPOSURE = 18;

        /// <summary>
        /// Minimum exposure for STT cameras in 1/100ths second.
        /// </summary>
        public const ushort MIN_STT_EXPOSURE = 12;

        /// <summary>
        /// Minimum exposure in 1/1000ths second since ST-i has E Shutter.
        /// </summary>
        public const ushort MIN_STU_EXPOSURE = 1;

        #endregion Enums

        //
        #region Types
        //

        //TODO: 檢查移植類型正確性
        /// <summary>
        /// Boolean type definition.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct MY_LOGICAL
        {
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

        /// <summary>
        /// Expanded parameters structure used to start SBIG camera exposures.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct StartExposureParams
        {
            /// <summary>
            /// <seealso cref="CCD_Request"/>
            /// </summary>
            public UInt16 ccd;
            /// <summary>
            /// Exposure time in hundredths of a second in least significant 24 bits. 
            /// Most significant bits are bit-flags described in exposureTime #define block.
            /// </summary>
            public UInt32 exposureTime;
            /// <summary>
            /// <seealso cref="AbgState7"/>
            /// </summary>
            public UInt16 abgState;
            /// <summary>
            /// <seealso cref="ShutterCommand"/>
            /// </summary>
            public UInt16 openShutter;
        }

        /// <summary>
        /// Parameters used to end SBIG camera exposures.
        /// Set ABORT_DONT_END flag in ccd to abort exposures in supported cameras.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct StartExposureParams2
        {
            /// <summary>
            /// <seealso cref="CCD_Request"/>
            /// </summary>
            public CCD_Request ccd;
            /// <summary>
            /// Exposure time in hundredths of a second in least significant 24 bits. Most significant bits are bit-flags described in exposureTime #define block.
            /// </summary>
            public uint exposureTime;
            /// <summary>
            /// <seealso cref="AbgState7"/>
            /// </summary>
            public AbgState7 abgState;
            /// <summary>
            /// <seealso cref="ShutterCommand"/>
            /// </summary>
            public ShutterCommand openShutter;
            /// <summary>
            /// <seealso cref="ReadoutBinningMode"/>
            /// </summary>
            public ReadoutBinningMode readoutMode;
            /// <summary>
            /// top-most row to read out. (0 based)
            /// </summary>
            public UInt16 top;
            /// <summary>
            /// left-most column to read out. (0 based)
            /// </summary>
            public UInt16 left;
            /// <summary>
            ///  image height in binned pixels.
            /// </summary>
            public UInt16 height;
            /// <summary>
            /// image width in binned pixels.
            /// </summary>
            public UInt16 width;
        }

        /// <summary>
        /// Parameters used to end SBIG camera exposures.
        /// Set ABORT_DONT_END flag in ccd to abort exposures in supported cameras.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct EndExposureParams
        {
            public CCD_Request ccd;
        }

        //TODO: 重新檢查是否符合類別動作
        /// <summary>
        /// Parameters used to readout lines of SBIG cameras during readout.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct ReadoutLineParams
        {
            /// <summary>
            /// <seealso cref="CCD_Request"/>
            /// </summary>
            public CCD_Request ccd;
            /// <summary>
            /// <seealso cref="ReadoutBinningMode"/>
            /// </summary>
            public ReadoutBinningMode readoutMode;
            /// <summary>
            /// left-most pixel to read out.
            /// </summary>
            public UInt16 pixelStart;
            /// <summary>
            /// number of pixels to digitize.
            /// </summary>
            public UInt16 pixelLength;

            //public static ushort MakeNBinMode(ReadoutBinningMode rlp, ushort n)
            //{
            //    // put the high byte in place, but only if it's one of those binning modes
            //    if (ReadoutBinningMode.RM_NX1 == rlp ||
            //        ReadoutBinningMode.RM_NX2 == rlp ||
            //        ReadoutBinningMode.RM_NX3 == rlp ||
            //        ReadoutBinningMode.RM_NXN == rlp)
            //        return (ushort)(((ushort)rlp) | (n << 8));
            //    else
            //        return (ushort)rlp;
            //}
        }

        /// <summary>
        /// Parameters used to dump/flush CCD lines during readout.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct DumpLinesParams
        {
            /// <summary>
            /// <seealso cref="CCD_Request"/>
            /// </summary>
            public CCD_Request ccd;
            /// <summary>
            /// <seealso cref="ReadoutBinningMode"/>
            /// </summary>
            public ReadoutBinningMode readoutMode;
            /// <summary>
            /// number of lines to dump.
            /// </summary>
            public UInt16 lineLength;
        }

        /// <summary>
        /// Parameters used to end SBIG camera readout.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct EndReadoutParams
        {
            /// <summary>
            /// <seealso cref="CCD_Request"/>
            /// </summary>
            public CCD_Request ccd;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct StartReadoutParams
        {
            /// <summary>
            /// <seealso cref="CCD_Request"/>
            /// </summary>
            public CCD_Request ccd;
            /// <summary>
            /// <seealso cref="ReadoutBinningMode"/>
            /// </summary>
            public ReadoutBinningMode readoutMode;
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
        }

        /// <summary>
        /// The Set Temperature Regulation command is used to enable or disable the CCD's temperature regulation.
        /// Uses special units for the CCD temperature.
        /// The Set Temperature Regulation 2 command described in the next section is easier to use with temperatures stated in Degrees Celsius.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct SetTemperatureRegulationParams
        {
            /// <summary>
            /// <seealso cref="TemperatureRegulation"/>
            /// </summary>
            public TemperatureRegulation state;
            /// <summary>
            /// CCD temperature setpoint in A/D units if regulation on or TE drive level (0-255 = 0-100%) if regulation override.
            /// </summary>
            public UInt16 ccdSetpointA2dUnits;
        }

        /// <summary>
        /// The Set Temperature Regulation 2 command is used to enable or disable the CCD's temperature regulation using temperatures in Degrees C instead of the funny A/D units described above.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct SetTemperatureRegulationParams2
        {
            /// <summary>
            /// <seealso cref="TemperatureRegulation"/>
            /// </summary>
            public TemperatureRegulation state;
            /// <summary>
            /// CCD temperature setpoint in degrees Celsius.
            /// </summary>
            public double ccdSetpointCelcius;
        }

        /// <summary>
        /// The Query Temperature Status command is used to monitor the CCD's temperature regulation. 
        /// The original version of this command took no Parameters (a NULL pointer) but the command has been expanded to allow a more user friendly result. 
        /// If you pass a NULL pointer in the Parameters variable youl get the classic result. 
        /// If you pass a pointer to a QueryTemperatureStatusParams struct you'll have access to the expanded results.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QueryTemperatureStatusParams
        {
            /// <summary>
            /// <seealso cref="TempStatusRequest"/>
            /// </summary>
            public TempStatusRequest request;
        }

        //TODO: 檢查MY_LOGICAL
        /// <summary>
        /// The results struct of a Temperature Status Query, with request set to TEMP_STATUS_STANDARD.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
        }

        //TODO: 檢查MY_LOGICAL
        /// <summary>
        /// The results struct of a Temperature Status Query, with request set to TEMP_STATUS_ADVANCED.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QueryTemperatureStatusResults2
        {
            /// <summary>
            /// temperature regulation is enabled when this is TRUE. &REGULATION_FROZEN_MASK is TRUE when TE is frozen.
            /// </summary>
            public MY_LOGICAL coolingEnabled;
            /// <summary>
            ///  fan state and is one of the following: FS_OFF (off), FS_ON (manual control) or FS_AUTOCONTROL (auto speed control).
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
        }

        /// <summary>
        /// The Activate Relay command is used to activate one or more of the telescope control outputs or to cancel an activation in progress.
        /// <para>The status for this command(from QueryCommandStatus) consists of four bit fields:</para>
        /// <para>b3 = +X Relay, 0=Off, 1= Active</para>
        /// <para>b2 = -X Relay, 0=Off, 1= Active</para>
        /// <para>b1 = +Y Relay, 0=Off, 1= Active</para>
        /// <para>b0 = -Y Relay, 0=Off, 1= Active</para>
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct ActivateRelayParams
        {
            public UInt16 tXPlus;
            public UInt16 tXMinus;
            public UInt16 tYPlus;
            public UInt16 tYMinus;
        }

        /// <summary>
        /// The Pulse Out command is used with the ST-7/8/etc to position the CFW-6A/CFW-8 and with the PixCel255 and PixCel237 to position the internal vane/filter wheel.
        /// <para>The status for this command is: </para>
        /// <para>b0 - Normal status, 0 = inactive, 1 = pulse out in progress</para>
        /// <para>b1-b3 - PixCel255/237 Filter state, 0=moving, 1-5=at position 1-5, 6=unknown</para>
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
        }

        /// <summary>
        /// The TX Serial Bytes command is for internal use by SBIG. 
        /// It's a very low level version of commands like AO Tip Tilt that are used to send data out the ST-7/8/etc's telescope port to accessories like the AO-7.
        /// There's no reason why you should need to use this command.
        /// Just use the dedicated commands like AO Tip Tilt.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct TXSerialBytesParams
        {
            /// <summary>
            /// Length of data buffer to send.
            /// </summary>
            public UInt16 dataLength;
            /// <summary>
            /// Buffer of data to send.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 256)]
            public UInt16[] data;
        }

        /// <summary>
        /// Results of a TXSerialBytes command.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct TXSerialBytesResults
        {
            /// <summary>
            /// Bytes sent out.
            /// </summary>
            public UInt16 bytesSent;
        }

        /// <summary>
        /// The Get Serial Status command is for internal use by SBIG. 
        /// It's a very low level version of commands like AO Tip Tilt that are used to send data out the ST-7/8/etc's telescope port to accessories like the AO-7.
        /// There's no reason why you should need to use this command.
        /// Just use the dedicated commands like AO Tip Tilt.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetSerialStatusResults
        {
            public MY_LOGICAL clearToCOM;
        }

        /// <summary>
        /// The Establish Link command is used by the application to establish a communications link with the camera.
        /// It should be used before any other commands are issued to the camera(excluding the Get Driver Info command).
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct EstablishLinkParams
        {
            /// <summary>
            /// Maintained for historical purposes. Keep set to 0.
            /// </summary>
            public UInt16 sbigUseOnly;
        }

        /// <summary>
        /// Results from an EstablishLink command.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct EstablishLinkResults
        {
            /// <summary>
            /// <seealso cref="CameraType"/>
            /// </summary>
            public CameraType cameraType;
        }

        /// <summary>
        /// The Get Driver Info command is used to determine the version and capabilities of the DLL/Driver.
        /// For future expandability this command allows you to request several types of information.
        /// Initially the standard request and extended requests will be supported but as the driver evolves additional requests will be added.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetDriverInfoParams
        {
            /// <summary>
            /// <seealso cref="DriverRequest"/>
            /// </summary>
            public DriverRequest request;
        }

        /// <summary>
        /// Standard, Extended and USB Loader Results Struct.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetDriverInfoResults0
        {
            /// <summary>
            /// driver version in BCD with the format XX.XX
            /// </summary>
            public UInt16 version;
            /// <summary>
            /// driver name, null terminated string
            /// </summary>
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string name;
            /// <summary>
            /// maximum request response available from this driver
            /// </summary>
            public UInt16 maxRequest;
        }

        /// <summary>
        /// The Get CCD Info command is used by the application to determine the model of camera being controlled and its capabilities. 
        /// For future expandability this command allows you to request several types of information.
        /// Currently 6 standard requests are supported but as the driver evolves additional requests will be added.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetCCDInfoParams
        {
            /// <summary>
            /// <seealso cref="CcdInfoRequest"/>
            /// </summary>
            public CcdInfoRequest request;
        }

        /// <summary>
        /// Internal structure for storing readout modes.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct READOUT_INFO
        {
            /// <summary>
            /// readout mode ID. (see also: <seealso cref="ReadoutBinningMode"/>)
            /// </summary>
            public ReadoutBinningMode mode;
            /// <summary>
            /// width of image in pixels.
            /// </summary>
            public UInt16 width;
            /// <summary>
            /// height of image in pixels.
            /// </summary>
            public UInt16 height;
            /// <summary>
            /// a four digit BCD number specifying the amplifier gain in e-/ADU in XX.XX format.
            /// </summary>
            public UInt16 gain;
            /// <summary>
            /// an eight digit BCD number specifying the pixel width in microns in the XXXXXX.XX format.
            /// </summary>
            public UInt32 pixel_width;
            /// <summary>
            /// an eight digit BCD number specifying the pixel height in microns in the XXXXXX.XX format.
            /// </summary>
            public UInt32 pixel_height;
        }
        
        /// <summary>
        /// Get CCD Info command results 0 and 1 request.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetCCDInfoResults0and1
        {
            /// <summary>
            /// version of the firmware in the resident microcontroller in BCD format (XX.XX, 0x1234 = 12.34).
            /// </summary>
            public UInt16 firmwareVersion;
            /// <summary>
            /// <seealso cref="CameraType"/>
            /// </summary>
            public CameraType cameraType;
            /// <summary>
            /// null terminated string containing the name of the camera.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string name;
            /// <summary>
            /// number of readout modes supported.
            /// </summary>
            public UInt16 readoutModes;
            /// <summary>
            /// <seealso cref="READOUT_INFO"/>
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, ArraySubType = UnmanagedType.Struct, SizeConst = 20)]
            public READOUT_INFO[] readoutInfo;
        }

        /// <summary>
        /// Get CCD Info command results second request.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetCCDInfoResults2
        {
            /// <summary>
            /// number of bad columns in imaging CCD.
            /// </summary>
            public UInt16 badColumns;
            /// <summary>
            /// bad columns.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public UInt16[] columns;
            /// <summary>
            /// type of Imaging CCD, 0= No ABG Protection, 1 = ABG Present.
            /// see also: <seealso cref="ImagingABG"/> enum.
            /// </summary>
            public UInt16 imagingABG;
            /// <summary>
            /// null terminated serial number string.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            public string serialNumber;
        }

        /// <summary>
        /// Get CCD Info command results third request. (For the PixCel255/237)
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetCCDInfoResults3
        {
            /// <summary>
            /// 0 = Unknown, 1 = 12 bits, 2 = 16 bits. see also: <seealso cref="AD_Size"/> enum.
            /// </summary>
            public AD_Size adSize;
            /// <summary>
            /// 0 = Unknown, 1 = External, 2 = 2 Position, 3 = 5 Position. see also: <seealso cref="FilterType"/> enum. 
            /// </summary>
            public FilterType filterType;
        }

        /// <summary>
        /// Get CCD Info command results fourth and fifth request. (For all cameras)
        /// <para>Capabilities bits:</para>
        /// <para>b0: 0 = CCD is Full Frame Device, 1 = CCD is Frame Transfer Device,</para>
        /// <para>b1: 0 = No Electronic Shutter, 1 = Interline Imaging CCD with Electronic Shutter and</para>
        /// <para>millisecond exposure capability</para>
        /// <para>b2: 0 = No hardware support for external Remote Guide Head, 1 = Detected hardware</para>
        /// <para>support for external Remote Guide Head.</para>
        /// <para>b3: 1 = Supports the special Biorad TDI acquisition mode.</para>
        /// <para>b4: 1 = AO8 detected.</para>
        /// <para>b5: 1 = Camera contains an internal frame buffer.</para>
        /// <para>b6: 1 = Camera requires the StartExposure2 command instead of the older depricated StartExposure command.</para>
        /// <para>Other: See the CB_XXX_XXX definitions in the sbigurdv.h header file.</para>
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetCCDInfoResults4and5
        {
            /// <summary>
            /// Camera capabilities. See the CB_XXX_XXX definitions in the sbigurdv.h header file.
            /// </summary>
            public UInt16 capabilitiesBits;
            /// <summary>
            /// Number of unbinned rows to dump to transfer image area to storage area.
            /// </summary>
            public UInt16 dumpExtra;
        }

        /// <summary>
        /// Get CCD Info command results sixth request. (For all cameras)
        /// 
        /// <para>Camera bits:</para>
        /// <para>b0: 0 = STX camera, 1 = STXL camera</para>
        /// <para>b1: 0 = Mechanical shutter, 1 = No mechanical shutter (only an electronic shutter)</para>
        /// <para>b2 ?b31: reserved for future expansion</para>
        /// 
        /// <para>CCD Bits:</para>
        /// <para>b0: 0 = Imaging Mono CCD, 1 = Imaging Color CCD</para>
        /// <para>b1: 0 = Bayer color matrix, 1 = Truesense color matrix</para>
        /// <para>b2 ?b31: reserved for future expansion</para>
        /// </summary>
        public struct GetCCDInfoResults6
        {
            /// <summary>
            /// Set of bits for additional camera capabilities.
            /// </summary>
            public UInt32 cameraBits;
            /// <summary>
            /// Set of bits for additional CCD capabilities.
            /// </summary>
            public UInt32 ccdBits;
            /// <summary>
            /// Set of bits for additional capabilities.
            /// </summary>
            public UInt32 extraBits;
        }

        /// <summary>
        /// The Query Command Status command is used to monitor the progress of a previously requested command.
        /// Typically this will be used to monitor the progress of an exposure, relay closure or CFW-6A move command.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QueryCommandStatusParams
        {
            /// <summary>
            /// command of which the status is desired.
            /// </summary>
            public PAR_COMMAND command;
        }

        /// <summary>
        /// Results for the Query Command Status command.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QueryCommandStatusResults
        {
            /// <summary>
            /// command status.
            /// </summary>
            public PAR_ERROR status;
        }

        /// <summary>
        /// Results for the Query Command Status command.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct MiscellaneousControlParams
        {
            /// <summary>
            /// set TRUE to turn on the Fan.
            /// </summary>
            public MY_LOGICAL fanEnable;
            /// <summary>
            /// see also: <seealso cref="ShutterCommand"/> enum.
            /// </summary>
            public ShutterCommand shutterCommand;
            /// <summary>
            /// see also: <seealso cref="LedState"/> enum.
            /// </summary>
            public LedState ledState;
        }

        /// <summary>
        /// The Read Offset command is used to measure the CCD's offset. 
        /// In the SBIG cameras the offset is adjusted at the factory and this command is for testing or informational purposes only.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct ReadOffsetParams
        {
            /// <summary>
            /// see also: <seealso cref="CCD_Request"/> enum.
            /// </summary>
            public CCD_Request ccd;
        }

        /// <summary>
        /// Results structure for the Read Offset command.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct ReadOffsetResults
        {
            /// <summary>
            /// the CCD's offset.
            /// </summary>
            public UInt16 offset;
        }

        /// <summary>
        /// The Read Offset 2 command is used to measure the CCD's offset and the noise in the readout register.
        /// In the SBIG cameras the offset is adjusted at the factory and this command is for testing or informational purposes only.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
        }

        /// <summary>
        /// The AO Tip Tilt Command is used to position an AO-7 attached to the telescope port of an ST-7/8/etc.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct AOTipTiltParams
        {
            /// <summary>
            /// this is the desired position of the mirror in the X axis.
            /// </summary>
            public UInt16 xDeflection;
            /// <summary>
            /// this is the desired position of the mirror in the Y axis.
            /// </summary>
            public UInt16 yDeflection;
        }

        /// <summary>
        /// This command is reserved for future use with motorized focus units.
        /// Prototypes of the AO-7 had motorized focus but the feature was removed in the production units.
        /// This command is a holdover from that.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct AOSetFocusParams
        {
            /// <summary>
            /// see also: <seealso cref="AO_FocusCommand"/> enum.
            /// </summary>
            public AO_FocusCommand focusCommand;
        }

        /// <summary> 
        /// The AO Delay Command is used to generate millisecond type delays for exposing the Tracking CCD.
        /// This sleep command is blocking.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct AODelayParams
        {
            /// <summary>
            /// this is the desired delay in microseconds.
            /// </summary>
            public UInt32 delay;
        }

        /// <summary>
        /// The current driver does not use this command.
        /// It was added in a previous version and never removed.
        /// It could be reassigned in the future.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetTurboStatusResults
        {
            /// <summary>
            /// TRUE if turbo is detected.
            /// </summary>
            public MY_LOGICAL turboDetected;
        }

        //TODO: 重新檢查是否符合類別動作
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct OpenDeviceParams
        {
            /// <summary>
            /// see also: <seealso cref="DeviceType"/> enum. specifies LPT, Ethernet, etc.
            /// </summary>
            public DeviceType deviceType;
            /// <summary>
            /// for deviceType::DEV_LPTN: Windows 9x Only, Win NT uses deviceSelect.
            /// </summary>
            public UInt16 lptBaseAddress;
            /// <summary>
            /// for deviceType::DEV_ETH: Ethernet address.
            /// </summary>
            public uint ipAddress;

            public OpenDeviceParams(string s) : this()
            {
                try
                {
                    // first, try to parse as an IP address
                    IPAddress ip = IPAddress.Parse(s);
                    byte[] b = ip.GetAddressBytes();
                    ipAddress = (((uint)b[0]) << 24) | (((uint)b[1]) << 16) | (((uint)b[2]) << 8) | ((uint)b[3]);
                    deviceType = DeviceType.Ethernet;
                }
                catch (FormatException)
                {
                    // if it's not an IP, it should be a string value of the enum
                    if (!Enum.TryParse(s, true, out deviceType))
                        throw new ArgumentException("must pass either an IP address or valid DeviceType enum string");
                }
            }
        }

        /// <summary>
        /// This command allows you to control the IRQ priority of the driver under Windows NT/2000/XP. 
        /// The default settings should work fine for all users and these commands should not need to be used.
        /// <para>
        /// We use three settings in our CCDOPS software: High = 27, Medium = 15, Low = 2. 
        /// Under fast machines Low will work fine. 
        /// On slower machines the mouse may get sluggish unless you select the Medium or High priority.
        /// </para>
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct SetIRQLParams
        {
            /// <summary>
            /// IRQ Level.
            /// </summary>
            public UInt16 level;
        }

        /// <summary>
        /// Results of Get IRQ Level command.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetIRQLResults
        {
            /// <summary>
            /// IRQ Level.
            /// </summary>
            public UInt16 level;
        }

        /// <summary>
        /// This command returns the status of the communications link established with the camera.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetLinkStatusResults
        {
            /// <summary>
            /// TRUE when a link has been established.
            /// </summary>
            public MY_LOGICAL linkEstablished;
            /// <summary>
            /// base address of the LPT port.
            /// </summary>
            public UInt16 baseAddress;
            /// <summary>
            /// see also: <seealso cref="CameraType"/> enum.
            /// </summary>
            public CameraType cameraType;
            /// <summary>
            /// total number of communications with camera.
            /// </summary>
            public UInt32 comTotal;
            /// <summary>
            /// total number of failed communications with camera.
            /// </summary>
            public UInt32 comFailed;
        }

        /// <summary>
        /// This command is of extremely limited (and unknown) use.
        /// When you have established a link to a parallel port based camera under Windows NT/2000/XP this command returns a counter with 1 microsecond resolution. 
        /// Under all other circumstances the counter is zero.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetUSTimerResults
        {
            /// <summary>
            /// counter value in microseconds.
            /// </summary>
            public UInt32 count;
        }

        //TODO: 移植uchar類型指標
        /// <summary>
        /// Intended for SBIG internal use only. Unimplemented.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
            /// <summary>
            /// Buffer of data to send.
            /// </summary>
            //public unsigned char* source;
        }

        /// <summary>
        /// Intended for SBIG internal use only. Unimplemented.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
        }

        /// <summary>
        /// Intended for SBIG internal use only. Clock the AD the number of times passed.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct ClockADParams
        {
            /// <summary>
            /// CCD to clock. see also: <seealso cref="CCD_Request"/> enum. (Unused)
            /// </summary>
            public CCD_Request ccd;
            /// <summary>
            /// Readout mode. see also: <seealso cref="ReadoutBinningMode"/> enum. (Unused)
            /// </summary>
            public ReadoutBinningMode readoutMode;
            /// <summary>
            /// Starting pixel. (Unused)
            /// </summary>
            public UInt16 pixelStart;
            /// <summary>
            /// Count of cycles to pass.
            /// </summary>
            public UInt16 pixelLength;
        }

        /// <summary>
        /// Intended for SBIG internal use only. Pass the SystemTest command to the micro.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
        }

        //TODO: 移植uchar類型指標
        /// <summary>
        /// Intended for SBIG internal use only. Unused.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct SendSTVBlockParams
        {
            /// <summary>
            /// Outgoing buffer length.
            /// </summary>
            public UInt16 outLength;
            /// <summary>
            /// Outgoing buffer.
            /// </summary>
            //public uchar* outPtr;
            /// <summary>
            /// Incoming buffer length.
            /// </summary>
            public UInt16 inLength;
            /// <summary>
            /// Incoming buffer.
            /// </summary>
            //public uchar* inPtr;
        }

        /// <summary>
        /// This command returns a null terminated C string in English (not Unicode) corresponding to the passed error number. 
        /// It's handy for reporting driver level errors to the user.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetErrorStringParams
        {
            /// <summary>
            /// Error code. see also: <seealso cref="PAR_ERROR"/> enum.
            /// </summary>
            public PAR_ERROR errorNo;
        }


        /// <summary>
        /// This command returns a null terminated C string in English (not Unicode) corresponding to the passed error number.
        /// It's handy for reporting driver level errors to the user.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetErrorStringResults
        {
            /// <summary>
            /// Error string in english (not unicode).
            /// </summary>
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string errorString;
        }

        /// <summary>
        /// The Get/Set Driver Handle commands are for use by applications that wish to talk to multiple cameras on various ports at the same time. 
        /// If your software only wants to talk to one camera at a time you can ignore these commands.
        /// </summary>
        /// <remarks>
        /// The Get Driver Handle command takes a NULL Parameters pointer and a pointer to a GetDriverHandleResults struct for Results. 
        /// The Set Driver Handle command takes a pointer to a SetDriverHandleParams struct for Parameters and a NULL pointer for Results. 
        /// To establish links to multiple cameras do the following sequence:
        /// 
        /// * Call Open Driver for Camera 1
        /// * Call Open Device for Camera 1
        /// * Call Establish Link for Camera 1
        /// * Call Get Driver Handle and save the result as Handle1
        /// * Call Set Driver Handle with INVALID_HANDLE_VALUE in the handle parameter
        /// * Call Open Driver for Camera 2
        /// * Call Open Device for Camera 2
        /// * Call Establish Link for Camera 2
        /// * Call Get Driver Handle and save the result as Handle2
        ///
        /// Then, when you want to talk to Camera 1, call Set Driver Handle with Handle1 and when you want to talk to Camera 2, call Set Driver Handle with Handle2. 
        /// To shut down you must call Set Driver Handle, Close Device and Close Driver in that sequence for each camera.
        /// 
        /// Each time you call Set Driver Handle with INVALID_HANDLE_VALUE you are allowing access to an additional camera up to a maximum of four cameras. 
        /// These cameras can be on different LPT ports, multiple USB4 cameras or at different Ethernet addresses. 
        /// There is a restriction though due to memory considerations. 
        /// You can only have a single readout in process at a time for all cameras and CCDs within a camera. 
        /// Readout begins with the Start Readout or Readout Line commands and ends with the End Readout command. 
        /// If you try to do multiple interleaved readouts the data from the multiple cameras will be commingled. 
        /// To avoid this, simply readout one camera/CCD at a time in an atomic process.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct SetDriverHandleParams
        {
            /// <summary>
            /// Handle to driver.
            /// </summary>
            public short handle;
        }

        /// <summary>
        /// The Get/Set Driver Handle commands are for use by applications that wish to talk to multiple cameras on various ports at the same time. 
        /// If your software only wants to talk to one camera at a time you can ignore these commands.
        /// </summary>
        /// <remarks>
        /// The Get Driver Handle command takes a NULL Parameters pointer and a pointer to a GetDriverHandleResults struct for Results. 
        /// The Set Driver Handle command takes a pointer to a SetDriverHandleParams struct for Parameters and a NULL pointer for Results. 
        /// To establish links to multiple cameras do the following sequence:
        /// 
        /// * Call Open Driver for Camera 1
        /// * Call Open Device for Camera 1
        /// * Call Establish Link for Camera 1
        /// * Call Get Driver Handle and save the result as Handle1
        /// * Call Set Driver Handle with INVALID_HANDLE_VALUE in the handle parameter
        /// * Call Open Driver for Camera 2
        /// * Call Open Device for Camera 2
        /// * Call Establish Link for Camera 2
        /// * Call Get Driver Handle and save the result as Handle2
        /// 
        /// Then, when you want to talk to Camera 1, call Set Driver Handle with Handle1 and when you want to talk to Camera 2, call Set Driver Handle with Handle2. 
        /// To shut down you must call Set Driver Handle, Close Device and Close Driver in that sequence for each camera.
        /// 
        /// Each time you call Set Driver Handle with INVALID_HANDLE_VALUE you are allowing access to an additional camera up to a maximum of four cameras. 
        /// These cameras can be on different LPT ports, multiple USB4 cameras or at different Ethernet addresses. 
        /// There is a restriction though due to memory considerations. 
        /// You can only have a single readout in process at a time for all cameras and CCDs within a camera. 
        /// Readout begins with the Start Readout or Readout Line commands and ends with the End Readout command. 
        /// If you try to do multiple interleaved readouts the data from the multiple cameras will be commingled. 
        /// To avoid this, simply readout one camera/CCD at a time in an atomic process.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetDriverHandleResults
        {
            /// <summary>
            /// Handle to driver.
            /// </summary>
            public short handle;
        }

        //TODO: 修理函數說明消失
        /// <summary>
        /// This command is used to modify the behavior of the driver by changing the settings of one of the driver control parameters. 
        /// Driver options can be enabled or disabled with this command. 
        /// There is one set of parameters for the whole DLL vs. one per handle.
        /// </summary>
        /// <remarks>
        /// * The DCP_USB_FIFO_ENABLE parameter defaults to TRUE and can be set FALSE to disable the FIFO and associated pipelining in the USB cameras. 
        ///   You would do this for example in applications using Time Delay Integration (TDI) where you don't want data in the CCD digitized until the actual call to ReadoutLine is made.
        ///   
        /// * The DCP_CALL_JOURNAL_ENABLE parameter defaults to FALSE and can be set to TRUE to have the driver broadcast Driver API calls. 
        ///   These broadcasts are handy as a debug tool for monitoring the sequence of API calls made to the driver. 
        ///   The broadcasts can be received and displayed with the Windows based SBIGUDRVJournalRx.exe application.
        ///   Only use this for testing purposes and do not enabled this feature in your released version of you application as the journaling mechanism can introduce minor artifacts in the readout.
        ///   
        /// * The DCP_IVTOH_RATIO parameter sets the number of Vertical Rows that are dumped (fast) before the Horizontal Register is dumped (not as fast) in the DumpRows command for Parallel Port based cameras. 
        ///   This is a very specialized parameter and you should think hard about changing it if you do. 
        ///   The default of 5 for the IHTOV_RATIO has been determined to offer a good compromise between the time it takes to clear the CCD or Dump Rows and the ability to effectively clear the CCD after imaging a bright object. 
        ///   Finally should you find it necessary to change it read the current setting and restore it when you're done.
        ///   
        /// * The DCP_USB_FIFO_SIZE parameter sets the size of the FIFO used to receive data from USB cameras. 
        ///   The default and maximum value of 16384 yields the highest download speeds.
        ///   Lowering the value will cause the camera to digitize and download pixels in smaller chunks.
        ///   Again this is a specialized parameter that 99.9% of programs out there will have no need for changing.
        ///   
        /// * The DCP_USB_PIXEL_DL_ENABLE parameter allows disabling the actual downloading of pixel data from the camera for testing purposes. 
        ///   This parameter defaults to TRUE.
        ///   
        /// * The DCP_HIGH_THROUGHPUT parameter allows configuring the driver for the highest possible imaging throughput at the expense of image noise and or artifacts. 
        ///   This parameter defaults to FALSE and you should only enable this for short periods of time. 
        ///   You might use this in Focus mode for example to get higher image throughput but you should never use it when you are taking keeper images. 
        ///   It does things that avoid timed delays in the camera like leaving the shutter motor on all the time, etc. 
        ///   At this time this feature is supported in the driver but not all cameras show a benefit from its use.
        ///   
        /// * The DCP_VDD_OPTIMIZED parameter defaults to TRUE which lowers the CCD's Vdd (which reduces amplifier glow) only for images 3 seconds and longer. 
        ///   This was done to increase the image throughput for short exposures as raising and lowering Vdd takes 100s of milliseconds.
        ///   The lowering and subsequent raising of Vdd delays the image readout slightly which causes short exposures to have a different bias structure than long exposures. 
        ///   Setting this parameter to FALSE stops the short exposure optimization from occurring.
        ///   
        /// * The DCP_AUTO_AD_GAIN parameter defaults to TRUE whereby the driver is responsible for setting the A/D gain in USB cameras. 
        ///   Setting this to FALSE allows overriding the driver imposed A/D gains.
        ///   
        /// * The DCP_NO_HCLKS_FOR_INTEGRATION parameter defaults to FALSE and setting it to TRUE disables the horizontal clocks during exposure integration and is intended for SBIG testing only.
        ///   
        /// * The DCP_TDI_MODE_ENABLE parameter defaults to FALSE and setting it to TRUE enables the special Biorad TDI mode.
        ///   
        /// * The DCP_VERT_FLUSH_CONTROL_ENABLE parameter defaults to TRUE and setting it to FALSE it disables the background flushing of the vertical clocks of KAI CCDs during exposure integration and is intended for SBIG testing only.
        /// 
        /// * The DCP_ETHERNET_PIPELINE_ENABLE parameter defaults to FALSE and setting it to TRUE can increase the throughput of Ethernet based cameras like the STX & STT but doing so is not recommended for robust operation.
        /// 
        /// * The DCP_FAST_LINK parameter defaults to FALSE and setting it to TRUE speeds up the Establish Link command by not dumping the pixel FIFOs in the camera, It is used internally to speed up the Query USB and Query Ethernet commands.
        /// 
        /// * The DCP_COLUMN_REPAIR_ENABLE defaults to FALSE and setting it to TRUE causes the Universal Driver Library to repair up to 7 columns in the Imaging CCD automatically. 
        ///   This is done in conjunction with column data stored in nonvolatile memory in the cameras. 
        ///   Under Windows the setting of this parameter persists in the Registry through the setting of the HKEY_CURRENT_USER\Software\SBIG\SBIGUDRV\Filter\ColumnRepairEnable setting.
        /// 
        /// * The DCP_WARM_PIXEL_REPAIR_ENABLE defaults to Zero and setting it to 1 through 8 causes the Universal Driver Library to repair warm pixels in the Imaging CCD automatically. 
        ///   A setting of 8 replaces approximately 5% of pixels and a setting of 1 replaces approximately 1 in a million. 
        ///   A decrease of 1 in the setting replaces approximately 1/10th the number of pixels of the higher setting (7 ~ 0.5%, 6 ~ 0.05%, etc). 
        ///   Under Windows the setting of this parameter persists in the Registry through the setting of the HKEY_CURRENT_USER\Software\SBIG\SBIGUDRV\Filter\WarmPixelRepairEnable setting.
        /// 
        /// * The DCP_WARM_PIXEL_REPAIR_COUNT parameter returns the total number of pixels replaced in the last image by the Warm Pixel Repair routine described above.
        ///   You can use this parameter to tweak the DCP_WARM_PIXEL_REPAIR_ENABLE parameter to filter as many warm pixels as your application requires.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct SetDriverControlParams
        {
            /// <summary>
            /// the parameter to modify. see also: <seealso cref="DriverControlParam"/> enum.
            /// </summary>
            public DriverControlParam controlParameter;
            /// <summary>
            /// the value of the control parameter.
            /// </summary>
            public UInt32 controlValue;
        }

        /// <summary>
        /// Requests the value of a driver control parameter.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetDriverControlParams
        {
            /// <summary>
            /// the driver parameter to be retrieved. see also: <seealso cref="DriverControlParam"/> enum.
            /// </summary>
            public DriverControlParam controlParameter;
        }

        /// <summary>
        /// Returns the value of a driver control parameter.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetDriverControlResults
        {
            /// <summary>
            /// The value of the requested driver parameter. see also: <seealso cref="DriverControlParam"/> enum.
            /// </summary>
            public DriverControlParam controlValue;
        }


        /// <summary>
        /// This command is used to modify the USB cameras A/D gain and offset registers.
        /// This command is intended for OEM use only. 
        /// The typical application does not need to use this command as the USB cameras initialize the A/D to factory set defaults when the camera powers up.
        /// </summary>
        /// <remarks>
        /// * For the USB_AD_IMAGING_GAIN and AD_USB_TRACKING_GAIN commands the allowed
        ///   setting for the data parameter is 0 through 63. The actual Gain of the A/D (in Volts/Volt) ranges
        ///   from 1.0 to 6.0 and is determined by the following formula:
        ///   Gain = 6.0 / ( 1.0 + 5.0 * ( (63 - data) / 63 )
        ///   Note that the default A/D Gain set by the camera at power up is 1.2 for the Imaging CCD and 2.0
        ///   for the Tracking CCD. Furthermore, the gain item reported by the Get CCD Info command will
        ///   always report the default factory-set gain and will not change based upon changes made to the
        ///   A/D gain by this command.
        /// * For the USB_AD_IMAGING_OFFSET and USB_AD_TRACKING_OFFSET commands the
        ///   allowed setting for the data parameter is -255 through 255. Positive offsets increase the video
        ///   black level in ADUs. The cameras are programmed at the factory to typically have a 900 to 1000
        ///   ADU black level offset.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct USBADControlParams
        {
            /// <summary>
            /// Imaging/Tracking Gain or offset. see also: <seealso cref="USB_AD_ControlCommand"/> enum.
            /// </summary>
            public USB_AD_ControlCommand command;
            /// <summary>
            /// Command specific.
            /// </summary>
            public short data;
        }

        /// <summary>
        /// Results for a single USB query.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QUERY_USB_INFO
        {
            /// <summary>
            /// TRUE if a camera was found.
            /// </summary>
            public MY_LOGICAL cameraFound;
            /// <summary>
            /// Camera type found. see also: <seealso cref="CameraType"/> enum.
            /// </summary>
            public CameraType cameraType;
            /// <summary>
            /// null terminated string. Name of found camera.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string name;
            /// <summary>
            /// null terminated string. Serial number of found camera.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            public string serialNumber;
        }

        /// <summary>
        /// Returns a list of up to four cameras found by the driver via USB.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QueryUsbResults
        {
            /// <summary>
            /// Number of cameras found. (Max 4)
            /// </summary>
            public UInt16 camerasFound;
            /// <summary>
            /// Information returned by cameras.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public QUERY_USB_INFO[] dev;
        }

        /// <summary>
        /// Returns a list of up to eight cameras found by the driver via USB.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QueryUSBResults2
        {
            /// <summary>
            /// Number of cameras found. (Max 8)
            /// </summary>
            public UInt16 camerasFound;
            /// <summary>
            /// Information returned by cameras.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public QUERY_USB_INFO[] usbInfo;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QueryUSBResults3
        {
            /// <summary>
            /// Number of cameras found. (Max 24)
            /// </summary>
            public UInt16 camerasFound;
            /// <summary>
            /// Information returned by cameras.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 24)]
            public QUERY_USB_INFO[] usbInfo;
        }

        /// <summary>
        /// Returned information for a single device over Ethernet.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
            /// Camera type found. see also: <seealso cref="CameraType"/> enum.
            /// </summary>
            public CameraType cameraType;
            /// <summary>
            /// null terminated string. Name of found camera.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string name;
            /// <summary>
            /// null terminated string. Serial number of found camera. 
            /// </summary>
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            public string serialNumber;
        }

        /// <summary>
        /// Returns a list of up to eight cameras found by the driver via Ethernet.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QueryEthernetResults
        {
            /// <summary>
            /// Number of cameras found
            /// </summary>
            public UInt16 camerasFound;
            /// <summary>
            /// Information of found devices.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public QUERY_ETHERNET_INFO[] ethernetInfo;
        }

        /// <summary>
        /// Returns a list of up to eight cameras found by the driver via Ethernet.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QueryEthernetResults2
        {
            /// <summary>
            /// Number of cameras found
            /// </summary>
            public UInt16 camerasFound;
            /// <summary>
            /// Information of found devices.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public QUERY_ETHERNET_INFO[] ethernetInfo;
        }

        /// <summary>
        /// This command is used to read a Pentium processor internal cycle counter. 
        /// Pentium processors have a 32 or 64 bit register that increments every clock cycle. 
        /// For example on a 1 GHz Pentium the counter advances 1 billion counts per second. 
        /// This command can be used to retrieve that counter.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetPentiumCycleCountParams
        {
            /// <summary>
            /// number of bits to shift the results to the right (dividing by 2)
            /// </summary>
            public UInt16 rightShift;
        }

        /// <summary>
        /// This command is used to read a Pentium processor internal cycle counter. 
        /// Pentium processors have a 32 or 64 bit register that increments every clock cycle. 
        /// For example on a 1 GHz Pentium the counter advances 1 billion counts per second. 
        /// This command can be used to retrieve that counter.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
        }

        //TODO: 移植uchar類型
        /// <summary>
        /// This command is used read or write data to the USB cameras I2C expansion port. 
        /// It writes the supplied data to the I2C port, or reads data from the supplied address.
        /// 
        /// This command is typically called by SBIG code in the Universal Driver.
        /// If you think you have some reason to call this function you should check with SBIG first.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct RWUSBI2CParams
        {
            /// <summary>
            /// Address to read from or write to
            /// </summary>
           // public uchar address;
            /// <summary>
            /// Data to write to the external I2C device, ignored for read
            /// </summary>
          //  public uchar data;
            /// <summary>
            /// TRUE when write is desired , FALSE when read is desired
            /// </summary>
            public MY_LOGICAL write;
            /// <summary>
            /// Device Address of the I2C peripheral
            /// </summary>
           // public uchar deviceAddress;
        }

        //TODO: 移植uchar類型
        /// <summary>
        /// This command is used read or write data to the USB cameras I2C expansion port.
        /// It returns the result of the read request.
        /// 
        /// This command is typically called by SBIG code in the Universal Driver.
        /// If you think you have some reason to call this function you should check with SBIG first.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct RWUSBI2CResults
        {
            /// <summary>
            /// Data read from the external I2C device
            /// </summary>
           // public uchar data;
        }

        //"bullet" | "number" | "table"
        //TODO: 移植uchar類型指標
        /// <summary>
        /// The CFW Command is a high-level API for controlling the SBIG color filter wheels. 
        /// It supports the 
        /// <list type="number">   
        ///     <item> CFW-2 (two position shutter wheel in the ST-5C/237), </item> 
        ///     <item> the CFW-5 (internal color filter wheel for the ST-5C/237), </item> 
        ///     <item> the CFW-8, </item> 
        ///     <item> the internal filter wheel (CFW-L) in the ST-L Large Format Camera, </item> 
        ///     <item> the internal filter wheel (CFW-402) in the ST-402 camera, </item> 
        ///     <item> the old 6-position CFW-6A, </item> 
        ///     <item> the 10-position CFW-10 in both I2C and RS-232 interface modes, </item> 
        ///     <item> the I2C based CFW-9 and 8-position CFW for the STL (CFW-L8), </item> 
        ///     <item> the five (FW5-STX) and seven (FW7-STX) position CFWs for the STX, </item> 
        ///     <item> the five (FW5-8300) and eight (FW8-8300) position CFWs for the ST-8300 and the eight (FW8-STT) position CFW for the STT cameras. </item> 
        /// </list>  
        /// </summary>
        /// <remarks>
        /// * CFW Command CFWC_QUERY
        ///   Use this command to monitor the progress of the Goto sub-command. 
        ///   This command takes no additional parameters in the CFParams.
        ///   You would typically do this several times a second after the issuing the Goto command until it reports CFWS_IDLE in the cfwStatus entry of the CFWResults. 
        ///   Additionally filter wheels that can report their current position (all filter wheels except the CFW-6A or CFW-8) have that position reported in cfwPosition entry of the CFWResults.
        /// * CFW Command CFWC_GOTO
        ///   Use this command to start moving the color filter wheel towards a given position.
        ///   Set the desired position in the cfwParam1 entry with entries defined by the CFW_POSITION enum.
        /// * CFW Command CFWC_INIT
        ///   Use this command to initialize/self-calibrate the color filter wheel. 
        ///   All SBIG color filter wheels self calibrate on power-up and should not require further initialization. 
        ///   We offer this option for users that experience difficulties with their color filter wheels or when changing between the CFW-2 and CFW-5 in the ST-5C/237. 
        ///   This command takes no additional parameters in the CFWParams struct.
        /// * CFW Command CFWC_GET_INFO
        ///   This command supports several sub-commands as determined by the cfwParam1 entry (see the CFW_GETINFO_SELECT enum). 
        ///   Command CFWG_FIRMWARE_VERSION returns the version of the CFW firmware in the cfwResults1 entry of the CFWResults and the number of filter positions the CFW supports in the cfwResults2 entry , commands CFWG_DATA_REGISTERS and CFWG_CAL_DATA are for internal SBIG use only and all other commands are undefined.
        /// * CFWC_OPEN_DEVICE and CFWC_CLOSE_DEVICE:
        ///   These commands are used to Open and Close any OS based communications port associated with the CFW and should proceed the first command sent and follow the last command sent to the CFW. 
        ///   While strictly only required for the RS-232 version of the CFW-10 calling these commands is a good idea for future compatibility. 
        ///   For the RS-232 based CFW-10 set the cfwParam1 entry to one of the settings CFW_COM_PORT enum to indicate which PC COM port is used to control the CFW-10. 
        ///   Again, only the RS232 controlled CFW-10 requires these calls.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct CFWParams
        {
            /// <summary>
            /// see also: <seealso cref="CFW_ModelSelect"/> enum.
            /// </summary>
            public CFW_ModelSelect cfwModel;
            /// <summary>
            /// see also: <seealso cref="CFW_Command"/> enum.
            /// </summary>
            public CFW_Command cfwCommand;
            /// <summary>
            /// command specific.
            /// </summary>
            public UInt32 cfwParam1;
            /// <summary>
            /// command specific.
            /// </summary>
            public UInt32 cfwParam2;
            /// <summary>
            /// command specific.
            /// </summary>
            public UInt16 outLength;
            /// <summary>
            /// command specific.
            /// </summary>
            //  public uchar* outPtr;
            /// <summary>
            /// command specific.
            /// </summary>
            public UInt16 inLength;
            /// <summary>
            /// command specific.
            /// </summary>
           // public uchar* inPtr;
        }


        /// <summary>
        /// The CFW Command is a high-level API for controlling the SBIG color filter wheels. 
        /// It supports the
        /// <list type="number">   
        ///     <item> CFW-2 (two position shutter wheel in the ST-5C/237), </item> 
        ///     <item> the CFW-5 (internal color filter wheel for the ST-5C/237), </item> 
        ///     <item> the CFW-8, </item> 
        ///     <item> the internal filter wheel (CFW-L) in the ST-L Large Format Camera, </item> 
        ///     <item> the internal filter wheel (CFW-402) in the ST-402 camera, </item> 
        ///     <item> the old 6-position CFW-6A, </item> 
        ///     <item> the 10-position CFW-10 in both I2C and RS-232 interface modes, </item> 
        ///     <item> the I2C based CFW-9 and 8-position CFW for the STL (CFW-L8), </item> 
        ///     <item> the five (FW5-STX) and seven (FW7-STX) position CFWs for the STX, </item> 
        ///     <item> the five (FW5-8300) and eight (FW8-8300) position CFWs for the ST-8300 and the eight (FW8-STT) position CFW for the STT cameras. </item> 
        /// </list>  
        /// </summary>
        /// <remarks>
        /// * CFW Command CFWC_QUERY
        ///   Use this command to monitor the progress of the Goto sub-command.
        ///   This command takes no additional parameters in the CFParams. 
        ///   You would typically do this several times a second after the issuing the Goto command until it reports CFWS_IDLE in the cfwStatus entry of the
        ///   CFWResults. 
        ///   Additionally filter wheels that can report their current position (all filter wheels except the CFW-6A or CFW-8) have that position reported in cfwPosition entry of the CFWResults.
        /// * CFW Command CFWC_GOTO
        ///   Use this command to start moving the color filter wheel towards a given position. 
        ///   Set the desired position in the cfwParam1 entry with entries defined by the CFW_POSITION enum.
        /// * CFW Command CFWC_INIT
        ///   Use this command to initialize/self-calibrate the color filter wheel. 
        ///   All SBIG color filter wheels self calibrate on power-up and should not require further initialization. 
        ///   We offer this option for users that experience difficulties with their color filter wheels or when changing between the CFW-2 and CFW-5 in the ST-5C/237. 
        ///   This command takes no additional parameters in the CFWParams struct.
        /// * CFW Command CFWC_GET_INFO
        ///   This command supports several sub-commands as determined by the cfwParam1 entry (see the <seealso cref="CFW_GetInfoSelect"/> enum). 
        ///   Command CFWG_FIRMWARE_VERSION returns the version of the CFW firmware in the cfwResults1 entry of the CFWResults and the number of filter positions the CFW supports in the cfwResults2 entry, commands CFWG_DATA_REGISTERS and CFWG_CAL_DATA are for internal SBIG use only and all other commands are undefined.
        /// * CFWC_OPEN_DEVICE and CFWC_CLOSE_DEVICE:
        ///   These commands are used to Open and Close any OS based communications port associated with the CFW and should proceed the first command sent and follow the last command sent to the CFW. 
        ///   While strictly only required for the RS-232 version of the CFW-10 calling these commands is a good idea for future compatibility. 
        ///   For the RS-232 based CFW-10 set the cfwParam1 entry to one of the settings CFW_COM_PORT enum to indicate which PC COM port is used to control the CFW-10. 
        ///   Again, only the RS232 controlled CFW-10 requires these calls.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct CFWResults
        {
            /// <summary>
            /// see also: <seealso cref="CFW_ModelSelect"/> enum.
            /// </summary>
            public CFW_ModelSelect cfwModel;
            /// <summary>
            /// see also: <seealso cref="CFW_POSITION"/> enum.
            /// </summary>
            public CFW_POSITION cfwPosition;
            /// <summary>
            /// see also: <seealso cref="CFW_Status"/> enum.
            /// </summary>
            public CFW_Status cfwStatus;
            /// <summary>
            /// see also: <seealso cref="CFW_Error"/> enum.
            /// </summary>
            public CFW_Error cfwError;
            /// <summary>
            /// command specific
            /// </summary>
            public UInt32 cfwResult1;
            /// <summary>
            /// command specific
            /// </summary>
            public UInt32 cfwResult2;
        }

        /// <summary>
        /// This command is used read or write control bits in the USB cameras.
        /// 
        /// On the ST-L camera you can use this command to monitor whether the input power supply has dropped to the point where you ought to warn the user. 
        /// Do this by issuing a Read operation on bit 0 and if that bit is set the power has dropped below 10 Volts.
        /// <para>
        /// bitName values:
        /// * 0=Read Power Supply Low Voltage, 
        /// * 1=Write Genl. Purp. Bit 1,
        /// * 2=Write Genl. Purp. Bit 2, 
        /// * 3=Read Genl. Purp. Bit 3
        /// </para>
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct BitIOParams
        {
            /// <summary>
            /// 0=Write, 1=Read. see also: <seealso cref="Bit_IO_Operation"/> enum.
            /// </summary>
            public Bit_IO_Operation bitOperation;
            /// <summary>
            /// see also: <seealso cref="BITIO_NAME"/> enum.
            /// </summary>
            public BIT_IO_Name bitName;
            /// <summary>
            /// 1=Set Bit, 0=Clear Bit
            /// </summary>
            public MY_LOGICAL setBit;
        }

        /// <summary>
        /// This command is used read or write control bits in the USB cameras.
        ///
        /// On the ST-L camera you can use this command to monitor whether the input power supply has dropped to the point where you ought to warn the user. 
        /// Do this by issuing a Read operation on bit 0 and if that bit is set the power has dropped below 10 Volts.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct BitIOResults
        {
            /// <summary>
            /// 1=Bit is set, 0=Bit is clear
            /// </summary>
            public MY_LOGICAL bitIsSet;
        }

        //TODO: 移植uchar類型指標
        /// <summary>
        /// Read or write a block of data the user space in the EEPROM.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct UserEEPROMParams
        {
            /// <summary>
            /// TRUE to write data to user EEPROM space, FALSE to read.
            /// </summary>
            public MY_LOGICAL writeData;
            /// <summary>
            /// Buffer of data to be written.
            /// </summary>
            /// public uchar data[32];
        }

        //TODO: 移植uchar類型指標
        /// <summary>
        /// Read or write a block of data the user space in the EEPROM.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct UserEEPROMResults
        {
            /// <summary>
            /// TRUE to write data to user EEPROM space, FALSE to read.
            /// </summary>
            public MY_LOGICAL writeData;
            /// <summary>
            /// Buffer of data to be written.
            /// </summary>
            /// public uchar data[32];
        }

        /// <summary>
        /// Internal SBIG use only.
        /// This command is used read or write the STF-8300's Column Repair data stored in the camera for use with that camera's Auto Filter Capability.
        /// </summary>
        /// <remarks>
        /// * The left most column is column 1 (not zero). 
        ///   Specifying a column zero doesn't filter any columns.
        /// * This command is somewhat unique in that the Parameters and the Results are the same struct.
        /// * To enable column filtering you must use this command and also the Set Driver Control command to set the DCP_COLUMN_REPAIR parameter to 1.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct ColumnEEPROMParams // ColumnEEPROMResults
        {
            /// <summary>
            /// TRUE to write data to specified EEPROM column, FALSE to read.
            /// </summary>
            public MY_LOGICAL writeData;
            /// <summary>
            /// Specify up to 7 columns to repair.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public UInt16[] columns;
            /// <summary>
            /// not used at this time.
            /// </summary>
            public UInt16 flags;
        }

        /// <summary>
        /// Internal SBIG use only.
        /// This command is used read or write the STF-8300's Column Repair data stored in the camera for use with that camera's Auto Filter Capability.
        /// </summary>
        /// <remarks>
        /// * The left most column is column 1 (not zero). 
        ///   Specifying a column zero doesn't filter any columns.
        /// * This command is somewhat unique in that the Parameters and the Results are the same struct.
        /// * To enable column filtering you must use this command and also the Set Driver Control command to set the DCP_COLUMN_REPAIR parameter to 1.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct ColumnEEPROMResults
        {
            /// <summary>
            /// TRUE to write data to specified EEPROM column, FALSE to read.
            /// </summary>
            public MY_LOGICAL writeData;
            /// <summary>
            /// Specify up to 7 columns to repair.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 7)]
            public UInt16[] columns;
            /// <summary>
            /// not used at this time.
            /// </summary>
            public UInt16 flags;
        }

        //TODO: 移植uchar類型
        /// <summary>
        /// Send the Biorad setup to the camera, returning any error.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct BTDISetupParams
        {
            /// <summary>
            /// Row period.
            /// </summary>
           // public uchar rowPeriod;
        }

        /// <summary>
        /// Results of the Biorad setup, returning any error.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct BTDISetupResults
        {
            /// <summary>
            /// Results of the command. see also: <seealso cref="BTDI_Error"/> enum.
            /// </summary>
            public BTDI_Error btdiErrors;
        }

        //TODO: 移植uchar類型指標
        /// <summary>
        /// The Motor Focus Command is a high-level API for controlling SBIG Motor Focus accessories. 
        /// It supports the new ST Motor Focus unit and will be expanded as required to support new models in the future.
        /// </summary>
        /// <remarks>
        /// * Motor Focus Command MFC_QUERY
        ///   Use this command to monitor the progress of the Goto sub-command. 
        ///   This command takes no additional parameters in the MFParams. You would typically do this several times a second after the issuing the Goto command until it reports MFS_IDLE in the mfStatus entry of the MFResults.
        ///   Motor Focus accessories report their current position in the mfPosition entry of the MFResults struct where the position is a signed long with 0 designating the center of motion or the home position. 
        ///   Also the Temperature in hundredths of a degree-C is reported in the mfResult1 entry.
        /// * Motor Focus Command MFC_GOTO
        ///   Use this command to start moving the Motor Focus accessory towards a given position. 
        ///   Set the desired position in the mfParam1 entry. 
        ///   Again, the position is a signed long with 0 representing the center or home position.
        /// * Motor Focus Command MFC_INIT
        ///   Use this command to initialize/self-calibrate the Motor Focus accessory. 
        ///   This causes the Motor Focus accessory to find the center or Home position. 
        ///   You can not count on SBIG Motor Focus accessories to self calibrate upon power-up and should issue this command upon first establishing a link to the Camera. 
        ///   Additionally you should retain the last position of the Motor Focus accessory in a parameter file and after initializing the Motor Focus accessory, you should return it to its last position. 
        ///   Finally, note that this command takes no additional parameters in the MFParams struct.
        /// * Motor Focus Command MFC_GET_INFO
        ///   This command supports several sub-commands as determined by the mfParam1 entry (see the MF_GETINFO_SELECT enum). 
        ///   Command MFG_FIRMWARE_VERSION returns the version of the Motor Focus firmware in the mfResults1 entry of the MFResults and the Maximum Extension (plus or minus) that the Motor Focus supports is in the mfResults2 entry. 
        ///   The MFG_DATA_REGISTERS command is internal SBIG use only and all other commands are undefined.
        /// * Motor Focus Command MFC_ABORT
        ///   Use this command to abort a move in progress from a previous Goto command. 
        ///   Note that this will not abort an Init.
        /// 
        /// Notes: 
        /// * The Motor Focus Command takes pointers to MFParams as parameters and MFResults as results.
        /// * Set the mfModel entry in the MFParams to the type of Motor Focus accessory you want to control. 
        ///   The same value is returned in the mfModel entry of the MFResults. 
        ///   If you select the MFSEL_AUTO option the driver will use the most appropriate model and return the model it found in the mfModel entry of the MFResults.
        /// * The Motor Focus Command is a single API call that supports multiple sub-commands through the mfCommand entry in the MFParams. 
        ///   Each of the sub-commands requires certain settings of the MFParams entries and returns varying results in the MFResults. 
        ///   Each of these sub-commands is discussed in detail above.
        /// * As with all API calls the Motor Focus Command returns an error code. 
        ///   If the error code is CE_MF_ERROR, then in addition the mfError entry in the MFResults further enumerates the error.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct MFParams
        {
            /// <summary>
            /// see also: <seealso cref="MF_ModelSelect"/> enum.
            /// </summary>
            public MF_ModelSelect mfModel;
            /// <summary>
            /// see also: <seealso cref="MF_Command"/> enum.
            /// </summary>
            public MF_Command mfCommand;
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
            /// <summary>
            /// command specific.
            /// </summary>
           // public uchar* outPtr;
            /// <summary>
            /// command specific.
            /// </summary>
            public UInt16 inLength;
            /// <summary>
            /// command specific.
            /// </summary>
           // public uchar* inPtr;
        }

        /// <summary>
        /// The Motor Focus Command is a high-level API for controlling SBIG Motor Focus accessories. 
        /// It supports the new ST Motor Focus unit and will be expanded as required to support new models in the future.
        /// </summary>
        /// <remarks>
        /// * Motor Focus Command MFC_QUERY
        ///   Use this command to monitor the progress of the Goto sub-command. 
        ///   This command takes no additional parameters in the MFParams. 
        ///   You would typically do this several times a second after the issuing the Goto command until it reports MFS_IDLE in the mfStatus entry of the MFResults. 
        ///   Motor Focus accessories report their current position in the mfPosition entry of the MFResults struct where the position is a signed long with 0 designating the center of motion or the home position. Also the Temperature in hundredths of a degree-C is reported in the mfResult1 entry.
        /// * Motor Focus Command MFC_GOTO
        ///   Use this command to start moving the Motor Focus accessory towards a given position. 
        ///   Set the desired position in the mfParam1 entry. 
        ///   Again, the position is a signed long with 0 representing the center or home position.
        /// * Motor Focus Command MFC_INIT
        ///   Use this command to initialize/self-calibrate the Motor Focus accessory.
        ///   This causes the Motor Focus accessory to find the center or Home position.
        ///   You can not count on SBIG Motor Focus accessories to self calibrate upon power-up and should issue this command upon first establishing a link to the Camera. 
        ///   Additionally you should retain the last position of the Motor Focus accessory in a parameter file and after initializing the Motor Focus accessory, you should return it to its last position. 
        ///   Finally, note that this command takes no additional parameters in the MFParams struct.
        /// * Motor Focus Command MFC_GET_INFO
        ///   This command supports several sub-commands as determined by the mfParam1 entry (see the
        ///   MF_GETINFO_SELECT enum). 
        ///   Command MFG_FIRMWARE_VERSION returns the version of the Motor Focus firmware in the mfResults1 entry of the MFResults and the Maximum Extension (plus or minus) that the Motor Focus supports is in the mfResults2 entry.
        ///   The MFG_DATA_REGISTERS command is internal SBIG use only and all other commands are undefined.
        /// * Motor Focus Command MFC_ABORT
        ///   Use this command to abort a move in progress from a previous Goto command. 
        ///   Note that this will not abort an Init.
        /// 
        /// Notes: 
        /// * The Motor Focus Command takes pointers to MFParams as parameters and MFResults as results.
        /// * Set the mfModel entry in the MFParams to the type of Motor Focus accessory you want to control. 
        ///   The same value is returned in the mfModel entry of the MFResults. 
        ///   If you select the MFSEL_AUTO option the driver will use the most appropriate model and return the model it found in the mfModel entry of the MFResults.
        /// * The Motor Focus Command is a single API call that supports multiple sub-commands through the mfCommand entry in the MFParams. Each of the sub-commands requires certain settings of the MFParams entries and returns varying results in the MFResults. 
        ///   Each of these sub-commands is discussed in detail above.
        /// * As with all API calls the Motor Focus Command returns an error code. 
        ///   If the error code is CE_MF_ERROR, then in addition the mfError entry in the MFResults further enumerates the error.
        /// </remarks>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct MFResults
        {
            /// <summary>
            /// see also: <seealso cref="MF_ModelSelect"/> enum. 
            /// </summary>
            public MF_ModelSelect mfModel;
            /// <summary>
            /// position of the Motor Focus, 0=Center, signed. 
            /// </summary>
            public Int32 mfPosition;
            /// <summary>
            /// see also: <seealso cref="MF_Status"/> enum.
            /// </summary>
            public MF_Status mfStatus;
            /// <summary>
            /// see also: <seealso cref="MF_Error"/>  enum.
            /// </summary>
            public MF_Error mfError;
            /// <summary>
            /// command specific.
            /// </summary>
            public Int32 mfResult1;
            /// <summary>
            /// command specific.
            /// </summary>
            public Int32 mfResult2;
        }

        //TODO: 移植uchar類型指標
        /// <summary>
        /// Differential Guider Command Guide:
        /// * DGC_DETECT detects whether a Differential Guide unit is connected to the camera.
        ///   Command takes no arguments.
        /// * DGC_GET_BRIGHTNESS obtains the brightness setting of the red and IR LEDs in the differential guide unit.
        ///   inPtr should be a pointer to a DGLEDState struct.
        /// * DGC_SET_BRIGHTNESS sets the brightness registers of the red and IR LEDs in the differential guide unit.
        ///   outPtr should be a pointer to a DGLEDState struct with the desired values register values set.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct DiffGuiderParams
        {
            /// <summary>
            /// Command for Differential Guider. see also: <seealso cref="DiffGuiderCommand"/> enum. 
            /// </summary>
            public DiffGuiderCommand diffGuiderCommand;
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
            /// <summary>
            /// output buffer. Command specific.
            /// </summary>
          //  public uchar* outPtr;
            /// <summary>
            /// Size of input buffer. Command specific.
            /// </summary>
            public UInt16 inLength;
            /// <summary>
            /// input buffer. Command specific.
            /// </summary>
           // public uchar* inPtr;
        }

        /// <summary>
        /// Returned results of a Differential Guider Command.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct DiffGuiderResults
        {
            /// <summary>
            /// see also: <seealso cref="DiffGuiderError"/> enum.
            /// </summary>
            public DiffGuiderError diffGuiderError;
            /// <summary>
            /// see also: <seealso cref="DiffGuiderStatus"/> enum.
            /// </summary>
            public DiffGuiderStatus diffGuiderStatus;
            /// <summary>
            /// Unused.
            /// </summary>
            public UInt32 diffGuiderResult1;
            /// <summary>
            /// Unused.
            /// </summary>
            public UInt32 diffGuiderResult2;
        }

        /// <summary>
        /// State of the Differential Guider LEDs.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
        }

        //TODO: 移植char類型指標
        /// <summary>
        /// Internal SBIG use only. Implement the Bulk IO command which is used for Bulk Reads/Writes to the camera for diagnostic purposes.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct BulkIOParams
        {
            /// <summary>
            /// see also: <seealso cref="BulkIO_Command"/> enum.
            /// </summary>
            public BulkIO_Command command;
            /// <summary>
            /// TRUE if reading/writing data to/from the Pixel pipe, FALSE to read/write from the com pipe.
            /// </summary>
            public MY_LOGICAL isPixelData;
            /// <summary>
            /// Length of data buffer.
            /// </summary>
            public UInt32 dataLength;
            /// <summary>
            /// data buffer.
            /// </summary>
           // public char* dataPtr;
        }

        /// <summary>
        /// Internal SBIG use only. Results of a Bulk I/O command.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct BulkIOResults
        {
            /// <summary>
            /// Bytes sent/received.
            /// </summary>
            public UInt32 dataLength;
        }

        /// <summary>
        /// This command is used read or write the STX/STXL/STT customer options.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct CustomerOptionsParams // CustomerOptionsResults
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
        }

        /// <summary>
        /// This command is used read or write the STX/STXL/STT customer options.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
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
        }

        /// <summary>
        /// Results of a CC_GET_AO_MODEL command.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetI2CAoModelResults
        {
            /// <summary>
            /// AO model.
            /// </summary>
            public UInt16 i2cAoModel;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetCcdInfoParams
        {
            public CcdInfoRequest req;

            public GetCcdInfoParams(CcdInfoRequest req)
            {
                this.req = req;
            }
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetReadoutInProgressResults
        {
            /// <summary>
            /// Readout In Progress. TRUE if RIP, FALSE otherwise.
            /// </summary>
            public MY_LOGICAL RIP;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct SetRBIPreflashParams
        {
            public UInt16 darkFrameLength;
            public UInt16 flushCount;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct GetRBIPreflashResults
        {
            public UInt16 darkFrameLength;
            public UInt16 flushCount;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct QueryFeatureSupportedParams
        {
            /// <summary>
            /// Feature to query for firmware support.
            /// </summary>
            public FeatureFirmwareRequirement ffr;
        }

        public struct QueryFeatureSupportedResults
        {
            /// <summary>
            /// TRUE if feature is supported, FALSE otherwise.
            /// </summary>
            public MY_LOGICAL result;
        }

        //TODO: 找尋正確的移植方式
        ///// <summary>
        ///// Internal SBIG use only. Queries Start/End exposure performance tracking.
        ///// </summary>
        //public struct QueryExposureTicksResults
        //{
        //    /// <summary>
        //    /// Start exposure tick initial value.
        //    /// </summary>
        //    public LARGE_INTEGER startExposureTicks0;
        //    /// <summary>
        //    /// Start exposure tick final value.
        //    /// </summary>
        //    public LARGE_INTEGER startExposureTicks1;
        //    /// <summary>
        //    /// End exposure tick initial value.
        //    /// </summary>
        //    public LARGE_INTEGER endExposureTicks0;
        //    /// <summary>
        //    /// End exposure tick final value.
        //    /// </summary>
        //    public LARGE_INTEGER endExposureTicks1;
        //}

        /// <summary>
        /// gets thrown whenever an SBIG operation doesn't return success (CE_NO_ERROR)
        /// </summary>
        public class FailedOperationException : Exception
        {
            public PAR_ERROR ErrorCode { get; private set; }

            public FailedOperationException(PAR_ERROR errorCode)
            {
                ErrorCode = errorCode;
            }

            public override string Message
            {
                get
                {
                    return ErrorCode.ToString();
                }
            }
        } // class FailedOperation

        #endregion Types

        //
        #region DEBUG
        //

        //TODO: 檢查列舉定義
        /// <summary>
        /// Flags for enabling debug messages of CC_***_*** commands.
        /// </summary>
        [Flags]
        public enum DEBUG_LOG_CC_FLAGS
        {
            /// <summary>
            /// Log MC_SYSTEM, CC_BREAKPOINT, CC_OPEN_*, CC_CLOSE_*, etc.
            /// </summary>
            DLF_CC_BASE = 0x0001,
            /// <summary>
            /// Log readout commands.
            /// </summary>
            DLF_CC_READOUT = 0x0002,
            /// <summary>
            /// Log status commands.
            /// </summary>
            DLF_CC_STATUS = 0x0004,
            /// <summary>
            /// Log temperature commands.
            /// </summary>
            DLF_CC_TEMPERATURE = 0x0008,
            /// <summary>
            /// Log filter wheel commands.
            /// </summary>
            DLF_CC_CFW = 0x0010,
            /// <summary>
            /// Log AO commands
            /// </summary>
            DLF_CC_AO = 0x0020,
            /// <summary>
            /// Unused.
            /// </summary>
            DLF_CC_40 = 0x0040,
            /// <summary>
            /// Unused.
            /// </summary>
            DLF_CC_80 = 0x0080
        }

        //TODO: 檢查列舉定義
        /// <summary>
        /// Flags for enabling debug messages of MC_***_*** commands.
        /// </summary>
        [Flags]
        public enum DEBUG_LOG_MC_FLAGS
        {
            /// <summary>
            /// Log MC_START_*, MC_END_*, MC_OPEN_*, MC_CLOSE_*, etc...
            /// </summary>
            DLF_MC_BASE = 0x0001,
            /// <summary>
            /// Log readout commands at microcommand level.
            /// </summary>
            DLF_MC_READOUT = 0x0002,
            /// <summary>
            /// Log status commands at microcommand level.
            /// </summary>
            DLF_MC_STATUS = 0x0004,
            /// <summary>
            /// Log temperature commands at microcommand level.
            /// </summary>
            DLF_MC_TEMPERATURE = 0x0008,
            /// <summary>
            /// Log EEPROM microcommands.
            /// </summary>
            DLF_MC_EEPROM = 0x0010,
            /// <summary>
            /// Unused.
            /// </summary>
            DLF_MC_20 = 0x0020,
            /// <summary>
            /// Unused.
            /// </summary>
            DLF_MC_40 = 0x0040,
            /// <summary>
            /// Unused.
            /// </summary>
            DLF_MC_80 = 0x0080
        }

        //TODO: 檢查列舉定義
        /// <summary>
        /// Flags for enabling debug messages of communication methods.
        /// </summary>
        [Flags]
        public enum DEBUG_LOG_FCE_FLAGS
        {
            /// <summary>
            /// Log Ethernet communication functions.
            /// </summary>
            DLF_FCE_ETH = 0x0001,
            /// <summary>
            /// Log USB communication functions.
            /// </summary>
            DLF_FCE_USB = 0x0002,
            /// <summary>
            /// Log FIFO communication functions.
            /// </summary>
            DLF_FCE_FIFO = 0x0004,
            /// <summary>
            /// Unused.
            /// </summary>
            DLF_FCE_0008 = 0x0008,
            /// <summary>
            /// Unused.
            /// </summary>
            DLF_FCE_0010 = 0x0010,
            /// <summary>
            /// Unused.
            /// </summary>
            DLF_FCE_0020 = 0x0020,
            /// <summary>
            /// Unused.
            /// </summary>
            DLF_FCE_0040 = 0x0040,
            /// <summary>
            /// Log camera communication responses.
            /// </summary>
            DLF_FCE_CAMERA = 0x0080
        }

        /// <summary>
        /// Flags for enabling debug messages of I/O operations.
        /// </summary>
        [Flags]
        public enum DEBUG_LOG_IO_FLAGS
        {
            /// <summary>
            /// Log reading from com pipe.
            /// </summary>
            DLF_IO_RD_COM_PIPE = 0x0001,
            /// <summary>
            /// Log writing to com pipe.
            /// </summary>
            DLF_IO_WR_COM_PIPE = 0x0002,
            /// <summary>
            /// Log reading from pixel pipe.
            /// </summary>
            DLF_IO_RD_PIXEL_PIPE = 0x0004,
            /// <summary>
            ///  Log reading from alternate pixel pipe.
            /// </summary>
            DLF_IO_RD_ALT_PIPE = 0x0008,
            /// <summary>
            /// Log writing to alternate pixel pipe.
            /// </summary>
            DLF_IO_WR_ALT_PIPE = 0x0010,
            /// <summary>
            /// Log reading from Async I/O.
            /// </summary>
            DLF_IO_RD = 0x0020, //!< 
            /// <summary>
            /// Log writing to Async I/O.
            /// </summary>
            DLF_IO_WR = 0x0040, //!< 
            /// <summary>
            /// Unused.
            /// </summary>
            DLF_IO_0080 = 0x0080
        }

        /// <summary>
        /// Change debug logging, and path to log file.
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct DebugLogParams
        {
            /// <summary>
            /// Command flags.
            /// </summary>
            public DEBUG_LOG_CC_FLAGS ccFlags;
            /// <summary>
            /// Microcommand flags.
            /// </summary>
            public DEBUG_LOG_MC_FLAGS mcFlags;
            /// <summary>
            /// Communication flags.
            /// </summary>
            public DEBUG_LOG_FCE_FLAGS fceFlags;
            /// <summary>
            /// I/O flags.
            /// </summary>
            public DEBUG_LOG_IO_FLAGS ioFlags;
            /// <summary>
            /// Path to SBIGUDRV log file.
            /// </summary>
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 1024)]
            public string logFilePathName;
        }

        //TODO: 檢查移植方式，還無法正常使用
        /// <summary>
        /// A function used to expose writing to the log file to calling programs.
        /// Useful for debugging purposes.
        /// </summary>
        /// <param name="pStr">pointer to an array of characters, null-terminated, which should be written to the log file.</param>
        /// <param name="length">buffer's length in bytes.</param>
        /// <returns></returns>
        [DllImport("SBIGUDrv.dll")]
        private static extern short SBIGLogDebugMsg(
            [MarshalAs(UnmanagedType.LPStr)]StringBuilder pStr, UInt16 length);

        #endregion

        //
        #region Methods
        // 

        /// <summary>
        /// Direct pass-through to SBIG Universal Driver
        /// </summary>
        /// <param name="Command">the command to be executed</param>
        /// <param name="Parameters">inputs to the operation, if any</param>
        /// <param name="Results">outputs from the operation, if any</param>
        /// <returns>error code from Error enum (e.g. CE_CAMERA_NOT_FOUND)</returns>
        [DllImport("SBIGUDrv.dll")]
        private static extern PAR_ERROR SBIGUnivDrvCommand(
            PAR_COMMAND Command, IntPtr Parameters, IntPtr Results);

        /// <summary>
        /// Direct pass-through to SBIG Universal Driver.
        /// </summary>
        /// <param name="Command">the command to be executed</param>
        /// <param name="Parameters">inputs to the operation, if any</param>
        /// <param name="Results">outputs from the operation, if any</param>
        /// <exception cref="FailedOperationException">when SBIGUnivDrvCommand error.</exception>
        private static void _UnivDrvCommand(
            PAR_COMMAND Command, IntPtr Parameters, IntPtr Results)
        {
            PAR_ERROR err = SBIGUnivDrvCommand(Command, Parameters, Results);
            if (PAR_ERROR.CE_NO_ERROR != err)
                throw new FailedOperationException(err);
        }

        /// <summary>
        /// Calls the SBIG Universal Driver with a (possibly null) input parameter struct
        /// </summary>
        /// <param name="Command">the command to be executed</param>
        /// <param name="Parameters">inputs to the operation, null if none</param>
        /// <exception cref="FailedOperationException">throws a FailedOperation exception if command doesn't return CE_NO_ERROR</exception>
        public static void UnivDrvCommand(
            PAR_COMMAND Command, object Parameters)
        {
            // marshall the input structure, if it exists
            var ParamGch = NullGch;
            var ParamPtr = IntPtr.Zero;
            if (!(Parameters is null))
            {
                ParamGch = GCHandle.Alloc(Parameters, GCHandleType.Pinned);
                ParamPtr = ParamGch.AddrOfPinnedObject();
            }

            //
            // make the call
            //
            _UnivDrvCommand(Command, ParamPtr, IntPtr.Zero);

            // clean up
            if (IntPtr.Zero != ParamPtr)
                ParamGch.Free();
        }

        /// <summary>
        /// Calls the SBIG Universal Driver with a (possibly null) input parameter struct DIRECTLY WRITING output into Results
        /// </summary>
        /// <param name="Command">the command to be executed</param>
        /// <param name="Parameters">inputs to the operation, null if none</param>
        /// <param name="Results">array or structure to write command output DIRECTLY into (no marshalling occurs)</param>
        /// <exception cref="FailedOperationException">throws a FailedOperation exception if command doesn't return CE_NO_ERROR</exception>
        public static void UnivDrvCommand(
            PAR_COMMAND Command, object Parameters, object Results)
        {
            // marshall the input structure, if it exists
            var ParamGch = NullGch;
            var ParamPtr = IntPtr.Zero;
            if (!(Parameters is null))
            {
                ParamGch = GCHandle.Alloc(Parameters, GCHandleType.Pinned);
                ParamPtr = ParamGch.AddrOfPinnedObject();
            }
            // pin the output bytes while we pass the buffer to the SBIG SDK
            var ResultsGch = GCHandle.Alloc(Results, GCHandleType.Pinned);

            //
            // make the call
            //
            _UnivDrvCommand(Command, ParamPtr, ResultsGch.AddrOfPinnedObject());

            // clean up
            ResultsGch.Free();
            if (IntPtr.Zero != ParamPtr)
                ParamGch.Free();
        }

        /// <summary>
        /// Calls the SBIG Universal Driver, MARSHALLING the input parameter struct to do any necessary type translation
        /// Only use this version only when type translation is required (as it's slower)
        /// </summary>
        /// <param name="Command">the command to be executed</param>
        /// <param name="Parameters">inputs to the operation, null if none</param>
        /// <exception cref="FailedOperationException">throws a FailedOperation exception if command doesn't return CE_NO_ERROR</exception>
        public static void UnivDrvCommandMarshal(PAR_COMMAND Command, object Parameters)
        {
            // marshall the input structure, if it exists
            IntPtr ParamPtr = IntPtr.Zero;
            if (!(Parameters is null))
            {
                ParamPtr = Marshal.AllocHGlobal(Marshal.SizeOf(Parameters));
                Marshal.StructureToPtr(Parameters, ParamPtr, false);
            }

            //
            // make the call
            //
            _UnivDrvCommand(Command, ParamPtr, IntPtr.Zero);

            // clean up
            if (IntPtr.Zero != ParamPtr)
                Marshal.FreeHGlobal(ParamPtr);
        }

        /// <summary>
        /// Calls the SBIG Universal Driver with a (possibly null) input parameter struct DIRECTLY WRITING output into return value
        /// </summary>
        /// <param name="Command">the command to be executed</param>
        /// <param name="Parameters">inputs to the operation, null if none</param>
        /// <returns>object with command output written DIRECTLY into it (no marshalling occurs)</returns>
        /// <exception cref="FailedOperationException">throws a FailedOperation exception if command doesn't return CE_NO_ERROR</exception>
        public static T UnivDrvCommand<T>(PAR_COMMAND Command, object Parameters) where T : new()
        {
            // marshall the input structure, if it exists
            GCHandle ParamGch = NullGch;
            IntPtr ParamPtr = IntPtr.Zero;
            if (!(Parameters is null))
            {
                ParamGch = GCHandle.Alloc(Parameters, GCHandleType.Pinned);
                ParamPtr = ParamGch.AddrOfPinnedObject();
            }
            // prepare the output structure and pin it while we pass it to the SBIG SDK
            T Results = new T();
            GCHandle ResultsGch = GCHandle.Alloc(Results, GCHandleType.Pinned);
            IntPtr ResultsPtr = ResultsGch.AddrOfPinnedObject();

            //
            // make the call
            //
            _UnivDrvCommand(Command, ParamPtr, ResultsPtr);

            // clean up
            ResultsGch.Free();
            if (IntPtr.Zero != ParamPtr)
                ParamGch.Free();

            return Results;
        }

        /// <summary>
        /// Calls the SBIG Universal Driver with a (possibly null) input parameter struct MARSHALLING the output into return value
        /// Only use this version only when type translation is required (as it's slower)
        /// </summary>
        /// <param name="Command">the command to be executed</param>
        /// <param name="Parameters">inputs to the operation, null if none</param>
        /// <returns>object with command output MARSHALLED into it (types are translated as necessary)</returns>
        /// <exception cref="FailedOperationException">throws a FailedOperation exception if command doesn't return CE_NO_ERROR</exception>
        public static T UnivDrvCommandMarshal<T>(PAR_COMMAND Command, object Parameters)
        {
            // marshall the input structure, if it exists
            IntPtr ParamPtr = IntPtr.Zero;
            if (!(Parameters is null))
            {
                ParamPtr = Marshal.AllocHGlobal(Marshal.SizeOf(Parameters));
                Marshal.StructureToPtr(Parameters, ParamPtr, false);
            }
            // marshall the output structure
            IntPtr ResultsPtr = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(T)));

            //
            // make the call
            //
            _UnivDrvCommand(Command, ParamPtr, ResultsPtr);

            // un-marshal the output
            T Results = (T)Marshal.PtrToStructure(ResultsPtr, typeof(T));
            Marshal.FreeHGlobal(ResultsPtr);

            // clean up
            if (IntPtr.Zero != ParamPtr)
                Marshal.FreeHGlobal(ParamPtr);

            return Results;
        }

        /// <summary>
        /// calls UnivDrvCommand while (possibly taking in, but definitely) marshalling out a complex struct
        /// (defined as a non-primitive or non-blittable type, like an array)
        /// </summary>
        /// <param name="Command"></param>
        /// <param name="Parameters"></param>
        /// <param name="Results"></param>
        /// <exception cref="FailedOperationException">throws a FailedOperation exception if command doesn't return CE_NO_ERROR</exception>
        public static void UnivDrvCommand_OutComplex<TResult>(
            PAR_COMMAND Command, object Parameters, out TResult Results)
            where TResult : struct
        {
            Results = default(TResult);
            // marshall the input structure, if it exists
            GCHandle ParamGch = NullGch;
            IntPtr ParamPtr = IntPtr.Zero;
            if (!(Parameters is null))
            {
                ParamGch = GCHandle.Alloc(Parameters, GCHandleType.Pinned);
                ParamPtr = ParamGch.AddrOfPinnedObject();
            }
            // translate the struct into bytes, which are pinned
            IntPtr ResultsPtr = Marshal.AllocHGlobal(Marshal.SizeOf(Results));
            // pass true to free up any incoming memory, since we're going to overwrite it
            Marshal.StructureToPtr(Results, ResultsPtr, true);

            //
            // make the call
            //
            _UnivDrvCommand(Command, ParamPtr, ResultsPtr);

            // Marshall back
            //Marshal.PtrToStructure(ResultsPtr, Results);
            Results = (TResult)Marshal.PtrToStructure(ResultsPtr, typeof(TResult));

            // clean up
            Marshal.FreeHGlobal(ResultsPtr);
            if (IntPtr.Zero != ParamPtr)
                ParamGch.Free();
        }


        //
        // Exposure helpers
        //

        /// <summary>
        /// Waits for any exposure in progress to complete, ends it, and reads it out into a 2D ushort array
        /// </summary>
        public static ushort[,] WaitEndAndReadoutExposure(StartExposureParams2 sep)
        {
            // wait for the exposure to be done
            QueryCommandStatusResults qcsr = new QueryCommandStatusResults()
            {
                status = PAR_ERROR.CE_NO_ERROR
            };
            while (PAR_ERROR.CE_NO_EXPOSURE_IN_PROGRESS != qcsr.status)
                qcsr = UnivDrvCommand<QueryCommandStatusResults>(
                    PAR_COMMAND.CC_QUERY_COMMAND_STATUS,
                    new QueryCommandStatusParams()
                    {
                        command = PAR_COMMAND.CC_START_EXPOSURE
                    });

            // prepare the CCD for readout
            UnivDrvCommand(
                PAR_COMMAND.CC_END_EXPOSURE,
                new EndExposureParams()
                {
                    ccd = CCD_Request.CCD_IMAGING
                });
            // then telling it where and how we're going to read
            StartReadoutParams srp = new StartReadoutParams
            {
                ccd = CCD_Request.CCD_IMAGING,
                readoutMode = 0,
                left = 0,
                top = 0,
                width = sep.width,
                height = sep.height
            };
            UnivDrvCommand(PAR_COMMAND.CC_START_READOUT, srp);

            // allocate the image buffer
            ushort[,] data = new ushort[sep.height, sep.width];
            GCHandle datagch = GCHandle.Alloc(data, GCHandleType.Pinned);
            IntPtr dataptr = datagch.AddrOfPinnedObject();

            // put the data into it
            ReadoutLineParams rlp = new ReadoutLineParams
            {
                ccd = CCD_Request.CCD_IMAGING,
                pixelStart = 0,
                pixelLength = sep.width,
                readoutMode = 0
            };
            GCHandle rlpgch = GCHandle.Alloc(rlp, GCHandleType.Pinned);
            // get the image from the camera, line by line
            for (int i = 0; i < sep.height; i++)
                SBIGUnivDrvCommand(PAR_COMMAND.CC_READOUT_LINE, rlpgch.AddrOfPinnedObject(), dataptr + (i * sep.width * sizeof(ushort)));

            // cleanup our memory goo
            rlpgch.Free();
            datagch.Free();
            /*Bitmap b3 = new Bitmap(sep.width, sep.height, PixelFormat.Format16bppGrayScale);
            BitmapData bd = b3.LockBits(new Rectangle(0, 0, sep.width, sep.height), ImageLockMode.WriteOnly, PixelFormat.Format16bppGrayScale);
            bd.Scan0 = datagch.AddrOfPinnedObject();
            b3.UnlockBits(bd);
            Color c2 = b3.GetPixel(0, 0);
            Bitmap bmp = new Bitmap(sep.width, sep.height, sep.width * sizeof(ushort), PixelFormat.Format16bppGrayScale, datagch.AddrOfPinnedObject());
            bmp.Save("foo.bmp");
            bmp.Dispose();*/

            return data;
        }

        // COMMENTED OUT: because GDI.net doesn't support our pixel format
        /// <summary>
        /// In addition to reading out the image data, converts it into a Bitmap
        /// </summary>
        //public static void SaveImageToVernacularFormat(StartExposureParams2 sep, ushort[,] data, string filename, ImageFormat format)
        //{
        //    // find min and max
        //    int min = Int32.MaxValue;
        //    int max = Int32.MinValue;
        //    for (int j = 0; j < sep.height; j++)
        //        for (int i = 0; i < sep.width; i++)
        //        {
        //            if (data[j, i] < min)
        //                min = data[j, i];
        //            if (data[j, i] > max)
        //                max = data[j, i];
        //        }

        //    // construct a new array with scales
        //    byte[,] b = new byte[sep.height, sep.width];
        //    for (int j = 0; j < sep.height; j++)
        //        for (int i = 0; i < sep.width; i++)
        //            b[j, i] = (byte) ( (data[j, i] - min) * 256.0 / (max - min) );

        //    // construct the bitmap object with the bytes
        //    GCHandle bgch = GCHandle.Alloc(b);
        //    using (Bitmap bmp = new Bitmap(sep.width, sep.height, /*sep.width * sizeof(byte),*/ PixelFormat.Format8bppIndexed))//, GCHandle.ToIntPtr(bgch)))
        //    {
        //        /*
        //        BitmapData bd = bmp.LockBits(new Rectangle(0, 0, sep.width, sep.height), ImageLockMode.WriteOnly, PixelFormat.Format8bppIndexed);
        //        Marshal.Copy(GCHandle.ToIntPtr(bgch), 0, bd.Scan0, sep.width * sep.height);
        //        bmp.UnlockBits(bd);
        //        */

        //        // create a pallette for the damn image
        //        for (int i = 0; i < 256; i++)
        //            bmp.Palette.Entries[i] = Color.FromArgb(i, i, i);

        //        for (int j = 0; j < sep.height; j++)
        //            for (int i = 0; i < sep.width; i++)
        //                bmp.SetPixel(i, j, Color.FromArgb(b[i, j]));

        //        // write it out
        //        bmp.Save(filename, format);
        //    }

        //    // clean up
        //    bgch.Free();
        //}

        /// <summary>
        /// Tries to autodetect ImageFormat based on filename
        /// </summary>
        //public static void WaitEndReadAndSaveExposure(StartExposureParams2 sep, string filename)
        //{
        //    // grab the extension
        //    string ext = Path.GetExtension(filename);
        //    // strip off the leading dot, if it's there
        //    if (ext.StartsWith("."))
        //        ext = ext.Substring(1);

        //    // pick the format based on it matching the strings used in the ImageFormat enum
        //    ImageFormat format = (ImageFormat) Enum.Parse(typeof(ImageFormat), ext);

        //    // do the heaving lifting
        //    throw new Exception("Not yet imnplemented, probably never will be, and probably shouldn't be");
        //    //WaitEndAndReadExposureAsBitmap(sep).Save(filename, format);
        //}

        //
        // Shortcut commands
        //

        public static CameraType EstablishLink()
        {
            EstablishLinkParams elp = new EstablishLinkParams();
            return UnivDrvCommand<EstablishLinkResults>(PAR_COMMAND.CC_ESTABLISH_LINK, elp).cameraType;
        }

        #endregion Methods

    } // class
} // namespace
