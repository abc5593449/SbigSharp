using System;
using System.Drawing;
using System.IO;
using System.Net;
using System.Runtime.InteropServices;
using System.Text;

namespace SbigSharp
{
    public class SBIG
    {
        //
        #region Constants
        //

        private static GCHandle NullGch = new GCHandle();

        #endregion

        //
        #region Enums
        //

        public enum Cmd : ushort
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

        } // enum Cmd

        /// <summary>
        /// Base value for all error IDs.
        /// </summary>
        public const ushort CE_ERROR_BASE = 1;
        public enum Error : ushort
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

        } // enum Error

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
        /// Motor Focus Command enum
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

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class EstablishLinkParams
        {
            public ushort sbigUseOnly;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class EstablishLinkResults
        {
            public CameraType cameraType;
        }


        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class StartExposureParams2
        {
            public CCD_Request ccd;
            public uint exposureTime;  //  integration time in hundredths of a second in the least significant 24 bits
            public AbgState7 abgState;
            public ShutterCommand openShutter;
            public ushort readoutMode;
            public ushort top;
            public ushort left;
            public ushort height;
            public ushort width;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class EndExposureParams
        {
            public CCD_Request ccd;

            public EndExposureParams() : this(CCD_Request.CCD_IMAGING) { }
            public EndExposureParams(CCD_Request ccd)
            {
                this.ccd = ccd;
            }
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class ReadoutLineParams
        {
            public CCD_Request ccd;
            public ushort readoutMode;
            public ushort pixelStart;
            public ushort pixelLength;

            public static ushort MakeNBinMode(ReadoutBinningMode rlp, ushort n)
            {
                // put the high byte in place, but only if it's one of those binning modes
                if (ReadoutBinningMode.RM_NX1 == rlp ||
                    ReadoutBinningMode.RM_NX2 == rlp ||
                    ReadoutBinningMode.RM_NX3 == rlp ||
                    ReadoutBinningMode.RM_NXN == rlp)
                    return (ushort)(((ushort)rlp) | (n << 8));
                else
                    return (ushort)rlp;
            }
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class QueryTemperatureStatusParams
        {
            public TempStatusRequest request;

            public QueryTemperatureStatusParams() : this(TempStatusRequest.TEMP_STATUS_STANDARD) { }
            public QueryTemperatureStatusParams(TempStatusRequest tsr)
            {
                request = tsr;
            }
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class QueryTemperatureStatusResults
        {
            public ushort enabled;
            public ushort ccdSetpoint;
            public ushort power;
            public ushort ccdThermistor;
            public ushort ambientThermistor;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class QueryTemperatureStatusResults2
        {
            public ushort coolingEnabled;
            public ushort fanEnabled;
            public double ccdSetpoint;
            public double imagingCCDTemperature;
            public double trackingCCDTemperature;
            public double externalTrackingCCDTemperature;
            public double ambientTemperature;
            public double imagingCCDPower;
            public double trackingCCDPower;
            public double externalTrackingCCDPower;
            public double heatsinkTemperature;
            public double fanPower;
            public double fanSpeed;
            public double trackingCCDSetpoint;
        }

        /// <summary>
        /// controls the thermoelectric cooler, using old school A2D units (see docs)
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class SetTemperatureRegulationParams
        {
            public TemperatureRegulation state;
            public ushort ccdSetpointA2dUnits;
        }

        /// <summary>
        /// controls the thermoelectric cooler in nice, simple degrees C
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class SetTemperatureRegulationParams2
        {
            public TemperatureRegulation state;
            public double ccdSetpointCelcius;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class QueryCommandStatusParams
        {
            public Cmd command;

            public QueryCommandStatusParams() : this(Cmd.CC_NULL) { }
            public QueryCommandStatusParams(Cmd cmd)
            {
                command = cmd;
            }
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class QueryCommandStatusResults
        {
            public Error status;

            public QueryCommandStatusResults() : this(Error.CE_NO_ERROR) { }
            public QueryCommandStatusResults(Error e)
            {
                status = e;
            }
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class MiscellaneousControlParams
        {
            public ushort fanEnable;
            public ShutterCommand shutterCommand;
            public LedState ledState;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class OpenDeviceParams
        {
            public DeviceType deviceType;       /* LPT, Ethernet, etc */
            public ushort lptBaseAddress;       /* DEV_LPTN: Windows 9x Only, Win NT uses deviceSelect */
            public uint ipAddress;			    /* DEV_ETH:  Ethernet address, the most significant byte specifying the first part of the address */

            public OpenDeviceParams()
            {
                deviceType = (DeviceType)0;    // illegal value to make things fail fast if unintialized
                ipAddress = 0;
                lptBaseAddress = 0;             // either it's irrelvant or the OS will handle it
            }

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
                    if (!Enum.TryParse<DeviceType>(s, true, out deviceType))
                        throw new ArgumentException("must pass either an IP address or valid DeviceType enum string");

                }
            }
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class SetDriverControlParams
        {
            public DriverControlParam controlParameter;
            public int controlValue;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct StartReadoutParams
        {
            public CCD_Request ccd;
            public ushort readoutMode;
            public ushort top;
            public ushort left;
            public ushort height;
            public ushort width;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct UsbInfo
        {
            public ushort cameraFound;
            public bool CameraFound { get { return 0 != cameraFound; } }
            public CameraType cameraType;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string name;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            public string serialNumber;
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class QueryUsbResults
        {
            public ushort camerasFound;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
            public UsbInfo[] dev;
        }


        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class GetCcdInfoParams
        {
            public CcdInfoRequest req;

            public GetCcdInfoParams(CcdInfoRequest req)
            {
                this.req = req;
            }
        }

        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public struct ReadoutInfo
        {
            public ushort mode;
            public ushort width;
            public ushort height;
            public ushort gain;        // amplifier gain in e-/ADU, e.g. 0x1234 = 12.34 e- per ADU
            public ulong pixelWidth;   // pixel width in microns in the form XXXXXX.XX
            public ulong pixelHeight;  // pixel height in microns in the form XXXXXX.XX
        }

        /// <summary>
        /// Result structure for CC_GET_CCD_INFO request types 0 and 1
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class GetCcdInfoResults01
        {
            public ushort firmwareVersion; // 0x1234 = v12.34
            public CameraType cameraType;
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 64)]
            public string name;
            public ushort readoutModeCount;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 20)]
            public ReadoutInfo[] readoutInfo;
        }

        /// <summary>
        /// Result structure for CC_GET_CCD_INFO request type 2
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class GetCcdInfoResults2
        {
            public ushort badColumns;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
            public ushort[] columns;
            public ImagingABG imagingABG; // 0 = no ABG, 1 = Anti-Blooming Gate protection
            [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 10)]
            public string serialNumber;
        }

        /// <summary>
        /// Result structure for CC_GET_CCD_INFO request type 3
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class GetCcdInfoResults3
        {
            public AD_Size a2dSize;
            public FilterType filterType;
        }

        /// <summary>
        /// Result structure for CC_GET_CCD_INFO request type 4 and 5
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class GetCcdInfoResults45
        {
            public ushort capabilitiesBits;
            public ushort dumpExtra;
        }

        /// <summary>
        /// Result structure for CC_GET_CCD_INFO request type 6
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class GetCcdInfoResults6
        {
            public ulong cameraBits;
            public ulong ccdBits;
            public ulong extraBits;
        }

        /// <summary>
        /// Input structure for CC_ACTIVATE_RELAY command to control telescope mount
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 8)]
        public class ActivateRelayParams
        {
            public ushort tXPlus;
            public ushort tXMinus;
            public ushort tYPlus;
            public ushort tYMinus;
        }


        /// <summary>
        /// gets thrown whenever an SBIG operation doesn't return success (CE_NO_ERROR)
        /// </summary>
        public class FailedOperationException : Exception
        {
            public Error ErrorCode { get; private set; }

            public FailedOperationException(Error errorCode)
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
        private static extern Error SBIGUnivDrvCommand(Cmd Command, IntPtr Parameters, IntPtr Results);

        /// <summary>
        /// Calls the SBIG Universal Driver with a (possibly null) input parameter struct
        /// </summary>
        /// <param name="Command">the command to be executed</param>
        /// <param name="Parameters">inputs to the operation, null if none</param>
        /// <exception cref="FailedOperationException">throws a FailedOperation exception if command doesn't return CE_NO_ERROR</exception>
        public static void UnivDrvCommand(Cmd Command, object Parameters)
        {
            // marshall the input structure, if it exists
            GCHandle ParamGch = NullGch;
            IntPtr ParamPtr = IntPtr.Zero;
            if (null != Parameters)
            {
                ParamGch = GCHandle.Alloc(Parameters, GCHandleType.Pinned);
                ParamPtr = ParamGch.AddrOfPinnedObject();
            }

            //
            // make the call
            //
            Error err = SBIGUnivDrvCommand(Command, ParamPtr, IntPtr.Zero);
            if (Error.CE_NO_ERROR != err)
                throw new FailedOperationException(err);

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
        public static void UnivDrvCommand(Cmd Command, object Parameters, object Results)
        {
            // marshall the input structure, if it exists
            GCHandle ParamGch = NullGch;
            IntPtr ParamPtr = IntPtr.Zero;
            if (null != Parameters)
            {
                ParamGch = GCHandle.Alloc(Parameters, GCHandleType.Pinned);
                ParamPtr = ParamGch.AddrOfPinnedObject();
            }
            // pin the output bytes while we pass the buffer to the SBIG SDK
            GCHandle ResultsGch = GCHandle.Alloc(Results, GCHandleType.Pinned);

            //
            // make the call
            //
            Error err = SBIGUnivDrvCommand(Command, ParamPtr, ResultsGch.AddrOfPinnedObject());
            if (Error.CE_NO_ERROR != err)
                throw new FailedOperationException(err);

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
        public static void UnivDrvCommandMarshal(Cmd Command, object Parameters)
        {
            // marshall the input structure, if it exists
            IntPtr ParamPtr = IntPtr.Zero;
            if (null != Parameters)
            {
                ParamPtr = Marshal.AllocHGlobal(Marshal.SizeOf(Parameters));
                Marshal.StructureToPtr(Parameters, ParamPtr, false);
            }

            //
            // make the call
            //
            Error err = SBIGUnivDrvCommand(Command, ParamPtr, IntPtr.Zero);
            if (Error.CE_NO_ERROR != err)
                throw new FailedOperationException(err);

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
        public static T UnivDrvCommand<T>(Cmd Command, object Parameters) where T : new()
        {
            // marshall the input structure, if it exists
            GCHandle ParamGch = NullGch;
            IntPtr ParamPtr = IntPtr.Zero;
            if (null != Parameters)
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
            Error err = SBIGUnivDrvCommand(Command, ParamPtr, ResultsPtr);
            if (Error.CE_NO_ERROR != err)
                throw new FailedOperationException(err);

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
        public static T UnivDrvCommandMarshal<T>(Cmd Command, object Parameters)
        {
            // marshall the input structure, if it exists
            IntPtr ParamPtr = IntPtr.Zero;
            if (null != Parameters)
            {
                ParamPtr = Marshal.AllocHGlobal(Marshal.SizeOf(Parameters));
                Marshal.StructureToPtr(Parameters, ParamPtr, false);
            }
            // marshall the output structure
            IntPtr ResultsPtr = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(T)));

            //
            // make the call
            //
            Error err = SBIGUnivDrvCommand(Command, ParamPtr, ResultsPtr);
            if (Error.CE_NO_ERROR != err)
                throw new FailedOperationException(err);

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
        public static void UnivDrvCommand_OutComplex(Cmd Command, object Parameters, object Results)
        {
            // marshall the input structure, if it exists
            GCHandle ParamGch = NullGch;
            IntPtr ParamPtr = IntPtr.Zero;
            if (null != Parameters)
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
            Error err = SBIGUnivDrvCommand(Command, ParamPtr, ResultsPtr);
            if (Error.CE_NO_ERROR != err)
                throw new FailedOperationException(err);

            // Marshall back
            Marshal.PtrToStructure(ResultsPtr, Results);

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
            QueryCommandStatusParams qcsp = new QueryCommandStatusParams(Cmd.CC_START_EXPOSURE);
            QueryCommandStatusResults qcsr = new QueryCommandStatusResults(Error.CE_NO_ERROR);
            while (Error.CE_NO_EXPOSURE_IN_PROGRESS != qcsr.status)
                qcsr = UnivDrvCommand<QueryCommandStatusResults>(Cmd.CC_QUERY_COMMAND_STATUS, qcsp);

            // prepare the CCD for readout
            UnivDrvCommand(Cmd.CC_END_EXPOSURE, new EndExposureParams(CCD_Request.CCD_IMAGING));
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
            UnivDrvCommand(Cmd.CC_START_READOUT, srp);

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
                SBIGUnivDrvCommand(Cmd.CC_READOUT_LINE, rlpgch.AddrOfPinnedObject(), dataptr + (i * sep.width * sizeof(ushort)));

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
            return UnivDrvCommand<EstablishLinkResults>(Cmd.CC_ESTABLISH_LINK, elp).cameraType;
        }

        #endregion Methods

    } // class
} // namespace
