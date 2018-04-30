using System;
using static SbigSharp.SBIG;

namespace SbigUsbDemo
{
    class Program
    {
        static void Main(string[] args)
        {
            #region 初始化
            // initialize the driver
            UnivDrvCommand(PAR_COMMAND.CC_OPEN_DRIVER);
            // connect to the camera
            UnivDrvCommand(
                PAR_COMMAND.CC_OPEN_DEVICE,
                new OpenDeviceParams
                {
                    deviceType = SBIG_DEVICE_TYPE.DEV_USB
                });
            var cameraType = EstablishLink();

            // start an exposure
            StartExposureParams2 sep2 = new StartExposureParams2
            {
                ccd = CCD_REQUEST.CCD_IMAGING,
                abgState = ABG_STATE7.ABG_LOW7,
                openShutter = SHUTTER_COMMAND.SC_LEAVE_SHUTTER,
                readoutMode = READOUT_BINNING_MODE.RM_1X1,
                exposureTime = 10,
                left = 1648, // 3296/2
                top = 1236,  // 2472/2
                width = 100,
                height = 100,
            };
            #endregion

            var s_expo = DateTime.Now;
            Exposure(sep2);
            WaitExposure();
            var e_expo = DateTime.Now;
            Console.WriteLine($"Exposure {e_expo - s_expo}");

            var s_sr = DateTime.Now;
            AbortExposure(sep2);
            var srp = new StartReadoutParams
            {
                ccd = sep2.ccd,
                readoutMode = sep2.readoutMode,
                left = sep2.left,
                top = sep2.top,
                width = sep2.width,
                height = sep2.height
            };
            UnivDrvCommand(PAR_COMMAND.CC_START_READOUT, srp);
            var e_sr = DateTime.Now;
            Console.WriteLine($"StartReadout { e_sr - s_sr}");

            // read a TDI line
            // input params
            var rlp = new ReadoutLineParams
            {
                ccd = sep2.ccd,
                readoutMode = sep2.readoutMode,
                pixelStart = srp.left,
                pixelLength = srp.width
            };
            // output
            var data = new ushort[rlp.pixelLength];
            // do it a lot
            var s_rl = DateTime.Now;
            for (int i = 0; i < srp.height; i++)
                UnivDrvCommand(PAR_COMMAND.CC_READOUT_LINE, rlp, out data);
            var e_rl = DateTime.Now;
            Console.WriteLine($"Read Line {e_rl - s_rl}");

            #region 結束
            // clean up
            UnivDrvCommand(PAR_COMMAND.CC_CLOSE_DEVICE);
            UnivDrvCommand(PAR_COMMAND.CC_CLOSE_DRIVER);
            #endregion
        }
    }
}
