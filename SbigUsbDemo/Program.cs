using System;
using static SbigSharp.SBIG;

namespace SbigUsbDemo
{
    class Program
    {
        static void Main(string[] args)
        {
            // initialize the driver
            UnivDrvCommand(PAR_COMMAND.CC_OPEN_DRIVER);

            // ask the SBIG driver what, if any, USB cameras are plugged in
            UnivDrvCommand(PAR_COMMAND.CC_QUERY_USB, out QueryUSBResults qur);
            for (int i = 0; i < qur.camerasFound; i++)
            {
                if (!qur.usbInfo[i].cameraFound)
                    Console.WriteLine($"Cam {i}: not found");
                else
                    Console.WriteLine(
                        $"Cam {i}: type={qur.usbInfo[i].cameraType} " +
                        $"name={ qur.usbInfo[i].name} " +
                        $"ser={qur.usbInfo[i].serialNumber}");
            }

            // connect to the camera
            UnivDrvCommand(
                PAR_COMMAND.CC_OPEN_DEVICE,
                new OpenDeviceParams
                {
                    deviceType = SBIG_DEVICE_TYPE.DEV_USB
                });
            var cameraType = EstablishLink();

            // query camera info
            var gcir0 = new GetCCDInfoResults0();
            UnivDrvCommand(
                PAR_COMMAND.CC_GET_CCD_INFO,
                new GetCCDInfoParams
                {
                    request = CCD_INFO_REQUEST.CCD_INFO_IMAGING
                },
                out gcir0);
            // now print it out
            Console.WriteLine($"Firmware version: {gcir0.firmwareVersion >> 8}.{gcir0.firmwareVersion & 0xFF}");
            Console.WriteLine($"Camera type: {gcir0.cameraType}");
            Console.WriteLine($"Camera name: {gcir0.name}");
            Console.WriteLine($"Readout modes: {gcir0.readoutModes}");
            for (int i = 0; i < gcir0.readoutModes; i++)
            {
                READOUT_INFO ri = gcir0.readoutInfo[i];
                Console.WriteLine($"Readout mode: {ri.mode}");
                Console.WriteLine($"Width: {ri.width}");
                Console.WriteLine($"Height: {ri.height}");
                Console.WriteLine($"Gain: {ri.gain >> 8}.{ri.gain & 0xFF} e-/ADU");
            }

            // get extended info
            var gcir2 = new GetCCDInfoResults2();
            UnivDrvCommand(
                PAR_COMMAND.CC_GET_CCD_INFO,
                new GetCCDInfoParams
                {
                    request = CCD_INFO_REQUEST.CCD_INFO_EXTENDED
                },
                out gcir2);
            // print it out
            Console.Write($"Bad columns: {gcir2.badColumns} = ");
            Console.WriteLine(
                $"{gcir2.columns[0]}, {gcir2.columns[1] }, " +
                $"{gcir2.columns[2]}, { gcir2.columns[3]}");
            Console.WriteLine($"ABG: {gcir2.imagingABG}");
            Console.WriteLine($"Serial number: {gcir2.serialNumber}");

            // query temperature
            UnivDrvCommand(
                PAR_COMMAND.CC_QUERY_TEMPERATURE_STATUS,
                 new QueryTemperatureStatusParams()
                 {
                     request = QUERY_TEMP_STATUS_REQUEST.TEMP_STATUS_ADVANCED2
                 },
                out QueryTemperatureStatusResults2 qtsr2);

            // start an exposure
            StartExposureParams2 sep2 = new StartExposureParams2
            {
                ccd = CCD_REQUEST.CCD_IMAGING,
                abgState = ABG_STATE7.ABG_LOW7,
                openShutter = SHUTTER_COMMAND.SC_LEAVE_SHUTTER,
                exposureTime = 10,
                width = 3296,
                height = 2472,
                readoutMode = READOUT_BINNING_MODE.RM_1X1
            };
            UnivDrvCommand(PAR_COMMAND.CC_START_EXPOSURE2, sep2);
            WaitExposure();

            AbortExposure(sep2);
            var srp = new StartReadoutParams
            {
                ccd = sep2.ccd,
                readoutMode = sep2.readoutMode,
                left = 0,
                top = 0,
                width = sep2.width,
                height = sep2.height
            };
            UnivDrvCommand(PAR_COMMAND.CC_START_READOUT, srp);

            // read a TDI line
            // input params
            var rlp = new ReadoutLineParams
            {
                ccd = sep2.ccd,
                pixelStart = 0,
                pixelLength = sep2.width,
                readoutMode = sep2.readoutMode
            };
            // output
            var data = new ushort[rlp.pixelLength];
            // make the call!!!
            UnivDrvCommand(PAR_COMMAND.CC_READOUT_LINE, rlp, out data);
            // do it a lot
            var start = DateTime.Now;
            for (int i = 1; i < sep2.height; i++)
                UnivDrvCommand(PAR_COMMAND.CC_READOUT_LINE, rlp, out data);
            var end = DateTime.Now;
            Console.WriteLine(end - start);

            // clean up
            UnivDrvCommand(PAR_COMMAND.CC_CLOSE_DEVICE);
            UnivDrvCommand(PAR_COMMAND.CC_CLOSE_DRIVER);
        }
    }
}
