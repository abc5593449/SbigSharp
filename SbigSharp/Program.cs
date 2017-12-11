using System;

namespace SbigSharp
{
    class Program
    {
        static void Main(string[] args)
        {
            // initialize the driver
            SBIG.UnivDrvCommand(SBIG.Cmd.CC_OPEN_DRIVER, null);

            // ask the SBIG driver what, if any, USB cameras are plugged in
            SBIG.QueryUsbResults qur = new SBIG.QueryUsbResults();
            SBIG.UnivDrvCommand_OutComplex(SBIG.Cmd.CC_QUERY_USB, null, qur);
            for (int i = 0; i < qur.camerasFound; i++)
            {
                if (0 == qur.dev[i].cameraFound)
                    Console.WriteLine("Cam " + i + ": not found");
                else
                    Console.WriteLine(String.Format("Cam {0}: type={1} name={2} ser={3}", i, qur.dev[i].cameraType, qur.dev[i].name, qur.dev[i].serialNumber));
            }

            //
            // connect to the camera
            //
            SBIG.UnivDrvCommand(SBIG.Cmd.CC_OPEN_DEVICE, new SBIG.OpenDeviceParams("127.0.0.1"));
            SBIG.CameraType ct = SBIG.EstablishLink();


            // query camera info
            SBIG.GetCcdInfoResults01 gcir0 = new SBIG.GetCcdInfoResults01();
            SBIG.UnivDrvCommand_OutComplex(SBIG.Cmd.CC_GET_CCD_INFO,
                                           new SBIG.GetCcdInfoParams(SBIG.CcdInfoRequest.ImagingCcdStandard),
                                           gcir0);
            // now print it out
            Console.WriteLine("Firmware version: " + (gcir0.firmwareVersion >> 8) + "." + (gcir0.firmwareVersion & 0xFF));
            Console.WriteLine("Camera type: " + gcir0.cameraType.ToString());
            Console.WriteLine("Camera name: " + gcir0.name);
            Console.WriteLine("Readout modes: " + gcir0.readoutModeCount);
            for (int i = 0; i < gcir0.readoutModeCount; i++)
            {
                SBIG.ReadoutInfo ri = gcir0.readoutInfo[i];
                Console.WriteLine(" Readout mode: " + ri.mode);
                Console.WriteLine("  Width: " + ri.width);
                Console.WriteLine("  Height: " + ri.height);
                Console.WriteLine("  Gain: " + (ri.gain >> 8) + "." + (ri.gain & 0xFF) + " e-/ADU");
            }

            // get extended info
            SBIG.GetCcdInfoResults2 gcir2 = new SBIG.GetCcdInfoResults2();
            SBIG.UnivDrvCommand_OutComplex(SBIG.Cmd.CC_GET_CCD_INFO,
                                           new SBIG.GetCcdInfoParams(SBIG.CcdInfoRequest.CameraInfoExtended),
                                           gcir2);
            // print it out
            Console.Write("Bad columns: " + gcir2.badColumns + " = ");
            Console.WriteLine(gcir2.columns[0] + ", " + gcir2.columns[1] + ", " + gcir2.columns[2] + ", " + gcir2.columns[3]);
            Console.WriteLine("ABG: " + gcir2.imagingABG.ToString());
            Console.WriteLine("Serial number: " + gcir2.serialNumber);

            // query temperature
            SBIG.QueryTemperatureStatusParams qtsp = new SBIG.QueryTemperatureStatusParams(SBIG.TempStatusRequest.TEMP_STATUS_ADVANCED2);
            var qtsr2 = SBIG.UnivDrvCommand<SBIG.QueryTemperatureStatusResults2>(SBIG.Cmd.CC_QUERY_TEMPERATURE_STATUS, qtsp);


            // start an exposure
            SBIG.StartExposureParams2 sep = new SBIG.StartExposureParams2
            {
                ccd = SBIG.CCD.Imaging,
                abgState = SBIG.AbgState.Off,
                openShutter = SBIG.ShutterState.Unchanged,
                exposureTime = 100,
                //sep.width = 765;
                //sep.height = 510;
                width = 1530,
                height = 1020
            };
            SBIG.UnivDrvCommand(SBIG.Cmd.CC_START_EXPOSURE, sep);


            // read out the image
            ushort[,] img = SBIG.WaitEndAndReadoutExposure(sep);
            //FitsUtil.WriteFitsImage("simcam.fits", img);
            //SBIG.SaveImageToVernacularFormat(sep, img, "foo.gif", ImageFormat.Gif);

            //
            // setup for TDI
            //
            SBIG.MiscellaneousControlParams mcp = new SBIG.MiscellaneousControlParams
            {
                fanEnable = 1,
                ledState = SBIG.LedState.BlinkHigh,
                shutterCommand = SBIG.ShutterState.Open
            };
            SBIG.UnivDrvCommand(SBIG.Cmd.CC_MISCELLANEOUS_CONTROL, mcp);

            // turn off pipelining for USB connected cameras
            SBIG.SetDriverControlParams sdcp = new SBIG.SetDriverControlParams
            {
                controlParameter = SBIG.DriverControlParam.DCP_USB_FIFO_ENABLE,
                controlValue = 0
            };
            SBIG.UnivDrvCommand(SBIG.Cmd.CC_SET_DRIVER_CONTROL, sdcp, null);

            //
            // read a TDI line
            //
            // input params
            SBIG.ReadoutLineParams rlp = new SBIG.ReadoutLineParams
            {
                ccd = SBIG.CCD.Imaging,
                pixelStart = 0,
                pixelLength = 1530,
                readoutMode = SBIG.ReadoutLineParams.MakeNBinMode(SBIG.ReadoutMode.BinNx1, 4)
            };
            // output
            ushort[] data = new ushort[rlp.pixelLength];
            // make the call!!!
            SBIG.UnivDrvCommand(SBIG.Cmd.CC_READOUT_LINE, rlp, data);
            // do it a lot
            DateTime start = DateTime.Now;
            for (int i = 0; i < 1000000; i++)
                SBIG.UnivDrvCommand(SBIG.Cmd.CC_READOUT_LINE, rlp, data);
            DateTime end = DateTime.Now;
            TimeSpan ts = end - start;
            Console.WriteLine(ts);

            // clean up
            SBIG.UnivDrvCommand(SBIG.Cmd.CC_CLOSE_DEVICE, null);
            SBIG.UnivDrvCommand(SBIG.Cmd.CC_CLOSE_DRIVER, null);
        } // Main

    } // class
} // namespace
