﻿using SbigSharp;
using System;

namespace SbigSharpDemo
{
    class Program
    {
        static void Main(string[] args)
        {
            // initialize the driver
            SBIG.UnivDrvCommand(SBIG.PAR_COMMAND.CC_OPEN_DRIVER);

            // ask the SBIG driver what, if any, USB cameras are plugged in
            SBIG.UnivDrvCommand(SBIG.PAR_COMMAND.CC_QUERY_USB, out SBIG.QueryUSBResults qur);
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
            SBIG.UnivDrvCommand(
                SBIG.PAR_COMMAND.CC_OPEN_DEVICE,
                new SBIG.OpenDeviceParams("127.0.0.1"));
            var cameraType = SBIG.EstablishLink();

            // query camera info
            var gcir0 = new SBIG.GetCCDInfoResults0();
            SBIG.UnivDrvCommand(
                SBIG.PAR_COMMAND.CC_GET_CCD_INFO,
                new SBIG.GetCCDInfoParams
                {
                    request = SBIG.CCD_INFO_REQUEST.CCD_INFO_IMAGING
                },
                out gcir0);
            // now print it out
            Console.WriteLine($"Firmware version: {gcir0.firmwareVersion >> 8}.{gcir0.firmwareVersion & 0xFF}");
            Console.WriteLine($"Camera type: {gcir0.cameraType}");
            Console.WriteLine($"Camera name: {gcir0.name}");
            Console.WriteLine($"Readout modes: {gcir0.readoutModes}");
            for (int i = 0; i < gcir0.readoutModes; i++)
            {
                SBIG.READOUT_INFO ri = gcir0.readoutInfo[i];
                Console.WriteLine($"Readout mode: {ri.mode}");
                Console.WriteLine($"Width: {ri.width}");
                Console.WriteLine($"Height: {ri.height}");
                Console.WriteLine($"Gain: {ri.gain >> 8}.{ri.gain & 0xFF} e-/ADU");
            }

            // get extended info
            var gcir2 = new SBIG.GetCCDInfoResults2();
            SBIG.UnivDrvCommand(
                SBIG.PAR_COMMAND.CC_GET_CCD_INFO,
                new SBIG.GetCCDInfoParams
                {
                    request = SBIG.CCD_INFO_REQUEST.CCD_INFO_EXTENDED
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
            SBIG.UnivDrvCommand(
                SBIG.PAR_COMMAND.CC_QUERY_TEMPERATURE_STATUS,
                 new SBIG.QueryTemperatureStatusParams()
                 {
                     request = SBIG.QUERY_TEMP_STATUS_REQUEST.TEMP_STATUS_ADVANCED2
                 },
                out SBIG.QueryTemperatureStatusResults2 qtsr2);

            // start an exposure
            SBIG.StartExposureParams2 sep = new SBIG.StartExposureParams2
            {
                ccd = SBIG.CCD_REQUEST.CCD_IMAGING,
                abgState = SBIG.ABG_STATE7.ABG_LOW7,
                openShutter = SBIG.SHUTTER_COMMAND.SC_LEAVE_SHUTTER,
                exposureTime = 100,
                //sep.width = 765;
                //sep.height = 510;
                width = 1530,
                height = 1020
            };
            SBIG.UnivDrvCommand(SBIG.PAR_COMMAND.CC_START_EXPOSURE, sep);


            // read out the image
            ushort[,] img = SBIG.WaitEndAndReadoutExposure(sep);
            //FitsUtil.WriteFitsImage("simcam.fits", img);
            //SBIG.SaveImageToVernacularFormat(sep, img, "foo.gif", ImageFormat.Gif);

            //
            // setup for TDI
            //
            SBIG.MiscellaneousControlParams mcp = new SBIG.MiscellaneousControlParams
            {
                fanEnable = true,
                ledState = SBIG.LED_STATE.LED_BLINK_HIGH,
                shutterCommand = SBIG.SHUTTER_COMMAND.SC_OPEN_SHUTTER
            };
            SBIG.UnivDrvCommand(SBIG.PAR_COMMAND.CC_MISCELLANEOUS_CONTROL, mcp);

            // turn off pipelining for USB connected cameras
            SBIG.SetDriverControlParams sdcp = new SBIG.SetDriverControlParams
            {
                controlParameter = SBIG.DRIVER_CONTROL_PARAM.DCP_USB_FIFO_ENABLE,
                controlValue = 0
            };
            SBIG.UnivDrvCommand(SBIG.PAR_COMMAND.CC_SET_DRIVER_CONTROL, sdcp);
            
            // read a TDI line

            // input params
            var rlp = new SBIG.ReadoutLineParams
            {
                ccd = SBIG.CCD_REQUEST.CCD_IMAGING,
                pixelStart = 0,
                pixelLength = 1530,
                readoutMode = SBIG.READOUT_BINNING_MODE.RM_NX1
                //SBIG.ReadoutLineParams.MakeNBinMode(SBIG.ReadoutBinningMode.RM_NX1, 4)
            };
            // output
            var data = new ushort[rlp.pixelLength];
            // make the call!!!
            SBIG.UnivDrvCommand(SBIG.PAR_COMMAND.CC_READOUT_LINE, rlp, out data);
            // do it a lot
            var start = DateTime.Now;
            for (int i = 0; i < 1000000; i++)
                SBIG.UnivDrvCommand(SBIG.PAR_COMMAND.CC_READOUT_LINE, rlp, out data);
            var end = DateTime.Now;
            Console.WriteLine(end - start);

            // clean up
            SBIG.UnivDrvCommand(SBIG.PAR_COMMAND.CC_CLOSE_DEVICE);
            SBIG.UnivDrvCommand(SBIG.PAR_COMMAND.CC_CLOSE_DRIVER);
        }
    }
}
