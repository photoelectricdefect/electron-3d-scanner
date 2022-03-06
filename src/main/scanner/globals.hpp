#ifndef GLOBALS_H_
#define GLOBALS_H_
#include <string>

namespace scanner
{
       const int FPS_60 = 60,
                 FPS_30 = 30;

       const static std::string ROTDIR_CLOCKWISE = "rotclockwise",
                                ROTDIR_COUNTERCLOCKWISE = "rotcounterclockwise";

       const static double ROTATION_RESOLUTION_HALFSTEP = 0.9,
                           ROTATION_RESOLUTION_FULLSTEP = 1.8;

       const float MODE_FULL_STEP = 0,
                   MODE_HALF_STEP = 1;

       const int ARDUINO_ERR_OK = 0,
                 ARDUINO_ERR_NOK = 1;

       const std::string EV_ERROR = "error",
                         EV_STATUS = "status",
                         EV_PROPCHANGED = "propchanged",
                         EV_IMUPDATE = "imupdate",
                         EV_VIDEOSTART = "videostart",
                         EV_VIDEOSTOP = "videostop",
                         EV_MAINSTART = "mainstart",
                         EV_MAINSTOP = "mainstop",
                         EV_CAMERACALIBSTART = "cameracalibstart",
                         EV_CAMERACALIBSTOP = "cameracalibstop",
                         EV_SCANNERCALIBSTART = "scannercalibstart",
                         EV_SCANNERCALIBSTOP = "scannercalibstop",
                         EV_SCANNERCALIBDATA = "scannercalibdata",
                         EV_SCANDATA = "scandata",
                         EV_SCANSTART = "scanstart",
                         EV_SCANSTOP = "scanstop",
                         EV_CONTROLLERSTART = "controllerstart",
                         EV_CONTROLLERSTOP = "controllerstop",
                         EV_LASERSET = "laserset",
                         EV_ROTATE = "rotate",
                         EV_CAMERACALIBCAPTURED = "cameracalibcaptured",
                         EV_DEBUGCAPTURE = "debugcapture";

       const std::string PROP_DISPLAYVIDEO = "displayvideo",
                        PROP_CALIBRATINGCAMERA = "calibratingcamera",
                        PROP_CALIBRATINGSCANNER = "calibratingscanner",
                        PROP_SCANNING = "scanning",
                        PROP_CAMERACALIBRATED = "cameracalibrated",
                        PROP_SCANNERCALIBRATED = "scannercalibrated",
                        PROP_CAMERA_CALIB_CAPTURES = "cameracalibcaptures",
                        PROP_CAMERALIST = "cameralist",
                        PROP_SELECTEDCAMERA = "selectedcamera",
                        PROP_SCANNER_CALIBRATION_DATA = "scannercalibrationdata",
                        PROP_SCAN_RENDER_DATA = "scanrenderdata";


       const int KEYCODE_SPACE = 32,
                 KEYCODE_C = 67,
                 KEYCODE_S = 83;

       const int
           COMM_MAINSTART = 0,
           COMM_MAINSTOP = 1,
           COMM_VIDEOSTART = 2,
           COMM_VIDEOSTOP = 3,
           COMM_SCANSTART = 12,
           COMM_SCANSTOP = 13,
           COMM_LOADMODEL = 5,
           COMM_SETPROP = 14,
           COMM_GETPROP = 16,
           COMM_CAMERACALIBSTART = 8,
           COMM_CAMERACALIBSTOP = 9,
           COMM_SCANNERCALIBSTART = 10,
           COMM_SCANNERCALIBSTOP = 11,
           COMM_CONTROLLERSTART = 18,
           COMM_CONTROLLERSTOP = 19,
           COMM_ROTATE = 20,
           COMM_TOGGLELASER = 21;

    const std::string CAMERA_CALIBRATION_FILE="camera_calibration.json",
                    SCANNER_CALIBRATION_FILE="scanner_calibration.json";
}

#endif