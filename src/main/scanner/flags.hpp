#ifndef FLAGS_H_
#define FLAGS_H_

namespace scanner {
    const int FPS_60 = 60,
           FPS_30 = 30;

    const int ROTATE_CLOCKWISE = 0,
           ROTATE_COUNTERCLOCKWISE = 1;

    const float MODE_FULL_STEP = 0,
           MODE_HALF_STEP = 1;

    const int ARDUINO_ERR_OK = 0,
           ARDUINO_ERR_NOK = 1;

    const std::string EV_ERROR = "error",
                EV_STATUS = "status",
                EV_IMUPDATE = "imupdate",
                EV_VIDEOSTART = "videostart",
                EV_VIDEOSTOP = "videostop",
                EV_IOSTART = "iostart",
                EV_IOSTOP = "iostop",
                EV_CAMERACALIBSTART = "cameracalibstart",
                EV_CAMERACALIBSTOP = "cameracalibstop",
                EV_SCANNERCALIBSTART = "scannercalibstart",
                EV_SCANNERCALIBSTOP = "scannercalibstop",
                EV_SCANNERCALIBDATA = "scannercalibdata",
                EV_PROPCHANGED = "propchanged",
                EV_SCANSTART = "scanstart",
                EV_SCANSTOP = "scanstop",
                EV_CONTROLLERSTART = "controllerstart",
                EV_CONTROLLERSTOP = "controllerstop",
                EV_LASERSET = "laserset",
                EV_ROTATE = "rotate",
                EV_CAMERACALIBCAPTURED="cameracalibcaptured",
                EV_DEBUGCAPTURE="debugcapture";
       //TODO: change to string
    const std::string PROP_VIDEOALIVE = "videoalive",
                PROP_CALIBRATINGCAMERA = "calibratingcamera",
                PROP_CALIBRATINGSCANNER = "calibratingscanner",
                PROP_SCANNING = "scanning",
                PROP_CAMERACALIBRATED="cameracalibrated",
                PROP_SCANNERCALIBRATED="scannercalibrated",
                PROP_CAMERA_CALIB_CAPTURES="cameracalibcaptures";

                
    const int KEYCODE_SPACE = 32,
              KEYCODE_C=67;

    const int
        COMM_IOSTART = 0,
        COMM_IOSTOP = 1,
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
        COMM_LASERSET = 21;
}

#endif