#ifndef FLAGS_H_
#define FLAGS_H_

namespace scanner {
    const int FPS_60 = 60,
           FPS_30 = 30;

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
                EV_SCANNERCALIBIMAGECAPTURED = "scannercalibimagecaptured",
                EV_PROPCHANGED = "propchanged",
                EV_SCANSTART = "scanstart",
                EV_SCANSTOP = "scanstop";

    const int PROP_VIDEOALIVE = 0,
                PROP_CALIBRATINGCAMERA = 1,
                PROP_CALIBRATINGSCANNER = 2,
                PROP_SCANNING = 3;
                
    const int KEYCODE_SPACE = 32;

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
        COMM_SCANNERCALIBSTART = 16,
        COMM_SCANNERCALIBSTOP = 17,
        COMM_KEYSTROKE = 15;
}

#endif