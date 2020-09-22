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
                EV_PROPCHANGED = "propchanged";

    const int PROP_VIDEOALIVE = 0,
                PROP_CALIBRATINGCAMERA = 1,
                PROP_CALIBRATINGSCANNER = 2,
                PROP_SCANNING = 3;

    const std::string EVCHANNEL_CAMERA = "camera",
                        EVCHANNEL_TABLE = "table";

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
        COMM_CAMERACALIBSTART = 8,
        COMM_CAMERACALIBSTOP = 9,
        COMM_INPUT = 15;
}

#endif