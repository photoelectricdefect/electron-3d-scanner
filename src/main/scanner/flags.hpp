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
                EV_IOSTOP = "iostop";

    const int
        COMM_IOSTART = 0,
        COMM_IOSTOP = 1,
        COMM_VIDEOSTART = 2,
        COMM_VIDEOSTOP = 3,
        COMM_SCAN = 4,
        COMM_LOADMODEL = 5,
        COMM_SETPROP = 6;
}

#endif