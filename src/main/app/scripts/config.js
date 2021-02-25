const debug=false;

const events =  {
    status: "status",
    imupdate: "imupdate",
    videostart: "videostart",
    videostop: "videostop",
    error: "error",
    iostart: "iostart",
    iostop: "iostop",
    propchanged: "propchanged",
    cameracalibstart: "cameracalibstart",
    cameracalibstop: "cameracalibstop",
    scannercalibstart: "scannercalibstart",
    scannercalibstop: "scannercalibstop",
    scanstart: "scanstart",
    scanstop: "scanstop",
    controllerstart: "controllerstart",
    controllerstop: "controllerstop",
    rotate: "rotate",
    cameracalibcaptured: "cameracalibcaptured",
    scannercalibdata: "scannercalibdata",
    debugcapture:"debugcapture"
  };
  
  const commands =  {
    iostart: 0,
    iostop: 1,
    videostart: 2,
    videostop: 3,
    loadmodel: 5,
    cameracalibstart: 8,
    cameracalibstop: 9,
    scannercalibstart: 10,
    scannercalibstop: 11,
    scanstart: 12,
    scanstop: 13,
    setprop: 14,
    getprop: 16,
    controllerstart:18,
    controllerstop:19,
    rotate:20,
    laserset:21
  };

  const properties = {
    videoalive: "videoalive",
    calibratingcamera: "calibratingcamera",
    calibratingscanner: "calibratingscanner",
    scanning: "scanning",
    cameracalibrated: "cameracalibrated",
    scannercalibrated: "scannercalibrated",
    cameracalibcaptures:"cameracalibcaptures"
};

  const rotation={
    clockwise:0,
    counterclockwise:1
  };

  exports.config = {};
  exports.config.debug = debug;
  exports.config.commands = commands;
  exports.config.events = events;
  exports.config.rotation = rotation;
  exports.config.properties = properties;