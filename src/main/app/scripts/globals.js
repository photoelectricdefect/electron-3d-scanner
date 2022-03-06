const debug=true;

const events =  {
    status: "status",
    imupdate: "imupdate",
    videostart: "videostart",
    videostop: "videostop",
    error: "error",
    mainstart: "mainstart",
    mainstop: "mainstop",
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
    scandata: "scandata",
    debugcapture:"debugcapture"
  };
  
  const commands =  {
    mainstart: 0,
    mainstop: 1,
    videostart: 2,
    videostop: 3,
    loadmodel: 5,
    cameracalibstart: 8,
    cameracalibstop: 9,
    scannercalibstart: 10,
    scannercalibstop: 11,
    scanstart: 12,
    scanstop: 13,
    // setprop: 14,
    // getprop: 16,
    controllerstart:18,
    controllerstop:19,
    rotate:20,
    togglelaser:21
  };

  const properties = {
    displayvideo: "displayvideo",
    calibratingcamera: "calibratingcamera",
    calibratingscanner: "calibratingscanner",
    cameracalibrated: "cameracalibrated",
    scannercalibrated: "scannercalibrated",
    cameracalibcaptures:"cameracalibcaptures",
    cameralist:"cameralist",
    selectedcamera:"selectedcamera",
    scannercalibrationdata:"scannercalibrationdata",
    scanrenderdata:"scanrenderdata",
  };

  const rotation={
    clockwise:0,
    counterclockwise:1
  };

  globals = {};
  globals.debug = debug;
  globals.commands = commands;
  globals.events = events;
  globals.rotation = rotation;
  globals.properties = properties;