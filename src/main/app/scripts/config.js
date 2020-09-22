const events =  {
    status: "status",
    imupdate: "imupdate",
    videostart: "videostart",
    videostop: "videostop",
    error: "error",
    iostart: "iostart",
    iostop: "iostop",
    propchanged: "propchanged"
};
  
  const commands =  {
    iostart: 0,
    iostop: 1,
    videostart: 2,
    videostop: 3,
    loadmodel: 5,
    input: 15,
    cameracalibstart: 8,
    cameracalibstop: 9,
    scannercalibstart: 10,
    scannercalibstop: 11,
    scanstart: 12,
    scanstop: 13,
    setprop: 14
  };

  const properties = {
    videoalive: 0,
    calibratingcamera: 1,
    calibratingscanner: 2,
    scanning: 3
  };

  exports.config = {};
  exports.config.commands = commands;
  exports.config.events = events;
  exports.config.properties = properties;