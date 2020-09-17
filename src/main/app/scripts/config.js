const events =  {
    status: "status",
    imupdate: "imupdate",
    videostart: "videostart",
    videostop: "videostop",
    error: "error",
    iostart: "iostart",
    iostop: "iostop"
  };
  
  const commands =  {
    iostart: 0,
    iostop: 1,
    videostart: 2,
    videostop: 3,
    scan: 4,
    loadmodel: 5,
    setprop: 6
  };

  exports.config = {};
  exports.config.commands = commands;
  exports.config.events = events;