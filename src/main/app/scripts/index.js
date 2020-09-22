const { config } = require("../scripts/config");
require("../scripts/util");
const { ipcRenderer } = require('electron');

var index = () => {
    let video = document.getElementById("video");
    let videobtn = document.getElementById("video-btn");
    let calibbtn = document.getElementById("calibration-btn");
    
    //TODO: bind these to events emitted by scanner module

    let videoalive = false;
    let calibratingcamera = false;
    let calibratingscanner = false;
    let scanning = false;

    const showImage = (e, base64) => {
        video.setAttribute(
            'src', "data:image/jpeg;base64, " + base64
        );
    };

    const videoalivechanged = (val) => {
        if(val) {
            videobtn.textContent = "Stop Video";
            ipcRenderer.on("imupdate", showImage);                    
        }
        else {
            videobtn.textContent = "Start Video";
            ipcRenderer.removeListener("imupdate", showImage);
            video.setAttribute(
                'src', ""
            );       
        }

        videoalive = val;
    };

    const calibratingcamerachanged = (val) => {
        calibratingcamera = val;
    };


    const stopvideo = () => {    
        if(calibratingcamera || calibratingscanner || scanning) setprop(config.properties.videoalive, false); 
        else ipcRenderer.send("forward-command", config.commands.videostop);
    };

    const startvideo = () => {
        if(calibratingcamera || calibratingscanner || scanning) setprop(config.properties.videoalive, true); 
        else ipcRenderer.send("forward-command", config.commands.videostart);
    };

    const startcameracalib = () => {
        //videobtn.textContent = "Stop Camera Calibration";
        ipcRenderer.send("forward-command", config.commands.cameracalibstart);
        //TODO: register
    };

    const stopcameracalib = () => {
        //videobtn.textContent = "Start Camera Calibration";
        ipcRenderer.send("forward-command", config.commands.cameracalibstop);
        //TODO: register
    };




    const startscannercalib = () => {
        ipcRenderer.send("forward-command", config.commands.startcameracalib);
        //TODO: register
    };

    const stopscannercalib = () => {
        ipcRenderer.send("forward-command", config.commands.stopcameracalib);
        //TODO: register
    };

    const startscanning = () => {
        ipcRenderer.send("forward-command", config.commands.startcameracalib);
        //TODO: register
    };

    const stopscanning = () => {
        ipcRenderer.send("forward-command", config.commands.stopcameracalib);
        //TODO: register
    };

    const setprop = (name, val) => {
        ipcRenderer.send("setprop", name, val);
    };

    const keycodes = {
        space: 32
    };

    return {
        init: () => {      
            document.addEventListener("keyup", (e) => {
                if(e.keyCode == keycodes.space) ipcRenderer.send("forward-input", keycodes.space);
            });
            
            videobtn.addEventListener("click", (e) => {
                if(videoalive) stopvideo();
                else startvideo();
            });

            calibbtn.addEventListener("click", (e) => {
                if(calibratingcamera) stopcameracalib();
                else startcameracalib();
            });
      
            ipcRenderer.on(config.events.propchanged, (e, msg) => {
                console.log(msg);
                separator();

                if(msg.prop == config.properties.videoalive) 
                    videoalivechanged(msg.value);
                if(msg.prop == config.properties.calibratingcamera) 
                    calibratingcamerachanged(msg.value);
            });
        }
    };
};