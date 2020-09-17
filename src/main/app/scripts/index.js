const { config } = require("../scripts/config");
const { ipcRenderer } = require('electron');

var index = () => {
    let video = document.getElementById("video");
    let videobtn = document.getElementById("video-btn");
    let videoalive = false;

    const showImage = (e, base64) => {
        video.setAttribute(
            'src', "data:image/jpeg;base64, " + base64
        );
    };

    return {
        init: () => {            
            videobtn.addEventListener("click", (e) => {
                if(videoalive) {
                    ipcRenderer.on("imupdate", showImage);                            
                    ipcRenderer.send("forward-command", config.commands.videostart);
                }
                else {
                    ipcRenderer.removeListener("imupdate", showImage);
                    video.setAttribute(
                        'src', ""
                    );            
                    ipcRenderer.send("forward-command", config.commands.videostop);
                }

                videoalive = !videoalive;
            });
        }
    };
};