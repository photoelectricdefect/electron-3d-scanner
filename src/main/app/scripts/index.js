const { ipcRenderer } = require('electron');

var index = () => {
    let video = document.getElementById("video");

    return {
        init: () => {            
            ipcRenderer.on("imupdate", (e, base64) => {
                video.setAttribute(
                    'src', "data:image/jpeg;base64, " + base64
                );
            });
        }
    };
};