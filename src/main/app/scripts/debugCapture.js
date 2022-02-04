const { ipcRenderer } = require('electron');

var debugCapture = () => {
    let container=document.getElementById("container");

    return {
        init: ()=> {
            ipcRenderer.on('data', (e,data)=> {
                let img=document.createElement("img");    
                let blob = new Blob([data], {type: "image/jpeg"});
                let frame = URL.createObjectURL(blob);
                img.src = frame;
                container.appendChild(img);
            });        
        }
    };
};