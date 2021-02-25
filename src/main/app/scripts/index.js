const { config } = require("../scripts/config");
const swal = require("sweetalert2");
require("../scripts/util");
const { ipcRenderer,remote } = require('electron');
const { data } = require("jquery");
const scanner=remote.getGlobal('scanner');

var index = () => {
    let video = document.getElementById("video");
    let videoBtn = document.getElementById("video-btn");
    let camCalibBtn = document.getElementById("camera-calibration-btn");
    let scannerCalibBtn = document.getElementById("scanner-calibration-btn");
    let rotateLeftBtn = document.getElementById("rotate-left-btn");
    let rotateRightBtn = document.getElementById("rotate-right-btn");
    let cameraCalibratedText = document.getElementById("camera-calibrated-text");
    let scannerCalibratedText = document.getElementById("scanner-calibrated-text");
    let camCalibCaptures = document.getElementById("captures");
    let camCalibCapturesMax = document.getElementById("max-captures");
    let slideshow = document.getElementById("slideshow");
    let videoSpinner = document.getElementById("video-spinner");
    let scannerCalibPlotButton = document.getElementById("scanner-calibration-plot-btn");
    let scannerCalibResetBtn = document.getElementById("scanner-calibration-reset-btn");

    let videoalive = false;
    let calibratingcamera = false;
    let calibratingscanner = false;
    let scanning = false;
    let cameracalibrated = false;
    let scannercalibrated = false;

    let scannerCalibImages=[];
    let scannerCalibPoints=[];
    let scannerCalibPlane=null;
    let scannerCalibPlotWin=null;

    let cameraCalibCaptures=0;      

//     <div class="slideshow-div">
//     <img src="/home/nejc/Pictures/lemon.jpg"></img>
// </div>
    // const createSlideshowImage=(id, base64)=> {
    //     let div = document.createElement("div");
    //     div.id=id;
    //     div.classList.add("slideshow-div");
    //     let img=document.createElement("img");        
    //     let blob = new Blob([base64], {type: "image/jpeg"});
    //     let frame = URL.createObjectURL(blob);
    //     img.src = frame;
    //     div.appendChild(img);
    //     document.getElementById("slideshow").appendChild(div);
    // }

    const openWin= (fname) => {
        let ref = new remote.BrowserWindow({
            parent: remote.getCurrentWindow(),
            // modal: true,
            webPreferences: {
                nodeIntegration: true
            }        
        });
        
        let url = 'file://' +__dirname+'/'+fname;
        ref.loadURL(url);
        
        return ref;
    };

    //https://stackoverflow.com/questions/9913765/rapidly-updating-image-with-data-uri-causes-caching-memory-leak
    const URL = window.URL || window.webkitURL;
    let frame;
    const showImage = (base64) => {
        if(frame) URL.revokeObjectURL(frame); 
        
        let blob = new Blob([base64], {type: "image/jpeg"});
        frame = URL.createObjectURL(blob);
        video.src = frame;
    };


    const videoaliveChanged = (val) => {
        if(val) {
            videoSpinner.classList.add("invisible");
            videoBtn.textContent = "Stop";
            scanner.addListener(config.events.imupdate, showImage);
        }
        else {
            videoBtn.textContent = "Start";
            scanner.removeListener(config.events.imupdate, showImage);
            video.setAttribute(
                'src', ""
            );       
        }

        videoalive = val;
    };

    const calibratingcameraChanged = (val) => {
        cameraCalibCaptures=0;
        camCalibCaptures.innerText = cameraCalibCaptures;     

        if(val)  {
            scanner.addListener(config.events.cameracalibcaptured, camCalibCaptured);          
            videoBtn.disabled=true;
            scannerCalibBtn.disabled=true;
            camCalibBtn.textContent = "Stop";
        }
        else {
            scanner.removeListener(config.events.cameracalibcaptured, camCalibCaptured);          
            videoBtn.disabled=false;
            scannerCalibBtn.disabled=false;
            camCalibBtn.textContent = "Start";
        }

        calibratingcamera = val;
    };

    const calibratingscannerChanged = (val) => {
        if(val) {
            scanner.addListener(config.events.scannercalibdata, scannerCalibData);
            slideshow.classList.remove("invisible");
            videoBtn.disabled=true;
            camCalibBtn.disabled=true;
            scannerCalibBtn.textContent = "Stop";
        }
        else {
            if(scannerCalibPlotWin!=null) scannerCalibPlotWin.close();

            scanner.removeListener(config.events.scannercalibdata, scannerCalibData);
            scannerCalibImages=[];
            slideshow.innerHTML='';
            slideshow.classList.add("invisible");
            videoBtn.disabled=false;
            camCalibBtn.disabled=false;
            scannerCalibBtn.textContent = "Start";
        }

        calibratingscanner = val;
    };

    const cameraCalibratedChanged = (val) => {
        if(val) {
            cameraCalibratedText.textContent="CALIBRATED";
            cameraCalibratedText.classList.add("badge-success");
            cameraCalibratedText.classList.remove("badge-danger");
        }
        else {
            cameraCalibratedText.textContent="UNCALIBRATED";
            cameraCalibratedText.classList.remove("badge-success");
            cameraCalibratedText.classList.add("badge-danger");
        }


        cameracalibrated = val;
    };

    const camCalibCaptured = () => {
        camCalibCaptures.innerText = ++cameraCalibCaptures;
    };

    const scannerCalibData = (msg,base64) => {
        msg=JSON.parse(msg);
        console.log(msg);

        if(msg["type"]=="image") {
            if(msg["size"]=="thumbnail") {
                let div = document.createElement("div");
                div.id="slim-"+msg["id"];
                div.classList.add("slideshow-div");
                let img=document.createElement("img");        
                let blob = new Blob([base64], {type: "image/jpeg"});
                let frame = URL.createObjectURL(blob);
                img.src = frame;
                div.appendChild(img);
    
                div.addEventListener("mouseenter", (e)=>{
                    e.target.classList.add("hover-thumbnail");
                });
                
                div.addEventListener("mouseleave", (e)=>{
                    e.target.classList.remove("hover-thumbnail");
                });
    
                div.addEventListener("click", (e)=>{
                    let win=openWin("imageSlideshow.html");
                    id=parseInt(e.currentTarget.id.split("-")[1]);
    
                    let data={
                        id: id,
                        images:scannerCalibImages
                    };

                    win.webContents.once("did-finish-load",()=>{
                        win.webContents.send('data', data);
                    });
    
                    win.webContents.openDevTools();
                });
    
                slideshow.appendChild(div);    
            }
            else if(msg["size"]=="large") {
                let obj={
                    id:msg["id"],
                    data:base64
                };
    
                scannerCalibImages.push(obj);
            }
        }
        else if(msg["type"]=="point") {
            scannerCalibPoints.push(msg);

            if(scannerCalibPlotWin!=null&&!scannerCalibPlotWin.isDestroyed()) {                
                scannerCalibPlotWin.webContents.send('data', [msg]);
            }
        }
        else if(msg["type"]=="plane") {
            scannerCalibPlane=msg;
            if(scannerCalibPlotWin!=null&&!scannerCalibPlotWin.isDestroyed()) scannerCalibPlotWin.webContents.send('data', [msg]);
        }
    };

    const scannercalibratedChanged = (val) => {
        if(val) {
            scannerCalibratedText.textContent="CALIBRATED";
            scannerCalibratedText.classList.add("badge-success");
            scannerCalibratedText.classList.remove("badge-danger");
        }
        else {
            scannerCalibratedText.textContent="UNCALIBRATED";
            scannerCalibratedText.classList.remove("badge-success");
            scannerCalibratedText.classList.add("badge-danger");
        }

        scannercalibrated = val;
    };


    const stopvideo = () => {
        videoSpinner.classList.add("invisible");
    
        if(calibratingcamera || calibratingscanner || scanning) setprop(config.properties.videoalive, false); 
        else scanner.sendCommand(JSON.stringify({code:config.commands.videostop})); 
    };

    const startvideo = () => {
        videoSpinner.classList.remove("invisible");

        if(calibratingcamera || calibratingscanner || scanning) {
            setprop(config.properties.videoalive, true);
        }  
        else scanner.sendCommand(JSON.stringify({code:config.commands.videostart}));
    };

    const startcameracalib = () => {
        videoSpinner.classList.remove("invisible");

        // ipcRenderer.send("forward-command", {code:config.commands.cameracalibstart});
        scanner.sendCommand(JSON.stringify({code:config.commands.cameracalibstart}));

    };

    const stopcameracalib = () => {
        videoSpinner.classList.add("invisible");

        // ipcRenderer.send("forward-command", {code: config.commands.cameracalibstop});
        scanner.sendCommand(JSON.stringify({code:config.commands.cameracalibstop}));
    };

    const startScannerCalib = () => {
        videoSpinner.classList.remove("invisible");

        // ipcRenderer.send("forward-command", {code:config.commands.scannercalibstart});
        scanner.sendCommand(JSON.stringify({code:config.commands.scannercalibstart}));
    };

    const stopScannerCalib = () => {
        videoSpinner.classList.add("invisible");

        // ipcRenderer.send("forward-command", {code:config.commands.scannercalibstop});
        scanner.sendCommand(JSON.stringify({code:config.commands.scannercalibstop}));
    };

    const rotateright = () => {
        // ipcRenderer.send("forward-command", {code:config.commands.rotate,direction:config.rotation.clockwise});
        scanner.sendCommand(JSON.stringify({code:config.commands.rotate,direction:config.rotation.clockwise}));
    };

    const rotateleft = () => {
        // ipcRenderer.send("forward-command", {code:config.commands.rotate,direction:config.rotation.counterclockwise});
        scanner.sendCommand(JSON.stringify({code:config.commands.rotate,direction:config.rotation.counterclockwise}));
    };

    const startscanning = () => {
        // ipcRenderer.send("forward-command", {code:config.commands.startcameracalib});
        //TODO: register
        scanner.sendCommand(JSON.stringify({code:config.commands.startcameracalib}));
    };

    const stopscanning = () => {
        // ipcRenderer.send("forward-command", {code:config.commands.stopcameracalib});
        //TODO: register
        scanner.sendCommand(JSON.stringify({code:config.commands.stopcameracalib}));
    };

    // let laserState=0;

    // const laserset = () => {
    //     laserState=(laserState+1)%2;
    //     ipcRenderer.send("forward-command", {code:config.commands.laserset,state:laserState});
    // };

    const setprop = (name, val) => {
        // ipcRenderer.setProp("setprop", name, val);
        scanner.setProp(JSON.stringify({code: config.commands.setprop, prop: name, value: val}));
    };

    return {
        init: () => {      
            document.addEventListener("keyup", (e) => {
                ipcRenderer.send("forward-keystroke", e.keyCode);
                //laserset();
            });
            
            videoBtn.addEventListener("click", (e) => {
                if(videoalive) stopvideo();
                else startvideo();
            });

            camCalibBtn.addEventListener("click", (e) => {
                if(calibratingcamera) stopcameracalib();
                else startcameracalib();
            });
      
            scannerCalibBtn.addEventListener("click", (e) => {
                if(calibratingscanner) stopScannerCalib();
                else startScannerCalib();
            });

            rotateLeftBtn.addEventListener("click", (e) => {
                rotateleft();
            });

            rotateRightBtn.addEventListener("click", (e) => {
                rotateright();

                // openWin();
            });

            scannerCalibResetBtn.addEventListener("click", (e) => {
                if(scannerCalibPlotWin!=null&&!scannerCalibPlotWin.isDestroyed()) scannerCalibPlotWin.close();

                scanner.postMessage(JSON.stringify({recipient:"camera_thread",data:"clear"}));
                scannerCalibImages=[];
                scannerCalibPoints=[];
                scannerCalibPlane=[];
                slideshow.innerHTML="";
                setprop(config.properties.scannercalibrated, false);
            });

            scannerCalibPlotButton.addEventListener("click",(e)=> {
                scannerCalibPlotWin=openWin("scannerCalibPlot.html");

                // let pts={
                //     type:"points",
                //     data:scannerCalibPoints
                // };

                // let plane={
                //     type:"plane",
                //     data:scannerCalibPlane
                // };

                // scannerCalibPlotWin.onbeforeunload = ()=>{
                //     scannerCalibPlotWin=null;
                // };

                scannerCalibPlotWin.webContents.once("did-finish-load",()=>{
                    if(scannercalibrated) scannerCalibPlotWin.webContents.send('data', [scannerCalibPlane]);

                    scannerCalibPlotWin.webContents.send('data', scannerCalibPoints);
                });

                /*if(config.debug)*/ scannerCalibPlotWin.webContents.openDevTools();
            });

            // debugCaptureBtn.addEventListener("click", (e) => {
            //     openWin("debugCapture.html");

            //     const fn = (base64) => {
            //         modal.webContents.send('data', base64);
            //     };

            //     scanner.addListener(config.events.debugcapture,fn);

            //     modal.onbeforeunload = (e) => {
            //         scanner.removeListener(config.events.debugcapture, fn);
            //     }

            //     modal.webContents.openDevTools();
            // });

            // scanner.addListener(config.events.propchanged, (msg) => {
            //     msg = JSON.parse(msg);
          
            //     if(msg.prop == config.properties.videoalive) {
            //       if(msg.value) {
            //         scanner.addListener(config.events.imupdate, forwardVideo);
            //       } 
            //       else {
            //         scanner.removeListener(config.events.imupdate, forwardVideo);
            //       }
            //     }
          
            //     index.webContents.send(config.events.propchanged, msg);
            //   });
          
            scanner.addListener(config.events.propchanged, (msg) => {                
                msg=JSON.parse(msg);

                if(msg["prop"] == config.properties.videoalive) 
                    videoaliveChanged(msg["value"]);
                else if(msg["prop"] == config.properties.calibratingcamera) 
                    calibratingcameraChanged(msg["value"]);
                else if(msg["prop"] == config.properties.calibratingscanner)
                    calibratingscannerChanged(msg["value"]);
                else if(msg["prop"] == config.properties.cameracalibrated)
                    cameraCalibratedChanged(msg["value"]);                
                else if(msg["prop"] == config.properties.scannercalibrated)
                    scannercalibratedChanged(msg["value"]);                
                
            });

            // scanner.addListener(config.events.cameracalibcaptured, camCalibCaptured);          
            // scanner.addListener(config.events.scannerCalibData,scannerCalibData);          

            // scanner.addListener(config.events.cameracalibcaptured, () => {                
            //     if(calibratingcamera) 
            //         camCalibCaptured();
            // });          

            scanner.addListener(config.events.error, (msg) => {
                swal.fire({
                    icon: 'error',
                    title: 'Error',
                    text: msg,
                });
            });

            if(/*config.debug*/false) {
                scanner.addListener(config.events.debugcapture, (base64)=> {
                    let win=openWin("debugCapture.html");
    
                    win.webContents.once("did-finish-load",()=>{
                        win.webContents.send('data', base64);
                    });
    
                    win.webContents.openDevTools();
                });    
            }

            scanner.sendCommand(JSON.stringify({code: config.commands.iostart}));
            scanner.sendCommand(JSON.stringify({code: config.commands.controllerstart}));

            scanner.getProp(config.properties.cameracalibrated).then((response)=> {
                cameraCalibratedChanged(response);
            });

            scanner.getProp(config.properties.scannercalibrated).then((response)=> {
                scannercalibratedChanged(response);
            });

            scanner.getProp(config.properties.cameracalibcaptures).then((response)=> {
                camCalibCapturesMax.innerText=response;
            });
        }
    };
};