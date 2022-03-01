const swal = require("sweetalert2");
const { ipcRenderer,remote } = require('electron');
const { data } = require("jquery");
const scanner=remote.getGlobal('scanner');

// const {globals} = require("globals");


var index = () => {
    let video = document.getElementById("video");
    let displayVideoBtn = document.getElementById("display-video-btn");
    let cameraCalibrationBtn = document.getElementById("camera-calibration-btn");
    let scannerCalibBtn = document.getElementById("scanner-calibration-btn");
    let rotateLeftBtn = document.getElementById("rotate-left-btn");
    let rotateRightBtn = document.getElementById("rotate-right-btn");
    let scannerCalibPlotButton = document.getElementById("scanner-calibration-plot-btn");
    let scannerCalibResetBtn = document.getElementById("scanner-calibration-reset-btn");
    let scanBtn = document.getElementById("scan-btn");
    let scanRenderBtn = document.getElementById("scan-render-btn");
    let cameraListLoadBtn=document.getElementById("camera-list-load-btn");
    let laserBtn=document.getElementById("laser-btn");
    
    
    let cameraCalibratedText = document.getElementById("camera-calibrated-text");
    let scannerCalibratedText = document.getElementById("scanner-calibrated-text");
    let cameraCalibCapturesText = document.getElementById("captures-text");
    let cameraCalibCapturesMaxText = document.getElementById("max-captures-text");
    let slideshow = document.getElementById("slideshow");
    let videoSpinner = document.getElementById("video-spinner");

    let cameraSelect=document.getElementById("camera-select");

    let displayVideo = false;
    let calibratingCamera = false;
    let calibratingScanner = false;
    let scanning = false;
    let cameracalibrated = false;
    let scannercalibrated = false;

    let scannerCalibImages=[];
    let scannerCalibPoints=[];
    let scannerCalibPlane=null;
    let scannerCalibPlotWin=null;
    let scanPlotWin=null;
    let scanRenderWin=null;
    let scanPoints=[];
    let scanPointCloud= {
        positions:[[],[],[]],
        colors:[[],[],[]]
    };

    let nCameraCalibCaptures=0;      

//     <div class="slideshow-div">
//     <img src="/home/nejc/Pictures/lemon.jpg"></img>
// </div>
    // const createSlideonImUpdate=(id, base64)=> {
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
    const onImUpdate = (base64) => {
        if(frame) URL.revokeObjectURL(frame); 
        
        let blob = new Blob([base64], {type: "image/jpeg"});
        frame = URL.createObjectURL(blob);
        video.src = frame;
    };


    const disableButtons = (value) => {
        displayVideoBtn.disabled=value;
        scannerCalibBtn.disabled=value;
        cameraCalibrationBtn.disabled=value;
        rotateLeftBtn.disabled=value;
        rotateRightBtn.disabled=value;
        scannerCalibPlotButton.disabled=value;
        scannerCalibResetBtn.disabled=value;
        scanBtn.disabled=value;
        scanRenderBtn.disabled=value;
        cameraListLoadBtn.disabled=value;
        laserBtn.disabled=value;
    };

    const setDefaultButtonText = () => {
        displayVideoBtn.textContent = "Start";
        scannerCalibBtn.textContent = "Start";
        cameraCalibrationBtn.textContent = "Start";
        rotateLeftBtn.textContent = "Left";
        rotateRightBtn.textContent = "Right";
        scannerCalibPlotButton.textContent = "Open point plot";
        scannerCalibResetBtn.textContent = "Clear";
        scanBtn.textContent = "Start";
        scanRenderBtn.textContent = "Open point plot";
        cameraListLoadBtn.textContent = "Load";
        laserBtn.textContent = "Toggle Laser";
    };

    const displayVideoChanged = (val) => {
        if(val) {
            videoSpinner.classList.add("invisible");
            displayVideoBtn.textContent = "Stop";
        }
        else {
            displayVideoBtn.textContent = "Start";
            video.setAttribute(
                'src', ""
            );       
        }

        displayVideo = val;
    };

    const calibratingCameraChanged = (val) => {
        setDefaultButtonText();
        disableButtons(true);
        cameraCalibrationBtn.disabled=false;

        if(val)  {
            nCameraCalibCaptures=0;
            cameraCalibCapturesText.parentElement.classList.remove("invisible");
            cameraCalibCapturesText.innerText = nCameraCalibCaptures;         
            cameraCalibrationBtn.textContent = "Stop";
        }
        else {
            cameraCalibCapturesText.parentElement.classList.add("invisible");
            cameraCalibrationBtn.textContent = "Start";
            disableButtons(false);
        }
        
        calibratingCamera = val;
    };

    const calibratingScannerChanged = (val) => {
        setDefaultButtonText();
        disableButtons(true);
        scannerCalibBtn.disabled=false;
        scannerCalibPlotButton.disabled=false;

        if(val) {
            slideshow.classList.remove("invisible");
            scannerCalibBtn.textContent = "Stop";
        }
        else {
            if(scannerCalibPlotWin!=null) scannerCalibPlotWin.close();

            scannerCalibImages=[];
            slideshow.innerHTML='';
            slideshow.classList.add("invisible");
            scannerCalibBtn.textContent = "Start";
            disableButtons(false);
        }

        calibratingScanner = val;
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

    const scanningChanged = (val) => {
        setDefaultButtonText();
        disableButtons(true);

        // if(val) {
        //     scanner.addListener(globals.events.scannercalibdata, onScannerCalibrationData);
        //     slideshow.classList.remove("invisible");
        //     displayVideoBtn.disabled=true;
        //     cameraCalibrationBtn.disabled=true;
        //     scannerCalibBtn.textContent = "Stop";
        // }
        // else {
        //     if(scannerCalibPlotWin!=null) scannerCalibPlotWin.close();

        //     scanner.removeListener(globals.events.scannercalibdata, onScannerCalibrationData);
        //     scannerCalibImages=[];
        //     slideshow.innerHTML='';
        //     slideshow.classList.add("invisible");
        //     displayVideoBtn.disabled=false;
        //     cameraCalibrationBtn.disabled=false;
        //     scannerCalibBtn.textContent = "Start";
        // }


        if(val) {
            scanBtn.textContent = "Stop";
            scanBtn.disabled=false;
            scanRenderBtn.disabled=false;    
        }
        else {
            scanBtn.textContent = "Start";
            disableButtons(false);
        }
        
        scanning = val;
    };

    const onScanData = (msg) => {
        msg=JSON.parse(msg);
        console.log(msg);

        if(msg["type"]=="points") {
            scanPoints.push(msg);
            
            for(let i=0;i<3;i++) {
                scanPointCloud.positions[i]=scanPointCloud.positions[i].concat(msg.positions[i]);
                scanPointCloud.colors[i]=scanPointCloud.colors[i].concat(msg.colors[i]);
            }
            
            if(scanPlotWin!=null&&!scanPlotWin.isDestroyed()) {                
                scanPlotWin.webContents.send('data', msg);
            }

            if(scanRenderWin!=null&&!scanRenderWin.isDestroyed()) {                
                scanRenderWin.webContents.send('data', {points:msg}); 
            }
        }
        else if(msg["type"]=="axis") {
            if(scanPlotWin!=null&&!scanPlotWin.isDestroyed()) {                
                scanPlotWin.webContents.send('data', msg);
            }
        }
    };


    const onCameraCalibrationCapture = () => {
        cameraCalibCapturesText.innerText = ++cameraCalibCapturesText;
    };

    const onScannerCalibrationData = (msg,base64) => {
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
            if(scannerCalibPlotWin!=null&&!scannerCalibPlotWin.isDestroyed()) 
                scannerCalibPlotWin.webContents.send('data', [msg]);
        }
    };

    const scannerCalibratedChanged = (val) => {
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

    const loadCameraList=()=> {
        scanner.getProp(globals.properties.cameralist).then((response)=> {
            response=JSON.parse(response);
            cameraSelect.innerHTML="";

            for(var i=0;i<response.data.length;i++) {
                var option = document.createElement("option");
                option.text = response.data[i].name;
                cameraSelect.add(option);  
            }
        });
    };

    const stopVideo = () => {
        videoSpinner.classList.add("invisible");
    
        if(calibratingCamera || calibratingScanner || scanning) 
            return; 
        
        setProp(globals.properties.displayvideo,false);
    };

    const startVideo = () => {
        videoSpinner.classList.remove("invisible");

        if(calibratingCamera || calibratingScanner || scanning) 
            return; 
        
        setProp(globals.properties.displayvideo,true);
    };

    const startcameracalib = () => {
        videoSpinner.classList.remove("invisible");
        scanner.sendCommand(JSON.stringify({code:globals.commands.cameracalibstart}));
    };

    const stopcameracalib = () => {
        videoSpinner.classList.add("invisible");
        scanner.sendCommand(JSON.stringify({code:globals.commands.cameracalibstop}));
    };

    const startScannerCalib = () => {
        videoSpinner.classList.remove("invisible");
        scanner.sendCommand(JSON.stringify({code:globals.commands.scannercalibstart}));
    };

    const stopScannerCalib = () => {
        videoSpinner.classList.add("invisible");
        scanner.sendCommand(JSON.stringify({code:globals.commands.scannercalibstop}));
    };

    const rotateRight = () => {
        scanner.sendCommand(JSON.stringify({code:globals.commands.rotate,direction:globals.rotation.clockwise}));
    };

    const rotateLeft = () => {
        scanner.sendCommand(JSON.stringify({code:globals.commands.rotate,direction:globals.rotation.counterclockwise}));
    };

    let laserState=false;

    const toggleLaser = () => {
        laserState=!laserState;
        scanner.sendCommand(JSON.stringify({code:globals.commands.togglelaser,state:laserState}));
    };

    const scanStart = () => {
        scanner.sendCommand(JSON.stringify({code:globals.commands.scanstart}));
    };

    const scanStop = () => {
        scanner.sendCommand(JSON.stringify({code:globals.commands.scanstop}));
    };

    const setProp = (name, val) => {
        scanner.setProp(JSON.stringify({prop: name, value: val}));
    };

    return {
        init: () => {      
            document.addEventListener("keyup", (e) => {
                // ipcRenderer.send("forward-keystroke", e.keyCode);
                //laserset();
                scanner.postMessage(JSON.stringify({recipient:"thread_camera",type:"keyup",keycode:e.key}));
            });
            
            displayVideoBtn.addEventListener("click", (e) => {
                if(displayVideo) stopVideo();
                else startVideo();
            });

            cameraCalibrationBtn.addEventListener("click", (e) => {
                if(calibratingCamera) stopcameracalib();
                else startcameracalib();
            });
      
            scannerCalibBtn.addEventListener("click", (e) => {
                if(calibratingScanner) stopScannerCalib();
                else startScannerCalib();
            });

            rotateLeftBtn.addEventListener("click", (e) => {
                rotateLeft();
            });

            rotateRightBtn.addEventListener("click", (e) => {
                rotateRight();
            });

            laserBtn.addEventListener("click",(e)=> {
                toggleLaser();
            });

            scanBtn.addEventListener("click", (e) => {
                if(scanning) scanStop();
                else scanStart();
            });

            cameraListLoadBtn.addEventListener("click", (e) => {
                loadCameraList();
            });

            scanRenderBtn.addEventListener("click",(e)=> {
                // scanPlotWin=openWin("scanPlot.html");
                scanRenderWin=openWin("scanRender.html");

                // scanPlotWin.webContents.once("did-finish-load",()=>{
                //     scanPlotWin.webContents.send('data', scanPoints);
                // });

                if(scannercalibrated) {
                    scanRenderWin.webContents.openDevTools();

                    scanRenderWin.webContents.once("did-finish-load",()=>{
                        scanner.getProp(globals.properties.scanrenderdata).then((response)=> {
                            scanRenderWin.webContents.send('init', {calibrationData:JSON.parse(response)});
                            scanRenderWin.webContents.send('data', {points:scanPointCloud});    
                        });            
                    });
    
                }

                // /*if(globals.debug)*/ scanPlotWin.webContents.openDevTools();
            });

            scannerCalibResetBtn.addEventListener("click", (e) => {
                if(scannerCalibPlotWin!=null&&!scannerCalibPlotWin.isDestroyed()) scannerCalibPlotWin.close();

                scanner.postMessage(JSON.stringify({recipient:"thread_camera",type:"clear_plane_points"}));
                scannerCalibImages=[];
                scannerCalibPoints=[];
                scannerCalibPlane=[];
                slideshow.innerHTML="";
                setProp(globals.properties.scannercalibrated, false);
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

                /*if(globals.debug)*/ scannerCalibPlotWin.webContents.openDevTools();
            });
          
            scanner.addListener(globals.events.scandata, onScanData);

            scanner.addListener(globals.events.propchanged, (msg) => {                
                msg=JSON.parse(msg);

                if(msg["prop"] == globals.properties.displayvideo) 
                    displayVideoChanged(msg["value"]);
                else if(msg["prop"] == globals.properties.calibratingcamera) 
                    calibratingCameraChanged(msg["value"]);
                else if(msg["prop"] == globals.properties.calibratingScanner)
                    calibratingScannerChanged(msg["value"]);
                else if(msg["prop"] == globals.properties.cameracalibrated)
                    cameraCalibratedChanged(msg["value"]);                
                else if(msg["prop"] == globals.properties.scannercalibrated)
                    scannerCalibratedChanged(msg["value"]);                                
                else if(msg["prop"] == globals.properties.scanning)
                    scanningChanged(msg["value"]);                
            });


            scanner.addListener(globals.events.imupdate, onImUpdate);
            scanner.addListener(globals.events.cameracalibcaptured, onCameraCalibrationCapture);          
            scanner.addListener(globals.events.scannercalibdata, onScannerCalibrationData);
            scanner.addListener(globals.events.scandata, onScanData);

            // scanner.addListener(globals.events.cameracalibcaptured, onCameraCalibrationCapture);          
            // scanner.addListener(globals.events.onScannerCalibrationData,onScannerCalibrationData);          

            // scanner.addListener(globals.events.cameracalibcaptured, () => {                
            //     if(calibratingCamera) 
            //         onCameraCalibrationCapture();
            // });          

            scanner.addListener(globals.events.error, (msg) => {
                swal.fire({
                    icon: 'error',
                    title: 'Error',
                    text: msg,
                });
            });

            if(globals.debug/*false*/) {
                scanner.addListener(globals.events.debugcapture, (base64)=> {
                    let win=openWin("debugCapture.html");

                    win.webContents.once("did-finish-load",()=>{
                        win.webContents.send('data', base64);
                    });
    
                    win.webContents.openDevTools();
                });    
            }

            scanner.addListener(globals.events.mainstart, ()=> {
                scanner.getProp(globals.properties.cameracalibrated).then((response)=> {
                    cameraCalibratedChanged(response);
                });
    
                scanner.getProp(globals.properties.scannercalibrated).then((response)=> {
                    // console.log("cal:"+response);
                    scannerCalibratedChanged(response);
                });    

                scanner.getProp(globals.properties.cameracalibcaptures).then((response)=> {
                    cameraCalibCapturesMaxText.innerText=response;
                });
    
                scanner.sendCommand(JSON.stringify({code: globals.commands.controllerstart}));
                scanner.sendCommand(JSON.stringify({code: globals.commands.videostart}));    
                loadCameraList();
            });   

            scanner.sendCommand(JSON.stringify({code: globals.commands.mainstart}));
        }
    };
};