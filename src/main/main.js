'use strict'

const { app, BrowserWindow, ipcMain, ipcRenderer } = require('electron');
const scannerapi = require("bindings")("scanner");

let index;

const events =  {
  status: "status",
  imupdate: "imupdate",
  videostart: "videostart",
  videostop: "videostop",
  error: "error"
};

const openIndex =  () => {
    index = new BrowserWindow({
      width: 800,
      height: 600,
      webPreferences: {
        nodeIntegration: true
      }
    });
  
    index.loadFile('app/views/index.html');
    index.webContents.openDevTools();
}
  
app.whenReady().then(openIndex).then(() => {
  index.webContents.on("did-finish-load", () => {
    //remove later
    scannerapi.addListener(events.imupdate, (base64) => {
      index.webContents.send(events.imupdate, base64);
    });
  
    scannerapi.addListener(events.videostart, () => {
      scannerapi.addListener(events.imupdate, (base64) => {
        index.webContents.send(events.imupdate, base64);
      });  
    });

    scannerapi.addListener(events.videostop, () => {
      index.webContents.removeListener(events.imupdate);
    });

    scannerapi.addListener(events.status, (msg) => {
    });

    scannerapi.addListener(events.error, (msg) => {
    });

    scannerapi.test_imemit("/home/nejc/projects/cv/improved-ChInEsE-3d-scanner/src/assets/thomas.png");
    // setTimeout(() => {
    //   scannerapi.test_imemit("/home/nejc/projects/cv/improved-ChInEsE-3d-scanner/src/assets/thomas.png");  
    // }, 3000);
  });
});