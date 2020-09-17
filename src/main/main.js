'use strict'

const { app, BrowserWindow, ipcMain, ipcRenderer } = require('electron');
const scanner = require("bindings")("scanner");

let index;

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
    scanner.addListener(events.imupdate, (base64) => {
      index.webContents.send(events.imupdate, base64);
    });
  
    scanner.addListener(events.videostart, () => {
      scanner.addListener(events.imupdate, (base64) => {
        index.webContents.send(events.imupdate, base64);
      });  
    });

    scanner.addListener(events.videostop, () => {
      index.webContents.removeListener(events.imupdate);
    });

    scanner.addListener(events.status, (msg) => {
    });

    scanner.addListener(events.error, (msg) => {
    });

    scanner.addListener(events.iostart, (msg) => {
      console.log(events.iostart);
    });

    scanner.addListener(events.iostop, (msg) => {
      console.log(events.iostop);
    });

    scanner.sendCommand(JSON.stringify({code:commands.iostart}));    
    // setTimeout(() => {
    //   scanner.sendCommand(JSON.stringify({code:commands.iostop}));    
    // }, 5000);

    // setTimeout(() => {
    //   scanner.sendCommand(JSON.stringify({code:commands.iostart}));    
    // }, 7000);

  });
});