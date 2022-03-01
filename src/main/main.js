'use strict'

const { app, BrowserWindow, ipcMain } = require('electron');
const scanner = require("./build/Release/scanner");
const { globals } = require("./app/scripts/globals");

global.scanner=scanner;
let index;

const openIndex =  () => {
    index = new BrowserWindow({
      width: 1200,
      height: 800,
      webPreferences: {
        nodeIntegration: true,
        enableRemoteModule: true
      }
    });
  
    index.loadFile('app/views/index.html');
    index.webContents.openDevTools();
}

app.whenReady().then(openIndex).then(() => {
  index.webContents.on("did-finish-load", () => {  
    // const forwardVideo = (base64) => {
    //   index.webContents.send(config.events.imupdate, base64);
    // };

    // scanner.addListener(config.events.propchanged, (msg) => {
    //   separator();
    //   msg = JSON.parse(msg);

    //   if(msg.prop == config.properties.videoalive) {
    //     if(msg.value) {
    //       scanner.addListener(config.events.imupdate, forwardVideo);
    //     } 
    //     else {
    //       scanner.removeListener(config.events.imupdate, forwardVideo);
    //     }
    //   }

    //   index.webContents.send(config.events.propchanged, msg);
    // });

    // scanner.addListener(config.events.error, (msg) => {
    //   index.webContents.send(config.events.error, msg);
    // });

    // ipcMain.on("getprop", (e, name, val) => {
    //   scanner.getProp(JSON.stringify({code: config.commands.setprop, prop: name, value: val}));
    // });

    // ipcMain.on("setprop", (e, name, val) => {
    //   scanner.setProp(JSON.stringify({code: config.commands.setprop, prop: name, value: val}));
    // });

    // ipcMain.on('forward-command', (e, comm) => {
    //   scanner.sendCommand(JSON.stringify(comm));
    //  });  

    //  ipcMain.on('forward-keystroke', (e, kyc) => {
    //   scanner.keyboardInput(JSON.stringify({code: -1, keycode: kyc}));
    //  });  
  });
});
