'use strict'

const { app, BrowserWindow, ipcMain } = require('electron');
const scanner = require("bindings")("scanner");
const { config } = require("./app/scripts/config");
const { separator } = require("./app/scripts/util");

let index;

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
    const forwardVideo = (base64) => {
      index.webContents.send(config.events.imupdate, base64);
    };

    // scanner.addListener(config.events.iostart, () => {
    //   console.log(config.events.iostart);
    //   separator();
    // });

    // scanner.addListener(config.events.iostop, () => {
    //   console.log(config.events.iostop);
    //   separator();
    // });

    // scanner.addListener(config.events.videostart, () => {
    //   console.log(config.events.videostart);
    //   separator();
    // });

    // scanner.addListener(config.events.videostop, () => {
    //   console.log(config.events.videostop);
    //   separator();
    // });

    // scanner.addListener(config.events.status, (msg) => {
    //   console.log(config.events.status);
    //   console.log(msg);
    //   separator();
    // });

    // scanner.addListener(config.events.error, (msg) => {
    //   console.log(config.events.error);
    //   console.log(msg);
    //   separator();
    // });

    scanner.addListener(config.events.propchanged, (msg) => {
      console.log(config.events.propchanged);
      console.log(msg);
      separator();
      msg = JSON.parse(msg);

      if(msg.prop == config.properties.videoalive) {
        if(msg.value) {
          scanner.addListener(config.events.imupdate, forwardVideo);
        } 
        else {
          scanner.removeListener(config.events.imupdate, forwardVideo);
        }
      }

      index.webContents.send(config.events.propchanged, msg);
    });

    ipcMain.on("setprop", (e, name, val) => {
      scanner.setProp(JSON.stringify({code: config.commands.setprop, prop: name, value: val}));
    });

    ipcMain.on('forward-command', (e, comm) => {
      scanner.sendCommand(JSON.stringify({code: comm}));
     });  

     ipcMain.on('forward-keystroke', (e, kyc) => {
      scanner.keyboardInput(JSON.stringify({code: config.commands.keystroke, keycode: kyc}));
     });  

    scanner.sendCommand(JSON.stringify({code: config.commands.iostart}));
  });
});
