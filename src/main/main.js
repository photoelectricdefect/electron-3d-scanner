'use strict'

const { app, BrowserWindow, ipcMain } = require('electron');
const scanner = require("bindings")("scanner");
const { config } = require("./app/scripts/config");

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
    scanner.addListener(config.events.iostart, () => {
      console.log(config.events.iostart);
    });

    scanner.addListener(config.events.iostop, () => {
      console.log(config.events.iostop);
    });

    scanner.addListener(config.events.videostart, () => {
      console.log(config.events.videostart);

      scanner.addListener(config.events.imupdate, (base64) => {
        index.webContents.send(config.events.imupdate, base64);
      });  
    });

    scanner.addListener(config.events.videostop, () => {
      console.log(config.events.videostop);
      index.webContents.removeListener(config.events.imupdate);
    });

    scanner.addListener(config.events.status, (msg) => {
      console.log(config.events.status);
      console.log(msg);
    });

    scanner.addListener(config.events.error, (msg) => {
      console.log(config.events.error);
      console.log(msg);
    });

    ipcMain.on('forward-command', (comm) => {
      scanner.sendCommand(JSON.stringify({code: comm}))
    });  

    scanner.sendCommand(JSON.stringify({code: config.commands.iostart}));
  });
});
