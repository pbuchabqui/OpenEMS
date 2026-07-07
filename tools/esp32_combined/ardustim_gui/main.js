const { app, BrowserWindow, ipcMain } = require('electron')
const {spawn} = require('child_process');
const {execFile} = require('child_process');

// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.
let win

var espToolErr = "";
var espToolIsRunning = false;

function createWindow () {
  // Create the browser window.
  windowWidth = 1024;
  windowHeight = 700;
  if(process.platform == "win32") 
  {
    windowWidth = 1037;
    windowHeight = 725;
  }

  win = new BrowserWindow({
    width: windowWidth,
    height: windowHeight, 
    backgroundColor: '#312450', 
    webPreferences: {
      contextIsolation: false,
      nodeIntegration: true,
      backgroundThrottling: false,
    },
  });

  // auto hide menu bar (Win, Linux)
  win.setMenuBarVisibility(false);
  win.setAutoHideMenuBar(true);

  // remove completely when app is packaged (Win, Linux)
  if (app.isPackaged) {
    win.removeMenu();
  }

  // and load the index.html of the app.
  win.loadFile('index.html')

  // Open the DevTools.
  //win.webContents.openDevTools()

  // Emitted when the window is closed.
  win.on('closed', () => {
    // Dereference the window object, usually you would store windows
    // in an array if your app supports multi windows, this is the time
    // when you should delete the corresponding element.
    win = null
  })

  // Open links in external browser
  win.webContents.setWindowOpenHandler(({ url }) => {
    if (url.startsWith('https:')) {
      require('electron').shell.openExternal(url);
    }
    return { action: 'deny' };
  });
}

app.allowRendererProcessReuse = false;

// Register handler before app.on/createWindow as this is used during window creation
ipcMain.handle('getAppVersion', async (e) => {
  return app.getVersion();
});

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
// Some APIs can only be used after this event occurs.
app.on('ready', createWindow)

// Quit when all windows are closed.
app.on('window-all-closed', () => {
  // On macOS it is common for applications and their menu bar
  // to stay active until the user quits explicitly with Cmd + Q
  if (process.platform !== 'darwin') {
    app.quit()
  }
})

app.on('activate', () => {
  // On macOS it's common to re-create a window in the app when the
  // dock icon is clicked and there are no other windows open.
  if (win === null) {
    createWindow()
  }
})

ipcMain.on('uploadFW', (e, args) => {

  if(espToolIsRunning == true) { return; }
  espToolIsRunning = true;

  // Use esptool.py for ESP32 flashing
  // First try to find esptool.py in PATH, then bundled
  const fs = require('fs');
  const path = require('path');

  // Try bundled esptool first, then system
  var esptoolScript = __dirname + '/bin/esptool.py';
  esptoolScript = esptoolScript.replace('app.asar', '');
  var pythonExe = 'python3';
  if(process.platform == 'win32') { pythonExe = 'python'; }

  // If bundled esptool doesn't exist, try system esptool.py
  if (!fs.existsSync(esptoolScript)) {
    esptoolScript = 'esptool.py'; // rely on PATH
  }

  var firmwareFile = __dirname + '/firmwares/esp32_combined.bin';
  firmwareFile = firmwareFile.replace('app.asar', '');

  var execArgs = [];
  // If esptoolScript is a full path, use python3 to run it
  if (fs.existsSync(esptoolScript) || esptoolScript.includes(path.sep)) {
    execArgs = ['--before', 'default_reset', '--baud', '921600',
                '--port', args.port,
                'write_flash', '0x0', firmwareFile];
    var child = execFile(pythonExe, [esptoolScript, ...execArgs]);
  } else {
    // esptool.py is in PATH
    execArgs = ['--before', 'default_reset', '--baud', '921600',
                '--port', args.port,
                'write_flash', '0x0', firmwareFile];
    var child = execFile('esptool.py', execArgs);
  }

  function logEspToolStdout(data) { console.log(`esptool stdout:\n${data}`); }
  child.stdout.on('data', logEspToolStdout);

  var burnStarted = false;
  var burnPercent = 0;

  child.stderr.on('data', (data) => {
    espToolErr = espToolErr + data;
    console.log(`esptool stderr: ${data}`);

    // Try to parse percentage from esptool output (e.g. "Writing at 0x00010000... (12 %)")
    if (burnStarted == false && data.includes('Writing at')) {
      burnStarted = true;
    }
    if (burnStarted) {
      // Look for percentage patterns like "(12 %)"
      const match = data.match(/\((\d+)\s*\%\)/);
      if (match) {
        burnPercent = parseInt(match[1]);
        e.sender.send('upload percent', burnPercent);
      }
      // Also count dots for old-style progress
      const dots = (data.match(/\./g) || []).length;
      if (dots > 0) {
        burnPercent = Math.min(burnPercent + dots * 2, 100);
        e.sender.send('upload percent', burnPercent);
      }
    }
  });

  child.on('error', (err) => {
    console.log('Failed to start esptool:', err);
    espToolIsRunning = false;
    e.sender.send('upload error', err.message);
  });

  child.on('close', (code) => {
    espToolIsRunning = false;
    if (code === 0) {
      e.sender.send('upload completed', code);
    } else {
      e.sender.send('upload error', espToolErr || `esptool exited with code ${code}`);
      espToolErr = '';
    }
  });
});

// In this file you can include the rest of your app's specific main process
// code. You can also put them in separate files and require them here.
