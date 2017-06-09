/* global require: false, setInterval: false, setTimeout: false, clearInterval: false, clearTimeout:false, module: false, console: false */

/**
 * Module dependencies.
 */

var mraa = require('mraa');
/*
If you get any mraa missing errors, run this on your board:
$  echo "src mraa-upm http://iotdk.intel.com/repos/1.1/intelgalactic" > /etc/opkg/mraa-upm.conf
$  opkg update
$  opkg install libmraa0
*/

var jsUpmI2cLcd = require('jsupm_i2clcd');
/*
If you get any upm missing errors, run this on your board:
$  opkg install upm
*/

var LcdTextHelper = require('./lcd_text_helper');
/*
Download lcd_text_helper.js from https://gist.github.com/pearlchen/31ac996f8688a01cc5b6
*/

/**
 * Initialization.
 */

// Initialize the LCD.
// The 1st param is the BUS ID:
//   Intel Edison: Use 6
//   Intel Galileo Gen 2: Use 6 (TODO: unconfirmed)
//   Intel Galileo Gen 1: Use 0
var lcd = new jsUpmI2cLcd.Jhd1313m1(6, 0x3E, 0x62);

var ip = require('ip');
var ipAddr = ip.address();

var lcdText = new LcdTextHelper(lcd);

/**
 * Try it out with some test messages.
 */

var LCD_MESSAGE_VERY_LONG = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvxyz"; // >16 characters long (48 chars)
var LCD_MESSAGE_LONG = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"; // >16 characters long (24 chars)
var LCD_MESSAGE_16CHAR = "1234567890ABCDEF"; // ==16 characters long
var LCD_MESSAGE_SHORT = "1234567890"; // <16 characters long (10 chars)
var LCD_MESSAGE_VERY_SHORT = "ABC"; // <16 characters long (3 chars)

lcdText.set([
  "Ambient Monitor ", 
  "    -CETYS-     "
]);

setTimeout(function(){
  lcdText.set([
    "Check " + ipAddr + " in web browser for real time monitoring through websocket", 
    "T=34°C H=24% L=32%"
  ]);
}, 5000);
