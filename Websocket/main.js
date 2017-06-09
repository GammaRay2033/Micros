var TH02 = require('th02js'); //TH02js Library
var mraa = require('mraa'); //MRAA Library
var app = require('express')(); //Express Library
var server = require('http').Server(app); //Create HTTP instance
var io = require('socket.io')(server); //Socket.IO Library
var jsUpmI2cLcd = require('jsupm_i2clcd'); //LCD Library
var LcdTextHelper = require('./lcd_text_helper'); //Text sliding library
var ip = require('ip'); //IP Address Library

var th02Sensor = new TH02(6); // Create th02 sensor object
var lightSensor = new mraa.Aio(0); //Create analog input objet in pin A0
var ledPin = new mraa.Gpio(13); //Create GPIO object in pin 13
ledPin.dir(mraa.DIR_OUT); //Set GPIO direction to output
var ledState = 0; //Default LED state
var ipAddr = ip.address(); //Get server IP address
var lcd = new jsUpmI2cLcd.Jhd1313m1(6, 0x3E, 0x62); //Create LCD object
var lcdText = new LcdTextHelper(lcd); //Create LCD text handling object

lcdText.set([ //Set initial text in LCD
  "Ambient Monitor ", 
  "    -CETYS-     "
]);

app.get('/', function(req, res) {                  
    res.sendFile(__dirname + '/files/index.html'); //serve the static html file
});

server.listen(3000); //Run on port 3000

setTimeout(function(){
  lcdText.set([
    "Check http://" + ipAddr + ":3000 in web browser for real time monitoring"
  ]);
  setInterval(function(){
    var tValue = Math.round(th02Sensor.getCelsiusTemp()*100)/100;
    var hValue = Math.round(th02Sensor.getHumidity()*100)/100;
    var lValue = Math.round(lightSensor.readFloat()*100*100)/100;
    lcd.setCursor(1,2);
    lcd.write(" T=" + parseInt(tValue) + " H=" + parseInt(hValue) + " L=" + parseInt(lValue) + " ");
  }, 100);
}, 5000);

io.on('connection', function(socket){
    var interval = setInterval(function(){
    tValue = Math.round(th02Sensor.getCelsiusTemp()*100)/100;
    hValue = Math.round(th02Sensor.getHumidity()*100)/100;
    lValue = Math.round(lightSensor.readFloat()*100*100)/100;
    socket.emit('ambient', {
        temperature: tValue,
        humidity: hValue,
        lighting: lValue
    });
    }, 100); //Read the temperature every 500ms and send the reading
    socket.on('disconnect', function(){
        clearInterval(interval);
    });
    socket.on('ledStatus', function(){ //on incoming websocket message...
        toggleLed();
    });
});

function toggleLed(){                                                                               
    ledPin.write(ledState); //Write the LED state
    ledState = 1 - ledState; //Toggle LED state
}
