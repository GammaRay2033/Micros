var TH02 = require('th02js'); //TH02js Library
var mraa = require('mraa'); //MRAA Library
var app = require('express')(); //Express Library
var server = require('http').Server(app); //Create HTTP instance
var io = require('socket.io')(server); //Socket.IO Library
var bus = 6;
var th02Sensor = new TH02(bus); // Create th02 sensor object
var lightSensor = new mraa.Aio(0); //Create light sensor object
var ledPin = new mraa.Gpio(13); //Set digital output in pin 13
var ledState = 0; //Default LED state
ledPin.dir(mraa.DIR_OUT); //Set the gpio direction to output

app.get('/', function(req, res) {                  
    res.sendFile(__dirname + '/files/index.html'); //serve the static html file
});

server.listen(3000); //Run on port 3000

io.on('connection', function(socket){
    var interval = setInterval(function(){
    socket.emit('ambient', {
        temperature: Math.round(th02Sensor.getCelsiusTemp()*100)/100,
        humidity: Math.round(th02Sensor.getHumidity()*100)/100,
        lighting: Math.round(lightSensor.readFloat()*100*100)/100
    });
    socket.on('ledStatus', function(){ //on incoming websocket message...
        toggleLed();
    });
    }, 100); //Read the temperature every 500ms and send the reading
    socket.on('disconnect', function(){
        clearInterval(interval);
    });    
});

function toggleLed(){                                                                               
    ledPin.write(ledState); //Write the LED state
    ledState = 1 - ledState; //Toggle LED state
}
