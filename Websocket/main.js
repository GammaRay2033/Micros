var TH02 = require('th02js'); //TH02js Library
var mraa = require('mraa'); //MRAA Library
var express = require('express'); //Express Library
var app = express();
var server = require('http').Server(app); //Create HTTP instance
var io = require('socket.io')(server); //Socket.IO Library

app.use(express.static(__dirname+'/files'));

app.get('/', function(req, res) {                  
    res.send('Hello World!'); //Serve the html file
});

server.listen(3001); //Run on port 3001

var bus = 6;
var th02Sensor = new TH02(bus); // Create th02 sensor object

var lightSensor = new mraa.Aio(0); //Create light sensor object

io.on('connection', function(socket){
    var interval = setInterval(function(){
    socket.emit('ambient', {
        temperature: Math.round(th02Sensor.getCelsiusTemp()*Math.pow(10,2))/Math.pow(10,2),
        humidity: Math.round(th02Sensor.getHumidity()*Math.pow(10,2))/Math.pow(10,2),
        lighting: Math.round(lightSensor.readFloat()*100*Math.pow(10,2))/Math.pow(10,2)
    });
    }, 500); //Read the temperature every 500ms and send the reading
    socket.on('disconnect', function(){
        clearInterval(interval);
    });    
});
