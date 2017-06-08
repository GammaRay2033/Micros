var TH02 = require('th02js'); //TH02js Library
var app = require('express')(); //Express Library
var server = require('http').Server(app); //Create HTTP instance
var io = require('socket.io')(server); //Socket.IO Library

server.listen(3000); //Run on port 3000

var bus = 6;
var sensor = new TH02(bus); // Create the temperature sensor object

app.get('/', function(req, res) {                  
    res.sendFile(__dirname + '/index.html'); //Serve the html file
});

io.on('connection', function(socket){
    var interval = setInterval(function(){
    socket.emit('temperature', {celsius: Math.round(sensor.getCelsiusTemp()*Math.pow(10,2))/Math.pow(10,2)});
    console.log(sensor.getCelsiusTemp());
    }, 500); //Read the temperature every 500ms and send the reading
    socket.on('disconnect', function(){
        clearInterval(interval);
    });    
});
