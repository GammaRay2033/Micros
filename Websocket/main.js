var TH02 = require('th02js'); //TH02js Library
var mraa = require('mraa'); //MRAA Library
var app = require('express')(); //Express Library
var server = require('http').Server(app); //Create HTTP instance
var io = require('socket.io')(server); //Socket.IO Library

server.listen(3000); //Run on port 3000

var bus = 6;
var th02Sensor = new TH02(bus); // Create th02 sensor object

var lightSensor = new mraa.Aio(0); //Create light sensor object

app.get('/', function(req, res) {                  
    res.sendFile(__dirname + '/index.html'); //Serve the html file
});

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

  var ctx = document.getElementById("myChart").getContext("2d");

  var data = {
    labels: ["January", "February", "March", "April", "May", "June", "July"],
    datasets: [{
      label: "My First dataset",
      fillColor: "rgba(220,220,220,0.2)",
      strokeColor: "rgba(220,220,220,1)",
      pointColor: "rgba(220,220,220,1)",
      pointStrokeColor: "#fff",
      pointHighlightFill: "#fff",
      pointHighlightStroke: "rgba(220,220,220,1)",
      data: [65, 59, 80, 81, 56, 55, 40]
    }, {
      label: "My Second dataset",
      fillColor: "rgba(151,187,205,0.2)",
      strokeColor: "rgba(151,187,205,1)",
      pointColor: "rgba(151,187,205,1)",
      pointStrokeColor: "#fff",
      pointHighlightFill: "#fff",
      pointHighlightStroke: "rgba(151,187,205,1)",
      data: [28, 48, 40, 19, 86, 27, 90]
    }]
  };
  var options = {
    animation: false,
    //Boolean - If we want to override with a hard coded scale
    scaleOverride: true,
    //** Required if scaleOverride is true **
    //Number - The number of steps in a hard coded scale
    scaleSteps: 10,
    //Number - The value jump in the hard coded scale
    scaleStepWidth: 10,
    //Number - The scale starting value
    scaleStartValue: 0
  };

  var myLineChart = new Chart(ctx).Line(data, options);

  setInterval(function() {
    setData(data.datasets[0].data);
    setData(data.datasets[1].data);
    setLabels(data.labels);

    var myLineChart = new Chart(ctx).Line(data, options);
  }, 2000);

  function setLabels(labels) {
    var nextMonthIndex = months.indexOf(labels[labels.length - 1]) + 1;
    var nextMonthName = months[nextMonthIndex] != undefined ? months[nextMonthIndex] : "January";
    labels.push(nextMonthName);
    labels.shift();
  }

  function setData(data) {
    data.push(Math.floor(Math.random() * 100) + 1);
    data.shift();
  }
  
  function convertMonthNameToNumber(monthName) {
    var myDate = new Date(monthName + " 1, 2016");
    var monthDigit = myDate.getMonth();
    return isNaN(monthDigit) ? 0 : (monthDigit + 1);
  }
  
  var months = ["January", "February", "March", "April", "May", "June",
    "July", "August", "September", "October", "November", "December"
  ];
