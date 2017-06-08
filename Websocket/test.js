var TH02 = require('th02js'); //TH02js Library

var bus = 6;
var sensor = new TH02(bus); // Create the temperature sensor object

var waiting = setInterval(function() {
        var celsius = sensor.getCelsiusTemp();
        console.log(celsius + " degrees Celsius");
        i++;
        if (i == 10) clearInterval(waiting);
        }, 1000);
