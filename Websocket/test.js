var TH02 = require('th02js'); //TH02js Library

var bus = 6;
var sensor = new TH02(bus); // Create the temperature sensor object

var waiting = setInterval(function() {
        var celsius = sensor.getCelsiusTemp();
        var fahrenheit = celsius * 9.0/5.0 + 32.0;
        console.log(celsius + " degrees Celsius, or " +
            Math.round(fahrenheit) + " degrees Fahrenheit");
        i++;
        if (i == 10) clearInterval(waiting);
        }, 1000);
