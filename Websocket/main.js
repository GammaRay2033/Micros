// Load Grove module
var TH02 = require('th02js');

// Create the temperature sensor object using AIO pin 0
var bus = 6;
var sensor = new TH02(bus);

// Read the temperature ten times, printing both the Celsius and
// equivalent Fahrenheit temperature, waiting one second between readings
var i = 0;
var waiting = setInterval(function() {
        var celsius = sensor.getCelsiusTemp();
        console.log(celsius + " degrees Celsius");
        i++;
        if (i == 10) clearInterval(waiting);
        }, 1000);
