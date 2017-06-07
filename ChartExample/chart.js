var helpers = Chart.helpers;
var canvas = document.getElementById('bar');


var barChartData = {
    labels: ["IBM", "Microsoft"],
    datasets: [{
      label: "Product A",
      fillColor: "rgba(220,220,220,0.5)",
      strokeColor: "rgba(220,220,220,0.8)",

      data: [25, 75]
    }, {
      label: "Product B",
      fillColor: "rgba(151,187,205,0.5)",
      strokeColor: "rgba(151,187,205,0.8)",

      data: [75, 25]
    }]

  }
  // 
var bar = new Chart(canvas.getContext('2d')).Bar(barChartData, {
  tooltipTemplate: "<%if (label){%><%=label%>: <%}%><%= value %>kb",
  animation: false,
})

;
