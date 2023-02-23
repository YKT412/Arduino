char webpage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<script>
var connection = new WebSocket('ws://'+location.hostname+':81/');
var button_1_status = 2;
var button_2_status = 4;
function button_1_on()
{
   button_1_status = 7; 
  console.log("HeatP 1 is ON");
  send_data();
}
function button_1_off()
{
  button_1_status = 2;
console.log("HeatP 1 is OFF");
send_data();
}
function button_2_on()
{
   button_2_status = 9; 
  console.log("HeatP 2 is ON");
  send_data();
}
function button_2_off()
{
  button_2_status = 4;
console.log("HeatP 2 is OFF");
send_data();
}
function send_data()
{
  var full_data = '{"Heater Control" :'+button_1_status+',"Fan Control":'+button_2_status+'}';
  connection.send(full_data);
}
</script>
<body>
<center>
<h1>Heat Pump Button Demo Test</h1>
<h3> HeatP 1 </h3>
<button onclick= "button_1_on()" >On</button><button onclick="button_1_off()" >Off</button>
<h3> HeatP 2 </h3>
<button onclick="button_2_on()">On</button><button onclick="button_2_off()">Off</button>
</center>
</body>
</html>
)=====";