#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
StaticJsonDocument<200> json;

#define username "R1nodemcu"
#define password "abc123456"
#define TX 13 // D7
#define RX 12 // D6
SoftwareSerial UART(RX, TX); // RX, TX

ESP8266WebServer server(80);

void r1() {

  String xaxis = server.arg("XAXIS");
  String yaxis = server.arg("YAXIS");
  String RPM = server.arg("RPM");
  String Throw = server.arg("Throw");
   Serial.println(xaxis.c_str()); 
   Serial.println(yaxis.c_str()); 
   Serial.println(RPM.c_str());
  //Serial.println(Throw.c_str());
  json["X_digit"] = xaxis.toInt();
  json["Y_digit"] = yaxis.toInt();
  json["RPM"] = RPM.toInt();
  json["Throw"] = Throw.toInt();
  serializeJson(json, UART);
  server.send(200, "text/plain", "");
  delay(10);
}

void setup()
{
  Serial.begin(74880);
  UART.begin(74880);
  WiFi.softAP(username, password);
  server.begin();
  server.on("/R1/", r1);

}

void loop() {
  
   // String Throw = server.arg("Throw");

//  Serial.println(Throw);
  server.handleClient();
}
