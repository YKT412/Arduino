#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#define RXD2 16
#define TXD2 17
const char* ssid = "Tlabs";
const char* password = "245234sv";
String D_name = "Ru8amDq2As";
int ph, tds, flow, D20, D24;
//Your Domain name with URL path or IP address with path
String serverName = "http://iot.thinkfinitylabs.com/rodata?key=";
//http://iot.thinkfinitylabs.com/rodata?key=Ru8amDq2As&ph=7.2&tds=140&fl=4&d20=24&d24=0
// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 5000;

void setup() {
  Serial.begin(115200);
  pinMode(14, OUTPUT);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: " + String(TX));
  Serial.println("Serial Rxd is on pin: " + String(RX));
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(14,LOW);
  Serial.println("Timer set to 5 seconds (timerDelay variable), it will take 5 seconds before publishing the first reading.");
}

void loop() {
  while (Serial2.available()) {
    StaticJsonDocument <500> data;
    String temp = Serial2.readString();
    Serial.println(temp);
    DeserializationError error = deserializeJson(data, temp);
    ph = data["ph"].as<int>();
    tds = data["tds"].as<int>();
    flow = data["flow"].as<int>();
    D20 = data["D20"].as<int>();
  }
  //Send an HTTP POST request every 10 minutes
  if ((millis() - lastTime) > timerDelay) {
    //Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;

      String serverPath = serverName + D_name + "&ph=" + ph + "&tds=" + tds + "&fl=" + flow + "&d20=" + D20 + "&d24=0";
      //      http://iot.thinkfinitylabs.com/rodata?key=Ru8amDq2As&ph=7.2&tds=140&fl=4&d20=24&d24=0

      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());

      // Send HTTP GET request
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        //        Serial.println(payload);
        StaticJsonDocument <500> command;  
        DeserializationError error = deserializeJson(command, payload);
        bool val = command["state"].as<bool>();
        Serial.print("command == "+(String)val+";");
        digitalWrite(14,val);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}
