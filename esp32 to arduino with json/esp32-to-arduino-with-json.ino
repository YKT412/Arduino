#include <SoftwareSerial.h>
#include <ArduinoJson.h>

/*ESP32 to Arduino*/
#define RX 16
#define TX 17

#define mega Serial2

/*work state*/
#define paused 1
#define busy 2
#define enter 3
#define complete 4

/*job state*/
#define hold 5
#define start 6
#define check 7
#define stop 8

//Timer to run Arduino code every 5 seconds
unsigned long previousMillis = 0;
unsigned long currentMillis;
const unsigned long period = 5000;
float hum = 0, temp = 0, target_time = 0;
uint8_t work_state = 0;
String mega_data;

void setup() {
  // Initialize Serial port
  Serial.begin(115200);
  StaticJsonDocument<200> doc;  
  mega.begin(115200);
  mega.println("hi  Mega!");
  // while (!mega.available()) continue;
}

void loop() {
delay(5000);  
 // // Get current time
  if(mega.available()) {
    StaticJsonDocument<200> doc;
    mega_data = mega.readString();
    Serial.println(mega_data);
    DeserializationError error = deserializeJson(doc, mega_data);
    hum = doc["hum"].as<float>();
    temp = doc["temp"].as<float>();
    work_state = doc["WS"].as<int>();
  Serial.println("JSON Object Recieved");
  Serial.print("Recieved Humidity:  ");
  Serial.println(hum);
  Serial.print("Recieved Temperature:  ");
  Serial.println(temp);
  Serial.println("-----------------------------------------");  // Test parsing
  }

  Serial.println("JSON Object Recieved");
  Serial.print("Recieved Humidity:  ");
  Serial.println(hum);
  Serial.print("Recieved Temperature:  ");
  Serial.println(temp);
  Serial.println("-----------------------------------------");  // Test parsing
 data_send();
}

void data_send(){
  StaticJsonDocument<200> doc;
  doc["hum"] = 50;
  doc["temp"] = 45;
  doc["min"] = target_min;
  doc["hr"] = target_hr; 
  doc["update"] = 1;
  doc["job"] = start;
  serializeJson(doc, Serial2);
  serializeJson(doc, Serial);
}