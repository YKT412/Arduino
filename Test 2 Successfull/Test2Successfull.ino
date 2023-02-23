#include "secrets.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "WiFi.h"
#include <SoftwareSerial.h>

#define AWS_IOT_PUBLISH_TOPIC "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

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
float hum = 0, temp = 0, target_time = 0, h = 0, t = 0, ws = 0, desh = 0, dest = 0, tt = 0, updt = 0, cmd = 0, rawh = 0, rawt = 0;
uint8_t work_state = 0;
String mega_data;

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

void connectAWS() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);

  // Create a message handler
  client.setCallback(messageHandler);

  Serial.print("Connecting to AWS IOT");

  while (!client.connect(THINGNAME)) {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected()) {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);

  Serial.println("AWS IoT Connected!");
}


void publishMessage() {
  data_receive();  //mega data received
  StaticJsonDocument<200> doc;
  // doc["h"] =rawh;
  // doc["t"] = rawt;
  // doc["wss"] = ws;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);  // print to client
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: ");
  Serial.println(topic);

  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  const char* message = doc["message"];
  // desh = doc["h"];
  // dest = doc["t"];
  // tt = doc["ttaws"];
  // updt = doc["updtaws"];
  // cmd = doc["cmdaws"];
  // data_send();
  Serial.println(message);
}

void setup() {
  Serial.begin(115200); 
  connectAWS();
}

void loop() {
  // delay(5000);
  // initialize();
  // data_receive();
   publishMessage();
  client.loop();
  
  //  data_receive(); // Test parsing
  
  //  data_send();
}

void data_send() {
  StaticJsonDocument<200> doc;
  doc["hum"] = desh;
  doc["temp"] = dest;
  doc["min"] = tt;
  doc["update"] = updt;
  doc["job"] = cmd;
  serializeJson(doc, mega);
}

void initialize() {

Serial.begin(115200);
  StaticJsonDocument<200> doc;  
  mega.begin(115200,SERIAL_8N1 , RX , TX );
  Serial.println("HI esp32");
  mega.println("hi  Mega!");
  while (!mega.available()) continue;
  
}

void data_receive() {

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
}