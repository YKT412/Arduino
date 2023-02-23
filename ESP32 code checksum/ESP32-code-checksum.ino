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

#define h1 0x59

/*work state*/
#define paused 1
#define busy 2
#define enter 3
#define complete 4

/*job state*/
#define hold 5
#define start 6
#define look 7
#define stop 8

//Receive variabes
uint8_t j = 0, rawh = 0, rawt = 0, ws = 0, data_receive[6], check = 0, receive_checksum = 0;
//Send variables
uint8_t i = 0, hr = 0, mint = 0, hum = 0, temp = 0, job = 0, data_send[9], send_checksuml = 0, send_checksumh = 0;
bool sf = 0, update = 0;
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

  StaticJsonDocument<200> doc;
  doc["h"] = rawh;
  doc["t"] = rawt;
  doc["wss"] = ws;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer);  // print to client
  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void messageHandler(char* topic, byte* payload, unsigned int length) {
  Serial.print("incoming: ");
  Serial.println(topic);
  StaticJsonDocument<200> doc;
  deserializeJson(doc, payload);
  temp = doc["temp"].as<int>();
  hum = doc["hum"].as<int>();
  hr = doc["hr"].as<int>();
  mint = doc["min"].as<int>();
  update = doc["update"].as<int>();
  job = doc["job"].as<int>();
  send_arduino();
  // Serial.print("temp:");
  // Serial.print(temp);
  // Serial.print("hum:");
  // Serial.print(hum);
  // Serial.print("hr:");
  // Serial.print(hr);
  // Serial.print("min:");
  // Serial.print(mint);
  // Serial.print("update:");
  // Serial.print(update);
  // Serial.print("job:");
  // Serial.println(job);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(38400);
  connectAWS();
}
void loop() {
  if (Serial2.available()) {
    receive_arduino();
  } 
  
  publishMessage();
  client.loop();
  delay(100);
}

void send_arduino() {
  send_checksuml = hum + temp + hr + mint;
  send_checksumh = update + job;
  data_send[0] = h1;
  data_send[1] = h1;
  data_send[2] = hum;
  data_send[3] = temp;
  data_send[4] = hr;
  data_send[5] = mint;
  data_send[6] = update;
  data_send[7] = job;
  data_send[8] = send_checksuml + send_checksumh;

  while (i < 9) {
    Serial2.write(data_send[i]);
    Serial.println(data_send[i]);
    i++;
  }
  i = 0;
}

void receive_arduino() {

  if (j < 2) {
    uint8_t x = Serial2.read();
    if (x == h1) {  //0, 1
      j++;
    } else {
      j = 0;
    }
  } else {  //2, 3, 4
    if (j < 5) {
      data_receive[j - 2] = Serial2.read();
      receive_checksum += data_receive[j - 2];
      j++;
    } else {
      check = Serial2.read();
      if (check == receive_checksum) {
        rawh = data_receive[0];
        rawt = data_receive[1];
        ws = data_receive[2];
        Serial.print(rawh);
        Serial.print("   ");
        Serial.print(rawt);
        Serial.print("   ");
        Serial.println(ws);
        j = 0;
        receive_checksum = 0;
      }
    }
  }
}