#include <esp_now.h>
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif
#define sendf false
#define CHANNEL 1
#define PRINTSCANRESULTS 0
esp_now_peer_info_t peers = {};

struct Message {
  uint16_t switches;
  uint8_t Lx, Ly, Rx, Ry;
} msg;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  delay(1000);
  //put target MAC ADDRESS HERE
  peers.peer_addr[0] = 0x30;
  peers.peer_addr[1] = 0xC6;
  peers.peer_addr[2] = 0xF7;
  peers.peer_addr[3] = 0x20;
  peers.peer_addr[4] = 0xB2;
  peers.peer_addr[5] = 0x0C;

  peers.channel = CHANNEL;
  peers.encrypt = 0;
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_STA);
  //  configDeviceAP();
  //Serial.println("ESPNow/Multi-Slave/Master Example");
  // This is the mac address of the Master in AP Mode
  Serial.println();
  Serial.print("AP MAC: "); Serial.println(WiFi.macAddress());
  Serial.println();
  InitESPNow();

  esp_err_t addStatus = esp_now_add_peer(&peers);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Peer added");
  } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Add Peer - Invalid Argument");
  } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
    Serial.println("Peer list full");
  } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("Out of memory");
  } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
    Serial.println("Peer Exists");
  } else {
    Serial.println("Not sure what happened");
  }


  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}
uint8_t i = 0, values_send[6] = {89,89,0,0,0,0}, checksum = 0;
#define h1 0x59
void loop() {
  // delay(1000);
  // if (sendf)Send();
  checksum = msg.Rx + msg.Ry + msg.switches;
   values_send[0] = h1;  
   values_send[1] = h1; 
   values_send[2] = msg.switches;
   values_send[3] = msg.Rx;
   values_send[4] = msg.Ry;  
   values_send[5] = checksum;
   while(i<6){
  Serial2.write(values_send[i]);
  i++;
  } 
  delay(1);
 i = 0; 
}


void configDeviceAP() {
  //  String Prefix = "Slave:";
  //  String Mac = WiFi.macAddress();
  String SSID = "AP2";
  String Password = "12345677";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.println("len: " + String(data_len));
  memcpy(&msg, data, data_len);
  Serial.println(msg.switches, BIN);
  Serial.println(String(msg.Lx) + ", " + msg.Ly + ", " + msg.Rx + ", " + msg.Ry);
  Serial.println();

  //  char macStr[18];
  //  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  //  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  //  Serial.println("");
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

uint8_t data = 0;
void Send() {
  data++;
  const uint8_t *peer_addr = peers.peer_addr;
  Serial.print("Sending: "); Serial.println(data);
  esp_err_t result = esp_now_send(peer_addr, &data, sizeof(data));
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }

}
//uint8_t data[10]={0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
//  const uint8_t *peer_addr = peers.peer_addr;//,*dataa=data;
//
//uint8_t data[5] = {50,12,45,12,45};
//  esp_err_t result = esp_now_send(peer_addr, &data[0], sizeof(data));
//    Serial.print("Send Status: ");
//    if (result == ESP_OK) {
//      Serial.println("Success");
//    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
//      // How did we get so far!!
//      Serial.println("ESPNOW not Init.");
//    } else if (result == ESP_ERR_ESPNOW_ARG) {
//      Serial.println("Invalid Argument");
//    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
//      Serial.println("Internal Error");
//    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
//      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
//    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
//      Serial.println("Peer not found.");
//    } else {
//      Serial.println("Not sure what happened");
//    }
//}

void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}
