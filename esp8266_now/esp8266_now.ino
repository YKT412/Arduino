
//#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <espnow.h>
//#elif defined(ARDUINO_ARCH_ESP32)
//#include <WiFi.h>
//#endif


//#define sendf false


#define CHANNEL 3
#define PRINTSCANRESULTS 0
//esp_now_peer_info_t peers={};
void setup() {
  Serial.begin(115200);

  delay(1000);
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  
  configDeviceAP();
  //Serial.println("ESPNow/Multi-Slave/Master Example");
  // This is the mac address of the Master in AP Mode
  Serial.println();
  Serial.print("AP MAC: "); Serial.println(WiFi.macAddress());
  Serial.println();
    InitESPNow();
esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    
  
        
  esp_now_register_recv_cb(OnDataRecv);
//  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  delay(1000);

}


void configDeviceAP() {
//  String Prefix = "Slave:";
//  String Mac = WiFi.macAddress();
  String SSID ="AP2";
  String Password = "12345677";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL,0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}


void OnDataRecv(unsigned char *mac_addr,  unsigned char *data,unsigned char data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr);
  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
  Serial.println("");
}
  
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == 0) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    //ESP.restart();
  }
}
