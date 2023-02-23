#include <esp_now.h>
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif
#define sendf true

#define CHANNEL 1
#define PRINTSCANRESULTS 0
esp_now_peer_info_t machine_peer = {};

struct Message {
  uint16_t switches;
  uint8_t Lx, Ly, Rx, Ry;
} msg;

struct IO {
  unsigned short ALx, ALy, ARx, ARy;
  short pins[10] = { 15, 2, 4, 16, 17, 5, 19, 21, 22, 23 };
  int N = 10;

  IO() {
    // config digital pin
    for (int i = 0; i < N; ++i) {
      pinMode(pins[i], INPUT_PULLUP);
    }
    // config analog pin
    ALx = 34;
    ALy = 35;
    ARx = 36;
    ARy = 39;
  }

  void read_digital() {
    unsigned int digital = 0;
    for (int j = 0; j < N; ++j) {
      digital |= digitalRead(pins[j]) << j;
    }
    msg.switches = digital;
  }

  void read_analog() {
    msg.Lx = analogRead(ALx) / 16;
    msg.Ly = analogRead(ALy) / 16;
    msg.Rx = analogRead(ARx) / 16;
    msg.Ry = analogRead(ARy) / 16;
  }

  // NOTE: THIS MUST BE FIRST LINE IN void loop() FOR CONTINOUS NEW INPUT VALUE
  void read_input() {
    read_digital();
    read_analog();
  }
} io;

void configDeviceAP() {
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
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  Serial.print("Last Packet Recv Data: ");
  Serial.println(*data);
  Serial.println("");
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: ");
  Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// UNCOMMENT FOR: send ondatachange only
Message prevMsg = msg;

void Send() {
  const uint8_t *peer_addr = machine_peer.peer_addr;
  Serial.print("Sending: ");
  //  Serial.println(data/);

  esp_err_t result;
  // UNCOMMENT FOR: send ondatachange only
  //  if (msg.switches != prevMsg.switches) {
  //    result = esp_now_send(peer_addr, (uint8_t *)&msg, sizeof(Message));
  //    prevMsg.switches = msg.switches;
  //  }
  result = result = esp_now_send(peer_addr, (uint8_t *)&msg, sizeof(Message));

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

void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void setup() {
  Serial.begin(115200);

  delay(1000);
  // For controller, machine is peer
  machine_peer.peer_addr[0] = 0x7C;
  machine_peer.peer_addr[1] = 0x9E;
  machine_peer.peer_addr[2] = 0xBD;
  machine_peer.peer_addr[3] = 0x60;
  machine_peer.peer_addr[4] = 0x79;
  machine_peer.peer_addr[5] = 0xA8;

  machine_peer.channel = CHANNEL;
  machine_peer.encrypt = 0;
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_STA);
  //  configDeviceAP();
  //Serial.println("ESPNow/Multi-Slave/Master Example");
  // This is the mac address of the Master in AP Mode
  Serial.println();
  Serial.print("AP MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println();
  InitESPNow();

  esp_err_t addStatus = esp_now_add_peer(&machine_peer);
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

  io = IO();
}

void loop() {
  io.read_input();  // reading pins for latest input
//  Serial.println(String(msg.Lx) + " " + msg.Ly + " " + msg.Rx + " " + msg.Ry);
  delay(10);
  if (sendf) Send();
}
