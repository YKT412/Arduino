#include<ESP8266WiFi.h>
#include<ESP8266WebServer.h>
#include<ArduinoJson.h>
#define username "base"
#define password "12345678"
#define h1 0x59
StaticJsonDocument<200> doc;

String json;
// NOTE: trigger pin output & echo pin input
ESP8266WebServer server(80);
//int r, b, g;
float kp = 0,kd = 0,a = 0,b = 0;
uint8_t values_send[8] = {89,89,0,0,0,0,0,0}, i =0, checksum = 0,button = 0, kpi = 0, kpf = 0, kdi =0, kdf = 0;


void BUTTON () {
  server.send(200, "application/json", json=server.arg("data"));
  DeserializationError error = deserializeJson(doc,json);
if(error)
{
Serial.print(F("Deserialzation is failed"));
Serial.println(error.f_str());
return;
}
  kp = doc["kp"].as<float>();
  kd = doc["kd"].as<float>();
  a = doc["a"].as<float>();
  b = doc["b"].as<float>();
  button = doc["button"].as<int>();
  kpi = (int)kp;
  kpf = (int)((kp - (int)(kp))*100);
   kdi = (int)kd;
  kdf = (int)((kd - (int)(kd))*100);
 checksum = kpi + kpf+ kdi + kdf + button;
   values_send[0] = h1;  
   values_send[1] = h1; 
   values_send[2] = button;
   values_send[3] = kpi;
   values_send[4] = kpf;
   values_send[5] = kdi;
   values_send[6] = kdf;  
   values_send[7] = checksum;
     
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(username, password);
  Serial.println(WiFi.softAPIP());
  server.on("/", BUTTON);
  server.begin();
}


void loop() {
  server.handleClient();
  // checksum = kp + kd + button;
  //  values_send[0] = h1;  
  //  values_send[1] = h1; 
  //  values_send[2] = kp;
  //  values_send[3] = kd;
  //  values_send[4] = button;  
  //  values_send[5] = checksum;
          // Serial.print("kp:");
          // Serial.print(kpf);
          // Serial.print("   ");
          // Serial.print("kd:");
          // Serial.println(kdf);
  while(i<8){
  Serial.write(values_send[i]);
  i++;
  } 
  delay(1);
 i = 0;  
}