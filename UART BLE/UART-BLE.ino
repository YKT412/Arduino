#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#define RXp2 16
#define TXp2 17

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_CLASSIC_BT"); //Bluetooth device name
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (Serial.available()) {
    // SerialBT.println("  Data: ");
    SerialBT.println(Serial2.readString()); //printing in ble screen
  }
  
  if (SerialBT.available()) {
    // Serial.write("The Given input is : ");
    Serial.write(SerialBT.read()); //printing in computer screen
    // Serial.println(Serial2.readString()); //I used this for printing value in terminal

  }
  delay(1000);
}