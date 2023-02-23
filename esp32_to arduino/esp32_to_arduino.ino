#define RX2 16
#define TX2 17
#define TX0 1
#define RX0 3

uint8_t data[3], humidity = 0, temp = 0, far = 0, checksum = 14, check = 0, j= 0;
void setup(){

 Serial.begin(38400);
 Serial2.begin(34800, SERIAL_8N1 , RX2 , TX2 );

}

void loop(){

delay(100);  

 if (j < 2) {   //1, 0
// Serial.print("yes");  
    if (Serial2.read() == 7) {
      j++;
     
    }
    else {
      j = 0;
    }
  }
  else {
    if (j < 5) {   //2, 3, 4
      data[j - 2] = Serial2.read();
      checksum += data[j - 2];
      j++;
// Serial.print("humidity:");      
    }
    else {
      check = Serial2.read();
      if (checksum == check) {
        Serial.print("yes");
        humidity = data[0];
        temp = data[1];
        far = data[2];
        checksum = 14;     
      }
      j = 0;
    }
  }

// Serial.print(Serial2.read());
Serial.print(humidity);
Serial.print(" ");
Serial.print("temp:");
Serial.print(temp);
Serial.print(" ");
Serial.print("far:");
Serial.println(far);
Serial.println(" ");     

  //  Serial.write(5);
  
}