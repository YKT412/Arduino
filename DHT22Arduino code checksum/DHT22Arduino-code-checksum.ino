#include <avr/io.h>
#include "DHT1.h"
#include "DHT2.h"
#include "DS3231.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define h1 0x59

/*parameters*/
#define standard 37
#define min_temp 40
#define max_temp 60
#define min_hum 30
#define max_hum 90

/*work state*/
#define paused 1
#define busy 2
#define enter 3
#define complete 4

/* job state*/
#define hold 5
#define start 6
#define look 7
#define stop 8

/*sensor pin*/
#define DHTPIN 2
#define DHTTYPE DHT22  // DHT 22  (AM2302)

/* pins for components*/
#define fan PB0     //8
#define heater PB1  //9

DHT dht(DHTPIN, DHTTYPE);
//EEPROM
unsigned char store[6];
//Send variables
float hum = 0, temp = 0, f = 0;
uint8_t work = 0, send_checksum = 0, send_data[6], i = 0;
//Receive variables
uint8_t job_state = 0, receive_checksum = 0, j = 0, check = 0, receive_data[9], desired_temp = 0, desired_hum = 0, hr = 0, mint = 0, job = 0;
bool query = 0;

void setup() {
  i2c_init();
  DDRB = (1 << fan) | (1 << heater);
  PORTB &= ~(1 << fan) | (1 << heater);
  Serial.begin(9600);    // to print data
  Serial1.begin(38400);  // communication with esp
  dht.begin();
  Serial.println("AM2302!");
}

void loop() {
  dht22_func();
  if (Serial1.available()) {
    data_receive();
  } else {
    desired_hum = EEPROM.read(0);
    desired_temp = EEPROM.read(1);
    hr = EEPROM.read(2);
    mint = EEPROM.read(3);
    query = EEPROM.read(4);
    job = EEPROM.read(5);
  }
  store[0] = desired_hum;
  store[1] = desired_temp;
  store[2] = hr;
  store[3] = mint;
  store[4] = query;
  store[5] = job;
    for (int i = 0; i < 6; i++) {  // to store new value in EEPROM
    // this performs as EEPROM.write(i, i)
    EEPROM.update(i, store[i]);  //address, value
  }
  if (query) {  // command to send data
    data_send();
  }
  RTC();
  if((hr == hours) && (mint == minutes)){
    regulation(standard, standard, stop);
  }
  else{
    regulation(desired_hum, desired_temp, job);  
  }
  // delay(500);
}

void data_send() {  //to send actual temp readings
  dht22_func();
  send_checksum = hum + temp + work;
  send_data[0] = h1;
  send_data[1] = h1;
  send_data[2] = hum;
  send_data[3] = temp;
  send_data[4] = work;
  send_data[5] = send_checksum;
  while (i < 6) {
    Serial1.write(send_data[i]);
    i++;
    delay(1);
  }
  i = 0;
   Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.print("   ");
  Serial.print("Temperature: ");
  Serial.println(temp);
}

void data_receive() {
  if (j < 2) {
    int8_t x = Serial1.read();
    if (x == h1) {  //0, 1
      j++;
    } else {
      j = 0;
    }
  } else {  //2, 3, 4, 5, 6, 7,
    if (j < 8) {
      receive_data[j - 2] = Serial1.read();
      receive_checksum += receive_data[j - 2];
      j++;
    } else {  //8
      check = Serial1.read();
      if (check == receive_checksum) {
        desired_hum = receive_data[0];
        desired_temp = receive_data[1];
        hr = receive_data[2];
        mint = receive_data[3];
        query = receive_data[4];
        job = receive_data[5];
        Serial.print(desired_hum);
        Serial.print("   ");
        Serial.print(desired_temp);
        Serial.print("   ");
        Serial.print(hr);
        Serial.print("   ");
        Serial.print(mint);
        Serial.print("  ");
        Serial.print(query);
        Serial.print("   ");
        Serial.println(job);
        j = 0;
        receive_checksum = 0;
      }
    }
  }
}

void dht22_func() {

  hum = dht.readHumidity();
  temp = dht.readTemperature();
  f = dht.readTemperature(true);

  if (isnan(hum) || isnan(temp) || isnan(f)) {
    // Serial.println("Failed to read from DHT sensor!");
    return;
  }

 
}

void regulation(uint8_t h, uint8_t t, uint8_t js) {

  if (js == stop) {  //stop
    PORTB &= ~((1 << heater) | (1 << fan));
     while(temp > standard) {
      PORTB &= (1 << fan);
    } 
      work = complete;
  } 
  else if (js == look) {
    if (temp > standard) {
      PORTB &= (1 << fan);
    } else {
      work = check;
    }
  } 
  else if (js == hold) {
    PORTB &= ~((1 << heater) | (1 << fan));
    work = paused;
  } 
  else{

    if ((t < max_temp) && (t > min_temp)) {
      if (temp < t) {
        PORTB &= (1 << heater);
      } else if (temp > t) {
        PORTB &= (1 << fan);
      } else {
        PORTB &= ~((1 << fan) | (1 << heater));    
      }
    }

    if (hum < h) {
      PORTB &= ~(1 << fan);
    }
    else if (hum > h) {
      PORTB |= (1 << fan);
    }
    else {
      PORTB &= ~((1 << fan) | (1 << heater));
    }
  }
}