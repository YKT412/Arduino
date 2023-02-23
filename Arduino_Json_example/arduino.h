

/*

   #define flow_sensor 19 // pulser in

  #define ib_select 40

  #define auto_manual 35

  #define motor_1 39

  #define tank_1 26// sig input

  #define tank_2 25 // sig input

  #define PH_s A1  // input

  #define I_M_I 12 // input

  #define M_I_I 8 // output

  #define tds_s A3 // input

  #define preassure_1 24 //input

  #define motor_2 29 // ouptut

  #define motor_2_S 33 //input

  #define display_s 36 //output

  #define tds_a_s 41 //input selsection b/w 4-20ma or built in sensor

  #define ph_a_s 38 //input selsection b/w 4-20ma or built in sensor

  #define iot_serv_s 37 //input

  #define motor_1_s 34 //input

  #define buzzer 31 //output

  #define preassure_2 23//input

  void setup() {

  Serial.begin(9600);

  pinMode(preassure_2,INPUT);

  pinMode(flow_sensor,INPUT);

  pinMode(ib_select,INPUT);

  pinMode(auto_manual,INPUT);

  pinMode(tank_1,INPUT);

  pinMode(tank_2,INPUT);

  pinMode(PH_s,INPUT);

  pinMode(I_M_I,INPUT);

  pinMode(motor_2_S,INPUT);

  pinMode(display_s,INPUT);

  pinMode(tds_a_s,INPUT);

  pinMode(ph_a_s,INPUT);

  pinMode(iot_serv_s,INPUT);

  pinMode(motor_1_s,INPUT);

  pinMode(tds_s,INPUT);

  pinMode(preassure_1,INPUT);

  pinMode(motor_2,OUTPUT);

  pinMode(buzzer,OUTPUT);

  pinMode(M_I_I,OUTPUT);

  pinMode(motor_1,OUTPUT);



  digitalWrite(motor_2,HIGH);

  digitalWrite(buzzer,HIGH);

  digitalWrite(M_I_I,HIGH);

  digitalWrite(motor_1,HIGH);

  delay(2000);



  }

  void loop() {

  digitalWrite(13,LOW);

  digitalWrite(motor_1,LOW);

  delay(2000);

  digitalWrite(motor_1,HIGH);

  digitalWrite(13,HIGH);

  }*/

/*

  /*

  Blink



  Turns an LED on for one second, then off for one second, repeatedly.



  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO

  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to

  the correct LED pin independent of which board is used.

  If you want to know what pin the on-board LED is connected to on your Arduino

  model, check the Technical Specs of your board at:

  https://www.arduino.cc/en/Main/Products



  modified 8 May 2014

  by Scott Fitzgerald

  modified 2 Sep 2016

  by Arturo Guadalupi

  modified 8 Sep 2016

  by Colby Newman



  This example code is in the public domain.



  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink

*/



// the setup function runs once when you press reset or power the board
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
bool tank1, tank2, am_s;

float ph;

int tds,  flow;
int D20, D24;
int waiting_t = 8000;
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 4000;
bool action,state, Send, action1, sensing;
SoftwareSerial mySerial(17, 16); // RX, TX

void setup() {

  Serial.begin(115200);

  pinMode(39, OUTPUT);//motor 1

  pinMode(29, OUTPUT);//motot_2

  pinMode(31, OUTPUT);//buzzer

  pinMode(36, OUTPUT);//display

  pinMode(26, INPUT); //tank 1

  pinMode(25, INPUT); //tank 2

  pinMode(35, INPUT); // auto / manual setup

  pinMode(34, INPUT); //Motor1_signal

  pinMode(33, INPUT); //Motor2_signal

  pinMode(38, INPUT); //PH SIGNAL

  pinMode(41, INPUT); //TDS SIGNAL

  pinMode(A1, INPUT); //PH_Sensor 4-20

  pinMode(A3, INPUT); //TDS_Sensor 4-20

  pinMode(A6, INPUT); //B_PH

  pinMode(A9, INPUT); //B_TDS
  pinMode(12,INPUT);//IMI

  digitalWrite(39, LOW);

  digitalWrite(29, LOW);
  mySerial.begin(115200);
  mySerial.println("Hello, world?");
  DynamicJsonDocument doc(1024);


  action = true;

  action1 = false;

  sensing = false;
  startMillis = millis();
}



void loop() {
  if (mySerial.available()) {
    StaticJsonDocument <500> data;
    String temp = mySerial.readString();
    Serial.println(temp);
    DeserializationError error = deserializeJson(data, temp);
    int st = (int)data["state"];
    state = st;
  }
  tank1 = digitalRead(26);
  tank2 = digitalRead(25);
  am_s = digitalRead(35);
  state = digitalRead(12);
  Serial.print(tank1); Serial.print("   "); Serial.print(tank2); Serial.print("   "); Serial.print(am_s);Serial.print("   "); Serial.println(state);
  if ((am_s == HIGH) && (state == true)) { //Auto Mode actions
    sensing = true;
    auto_mode();
  }
  else if (am_s == LOW) {//manual mode actions
    int m1s, m2s;
    Serial.println("Manual mode");
    m1s = digitalRead(34);
    m2s = digitalRead(33);
    Serial.print(m1s); Serial.print("  "); Serial.println(m2s);
    digitalWrite(39, m1s);
    digitalWrite(29, m2s);
  }
  else if (sensing == true) {
    ph = ph_reader();
    tds = tds_reader();
    sensing = false;
  }
  {
    currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
    if (currentMillis - startMillis >= period)  //test whether the period has elapsed
    {
      Send = true;
      startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
    }
  }

  data_send();
  delay(1000);                       // wait for a second
}
int buzzer_fun() {

  digitalWrite(31, HIGH);

  delay(100);

  digitalWrite(31, LOW);

}

void auto_mode() {

  if ((tank1 == HIGH) && (tank2 == LOW)) {

    if (action == true) {

      digitalWrite(39, HIGH);

      digitalWrite(29, LOW);

      buzzer_fun();

      action = false;

      action1 = true;
      D20 = 2;
      Serial.println("motor one started");

      delay(waiting_t);

      digitalWrite(29, HIGH);

    }



  }

  if (action1 == true) {

    if ((tank1 == LOW) || (tank2 == HIGH)) {

      digitalWrite(39, LOW);

      digitalWrite(29, LOW);

      Serial.println("motor's stopped");
      D20 = 24;
      buzzer_fun();

      action = true;

      action1 = false;

    }

  }



}

int ph_reader() {

  int reading, val, result;

  bool rd_type = digitalRead(38);

  if (rd_type == LOW) { //read in 4-20mA port.

    reading = analogRead(A1);

    val = (reading / 1024) * 100;

    result = (14 * val) / 100;
    Serial.println("ph = "+(String)result+";");
    return result;

  }

  else { //read with default built in sensor

    reading = analogRead(A6);

    val = (reading / 1024) * 100;

    result = (14 * val) / 100;
    Serial.println("ph = "+(String)result+";");
    return result;

  }

}

int tds_reader() {

  int reading, val, result;

  bool rd_type = digitalRead(38);

  if (rd_type == LOW) { //read in 4-20mA port.

    reading = analogRead(A1);

    val = (reading / 1024) * 100;

    result = (14 * val) / 100;
    Serial.println("tds = "+(String)result+";");
    return result;

  }

  else { //read with default built in sensor

    reading = analogRead(41);

    val = (reading / 1024) * 100;

    result = (1000 * val) / 100;
  Serial.println("tds = "+(String)result+";");
    return result;

  }
}

void data_send() {
  if (Send == true) {
    StaticJsonDocument <500> value;
    value["ph"] = ph;
    value["tds"] = tds;
    value["flow"] = flow;
    value["D20"] = D20 ;
    serializeJson(value, mySerial);
    Send = false;
  }
}
