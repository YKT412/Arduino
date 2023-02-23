#include <ArduinoJson.h>
#include <util/delay.h>
//#include "encoder.h"

StaticJsonDocument<200> json;
int DATA, X_angle = 0, Y_angle = 0, RPM = 0, THROW = 40;



void getDataFromApp() {

  String str = "";
  while (Serial2.available()) {
    char c = Serial2.read();
    str += c;
  }

  deserializeJson(json, str.c_str());

  int x_angle = json["X_digit"].as<int>();
  int y_angle = json["Y_digit"].as<int>();
  int rpm = json["RPM"].as<int>();

  // adding thorwing mech.
  int THR = json["Throw"].as<int>();

  int LOCALDATA_x_angle, LOCALDATA_y_angle, LOCALDATA_RPM;
  int LOCAL_THROW;

  if (THR) {
    LOCAL_THROW = THR;
    THROW = LOCAL_THROW;
  } else {
    LOCAL_THROW = THROW;
  }
  //    Serial.print("THROW VALUE [0 Default/push | 1 Back]:");
  //    Serial.print(THROW);

  if (x_angle) {
    if (x_angle == 1) {
      x_angle = 0;
    }
    LOCALDATA_x_angle = x_angle;
    X_angle = LOCALDATA_x_angle;
  } else {
    LOCALDATA_x_angle = X_angle;
  }


  if (y_angle) {
    if (y_angle == 1) {
      y_angle = 0;
    }
    LOCALDATA_y_angle = y_angle;
    Y_angle = LOCALDATA_y_angle;
  } else {
    LOCALDATA_y_angle = Y_angle;
  }

  if (rpm) {
    if (rpm == 1) {
      rpm = 0;
    }
    LOCALDATA_RPM = rpm;
    RPM = LOCALDATA_RPM;
  } else {
    LOCALDATA_RPM = RPM;
  }

  delay(9);

}


// RESET BUTTON
// THROW MECHANISM


// ARDUINO LOGIC
//if (THROW) {
//  PORTA |= (1 << pneumatic_pin);
//  Serial.println("Release Ball");
//} else {
//  PORTA &= ~(1 << pneumatic_pin);
//}
