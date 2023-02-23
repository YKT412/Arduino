#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "functions.h"
//#include "ps3.h"
//*****************************************************************************************************************
unsigned char serial_input_flag = 0, switch_pneumatic = 0;
unsigned long h2 = 0; //To use serial monitor to input multiple data
int pwm1 = 0, pwm2 = 0;

//*****************************************************************************************************************

int main() {
  init();
  Serial.begin(115200);
  //  ps3_setup();
  pin_setup();
  Serial_int_setup();
  timer_setup(300, 100); //50 -> duty cycle of linear actuator
  encoder_setup();
  IR_setup();
  //    while (!(PINL & (1 << actuator_limit_pin))) { //set Linear actuator to zero pos using actuator limit pin
  //      PORTL &= ~(1 << actuator_dir);
  //      OCR5B = set_ocr;
  //      _delay_us(500);
  ////      Serial.println(encoder_counts);
  //    }
  PORTL |= (1 << actuator_dir);
  OCR5B = 0;
  //  while ((PINL & (1 << BTM_limit_pin))) { //set BTM - Stepper to zero pos using BTM - Stepper limit pin
  //    if ((PINL & (1 << BTM_limit_pin))) {
  //      PORTH &= ~(1 << BTM_dir);
  //      PORTB ^= (1 << BTM_pulse);
  //    }
  //    else {
  //      PORTH &= ~(1 << BTM_dir);
  //      PORTB &= ~(1 << BTM_pulse);
  //    }
  ////    Serial.println(digitalRead(48));
  //    _delay_us(500);
  //  }
  desired_deg = 0;
  prev_pos = 0;
  //  prev_deg = 0;
  encoder_counts = 0; //To set the intial position as zero
  _delay_ms(1000);
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  while (1) {
    //    ps3_data();
    //    if (Serial.available()) {
    //      if (serial_input_flag == 0) {
    //        desired_rpm = Serial.parseInt();
    //        if (desired_rpm == 0) {
    //          OCR4A = 0;
    //          OCR4B = 0;
    //        }
    //        serial_input_flag = 1;
    //      }
    //      else if (serial_input_flag == 1) { //To enter vertical angle for linear actuator
    //        desired_V_pos = Serial.parseInt();
    //        //        if (desired_V_pos <= 3) desired_V_pos = 0; //for compensating offset
    //        //        else desired_V_pos = desired_V_pos - actuator_offset;
    //        serial_input_flag = 2;
    //      }
    //      else if (serial_input_flag == 2) { //To enter vertical angle for linear actuator
    //        desired_H_pos = Serial.parseInt();
    //        //        if (desired_V_pos <= 3) desired_V_pos = 0; //for compensating offset
    //        //        else desired_V_pos = desired_V_pos - actuator_offset;
    //        serial_input_flag = 3;
    //      }
    //      else if (serial_input_flag == 3){
    //        switch_pneumatic = Serial.parseInt();
    //        h2=h;
    //        serial_input_flag = 0;
    //      }
    //  }
    Serial.print(desired_H_pos);
    Serial.print("      ");
    Serial.print(desired_V_pos);
    Serial.print("      ");
    //      Serial.print(rpm1);
    //      Serial.print("      ");
    //      Serial.print(rpm2);
    //      Serial.print("      ");
    //      Serial.print(pwm1);
    //      Serial.print("      ");
    //      Serial.print(pwm2);
    //      Serial.print("      ");
    Serial.print(desired_rpm);
    Serial.print("      ");
    Serial.print(input_values[6]);
    Serial.print("      ");
    Serial.println(checksum);
//    Serial.println("      ");
    if (input_values[0] == header_byte) {
      if (input_values[1] == header_byte) {
        if (input_values[6] == checksum) {
          desired_H_pos = input_values[2];
          desired_V_pos = input_values[3];
          desired_rpm = ((input_values[4]) | (input_values[5] << 8));
          desired_H_pos = constrain(desired_H_pos, 0, 110);
          desired_V_pos = constrain(desired_V_pos, 0, 45);
          desired_rpm = constrain(desired_rpm, 0, 2100);
          checksum = 0;
        
        }
      }
    }
    //    if (switch_pneumatic == 0x01){
    //      PORTA |= (1 << pneumatic_pin);
    //      if ((h - h2) > 100000){
    //        switch_pneumatic = 0x00;
    //      }
    //    }
    //    else {
    //      PORTA &= ~(1 << pneumatic_pin);
    //    }
    set_pos(desired_H_pos);
    set_actuator(desired_V_pos);
    PID_set(desired_rpm);
    spd_constrain(desired_rpm);
    pwm1 = desired_rpm + z1;
    pwm2 = desired_rpm + z2;
    pwm1 = constrain(pwm1, 0, 2100);
    pwm2 = constrain(pwm2, 0, 2100);
    OCR4A = pwm1;
    OCR4B = pwm2;
  }
}
