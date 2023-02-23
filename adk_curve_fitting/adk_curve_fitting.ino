#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "functions.h"
#include "ps3.h"

/*
1 left 24 946  
2 down 21 809
3 right 24 670
4 square 25 538
5 up 27 891
6 circle 35 582 1150
7
*/
float h_pos = 133.0,v_pos=0.0;
int v_off=0;
int main() {
  init();
  Serial.begin(115200);
  
  pin_setup();
  Serial_int_setup();
  timer_setup(600, 100); //100 -> duty cycle of linear actuator
  encoder_setup();
  IR_setup();
  ps3_setup();
  while (!(PINL & (1 << actuator_limit_pin))) { //set Linear actuator to zero pos using actuator limit pin
    PORTL &= ~(1 << actuator_dir);
    OCR5B = set_ocr;
    _delay_us(500);
  }
  PORTL |= (1 << actuator_dir);
  OCR5B = 0;
  while ((PINL & (1 << BTM_limit_pin))) { //set BTM - Stepper to zero pos using BTM - Stepper limit pin
    if ((PINL & (1 << BTM_limit_pin))) {
      PORTH &= ~(1 << BTM_dir);
      PORTB ^= (1 << BTM_pulse);
    }
    else {
      PORTH &= ~(1 << BTM_dir);
      PORTB &= ~(1 << BTM_pulse);
    }
    _delay_us(500);
  }
  desired_deg = 0;
  prev_pos = 533;
  encoder_counts = 0; //To set the intial position as zero
  _delay_ms(1000);

  while (1) {
//    desired_H_pos = 91;
//      desired_V_pos = 8;
    ps3_data();
    if (Serial.available()) {
//      if (serial_input_flag == 0) { //rpm
        desired_rpm = Serial.parseInt();
        if (desired_rpm == 0) {
          OCR4A = 0;
          OCR4B = 0;
        }
//        serial_input_flag = 1;
//      }
//      else if (serial_input_flag == 1) {  //linear actuator
//        desired_V_pos = Serial.parseInt();
//        serial_input_flag = 2;
//      }
//      else if (serial_input_flag == 2) { //BTM stepper
//        desired_H_pos = Serial.parseInt();
//        serial_input_flag = 0;
//      }
    }
//        Serial.print(serial_input_flag);
//        Serial.print("      ");
        Serial.print(desired_H_pos);
        Serial.print("  ");
        Serial.print(desired_V_pos);
        Serial.print("  ");
//        Serial.print(rpm1);
//        Serial.print("      ");
//        Serial.print(rpm2);
//        Serial.print("      ");
//        Serial.print(pwm1);
//        Serial.print("      ");
//        Serial.print(pwm2);
//        Serial.print("      ");
        Serial.println(rpm_mega);
//    get_values_BOH();
    if (ps3.cross) {
//      desired_H_pos = 91;
//      desired_V_pos = 0;
      desired_rpm = 0;
    }
   else if (ps3.up) {
    v_pos += 0.01;
    desired_V_pos = v_pos;
    }

   else if (ps3.down){
    v_pos -= 0.01;
    desired_V_pos = v_pos;
   }
   else if (ps3.left){
    h_pos +=0.1;
    desired_H_pos = h_pos;
   }
   else if (ps3.right){
    h_pos -= 0.1;
    desired_H_pos = h_pos;
      
   }
   else if(ps3.circle){
  desired_rpm =1400;
   }
   else if(ps3.square){
//    desired_H_pos = ;
//      desired_V_pos = ;
//      desired_rpm =;
   }
   else if(ps3.L1){
    desired_rpm = 1600;
    
    
   }
   else if(ps3.R1){
    v_off=0;
    desired_rpm = 1550;
   }
  
   

if (ps3.tri){
      PORTA |= (1 << pneumatic_pin);
    }
    else {
      PORTA &= ~(1 << pneumatic_pin);
    }
    input_constrain();
    set_pos(desired_H_pos);
    set_actuator(desired_V_pos);
    PID_set(desired_rpm);
    rpm_constrain(desired_rpm);
    pwm1 = desired_rpm + z1;
    pwm2 = desired_rpm + z2;
    pwm_constrain();
    OCR4A = pwm1;
    OCR4B = pwm2;
//   else if (ps3.right){
//      desired_H_pos = 92;
//      desired_V_pos = 11;
//      desired_rpm = 1550;
//   }
//   else if (ps3.left){
//      desired_H_pos = 92;
//      desired_V_pos = 11;
//      desired_rpm = 1400;
//   }
//    if (switch_pneumatic == 0x01) {
  }
}
