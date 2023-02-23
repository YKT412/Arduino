#include <avr/io.h>
#include <avr/interrupt.h>
#include "node_mcu.h"
#include <util/delay.h>
#include "functions.h"
//#include "ps3.h"
// G 2500 68 19
// F 2500 98 22
// D 2500 122 23
// H 2500 48  17
// E 2500 135 19
// C 2500 160 19
// B 2500 133 20 STEPPER ANGLE MAY CHANGE
// A 2500 118/120 28 VERTICAL ANGLE MAY CHANGE
// K 2500 20 22
// J 2100 45 27
// I 2400 60 30
//1 2300 95 8 better
//2 2300 95 7 bestem best
//3 2400 95 7
//4 2400 95 10 good
//5 2400 95 8 & 2300 95 8 best


void debug_vars() {
  //  Serial.println("=== DEBUG (ON) ===");
  //  Serial.print("X-angle: ");
  //  Serial.print(X_angle);
  //  Serial.println();
  //
  //  Serial.print("Y-angle: ");
  //  Serial.print(Y_angle);
  //  Serial.println();
  //
  //  Serial.print("RPM: ");
  //  Serial.print(RPM);
  //  Serial.println();
  //
  //  Serial.print("THROW: ");
  //  Serial.print(THROW);
  //  Serial.println("=== X ===\n");
}

int main() {
  init();
  Serial.begin(115200);
  Serial2.begin(74880); //for node mcu communication
  pin_setup();
  timer_setup(350, 50); //300 -> Stepper speed | 100 -> duty cycle of linear actuator
  encoder_setup(); //for actuator feedback
  IR_setup(); // for roller feedback
  //  ps3_setup();
  while ((PINH & (1 << actuator_limit_pin)) || (PINH & (1 << BTM_limit_pin))) { //set Linear actuator to zero pos using actuator limit pin
    if ((PINH & (1 << actuator_limit_pin))) {
      PORTB |= (1 << actuator_dir);
      OCR1B = set_ocr;
    }
    else {
      PORTB |= (1 << actuator_dir);
      OCR1B = 0;
    }
    if ((PINH & (1 << BTM_limit_pin))) {
      PORTB |= (1 << BTM_dir);
      //      PORTB ^= (1 << BTM_pulse);
      OCR1A = set_ocr;
    }
    else {
      PORTB |= (1 << BTM_dir);
      //      PORTB &= ~(1 << BTM_pulse);
      OCR1A = 0;
    }
    //    Serial.println("YES");
//    _delay_us(500);
  }
  PORTH |= (1 << actuator_dir);
  OCR1B = 0;
  PORTB |= (1 << BTM_dir);
  OCR1A = 0;

  //  while ((PINH & (1 << BTM_limit_pin))) { //set BTM - Stepper to zero pos using BTM - Stepper limit pin
  //    if ((PINH & (1 << BTM_limit_pin))) {
  //      PORTB |= (1 << BTM_dir);
  //      PORTB ^= (1 << BTM_pulse);
  //    }
  //    else {
  //      PORTB |= (1 << BTM_dir);
  //      PORTB &= ~(1 << BTM_pulse);
  //    }
  //    _delay_us(500);
  //  }
  //To set the intial position as zero
  //  PORTH |= (1 << actuator_dir);
  //  OCR1B = 0;
  //  desired_deg = 0;
  //  prev_pos = 0;
  actuator_encoder_counts = 0;
  stepper_encoder_counts = 0;
  prev_stepper_counts = 0;
  while (1) {
//    debug_vars();
    //    ps3_data();
    getDataFromApp(); //for BOH Hitter Round
    //      Serial.print(DATA);
    PNEUMATIC = THROW;
    desired_V_pos = Y_angle; //for BOH Hitter Round
    desired_H_pos = X_angle; //for BOH Hitter Round
    desired_rpm = RPM;

    ////    if (X_angle == 0 && Y_angle == 0 && RPM == 0) {  // reset with app
    ////      OCR4A = 0;
    ////      OCR4B = 0;
    ////      while ((PINH & (1 << actuator_limit_pin)) || (PINH & (1 << BTM_limit_pin))) { //set Linear actuator to zero pos using actuator limit pin
    ////        if ((PINH & (1 << actuator_limit_pin))) {
    ////          PORTB |= (1 << actuator_dir);
    ////          OCR1B = set_ocr;
    ////        }
    ////        else {
    ////          PORTB |= (1 << actuator_dir);
    ////          OCR1B = 0;
    ////        }
    ////        if ((PINH & (1 << BTM_limit_pin))) {
    ////          PORTB |= (1 << BTM_dir);
    ////          PORTB ^= (1 << BTM_pulse);
    ////        }
    ////        else {
    ////          PORTB |= (1 << BTM_dir);
    ////          PORTB &= ~(1 << BTM_pulse);
    ////        }
    ////        //    Serial.println("YES");
    ////        _delay_us(500);
    ////      }
    ////      PORTH |= (1 << actuator_dir);
    ////      OCR1B = 0;
    ////      desired_deg = 0;
    ////      prev_pos = 0;
    ////      encoder_counts = 0;
    // //set Linear actuator to zero pos using actuator limit pin
    //        if ((PINH & (1 << actuator_limit_pin))) {
    //          PORTB |= (1 << actuator_dir);
    //          OCR1B = set_ocr;
    //        }
    //        else {
    //          PORTB |= (1 << actuator_dir);
    //          OCR1B = 0;
    //          encoder_counts = 0;
    //        }
    //        if ((PINH & (1 << BTM_limit_pin))) {
    //          PORTB |= (1 << BTM_dir);
    //          PORTB ^= (1 << BTM_pulse);
    //        }
    //        else {
    //          PORTB |= (1 << BTM_dir);
    //          PORTB &= ~(1 << BTM_pulse);
    //           prev_pos = 0;
    //        }
    //        //    Serial.println("YES");
    //        _delay_us(500);
    //    }


    //    if (Serial.available()) {
    //            desired_H_pos = Serial.parseInt();
    //          }
    //      if (serial_input_flag == 0) { //rpm
    //        desired_rpm = Serial.parseInt();
    //        //        if (desired_rpm == 0) {
    //        //          OCR4A = 0;
    //        //          OCR4B = 0;
    //        //        }
    //        serial_input_flag = 1;
    //      }
    //      else if (serial_input_flag == 1) {  //linear actuator -> enter degree
    //        desired_V_pos = Serial.parseInt();
    //        serial_input_flag = 2;
    //      }
    //      else if (serial_input_flag == 2) { //BTM stepper -> enter degree
    //        desired_H_pos = Serial.parseInt();
    //        serial_input_flag = 0;
    //      }
    //    }
    //        Serial.print(serial_input_flag);

    //    Serial.print(PNEUMATIC);
    //    Serial.print("      ");
    Serial.print(desired_H_pos);
    Serial.print("      ");
    //    Serial.print(desired_V_pos);
    //    Serial.print("      ");
    //    Serial.print(desired_V_pos * 6.3); //
    //    Serial.print("      ");
    //    Serial.println(desired_rpm);
    //    Serial.print(encoder_counts);
    //    get_values_BOH();
    //    if (ps3.cross) {
    //      Serial.print("Roller Stop");
    //      X_angle = 0;
    //      Y_angle = 0;
    //      RPM = 100;
    //    }
    //       else if (ps3.up) {
    //          Serial.print("BTM Set");
    //          desired_H_pos = 90;
    //          desired_V_pos = 50;
    //          desired_rpm = 1500;
    //        }

    //// === Using PS3 ===
    //        if (ps3.R1) {
    //          PORTA |= (1 << pneumatic_pin);
    //          Serial.println("Release Ball");
    //        }
    //        else {
    //          PORTA &= ~(1 << pneumatic_pin);
    //        }
    // ======

    //    == = Throw Using ESP == =
    //    if (PNEUMATIC == 20) {
    //      PORTA |= (1 << pneumatic_pin);
    //      Serial.println("Release Ball");
    //    } else if (PNEUMATIC == 40) {
    //      PORTA &= ~(1 << pneumatic_pin);
    //    }
    //    //    == == ==
    //    input_constrain();
    //    //    rpm_constrain();
    ////    set_pos(desired_H_pos);
    stepper_encoder(desired_H_pos * 6.3 * 4);
    //    set_actuator(desired_V_pos * 6.3); //6.3 counts of encoder = 1 degree (approx)
    //    PID_set(desired_rpm);
    //    PID_constrain();
    //    pwm1 = desired_rpm + z1;
    //    pwm2 = desired_rpm + z2;
    //    pwm_constrain();
    //    OCR4A = pwm1;
    //    OCR4B = pwm2;
    Serial.print("      ");
    Serial.print(prev_stepper_counts);
    Serial.print("      ");
    //        Serial.print(rpm2);
    Serial.println(stepper_encoder_counts);
    //    Serial.print("      ");
    //    Serial.println(desired_rpm);
  }
}
