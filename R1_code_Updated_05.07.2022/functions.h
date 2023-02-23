#include "pin_setup.h"

/* BTM - Stepper */
uint8_t gear_ratio = 4;
int8_t BTM_offset = 0; // in degrees
uint16_t prev_pos = 0, pulse = 0;
int16_t p = 0;
float steps_per_deg = 8.889;

/* Set + Map linear actuator positon wrt encoder counts*/
uint16_t prev_deg = 0, desired_deg = 0;
int16_t diff_counts = 0, prev_counts = 0, diff = 0;
uint8_t margin_of_error = 3, actuator_offset = 31; //offset in degrees
float scaling_factor = 6.3; //mapping factor of encoder and degree values

/* Stepper encoder */
int stepper_diff_counts = 0, prev_stepper_counts = 0;

/* RPM */
long int prev_time1 = 0, prev_time2 = 0, duration1 = 0, duration2 = 0;
long int rpm1 = 0, rpm2 = 0, prev_rpm2 = 0, prev_rpm1 = 0;
long int rpm2_diff = 0, rpm1_diff = 0;

/* Custom Millis Function for RPM*/
unsigned long h = 0, prev_h = 0, h2 = 0, prev_h2 = 0;
unsigned char serial_input_flag = 0, switch_pneumatic = 0;

/* PID of M1 and M2 */
float kp1 = 20, kd1 = 10 , P1 = 0, D1 = 0, z1 = 0, kp2 = 20, kd2 = 20, P2 = 0, D2 = 0, z2 = 0;
float e1 = 0, pe1 = 0, e2 = 0, pe2 = 0;
int pwm1 = 0, pwm2 = 0, pwm3 = 0;

/* Input values */
unsigned int desired_H_pos = 0, desired_V_pos = 0, PNEUMATIC = 0;
int desired_rpm = 0;

/* BOH position values for Hitter Round */
unsigned char BOH_V_POS[11] = {14, 19, 25, 18, 27, 16, 18, 27, 14, 19, 25}; //actuator theoretical degrees (A ... K)
unsigned char BOH_H_POS[11] = {120, 135, 164, 118, 138, 90, 62, 42, 60, 45, 16}; //BTM stepper theoretical degrees (A ... K)

/* Encoder Counts */
int actuator_encoder_counts = 0, stepper_encoder_counts = 0;
unsigned char a0, b0, a1, b1, c0, c1, d1, d0;

/* Actuator direction */
void set_actuator_dir(int d) {
  if (d > 0) {
    PORTB &= ~(1 << actuator_dir); //(Up) Increase length
  }
  if (d < 0) {
    PORTB |= (1 << actuator_dir); //(Down) Decrease length
  }
}

/* Stepper Direction */
void set_stepper_dir(int d) {
  if (d < 0) {
    PORTB &= ~(1 << BTM_dir);  //clockwise
  }
  if (d < 0) {
    PORTB |= (1 << BTM_dir); //anticlockwise
  }
}

/* PWM to actuator based on feedback position */
void set_actuator (uint16_t desired_counts) {
  if (desired_counts <= actuator_offset) desired_counts = 0; //for making sure actuator does not go below limit pin
  else desired_counts = desired_counts - actuator_offset; //for compensating offset
  diff_counts = (int)((desired_counts) - actuator_encoder_counts) - diff;
  set_actuator_dir(diff_counts);
  //  if ((abs(diff_counts)) > 10) {
  //    OCR1B = set_ocr; //high speed
  //  }
  //  else {
  //    if (abs(diff_counts) > 0)
  //    {
  //      OCR1B = (set_ocr/2); //low speed
  //    }
  if ((abs(diff_counts)) > 10) {
    OCR1B = set_ocr;
  }
  else {
    OCR1B = 0;
    prev_counts = desired_counts;
    diff = actuator_encoder_counts - desired_counts;
  }
}

/* PID of M1 and M2 */
void PID_set (unsigned int s) {
  e1 = s - rpm1;
  e2 = s - rpm2;
  P1 = kp1 * e1;
  P2 = kp2 * e2;
  D1 = kd1 * ((e1 - pe1) / (h - prev_h));
  D2 = kd2 * ((e2 - pe2) / (h - prev_h));
  z1 = P1 + D1;
  z2 = P2 + D2;
  pe1 = e1;
  pe2 = e2;
  prev_h = h;
}

/* Input values constrain */
void input_constrain () {
  desired_H_pos = constrain(desired_H_pos, 0, 200);
  desired_V_pos = constrain(desired_V_pos, 0, 280);
  desired_rpm = constrain(desired_rpm, 0, 4000);
}

/* PID error constrain */
void PID_constrain() {
  z1 = constrain(z1, -500, 500);
  z2 = constrain(z2, -500, 500);
}

/* PWM constrain*/
void pwm_constrain() {
  pwm1 = constrain(pwm1, 0, 4000);
  pwm2 = constrain(pwm2, 0, 4000);
}

/* BTM - Stepper direction and pulse */
//void set_pos(int pos) {
//  if (pos <= abs(BTM_offset)) {
//    pos = 0;
//  }
//  else {
//    pos = ((pos + BTM_offset) * gear_ratio * steps_per_deg);
//  }
//  if (pos > prev_pos) {
//    PORTB &= ~(1 << BTM_dir);           //clockwise
//  }
//  else {
//    PORTB |= (1 << BTM_dir);          //anticlockwise
//  }
//  p = (prev_pos - pos);
//  pulse = abs(p);
//  TIMSK3 |= (1 << OCIE3A);
//}

/* SET stepper encoder*/
void stepper_encoder(uint16_t desired_counts) {
  stepper_diff_counts = desired_counts - stepper_encoder_counts;
  set_stepper_dir(stepper_diff_counts);
//  if (stepper_diff_counts == 0) {
//    pulse = 0;
//    TIMSK3 &= ~(1 << OCIE3A);
//  }
//  else {
//    pulse = 1;
//    TIMSK3 |= (1 << OCIE3A);
//  }
if ((abs(stepper_diff_counts)) > 10) {
    OCR1A = set_ocr;
  }
  else {
    OCR1A = 0;
    prev_stepper_counts = desired_counts;
//    diff = stepper_encoder_counts - desired_counts;
  }
}
/* BTM - Stepper Timer for pulse */
//ISR(TIMER3_COMPA_vect)
//{
//  if (pulse > 0) {
//    PORTB ^= (1 << BTM_pulse);
//    if ((PINB & (1 << BTM_pulse)) == 0) {
//      pulse--;
//      if ((PINB & (1 << BTM_dir)) == 0x10) {
//        prev_pos--;
//      }
//      else prev_pos++;
//    }
//  }
//  else if (pulse == 0)
//  {
//    TIMSK3 &= ~(1 << OCIE3A);
//  }
//}

/* For Encoder Counts (Feedback of Linear actuator (vertical angle)) */
/* encoder_counts++ -> Increase in actuator length (Up)
   encoder_counts-- -> Decrease in actuator length (Down) */
ISR (PCINT1_vect)
{
  a0 = ~PINJ & 0x01; //encoder_signal_1 (PJ0)
  b0 = ~PINJ & 0x02; //encoder_signal_2 (PJ1)
  if (a0 != a1) {
    if (a0) {
      if (b0) actuator_encoder_counts++;
      else actuator_encoder_counts--;
    }
    else {
      if (b0) actuator_encoder_counts--;
      else actuator_encoder_counts++;
    }
  }
  else if (b0 != b1) {
    if (b0) {
      if (a0) actuator_encoder_counts--;
      else actuator_encoder_counts++;
    }
    else {
      if (a0) actuator_encoder_counts++;
      else actuator_encoder_counts--;
    }
  }
  a1 = a0;
  b1 = b0;
}
/* STEPPER ENCODER INTERRUPT*/
ISR (PCINT2_vect)
{
  c0 = ~PINK & 0x02; //encoder_signal_1 (Pk1)
  d0 = ~PINK & 0x04; //encoder_signal_2 (Pk2)
  if (c0 != c1) {
    if (c0) {
      if (d0) stepper_encoder_counts++;
      else stepper_encoder_counts--;
    }
    else {
      if (d0) stepper_encoder_counts--;
      else stepper_encoder_counts++;
    }
  }
  else if (d0 != d1) {
    if (d0) {
      if (c0) stepper_encoder_counts--;
      else stepper_encoder_counts++;
    }
    else {
      if (c0) stepper_encoder_counts++;
      else stepper_encoder_counts--;
    }
  }
  c1 = c0;
  d1 = d0;
}

/* IR input of M1 (Right Motor (from behind BTM)) -> ADK PIN 3*/
ISR (INT5_vect) {
  duration1 = (h - prev_time1);
  prev_time1 = h;
  rpm1 = ((60 * 100000) / duration1);
  rpm1_diff = (rpm1 - desired_rpm);
  if (abs(rpm1_diff) > 500) {
    rpm1 = prev_rpm1;
  }
  rpm1 = rpm1 * (0.1) + prev_rpm1 * (0.9);
  prev_rpm1 = rpm1;
}

/* IR input of M2 (Left Motor (from behind BTM)) -> ADK PIN 2*/
ISR (INT4_vect) {
  duration2 = (h - prev_time2);
  prev_time2 = h;
  rpm2 = ((60 * 100000.0) / duration2);
  rpm2_diff = (rpm2 - desired_rpm);
  if (abs(rpm2_diff) > 500) {
    rpm2 = prev_rpm2;
  }
  rpm2 = rpm2 * (0.1) + prev_rpm2 * (0.9);
  prev_rpm2 = rpm2;
}

/* Custom Millis function for RPM */
ISR(TIMER5_COMPA_vect) {
  h++;
}
