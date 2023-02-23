#include "pin_setup.h"

/* BTM - Stepper */
uint8_t gear_ratio = 4;
int8_t BTM_offset = (-15);
uint16_t prev_pos = 0, pulse = 0;
int16_t p = 0;
float steps_per_deg = 8.889;

/* Set + Map linear actuator positon wrt encoder/potentiometer counts*/
uint16_t prev_deg = 0, desired_deg = 0;
//uint16_t max_deg = 360, max_analog = 1024; // to map encoder/ potentiometer values with degrees input
int16_t diff_counts = 0;
uint8_t margin_of_error = 3, actuator_offset = 3; //offset in degrees
float scaling_factor = 6.3; //mapping factor of encoder and degree values
//uint16_t pot_value = 0;  // for potentiometer -> UNUSED

/* RPM */
long int prev_time1 = 0, prev_time2 = 0, duration1 = 0, duration2 = 0;
long int rpm1 = 0, rpm2 = 0, prev_rpm2 = 0, prev_rpm1 = 0;
long int rpm2_diff = 0, rpm1_diff = 0;

/* Custom Millis Function  for RPM*/
unsigned long h = 0, prev_h = 0;

/* PID of M1 and M2 */
float kp1 = 17, kd1 = 33, P1 = 0, D1 = 0, z1 = 0, kp2 = 17, kd2 = 53, P2 = 0, D2 = 0, z2 = 0;
float e1 = 0, pe1 = 0, e2 = 0, pe2 = 0;

/* Received data from Mega */
uint8_t  i = 0;
uint16_t desired_H_pos = 0, desired_V_pos = 0;
int desired_rpm = 0;
unsigned char checksum = 0,input_values[7], header_byte = 0x57;

/* Encoder Counts */
int encoder_counts = 0;
unsigned char a0, b0, a1, b1;

/* To convert degree into encoder/potentiometer reading */ //-> UNUSED
//uint16_t deg_to_analog (float deg) {
//  uint16_t analog;
//  analog = (uint16_t)((max_analog * deg) / max_deg);
//  return analog;
//}

/* Actuator direction */
void set_actuator_dir(int d) {
  if (d > 0) {
    PORTL |= (1 << actuator_dir); //Up / Increase length
  }
  if (d < 0) {
    PORTL &= ~(1 << actuator_dir); //Down / Decrease length
  }
}

/* PWM to actuator based on feedback position */
void set_actuator (uint16_t desired_deg) {
  if (desired_deg <= 3) desired_deg = 0; //for compensating offset
  else desired_deg = desired_deg - actuator_offset;
  diff_counts = (int)((desired_deg * scaling_factor) - encoder_counts);
  set_actuator_dir(diff_counts);
  if ((abs(diff_counts)) > margin_of_error) {
    OCR5B = set_ocr;
    //      prev_deg = (int)(encoder_counts / scaling_factor);
  }
  else {
    OCR5B = 0;
    //      prev_deg = (int)(encoder_counts / scaling_factor);
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

/* RPM constrain for M1 and M2 */
void spd_constrain(int s) {
  //  if (rpm1 > desired_rpm + 100) {
  //    OCR4A = 0;
  //  }
  //  if (rpm2 > desired_rpm + 100) {
  //    OCR4B = 0;
  //  }
  z1 = constrain(z1, -200, 200);

  z2 = constrain(z2, -200, 200);
}

/* BTM - Stepper direction and pulse */
void set_pos(int pos) {
  if (pos <= abs(BTM_offset)) {
    pos = 0;
  }
  else {
    pos = (pos + BTM_offset) * gear_ratio * steps_per_deg;
  }
  if (pos > prev_pos) {
    PORTH |= (1 << BTM_dir);           //clockwise
  }
  else {
    PORTH &= ~(1 << BTM_dir);          //anticlockwise
  }
  p = prev_pos - pos;
  pulse = abs(p);
  TIMSK3 |= (1 << OCIE3A);
}

/* BTM - Stepper Timer for pulse */
ISR(TIMER3_COMPA_vect)
{
  if (pulse > 0) {
    PORTB ^= (1 << BTM_pulse);
    if ((PINB & (1 << BTM_pulse)) == 0) {
      pulse--;
      if ((PINH & (1 << BTM_dir)) == 0x40) {
        prev_pos++;
      }
      else prev_pos--;
    }
  }
  else if (pulse == 0)
  {
    TIMSK3 &= ~(1 << OCIE3A);
  }
}

/* For Encoder Counts (Feedback of Linear actuator (vertical angle)) */
/* encoder_counts++ -> Increase in actuator length (Up)
   encoder_counts-- -> Decrease in actuator length (Down) */
ISR (PCINT0_vect)
{
  a0 = ~PINB & 0x01; //encoder_signal_1 (PB0)
  b0 = ~PINB & 0x04; //encoder_signal_2 (PB2)
  if (a0 != a1) {
    if (a0) {
      if (b0) encoder_counts++;
      else encoder_counts--;
    }
    else {
      if (b0) encoder_counts--;
      else encoder_counts++;
    }
  }
  else if (b0 != b1) {
    if (b0) {
      if (a0) encoder_counts--;
      else encoder_counts++;
    }
    else {
      if (a0) encoder_counts++;
      else encoder_counts--;
    }
  }
  a1 = a0;
  b1 = b0;
}

/* Mega -> ADK (Receive 4 bytes of data :
    input_values[0] = desired_H_angle
    input_values[1] = desired_V_angle
    input_values[2] = desired_RPM_Low_byte
    input_values[3] = desired_RPM_High_byte
    RPM bytes have been combined in a single integer variable -> desired_rpm)
*/
ISR(USART1_RX_vect) {
  if (i < 7) {
    input_values[i] = UDR1;
    if (i < 6) {
      checksum += input_values[i];
    }
  }
  i++;
  if (i > 6) {
    i = 0;
  }
}

/* IR input of M1 (Left Motor (from behind BTM)) -> ADK PIN 3*/
ISR (INT5_vect) {
  duration1 = (h - prev_time1);
  prev_time1 = h;
  rpm1 = ((60 * 100000) / duration1);
  //  rpm2=rpm2*(0.05)+prev_rpm2*(0.95);
  rpm1_diff = (rpm1 - desired_rpm);
  if (abs(rpm1_diff) > 500) {
    rpm1 = prev_rpm1;
  }
  rpm1 = rpm1 * (0.1) + prev_rpm1 * (0.9);
  prev_rpm1 = rpm1;
}

/* IR input of M2 (Right Motor (from behind BTM)) -> ADK PIN 2*/
ISR (INT4_vect) {
  duration2 = (h - prev_time2);
  prev_time2 = h;
  rpm2 = ((60 * 100000) / duration2);
  //  rpm2=rpm2*(0.05)+prev_rpm2*(0.95);
  rpm2_diff = (rpm2 - desired_rpm);
  if (abs(rpm2_diff) > 500) {
    rpm2 = prev_rpm2;
  }
  rpm2 = rpm2 * (0.1) + prev_rpm2 * (0.9);
  prev_rpm2 = rpm2;
}

/* Custom Millis function for RPM */
ISR(TIMER1_COMPA_vect) {
  h++;
}
