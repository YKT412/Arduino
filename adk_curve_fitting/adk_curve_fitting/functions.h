#include "pin_setup.h"

int rpm_mega = 0, act_mega = 0, step_mega = 91;
/* BTM - Stepper */
uint8_t gear_ratio = 4;
int8_t BTM_offset = (-15);
int prev_pos = 0, pulse = 0;
int16_t p = 0;
float steps_per_deg = 8.889;

/* Set + Map linear actuator positon wrt encoder/potentiometer counts*/
uint16_t prev_deg = 0, desired_deg = 0;
//uint16_t max_deg = 360, max_analog = 1024; // to map encoder/ potentiometer values with degrees input
int16_t diff_counts = 0;
uint8_t margin_of_error = 3, actuator_offset = 3; //offset in degrees
float scaling_factor = 6.3; //mapping factor of encoder and degree values

/* RPM */
long int prev_time1 = 0, prev_time2 = 0, duration1 = 0, duration2 = 0;
long int rpm1 = 0, rpm2 = 0, prev_rpm2 = 0, prev_rpm1 = 0;
long int rpm2_diff = 0, rpm1_diff = 0;

/* Custom Millis Function  for RPM*/
unsigned long h = 0, prev_h = 0, h2 = 0;
unsigned char serial_input_flag = 0, switch_pneumatic = 0;

/* PID of M1 and M2 */
float kp1 = 17, kd1 = 33, P1 = 0, D1 = 0, z1 = 0, kp2 = 17, kd2 = 53, P2 = 0, D2 = 0, z2 = 0;
float e1 = 0, pe1 = 0, e2 = 0, pe2 = 0;
int pwm1 = 0, pwm2 = 0;

/* Received data from Mega */
unsigned char input_values[5], i = 0;
uint16_t desired_H_pos = 133, desired_V_pos = 0;
int desired_rpm = 0;
unsigned char checksum = 0xAE, header_byte = 0x57;

/* Encoder Counts */
int encoder_counts = 0;
unsigned char a0, b0, a1, b1;

/* Actuator direction */
void set_actuator_dir(int d) {
  if (d > 0) {
    PORTL |= (1 << actuator_dir); //Up / Increase length
  }
  if (d < 0) {
    PORTL &= ~(1 << actuator_dir); //Down / Decrease length
  }}
/* PWM to actuator based on feedback position */
void set_actuator (uint16_t desired_deg) {
  if (desired_deg <= 3) desired_deg = 0; //for compensating offset
  else desired_deg = desired_deg - actuator_offset;
  diff_counts = (int)((desired_deg * scaling_factor) - encoder_counts);
  set_actuator_dir(diff_counts);
  if ((abs(diff_counts)) > margin_of_error) {
    OCR5B = set_ocr;
  }
  else {
    OCR5B = 0;
  }}

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
void rpm_constrain(int s) {
  z1 = constrain(z1, -100, 100);
  z2 = constrain(z2, -100, 100);
}

void input_constrain () {
  desired_H_pos = constrain(desired_H_pos, 0, 1600);
  desired_V_pos = constrain(desired_V_pos, 0, 45);
  desired_rpm = constrain(desired_rpm, 0, 2100);
}

void pwm_constrain() {
  pwm1 = constrain(pwm1, 0, 2100);
  pwm2 = constrain(pwm2, 0, 2100);
}

void get_values_BOH () {
  step_mega = input_values[0];
  act_mega = input_values[1];
  rpm_mega = ((input_values[2]) | (input_values[3] << 8));
}

/* BTM - Stepper direction and pulse */
void set_pos(int pos) {
//  if (pos <= abs(BTM_offset)) {
//    pos = 0;
//  }
//  else {
//    pos = ((pos + BTM_offset) * gear_ratio * steps_per_deg)+60;
//  }
    pos=pos*4;
  if (pos > prev_pos) {
    PORTH |= (1 << BTM_dir);           //clockwise
  }
  else {
    PORTH &= ~(1 << BTM_dir);          //anticlockwise
  }
  p = (prev_pos - pos);
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
    }}
  else if (pulse == 0)
  {
    TIMSK3 &= ~(1 << OCIE3A);
  }}

/* For Encoder Counts (Feedback of Linear actuator (vertical angle)) */
/* encoder_counts++ -> Increase in actuator length (Up)
   encoder_counts-- -> Decrease in actuator length (Down) */
ISR (PCINT1_vect)
{
  //  a0 = ~PINB & 0x01; //encoder_signal_1 (PB0)
  //  b0 = ~PINB & 0x04; //encoder_signal_2 (PB2)
  a0 = ~PINJ & 0x01; //encoder_signal_1 (PJ0)
  b0 = ~PINJ & 0x02; //encoder_signal_2 (PJ1)
  if (a0 != a1) {
    if (a0) {
      if (b0) encoder_counts++;
      else encoder_counts--;
    }
    else {
      if (b0) encoder_counts--;
      else encoder_counts++;
    }}
  else if (b0 != b1) {
    if (b0) {
      if (a0) encoder_counts--;
      else encoder_counts++;
    }
    else {
      if (a0) encoder_counts++;
      else encoder_counts--;
    }}
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
  if (i < 2) {
    if (UDR1 == 0x57) {
      i++;
    }
    else {
      i = 0;
    }}
    else {
      if (i < 6) {
        input_values[i - 2] = UDR1;
        checksum += input_values[i - 2];
        i++;
      }
      else {
        if (checksum == UDR1) {
          step_mega = input_values[0];
          act_mega = input_values[1];
          rpm_mega = ((input_values[2]) | (input_values[3] << 8));
        }
        checksum = 0xAE;
        i = 0;
      }}}
//  }
//  if (i < 7) {
//    input_values[i] = UDR1;
//    if (i < 6) {
//      checksum += input_values[i];
//    }
//  }
//  i++;
//  if (i > 6) {
//    i = 0;
//  }


//ISR(USART2_RX_vect) {         //tfmini (for coding guide)
//  if (i < 2) {
//    if (UDR2 == 0x59) {
//      i++;
//    }
//    else {
//      i = 0;
//    }
//  }
//  else {
//    if (i < 8) {
//      tf_data[i - 2] = UDR2;
//      checksum += tf_data[i - 2];
//      i++;
//    }
//    else {
//      check = UDR2;
//      if (checksum == check) {
//        dist = tf_data[0] + tf_data[1] * 256;
//        if(dist > 65000)dist=0;
//        checksum = 0xB2;
//      }
//      i = 0;}}}


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
