#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//1 -> stepper to the right of BTM
//2 -> stepper to the left of BTM
#define limit_pin1 PL3  //PIN 46
#define limit_pin2 PL1  //PIN 48
#define pulse_pin1 PB4  //PIN 10
#define pulse_pin2 PB5  //PIN 11
#define dir1 PH6        //PIN 9
#define dir2 PH5        //PIN 8

bool tf1 = 0, tf2 = 0;
int8_t dirf = 1;
uint8_t i = 0, tf_data1[7], checksum1 = 0xB2, check1 = 0;
uint8_t j = 0, tf_data2[7], checksum2 = 0xB2, check2 = 0;
uint16_t dist1 = 0, prev_dist1 = 0, min_step1 = 0, pulse1 = 0;
int16_t prev_pos1 = 0, pos1 = 0, prev_pos2 = 0, pos2 = 0;
uint16_t dist2 = 0, prev_dist2 = 0, min_step2 = 0, pulse2 = 0;
uint16_t set = 0;
double target_dist = 0, theta = 0;
double target_dist1 = 0, phi1 = 0, phi_rad1 = 0, theta1 = 0, theta_rad1 = 0;
double target_dist2 = 0, phi2 = 0, phi_rad2 = 0, theta2 = 0, theta_rad2 = 0;
double dist_of_sep = 7.65, offset1 = 4.0, offset2 = 4.4, pi = 3.1415;
//unsigned char tf_data1[7], tf_data2[7], checksum1 = 0xB2, check1 = 0, checksum2 = 0xB2, check2 = 0;

int main() {
  DDRL &= ~((1 << limit_pin1) | (1 << limit_pin2));
  DDRB |= ((1 << pulse_pin1) | (1 << pulse_pin2));
  DDRH |= ((1 << dir1) | (1 << dir2));
  PORTL |= ((1 << limit_pin1) | (1 << limit_pin2));
  Serial.begin(115200);
  timer_setup(500);
  while ((!(PINL & (1 << limit_pin1)))  | (!(PINL & (1 << limit_pin2)))) {
    PORTH &= ~((1 << dir1) | (1 << dir2));
    if (!(PINL & (1 << limit_pin1))) {
      PORTB ^= (1 << pulse_pin1);
    }
    else {
      PORTB &= ~(1 << pulse_pin1);
    }
    if (!(PINL & (1 << limit_pin2))) {
      PORTB ^= (1 << pulse_pin2);
    }
    else {
      PORTB &= ~(1 << pulse_pin2);
    }
    _delay_us(500);
  }
//    while (!(PINL & (1 << limit_pin1))) {
//    PORTH &= ~(1 << dir1);
//      PORTB ^= (1 << pulse_pin1);
//      _delay_us(500);
//    }
//    while (!(PINL & (1 << limit_pin2))) {
//    PORTH &= ~(1 << dir2);
//      PORTB ^= (1 << pulse_pin2);
//      _delay_us(500);
//    }
  prev_dist1 = 65535;
  prev_dist2 = 65535;
  prev_pos1 = -40;
  prev_pos2 = 0;
  _delay_ms(2000);
  Serial_int_setup();
  while (1) {
//    Serial.print(target_dist1);
//    Serial.print("      ");
//    Serial.print(phi1);
//    Serial.print("      ");
    Serial.print(prev_dist1);
    Serial.print("      ");
    Serial.print(prev_dist2);
    Serial.print("      ");
    Serial.print(target_dist);
    Serial.print("      ");
    Serial.println(theta);
  }
}

void Serial_int_setup() {
  cli();
  UBRR1L = 8;
  UBRR1H = 0x00;
  UCSR1B = 0x38;
  UCSR1C = 0x06;
  UBRR2L = 0x08;
  UBRR3L = 8;
  UBRR3H = 0x00;
  UCSR2C = (1 << UCSZ20) | (1 << UCSZ21);
  UCSR2B = (1 << RXEN2) | (1 << RXCIE2);
  UCSR3B = 0x98;
  UCSR3C = 0x06;
  sei();
}

void timer_setup(int w) {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT1 = 0;
  TCNT3 = 0;
  OCR1A = pulse_width(w);
  OCR3A = pulse_width(w);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  sei();
}

int pulse_width(int width) {
  return (width * 2) - 1;
}

void set_stepper() {
  
  set += 8 * dirf;    // 8 = 0.9 *8.8889    set is the step incrimented per reading
  set_pos1(set);      //setting both steppers at incrimented count
  set_pos2(set);
  if (set == 888) {
    dirf = (-1);
    angle_calculation1((1600 - min_step1), prev_dist1);
    angle_calculation2((1600 - min_step2), prev_dist2);
    compare();
    prev_dist1 = 65535;
    prev_dist2 = 65535;
  }
  if (set == 0) {
    dirf = 1;
    angle_calculation1((1600 - min_step1), prev_dist1);
    angle_calculation2((1600 - min_step2), prev_dist2);
    compare();
    prev_dist1 = 65535;
    prev_dist2 = 65535;
  }
}

void set_pos1(int pos) {
  if (pos < prev_pos1) {
    PORTH &= ~(1 << dir1);
  }
  else {
    PORTH |= (1 << dir1);
  }
  pulse1 = abs(prev_pos1 - pos);
  TIMSK1 |= (1 << OCIE1A);
}

void set_pos2(int pos) {
  if (pos < prev_pos2) {
    PORTH &= ~(1 << dir2);
  }
  else {
    PORTH |= (1 << dir2);
  }
  pulse2 = abs(prev_pos2 - pos);
  TIMSK3 |= (1 << OCIE3A);
}

void min_dist2()
{
  if (dist2 == 0)dist2 = 65535;
  if (dist2 < prev_dist2)
  {
    prev_dist2 = dist2;
    min_step2 = prev_pos2;
  }
}

void min_dist1()
{
  if (dist1 == 0)dist1 = 65535;
  if (dist1 < prev_dist1)
  {
    prev_dist1 = dist1;
    min_step1 = prev_pos1;
  }
}

void angle_calculation1 (float steps, float d) {
  theta1 = (steps * 0.1125);
  theta_rad1 = (theta1 * 0.0174);
  target_dist1 = sq(d + offset1) + sq(dist_of_sep) - ((d + offset1) * 2.0 * dist_of_sep * cos(theta_rad1));
  target_dist1 = sqrt(target_dist1);
  phi_rad1 = asin(((sin(theta_rad1) * (d + offset1)) / target_dist1));
  phi1  = (phi_rad1 * 57.3248); //+1.0
}

void angle_calculation2(float steps, float d) {
  theta2 = (steps * 0.1125);
  theta_rad2 = (theta2 * 0.0174);
  target_dist2 = sq(d + offset2) + sq(dist_of_sep) - ((d + offset2) * 2.0 * dist_of_sep * cos(theta_rad2));
  target_dist2 = sqrt(target_dist2);
  phi_rad2 = asin(((sin(theta_rad2) * (d + offset2)) / target_dist2));
  phi2  = (phi_rad2 * 57.3248);
}

void compare() {
  if (target_dist2 > target_dist1) {
    target_dist = target_dist1;
    theta = phi1;
  }
  else {
    target_dist = target_dist2;
    theta = (180.0 - phi2);
  }
}

ISR(USART3_RX_vect) {
  if (i < 2) {
    if (UDR3 == 0x59) {
      i++;
    }
    else {
      i = 0;
    }
  }
  else {
    if (i < 8) {
      tf_data1[i - 2] = UDR3;
      checksum1 += tf_data1[i - 2];
      i++;
    }
    else {
      check1 = UDR3;
      if (checksum1 == check1) {
        dist1 = tf_data1[0] + tf_data1[1] * 256;
        pos1 = prev_pos1;
        checksum1 = 0xB2;
        //        if (tf2 == 1) {
        //          set_stepper();
        //          tf2 = 0;
        //          min_dist1();
        //        }
        tf1 = 1;
        min_dist1();
      }
      i = 0;
    }
  }
}

ISR(USART2_RX_vect) {
  if (j < 2) {
    if (UDR2 == 0x59) {
      j++;
    }
    else {
      j = 0;
    }
  }
  else {
    if (j < 8) {
      tf_data2[j - 2] = UDR2;
      checksum2 += tf_data2[j - 2];
      j++;
    }
    else {
      check2 = UDR2;
      if (checksum2 == check2) {
        dist2 = tf_data2[0] + tf_data2[1] * 256;
        pos2 = prev_pos2;
        checksum2 = 0xB2;
        if (tf1 == 1) {
          set_stepper();
          tf1 = 0;
        }
        min_dist2();
      }
      j = 0;
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  if (pulse1 > 0) {
    PORTB ^= (1 << pulse_pin1);
    if ((PINB & 0x10) == 0) {
      pulse1--;
      if ((PINH & 0x40) == 0x40) {
        prev_pos1++;
      }
      else prev_pos1--;
    }
  }
  else if (pulse1 == 0)
  {
    TIMSK1 &= ~(1 << OCIE1A);
  }
}

ISR(TIMER3_COMPA_vect) {
  if (pulse2 > 0) {
    PORTB ^= (1 << pulse_pin2);
    if ((PINB & 0x20) == 0) {
      pulse2--;
      if ((PINH & 0x20) == 0x20) {
        prev_pos2++;
      }
      else prev_pos2--;
    }
  }
  else if (pulse2 == 0)
  {
    TIMSK3 &= ~(1 << OCIE3A);
  }
}

ISR(USART1_UDRE_vect) {
  UDR1 = theta;
}

//char uread() {
//  while (!(UCSR3A & (1 << RXC3)));
//  return UDR3;
//}
//
//char uread_2() {
//  while (!(UCSR2A & (1 << RXC2)));
//  return UDR2;
//}
