#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define pulse_pin1 PB4   //stepper_1 pulse pin 10
#define dir1 PH6       //stepper_1 direction pin 9
#define limit_pin1 PL3   //stepper_1 limit pin 46
#define pulse_pin2 PB5   //stepper_2 pulse pin 11
#define dir2 PH5         //stepper_2 direction pin 8
#define limit_pin2 PL1  //stepper_2 limit pin 48
#define pi 3.1415

int step_t = 3200, step_c = 0 , target_step1 = 0 , target_step2 = 0, i = 0, j = 0;
uint8_t cnt = 0;
char tf_data1[4], tf_data2[4];
uint16_t dist1 = 0, dist2 = 0, prev_dist1 = 0, prev_dist2 = 0;
float target_dist = 0;
int pulse1 = 0, prev_pos1 = 0, c = 0, a = 0, pulse2 = 0, prev_pos2 = 0;
unsigned int min_value1 = 0, steps = 3200;
float phi = 0, x = 39, phi_rad = 0, theta = 0, theta_rad = 0 , b = 0;

int main() {
  DDRL &= ~((1 << limit_pin1) | (1 << limit_pin2));
  DDRB |= ((1 << pulse_pin1) | (1 << pulse_pin2));
  DDRH |= ((1 << dir1) | (1 << dir2));
  PORTL |= ((1 << limit_pin1) | (1 << limit_pin2));
  Serial_int_setup();
  Serial.begin(115200);
  timer_setup(500);
  while ((PINL & (1 << limit_pin1))  | (PINL & (1 << limit_pin2))) {  //while both limit pin are high
    PORTH &= ~((1 << dir1) | (1 << dir2));                            //clock wise
    if (PINL & (1 << limit_pin1)) {                                   //if pin is high
      PORTB ^= (1 << pulse_pin1);
    }
    else {
      PORTB &= ~(1 << pulse_pin1);
    }
    if (PINL & (1 << limit_pin2)) {
      PORTB ^= (1 << pulse_pin2);
    }
    else {
      PORTB &= ~(1 << pulse_pin2);
    }
    _delay_us(500);
  }
  prev_dist1 = dist1;
  prev_pos1 = 190;
  prev_pos2 = 0;
  _delay_ms(1000);
  //  while ((PINL & (1 << limit_pin2))) {
  //    PORTH &= ~(1 << dir2);
  //
  //    _delay_us(500);
  //  }

  while (1) {
    Serial.print(theta);
    Serial.print("    ");
    Serial.print(phi);
    Serial.print("    ");
    Serial.print(dist1);
    Serial.print("    ");
    Serial.print(dist2);
    Serial.print("    ");
     Serial.print(prev_pos1);
    Serial.print("    ");
    Serial.print(prev_pos2);
    Serial.print("    ");
    Serial.print(target_dist);
    Serial.print("    ");
    Serial.println(min_value1);
    if (a == 0) {
      if (prev_pos1 == 190)
      {
        set_pos1(1300);
        prev_dist1 = 10000;
      }
      else {
        if (prev_pos1 == 1300) {
          angle_calculation(target_step1, min_value1);
          set_pos2(target_step2 - 130); //130
          set_pos1(target_step1);
          a = 1;
        }
      }
    }
    else if (a == 1)
    {
      if (prev_pos1 == 1300)
      {
        set_pos1(190);
        prev_dist1 = 10000;
      }
      else {
        if (prev_pos1 == 190) {
          a = 0;
          angle_calculation(target_step1, min_value1);
          set_pos2(target_step2); //116
        }
      }
    }
  }
}


//void targetStep()
//{
//  if ((distance < prev_dist) && (distance < 10000) && (distance > 200))
//  {
//    prev_dist = distance;
//    target_step = (prev_pos - 35);
//    value = prev_dist;
//  }
//}

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

void Serial_int_setup() {
  cli();
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
  OCR3A = pulse_width(w);
  OCR1A = pulse_width(w);
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  sei();
}

int pulse_width(int width) {
  return (width * 2) - 1;
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
    tf_data1[i - 2] = UDR3;
    i++;
    if (i > 5) {
      dist1 = tf_data1[0] + tf_data1[1] * 256;
      if ((dist1 < prev_dist1) && (dist1 < 10000) && (dist1 > 200))
      {
        prev_dist1 = dist1;
        target_step1 = prev_pos1;
        min_value1 = prev_dist1;
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
    tf_data2[j - 2] = UDR2;
    j++;
    if (j > 5) {
      dist2 = tf_data2[0] + tf_data2[1] * 256;
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

char uread() {
  while (!(UCSR3A & (1 << RXC3)));
  return UDR3;
}

char uread_2() {
  while (!(UCSR2A & (1 << RXC2)));
  return UDR2;
}

void angle_calculation (float steps, unsigned int d) {
  theta = (steps * 0.1125);
  theta_rad = (theta * 0.0174);
  b = (pi - acos(x / d));
  if ((theta_rad > b) && (theta_rad < pi)) {
    phi_rad = pi - atan((d * sin(pi - theta_rad)) / (d * cos(pi - theta_rad) - x));
    target_dist = ((d * sin(pi - theta_rad)) / sin(pi - phi_rad));
  }
  else if ((theta_rad > (pi / 2)) &&  (theta_rad <= b)) {
    phi_rad = atan((d * sin(pi - theta_rad)) / (x - d * cos(pi - theta_rad)));
    target_dist = ((d * sin(pi - theta_rad)) / sin(phi_rad));
  }
  else if ((theta_rad > 0) && (theta_rad <= (pi / 2))) {
    phi_rad = atan((d * sin(theta_rad)) / (x + (d * cos(theta_rad))));
    target_dist = ((d * sin(theta_rad)) / sin(phi_rad));
  }
  phi  = (phi_rad * 57.3248);
  target_step2 = (phi * 8.88888); //8.88888 = 1600/180
}
