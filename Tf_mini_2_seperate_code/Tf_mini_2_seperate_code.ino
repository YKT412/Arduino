#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define limit_pin2 PL3  //PIN 48
#define pulse_pin2 PB5  //PIN 11
#define dir2 PH5        //PIN 8

bool tf2 = 0;
int8_t dirf = 1;
uint8_t j = 0, tf_data2[7], checksum2 = 0xB2, check2 = 0;
uint16_t dist2 = 0, prev_dist2 = 0, min_step2 = 0, pulse2 = 0;
int16_t prev_pos2 = 0, pos2 = 0;
uint16_t set = 0;
double target_dist2 = 0, phi2 = 0, phi_rad2 = 0, theta2 = 0, theta_rad2 = 0;
double dist_of_sep = 7.65, offset1 = 4.0, offset2 = 4.4, pi = 3.1415;

int main() {
  DDRL &= ~(1 << limit_pin2);
  DDRB |= (1 << pulse_pin2);
  DDRH |= (1 << dir2);
  PORTL |= (1 << limit_pin2);
  Serial.begin(115200);
  timer_setup(500);
//  Serial.println(digitalRead(46));
    while (!(PINL & (1 << limit_pin2))) {
    PORTH &= ~(1 << dir2);
      PORTB ^= (1 << pulse_pin2);
      _delay_us(500);
//      Serial.println(digitalRead(46));
    }
  prev_dist2 = 65535;
  prev_pos2 = 0;
  _delay_ms(2000);
  Serial_int_setup();
  while (1) {

    Serial.print(prev_dist2);
     Serial.print("   ");
    Serial.print(target_dist2);
    Serial.print("   ");
     Serial.println(phi2);
    
//   Serial.println(digitalRead(46));
  }
}

void Serial_int_setup() {
  cli();
//  UBRR1L = 8;
//  UBRR1H = 0x00;
//  UCSR1B = 0x38;
//  UCSR1C = 0x06;
  UBRR2L = 0x08;
  UCSR2C = (1 << UCSZ20) | (1 << UCSZ21);
  UCSR2B = (1 << RXEN2) | (1 << RXCIE2);
  sei();
}

void timer_setup(int w) {
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = pulse_width(w);
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  sei();
}

int pulse_width(int width) {
  return (width * 2) - 1;
}

void set_stepper() {
  set += 8 * dirf;    // 8 = 0.9 *8.8889    set is the step incrimented per reading
  set_pos2(set);      //setting both steppers at incrimented count
  if (set == 888) {
    dirf = (-1);
    angle_calculation2((1600 - min_step2), prev_dist2);
    prev_dist2 = 65535;
  }
  if (set == 0) {
    dirf = 1;
    angle_calculation2((1600 - min_step2), prev_dist2);
    prev_dist2 = 65535;
  }
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

void angle_calculation2(float steps, float d) {
  theta2 = (steps * 0.1125);
  theta_rad2 = (theta2 * 0.0174);
  target_dist2 = sq(d + offset2) + sq(dist_of_sep) - ((d + offset2) * 2.0 * dist_of_sep * cos(theta_rad2));
  target_dist2 = sqrt(target_dist2);
  phi_rad2 = asin(((sin(theta_rad2) * (d + offset2)) / target_dist2));
  phi2  = (phi_rad2 * 57.3248);
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
          set_stepper();
          min_dist2();
      }
      j = 0;
    }
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
