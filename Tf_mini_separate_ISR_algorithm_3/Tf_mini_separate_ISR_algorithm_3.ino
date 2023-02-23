#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//1 -> stepper to the right of BTM
//2 -> stepper to the left of BTM
#define limit_pin1 PL1  //PIN 46
#define limit_pin2 PL3  //PIN 48
#define pulse_pin1 PB4  //PIN 10
#define pulse_pin2 PB5  //PIN 11
#define dir1 PH6        //PIN 9
#define dir2 PH5        //PIN 8

bool dir_flag;
uint8_t c = 0, check = 2, a = 1;
uint16_t min_dist = 65535, min_steps = 0;
uint16_t prev_pos = 0, pulse = 0, min_steps2 = 0, min_steps1 = 0;
uint16_t dist1 = 0, min_dist1 = 65535, dist2 = 0, min_dist2 = 65535;
unsigned char i = 0, tf_data1[7], check1 = 0, checksum1 = 0xB2;
unsigned char j = 0, tf_data2[7], check2 = 0, checksum2 = 0xB2;
double target_dist = 0, target_angle = 0, dist_of_sep = 7.5, offset = 4.2;
int main() {
  DDRL &= ~((1 << limit_pin1) | (1 << limit_pin2));
  DDRB |= ((1 << pulse_pin1) | (1 << pulse_pin2));
  DDRH |= ((1 << dir1) | (1 << dir2));
  PORTL |= ((1 << limit_pin1) | (1 << limit_pin2));
  Serial_int_setup();
  timer_setup(500);
  Serial.begin(115200);
  while ((!(PINL & (1 << limit_pin1)))  | (!(PINL & (1 << limit_pin2)))) {
    if (!(PINL & (1 << limit_pin1))) {
      PORTH &= ~(1 << dir1);
      PORTB ^= (1 << pulse_pin1);
    }
    else {
      PORTH &= ~(1 << dir1);
      PORTB &= ~(1 << pulse_pin1);
    }
    if (!(PINL & (1 << limit_pin2))) {
      PORTH &= ~(1 << dir2);
      PORTB ^= (1 << pulse_pin2);
    }
    else {
      PORTH &= ~(1 << dir2);
      PORTB &= ~(1 << pulse_pin2);
    }
    _delay_us(500);
  }
  c = 80;
  while (c > 0) {
    PORTH |= (1 << dir1);
    PORTB ^= (1 << pulse_pin1);
    _delay_us(500);
    c--;
  }
  min_dist = 900;
  min_steps = 0;
  prev_pos = 0;
  _delay_ms(2000);
  while (1) {
    Serial.println(min_dist2);
    if (prev_pos == 845) {
//            find_min_dist();
            Serial.println(min_steps);
            angle_calculation(min_steps, min_dist);
      set_pos(0);
       min_dist2 = 900;
    }
    if (prev_pos == 0) {
//            find_min_dist();
            Serial.println(min_steps);
            angle_calculation(min_steps, min_dist);
      set_pos(845);
       min_dist2 == 900;
    }
  }
}

void Serial_int_setup() {
  cli();
  //  UBRR1L = 8;
  //  UBRR1H = 0x00;
  //  UCSR1B = 0x38;
  //  UCSR1C = 0x06;
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
  TCNT1 = 0;
  OCR1A = pulse_width(w);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  sei();
}

int pulse_width(int width) {
  return (width * 2) - 1;
}

void set_pos(int pos) {
  if (pos < prev_pos) {
    dir_flag = 0;
    //        PORTH &= ~((1 << dir1) | (1 << dir2));
  }
  else {
    dir_flag = 1;
    //        PORTH |= ((1 << dir1) | (1 << dir2));
  }
   if (pos < prev_pos2) {
     PORTH &= ~(1 << dir2);
   }
   else {
     PORTH |= (1 << dir2);
   }
   pulse1 = abs(prev_pos1 - pos);
  pulse = abs(prev_pos - pos);
  TIMSK1 |= (1 << OCIE1A);
}

void find_min_dist() {
 if (min_dist1 < min_dist2) {
   min_dist = min_dist1;
   min_steps = min_steps1;
 }
 else if (min_dist2 < min_dist1) {
   min_dist = min_dist2;
   min_steps = min_steps2;
 }
}
void angle_calculation(float steps, float d) {
  double rad = (steps * 0.00196);
  target_dist = sq(d + offset) + sq(dist_of_sep) - ((d + offset) * 2.0 * dist_of_sep * cos(rad));
  target_dist = sqrt(target_dist);
  target_angle = (asin(((sin(rad) * (d + offset)) / target_dist)) * 57.3248);
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
        checksum1 = 0xB2;
        //        if (dist1 < min_dist) {
        //          min_dist = dist1;
        //          min_steps = prev_pos;
        //          //                    check = 1;
        //        }
        if (dist1 < min_dist1) {
          min_dist1 = dist1;
          min_steps1 = prev_pos;
        }
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
        if (dist2 == 0) dist2 = 65535;
        checksum2 = 0xB2;
        //                if (dist2 < min_dist) {
        //                  min_dist = dist2;
        //                  min_steps = prev_pos;
        //                }
        //        check++;
        if (dist2 < min_dist2) {
          min_dist2 = dist2;
          min_steps2 = prev_pos;
        }
      }
      j = 0;
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  if (dir_flag == 0) PORTH &= ~((1 << dir1) | (1 << dir2));
  else PORTH |= ((1 << dir1) | (1 << dir2));

  if (pulse > 0) {
    PORTB ^= ((1 << pulse_pin1) | (1 << pulse_pin2));
    if ((PINB & (1 << pulse_pin1)) == 0) {
      pulse--;
      if ((PINH & 0x20) == 0x20) {
        prev_pos++;
      }
      else prev_pos--;
      //        Serial.println(prev_pos);
    }
  }
  else {
    TIMSK1 &= ~(1 << OCIE1A);
  }
  //    else if (pulse1 == 0)
  //    {
  //      PORTB &= ~(1 << pulse_pin1);
  //    }
  //  if (pulse2 > 0){
  //    PORTB ^= (1 << pulse_pin2);
  //    if ((PINB & 0x20) == 0){
  //      pulse2--;
  //      if ((PINH & 0x20) == 0x20){
  //        prev_pos2++;
  //      }
  //      else prev_pos2--;
  //    }
  //    }
  //    else if (pulse2 == 0)
  //  {
  //    PORTB &= ~(1 << pulse_pin2);
  //  }
  //  }
}

//ISR(USART1_UDRE_vect) {
//  UDR1 = theta;
//}
