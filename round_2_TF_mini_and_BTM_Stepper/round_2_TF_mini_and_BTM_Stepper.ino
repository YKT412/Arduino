#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//1 -> stepper to the right of BTM
//2 -> stepper to the left of BTM
#define limit_pin1 PL1  //PIN 46
#define limit_pin2 PL3  //PIN 48
#define limit_pin3 PL5  //PIN 44
#define pulse_pin1 PB4  //PIN 10
#define pulse_pin2 PB5  //PIN 11
#define pulse_pin3 PB7  //PIN 7
#define dir1 PH6        //PIN 9
#define dir2 PH5        //PIN 8
#define dir3 PB6        //PIN 6
uint8_t flag_temp = 1;
uint8_t c = 0, check = 2, m = 0, n = 0, p = 0, q = 0, flag_for_angle = 0, min_dist_flag = 0;
uint16_t min_dist = 1200, min_steps = 0, deg = 0, med_steps = 0, f = 0, sum_min_steps = 0;
uint16_t prev_pos1 = 0, prev_pos2 = 0, prev_pos3 = 0, pulse1 = 0, pulse2 = 0, pulse3 = 0, min_steps2 = 0, min_steps1 = 0;
uint16_t dist1 = 0, min_dist1 = 65535, dist2 = 0, min_dist2 = 65535, prev_min_dist = 65535;
unsigned char i = 0, tf_data1[7], check1 = 0, checksum1 = 0xB2;
unsigned char j = 0, tf_data2[7], check2 = 0, checksum2 = 0xB2;
float target_dist = 0, target_angle = 0, dist_of_sep = 15, offset = 4.2, rad = 0, a = 1, b = 1;
uint16_t lower_range_1 = 0, lower_range_2 = 0, upper_range_1 = 1300, upper_range_2 = 1300;
int16_t diff1 = 0, diff2 = 0, p1 = 0, p2 = 0, p3 = 0;
int main() {
  DDRL &= ~((1 << limit_pin1) | (1 << limit_pin2) | (1 << limit_pin3));
  PORTL |= ((1 << limit_pin1) | (1 << limit_pin2) | (1 << limit_pin3));
  DDRB |= ((1 << pulse_pin1) | (1 << pulse_pin2) | (1 << dir3) | (1 << pulse_pin3));
  DDRH |= ((1 << dir1) | (1 << dir2) );
  Serial_int_setup();
  timer_setup(700, 300, 700);  // 1st , 2nd , 3rd
  Serial.begin(115200);
  while ((!(PINL & (1 << limit_pin1)))  | (!(PINL & (1 << limit_pin2))) | ((PINL & (1 << limit_pin3)))) {
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
    if ((PINL & (1 << limit_pin3))) {
      PORTB &= ~(1 << dir3);
      PORTB ^= (1 << pulse_pin3);
    }
    else {
      PORTB &= ~(1 << dir3);
      PORTB &= ~(1 << pulse_pin3);
    }
    _delay_us(500);
  }
  c = 60;
  while (c > 0) {
    PORTH |= (1 << dir1);   // anticlockwise
    PORTB ^= (1 << pulse_pin1);
    _delay_us(500);
    c--;
  }
  min_dist1 = 900;
  min_dist2 = 900;
  min_steps = 0;
  //  pulse1 = 0;
  //  pulse2 = 0;
  prev_pos1 = 0;
  prev_pos2 = 0;
  prev_pos3 = 0;
  _delay_ms(2000);
  //  while(1){
  //    set_pos3(800);
  ////    Serial.println(digitalRead(44));
  //  }

  while (1) {
    if (min_dist_flag < 7) {
      if (min_dist1 < min_dist2) {
        min_dist = min_dist1;
        min_steps = min_steps1;
        flag_for_angle = 0;
      }
      else {
        min_dist = min_dist2;
        min_steps = min_steps2;
        flag_for_angle = 1;
      }
    }
    else {
      if (min_dist1 < min_dist2) {
        min_dist = min_dist1;
        min_steps = min_steps1;
        flag_for_angle = 0;
      }
      else {
        min_dist = min_dist2;
        min_steps = min_steps2;
        flag_for_angle = 1;
      }
      Serial.println(min_dist);
    }

    if (prev_pos1 == upper_range_1) {
      if (m == 0) {
        UCSR3B = (1 << RXEN3) | (1 << RXCIE3);
        lower_range_1 = 450;
        set_pos1(lower_range_1);
        m = 1;
        n = 0;
        med_steps = 0;
//        if (min_dist_flag > 8) {
//          min_dist1 = min_dist;
//          min_dist2 = min_dist;
//          angle_calculation((1600 - min_steps), min_dist);
//          //          set_pos3(target_angle*8.889);
//          Serial.println(min_dist);
//        }
      }
    }
    if (prev_pos1 == lower_range_1) {
      if (n == 0) {
        set_pos1(upper_range_1);
        n = 1;
        m = 0;
        //        if (min_dist_flag > 8){
        //          min_dist1 = min_dist;
        //          min_dist2 = min_dist;
        //           angle_calculation((1600 - min_steps), min_dist);
        ////          set_pos3(target_angle*8.889);
        //Serial.println(min_dist);
        //        }
      }
    }
    if (prev_pos2 == upper_range_2) {
      if (p == 0) {
        UCSR2B = (1 << RXEN2) | (1 << RXCIE2);
        set_pos2(lower_range_2);
        p = 1;
        q = 0;
        med_steps = 0;
        if (min_dist_flag <= 7) {
          min_dist_flag++;
        }
        if (min_dist_flag == 8) {
          angle_calculation((1600 - min_steps), min_dist);
          //          set_pos3(target_angle*8.889);
          Serial.println(min_dist);
        }
        if (min_dist_flag > 7) {
          min_dist1 = min_dist;
          min_dist2 = min_dist;
          angle_calculation((1600 - min_steps), min_dist);
          //          set_pos3(target_angle*8.889);
          Serial.println(min_dist);
        }
      }
    }
    if (prev_pos2 == lower_range_2) {
      if (q == 0) {
        lower_range_2 = 400;
        set_pos2(upper_range_2);
        q = 1;
        p = 0;
        //         Serial.println(min_dist);
        if (min_dist_flag > 7) {
          min_dist1 = min_dist;
          min_dist2 = min_dist;
          angle_calculation((1600 - min_steps), min_dist);
          //          set_pos3(target_angle*8.889);
          Serial.println(min_dist);
        }
      }
    }
  }
}

void Serial_int_setup() {
  cli();
  UBRR2L = 0x08;
  UBRR3L = 8;
  UBRR3H = 0x00;
  UCSR2C = (1 << UCSZ20) | (1 << UCSZ21);
  UCSR3B = 0x00;
  UCSR2B = 0x00;
  UCSR3C = 0x06;
  sei();
}

void timer_setup(int w1, int w2, int w3) {
  cli();
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = pulse_width(w1);
  OCR3A = pulse_width(w2);
  OCR4A = pulse_width(w3);
  TCCR4B |= (1 << WGM42);
  TCCR4B |= (1 << CS41);
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  sei();
}

int pulse_width(int width) {
  return (width * 2) - 1;
}

void set_pos1(int pos1) {
  if (pos1 > prev_pos1) {
    PORTH |= (1 << dir1);           //clockwise
  }
  else {
    PORTH &= ~(1 << dir1);          //anticlockwise
  }
  p1 = prev_pos1 - pos1;
  pulse1 = abs(p1);
  TIMSK1 |= (1 << OCIE1A);
}

void set_pos2(int pos2) {
  if (pos2 > prev_pos2) {
    PORTH |= (1 << dir2);           //clockwise
  }
  else {
    PORTH &= ~(1 << dir2);         //anticlockwise
  }
  p2 = prev_pos2 - pos2;
  pulse2 = abs(p2);
  TIMSK3 |= (1 << OCIE3A);
}

void set_pos3(int pos3) {
  if (pos3 > prev_pos3) {
    PORTB |= (1 << dir3);           //clockwise
  }
  else {
    PORTB &= ~(1 << dir3);          //anticlockwise
  }
  p3 = prev_pos3 - pos3;
  pulse3 = abs(p3);
  TIMSK4 |= (1 << OCIE4A);
}

void angle_calculation(float steps, float d) {
  rad = (steps * 0.00196);
  target_dist = sq(d + offset) + sq(dist_of_sep) - ((d + offset) * 2.0 * dist_of_sep * cos(rad));
  target_dist = sqrt(target_dist);
  target_angle = ((asin(((sin(rad) * (d + offset)) / target_dist))) * 57.3248);
  if (flag_for_angle == 1) {
    target_angle = 180 - target_angle;
  }
  else {
    target_angle = target_angle;
  }
}

ISR(USART3_RX_vect) {             //tfmini 1
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
        if (min_dist_flag < 7) {
          if (dist1 < min_dist1) {
            min_dist1 = dist1;
            min_steps1 = prev_pos1;
          }
        }
        else {
          diff1 = (min_dist - dist1);
          if ((abs(diff1) < 100) && (abs(diff1) >= 0) && (dist1 > 400) && (dist1 <= 900)) {
            min_dist1 = dist1;
            min_steps1 = prev_pos1;
          }
        }
      }
      i = 0;
    }
  }
}

ISR(USART2_RX_vect) {         //tfmini 2
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
        checksum2 = 0xB2;
        if (dist2 == 0) dist2 = 65535;
        if (min_dist_flag < 7) {
          if (dist2 < min_dist2) {
            min_dist2 = dist2;
            min_steps2 = prev_pos2;
          }
        }
        else {
          diff2 = (min_dist - dist2);
          if ((abs(diff2) < 50) && (abs(diff2) >= 0) && (dist2 <= 400)) {
            min_dist2 = dist2;
            min_steps2 = prev_pos2;
          }
        }
      }
      j = 0;
    }
  }
}

ISR(TIMER1_COMPA_vect)                     // for 1st stepper
{
  if (pulse1 > 0) {
    PORTB ^= (1 << pulse_pin1);
    if ((PINB & (1 << pulse_pin1)) == 0) {
      pulse1--;
      if ((PINH & (1 << dir1)) == 0x40) {
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

ISR(TIMER3_COMPA_vect) {                // for 2nd stepper
  if (pulse2 > 0) {
    PORTB ^= (1 << pulse_pin2);
    if ((PINB & (1 << pulse_pin2)) == 0) {
      pulse2--;
      if ((PINH & (1 << dir2)) == 0x20) {
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

ISR(TIMER4_COMPA_vect) {                      // for 3rd stepper
  if (pulse3 > 0) {
    PORTB ^= (1 << pulse_pin3);
    if ((PINB & (1 << pulse_pin3)) == 0) {
      pulse3--;
      if ((PINB & (1 << dir3)) == 0x40) {
        prev_pos3++;
      }
      else prev_pos3--;
    }
  }
  else if (pulse3 == 0)
  {
    TIMSK4 &= ~(1 << OCIE4A);
  }
}
