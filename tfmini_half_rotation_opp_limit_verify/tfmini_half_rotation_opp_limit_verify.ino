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

int step_t = 3200, step_c = 0 , target_step1 = 0 , target_step2 = 0, i = 0, j = 0, dirf = 1, set = 0, tf1n = 0, tf2n = 0;
bool tf1 = 0, tf2 = 0;
long a2 = 0, b2 = 0;
uint8_t cnt = 0;
unsigned char tf_data1[7], tf_data2[7], checksum1 = 0xB2, check1 = 0, checksum2 = 0xB2, check2 = 0;
uint16_t dist1 = 0, dist2 = 0, prev_dist1 = 0, prev_dist2 = 0;
double target_dist = 0, target_dist1 = 0, target_dist2 = 0;
int pulse1 = 0, prev_pos1 = 0, c = 0, a = 0, pulse2 = 0, prev_pos2 = 0, pos1 = 0, pos2 = 0;
unsigned int value1 = 0, steps = 3200, flag = 0 ;
double phi1 = 0, phi_rad1 = 0, theta1 = 0, theta_rad1 = 0 , phi2 = 0,  phi_rad2 = 0, theta2 = 0, theta_rad2 = 0;

int main() {

  DDRL &= ~((1 << limit_pin1) | (1 << limit_pin2));
  DDRB |= ((1 << pulse_pin1) | (1 << pulse_pin2));
  DDRH |= ((1 << dir1) | (1 << dir2));
  PORTL |= ((1 << limit_pin1) | (1 << limit_pin2));

  Serial.begin(115200);
  timer_setup(1000);
  while ((PINL & (1 << limit_pin1))  | (PINL & (1 << limit_pin2))) { //if either of the limit pin is not pressed
    PORTH &= ~((1 << dir1)|(1 << dir2));         //clockwise
//    Serial.print(digitalRead(46));
//    Serial.print("   ");
//    Serial.println(digitalRead(48));
    if (PINL & (1 << limit_pin1)) {                // if limit pin 1 is pressed
      PORTB ^= (1 << pulse_pin1);                  //
    }
    else {
      PORTB &= ~(1 << pulse_pin1);                 //
    }
    if (PINL & (1 << limit_pin2)) {
      PORTB ^= (1 << pulse_pin2);
    }
    else {
      PORTB &= ~(1 << pulse_pin2);
    }
    _delay_us(500);
  }
  prev_dist1 = 65535;
  prev_dist2 = 65535;
  prev_pos1 = 0;
  prev_pos2 = 0;
  Serial_int_setup();
  _delay_ms(1000);
  while (1) {
//     Serial.print(digitalRead(46));
//    Serial.print("   ");
//    Serial.println(digitalRead(48));
    print_dist();
//    set_stepper();
  }
}

void min_dist2()
{
  if (dist2 == 0)dist2 = 65535;
  if (dist2 < prev_dist2)
  {
    prev_dist2 = dist2;
    target_step2 = prev_pos2;
  }
}
void min_dist1()
{
  if (dist1 == 0)dist1 = 65535;
  if (dist1 < prev_dist1)
  {
    prev_dist1 = dist1;
    target_step1 = prev_pos1;
  }
}

void set_pos1(int pos) {
//  Serial.println("yes");
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
void print_dist () {
  Serial.print(set);
  Serial.print("      ");
  Serial.print(prev_pos2);
   Serial.print("      ");
  Serial.println(prev_pos1);
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
        tf1 = 1;
        min_dist1();
      }
      i = 0;
    }
  }
}

void set_stepper() {
    set += 8 * dirf;    // 8 = 0.9 *8.8889    set is the step incrimented per reading
    set_pos1(set);      //setting both steppers at incrimented count
    set_pos2(set);
    if (set == 1000){
      dirf = -1;
//      Serial.println( "yes");
    }
    if (set == 0){
      dirf = 1;
    }
//    Serial.println( dirf);
    _delay_us(40);
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
          min_dist2();
          tf1 = 0;
          
        }
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

char uread() {
  while (!(UCSR3A & (1 << RXC3)));
  return UDR3;
}

char uread_2() {
  while (!(UCSR2A & (1 << RXC2)));
  return UDR2;
}

void angle_calculation1 (float steps, float d) {
  //  if (!d)d = 10000.0;
  //d-=1.5;
  theta1 = (steps * 0.1125);
  theta_rad1 = (theta1 * 0.0174);
  target_dist1 = sq(d + 3.6) + sq(40.3) - ((d + 3.6) * 80.6 * cos(theta_rad1));
  target_dist1 = sqrt(target_dist1);
  phi_rad1 = asin(sin(theta_rad1) * (target_dist1 / (d + 3.6)));
  phi1  = (phi_rad1 * 57.3248) + 1.0;
}

void angle_calculation2(float steps, float d) {
  //  if (!d)d = 10000.0;
  //  d-=2.5;
  theta2 = (steps * 0.1125);
  theta_rad2 = (theta2 * 0.0174);
  target_dist2 = sq(d + 4.0) + sq(40.3) - ((d + 4.0) * 2.0 * 40.3 * cos(theta_rad2));
  target_dist2 = sqrt(target_dist2);
  phi_rad2 = asin(sin(theta_rad2) *  (target_dist2 / (d + 4.0)));
  phi2  = (phi_rad2 * 57.3248);
}

//void compare() {
  //  if (target_dist2 > target_dist1){
  //    target_dist = target_dist1;
  //
  //  }
  //    else{
  //    target_dist = target_dist2;
  //
  //  }
//}
