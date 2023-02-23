#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define pulse_pin PB4   //stepper_1 pulse pin 10
#define limit_pin PL3   //stepper_1 limit pin 46
#define limit_pin2 PL1  //stepper_2 limit pin 48
#define dir PH6       //stepper_1 direction pin 9
#define pulse_pin2 PB5   //stepper_2 pulse pin 11
#define dir2 PH5         //stepper_2 direction pin 8
#define pi 3.1415

int step_t = 3200, step_c = 0 , target_step = 0 , target_step2 = 0;
uint8_t cnt = 0;
char tf_data[4];
uint16_t distance = 0, distance_2 = 0, prev_dist = 0;
float target_dist = 0;
int pulse = 0, prev_pos = 0, c = 0, a = 0, pulse2 = 0, prev_pos2 = 0;
unsigned int value = 0, steps = 3200;
float phi = 0, x = 39, phi_rad = 0, theta = 0, theta_rad = 0 , b = 0;

int main() {
  DDRL &= ~((1 << limit_pin) | (1 << limit_pin2));
  DDRB |= ((1 << pulse_pin) | (1 << pulse_pin2));
  DDRH |= ((1 << dir) | (1 << dir2));
  PORTL |= ((1 << limit_pin) | (1 << limit_pin2));
  Serial_int_setup();
  Serial.begin(115200);
  timer_setup(500);
  while ((PINL & (1 << limit_pin)) ) {
    PORTH &= ~(1 << dir);
    PORTB ^= (1 << pulse_pin);
    _delay_us(500);
  }

  while ((PINL & (1 << limit_pin2))) {
    PORTH &= ~(1 << dir2);
    PORTB ^= (1 << pulse_pin2);
    _delay_us(500);
  }
  prev_dist = distance;
  prev_pos = 0;
  prev_pos2 = 0;
  _delay_ms(1000);

  while (1) {
        Serial.print(theta);
        Serial.print("    ");
        Serial.print(phi);
        Serial.print("    ");
        Serial.print(distance);
        Serial.print("    ");
        Serial.print(distance_2);
        Serial.print("    ");
        Serial.print(target_dist);
        Serial.print("    ");
    Serial.println(abs(distance_2 - target_dist));
    if (a == 0) {
      if (prev_pos == 0)
      {
        set_pos(1300);
      }
      else {
        angle_calculation(target_step, value);
        set_pos2(target_step2 + 130); //110 edge  //130 mid  : offsets
        //        calibrate();
        if (prev_pos == 1300) {
          a = 1;
          //          set_pos(target_step);
        }
      }
    }
    //      else if (a == 1)
    //      {
    //        if (prev_pos == 1300)
    //        {
    //          set_pos(0);
    //        }
    //        else {
    //          targetStep();
    //          angle_calculation(target_step, value);
    //          set_pos2(target_step2 + 116);  //offset
    //          if (prev_pos == 0) {
    //            a = 2;
    ////            set_pos(target_step);
    //          }
    //        }
    //      }
  }
}
void targetStep()
{
  //  if (abs(prev_dist - target_dist) > 30){
  //    prev_dist = 10000;
  //  }
  if ((distance < prev_dist) && (distance < 10000) && (distance > 200))
  {
    prev_dist = distance;
    target_step = (prev_pos - 35);
    value = prev_dist;
  }
}

void set_pos(int pos) {
  if (pos < prev_pos) {
    PORTH &= ~(1 << dir);
  }
  else {
    PORTH |= (1 << dir);
  }
  pulse = abs(prev_pos - pos);
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

int calculate_pulse(float a) {
  return 3200 * a / 360.0;
}

ISR(USART3_RX_vect) {
  c++;
  targetStep();
  measurement();
}

void measurement() {
  int check;
  unsigned char uart[9];
  const int HEADER = 0x59;
  if (uread() == HEADER)
  {
    uart[0] = HEADER;
    check += uart[0];
    if (uread() == HEADER)
    {
      uart[1] = HEADER;
      check += uart[1];
      for (int i = 2; i < 8; i++)
      {
        uart[i] = uread();
        check += uart[i];
      }
      uart[8] = uread();
      distance = (uart[2] + uart[3] * 256);
    }
  }
}

void measurement_2() {
  int check;
  unsigned char uart[9];
  const int HEADER = 0x59;
  if (uread_2() == HEADER)
  {
    uart[0] = HEADER;
    check += uart[0];

    if (uread_2() == HEADER)
    {
      uart[1] = HEADER;
      check += uart[1];
      for (int i = 2; i < 8; i++)
      {
        uart[i] = uread_2();
        check += uart[i];
      }
      uart[8] = uread_2();
      distance_2 = (uart[2] + uart[3] * 256);
    }
  }
}

ISR(USART2_RX_vect) {
  measurement_2();
}

ISR(TIMER1_COMPA_vect)
{
  if (pulse > 0) {
    PORTB ^= (1 << pulse_pin);
    if ((PINB & 0x10) == 0) {
      pulse--;
      if ((PINH & 0x40) == 0x40) {
        prev_pos++;
      }
      else prev_pos--;
    }
  }
  else if (pulse == 0)
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
  if (a == 1) {
    error_compensate(abs(distance_2 - target_dist));
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

void error_compensate(float error) {
  if (error > 10) {
    set_pos2 (target_step2 + 10);
    error = abs(distance_2 - target_dist);
    error_compensate(error);
    Serial.println("no");
  }
  Serial.println("YES");
}
