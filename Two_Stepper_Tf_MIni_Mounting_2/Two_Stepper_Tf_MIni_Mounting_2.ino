
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define pulse_pin PB4   //stepper_1 pulse pin 10
#define limit_pin PL3   //stepper_1 limit pin 46
#define limit_pin2 PL7  //stepper_2 limit pin 42
#define dir PH6       //stepper_1 direction pin 9
#define pulse_pin2 PB5   //stepper_2 pulse pin 11
#define dir2 PH5         //stepper_2 direction pin 8
#define pi 3.1415

int step_t = 3200, step_c = 0 , target_step = 0 ;
uint8_t cnt = 0;
char tf_data[4];
uint16_t distance = 0, distance_2 = 0, prev_dist = 0;   //distance=instantaneous value by TFmini
int pulse = 0, prev_pos = 0, c = 0, a = 0, pulse2 = 0, prev_pos2 = 0;
unsigned int value = 0, max_span = 1511; //for 160 1422

float phi = 0, x = 38, phi_rad = 0, theta = 0, theta_rad = 0 , b = 0;

unsigned int D = 0;
long int t = 0;
uint16_t dist_f = 0, dist_p = 0;
void setup() {
  x = (x); //x = x while input is on rhs facing towards ball , for lhs x = -x
  DDRL &= ~((1 << limit_pin) | (1 << limit_pin2));
  DDRB |= ((1 << pulse_pin) | (1 << pulse_pin2));
  DDRH |= ((1 << dir) | (1 << dir2));
  PORTL |= ((1 << limit_pin) | (1 << limit_pin2));
  Serial_int_setup();
  Serial.begin(115200);
  timer_setup(390); //1.5 sec
  while ((PINL & (1 << limit_pin)) ) {     //when the limit switch is close the stepper pins will be high
    PORTH &= ~(1 << dir);
    PORTB ^= (1 << pulse_pin);
    _delay_us(1500);
  }

  while ((PINL & (1 << limit_pin2))) {    //when the limit switch is close the stepper2 pins will be high
    PORTH &= ~(1 << dir2);
    PORTB ^= (1 << pulse_pin2);
    _delay_us(500);
  }
  //  dist_f=0.969*dist_f+0.0155*distance+0.0155*dist_p;
  //  dist_p=distance;
  prev_dist = distance;
  // prev_dist = dist_f;
  prev_pos = 0;
  prev_pos2 = 0;
  _delay_ms(1000);
}
void loop () {
    Serial.print(theta);
    Serial.print("    ");
    Serial.print(phi);
    Serial.print("    ");
    Serial.print(distance);
    Serial.print("    ");
    Serial.print(distance_2);
    Serial.print("      ");
    Serial.println(value);
//  Serial.print(distance_2);
//  Serial.print("      ");
//  Serial.println(D);
//Serial.print(digitalRead(42));
//Serial.print("      ");
//Serial.println(digitalRead(46));

  if (a == 0) {
    if (prev_pos == 0)
    {
      //        t = millis();
      set_pos(1400);                    //the steps from 0 to half rotation| desired steps position
      TIMSK1 |= (1 << OCIE1A);
      //        a = 1;
      //      prev_dist = 10000;
    }
    else {
      targetStep();                     //to return stepper to the closest distance
      value = prev_dist;
      if (prev_pos == 1400) {
        angle_calculation(target_step, value);
        set_pos2((phi * 8.89) - 13.35);
        TIMSK3 |= (1 << OCIE3A);
        //          t = millis() - t;
        //           Serial.println(t);
        //          set_pos(target_step);
        //          TIMSK1 |= (1 << OCIE1A);
        a = 1;
      }
    }
  }

  else if (a == 1)
  {
    if (prev_pos == 1400)
    {
      //        t = millis();
      set_pos(0);
      TIMSK1 |= (1 << OCIE1A);
      //        a = 2;
      //      prev_dist = 10000;
    }
    else {
      targetStep();
      value = prev_dist;
      if (prev_pos == 0) {
        angle_calculation(target_step, value);
        set_pos2((phi * 8.89) - 13.35);
        TIMSK3 |= (1 << OCIE3A);
        //          t = millis() - t;
        //           Serial.println(t);
        a = 0;
      }
    }
  }
  //    else if (a == 2) {
  //      if (prev_pos == 0)
  //      {
  //        set_pos(target_step);                    //the steps from 0 to half rotation| desired steps position
  //        TIMSK1 |= (1 << OCIE1A);
  //        angle_calculation(target_step, value);
  //
  //        a = 0;
  //        prev_dist = 10000;
  //      }
  //      else {
  //        targetStep();                     //to return stepper to the closest distance
  //        value = prev_dist;
  //      }
  //    }
}


void targetStep()
{
  if (distance < prev_dist)
  {
    prev_dist = distance;
    target_step = prev_pos - 15;
  }
//  if (abs(distance_2 - D) > 20) {
//    prev_dist = 10000;
//  }
}

void set_pos(int pos) {            //ACW: prev_pos < pos    |   CW: prev_pos > pos
  if (pos < prev_pos) {
    PORTH &= ~(1 << dir);
  }
  else {
    PORTH |= (1 << dir);    //to toggle the dir pin
  }
  pulse = abs(prev_pos - pos);    // number of steps left to turn
}

void set_pos2(int pos) {            //ACW: prev_pos < pos    |   CW: prev_pos > pos
  if (pos < prev_pos2) {
    PORTH &= ~(1 << dir2);
  }
  else {
    PORTH |= (1 << dir2);
  }
  pulse2 = abs(prev_pos2 - pos);
}

void Serial_int_setup() {
  cli();
  UBRR2L = 0x08;
  UBRR3L = 8;
  UBRR3H = 0x00;
  UCSR2C = (1 << UCSZ20) | (1 << UCSZ21); //for 2nd TF mini  servo
  UCSR2B = (1 << RXEN2) | (1 << RXCIE2);
  UCSR3B = 0x98;                          //for 1st TF mini stepper
  UCSR3C = 0x06;
  sei();
}

void timer_setup(int w) {
  cli();
  TCCR1A = 0;                                             //stepper
  TCCR1B = 0;                                             //limit switch
  TCCR3A = 0;                                             //servo
  TCCR3B = 0;                                             //servo
  TCNT1 = 0;                                              //stepper
  TCNT3 = 0;
  OCR3A = pulse_width(w);                                           //servo
  OCR1A = pulse_width(w);                                 //variable pulse width for stepper
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  //TIMSK1|=(1<<OCIE1A);
  sei();
}

int pulse_width(int width) {
  return (width * 2) - 1;
}

int calculate_pulse(float a) {
  return 3200 * a / 360.0;
}

ISR(USART3_RX_vect) {                                       //interrupt or stepper TF mini
  c++;
  measurement();
}

void measurement() {                                        //for stepper TF mini
  //int check;                //no' of value in uart array
  unsigned char uart[9];
  const int HEADER = 0x59;  // header of TFmini stepper
  if (uread() == HEADER)    // 1st header
  {
    uart[0] = HEADER;
    //check += uart[0];       //next value in array

    if (uread() == HEADER)
    {
      uart[1] = HEADER;
      //check += uart[1];
      for (int i = 2; i < 8; i++)// distance/strenth/temp
      {
        uart[i] = uread();
        //    check += uart[i];
      }
      uart[8] = uread();
      distance = (uart[2] + uart[3] * 256);//distance of stepper TFmini
      //      str = uart[4] + uart[5] * 256;
    }
  }
}

void measurement_2() {                                       //for TF mini of servo
  //int check;
  unsigned char uart[9];
  const int HEADER = 0x59;
  if (uread_2() == HEADER)
  {
    uart[0] = HEADER;
    // check += uart[0];

    if (uread_2() == HEADER)
    {
      uart[1] = HEADER;
      //check += uart[1];
      for (int i = 2; i < 8; i++)
      {
        uart[i] = uread_2();
        //check += uart[i];
      }
      uart[8] = uread_2();
      distance_2 = (uart[2] + uart[3] * 256);
    }
  }
}

ISR(USART2_RX_vect) {                                      //interrupt of servo TFmini
  //  c++;
  measurement_2();
}

ISR(TIMER1_COMPA_vect)                                     //interrupt for stepper
{
  if (pulse > 0) {
    PORTB ^= (1 << pulse_pin);
    if ((PINB & 0x20) == 0) {
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

ISR(TIMER3_COMPA_vect)                                     //interrupt for stepper
{
  if (pulse2 > 0) {
    PORTB ^= (1 << pulse_pin2);
    if ((PINB & 0x10) == 0) {
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

char uread() {                              //to read data us of Stepper TFmini
  while (!(UCSR3A & (1 << RXC3)));
  return UDR3;
}

char uread_2() {                             //to read data bus of servo TFmini
  while (!(UCSR2A & (1 << RXC2)));
  return UDR2;
}

void angle_calculation (float steps, unsigned int d) {
  theta = (steps * 0.1125);
  theta_rad = (theta * 0.0174);
  b = pi - acos(x / d);
  if ((theta_rad > b) && (theta_rad < pi))
  { phi_rad = pi - atan((d * sin(pi - theta_rad)) / (d * cos(pi - theta_rad) - x));
    D = ((d * sin(pi - theta_rad)) / sin(pi - phi_rad));
//    d2 = sqrt(pow((d * sin(pi - theta_rad)), 2) + pow(((d * cos(pi - theta_rad) - x)), 2));
  }
  else if ((theta_rad > (pi / 2)) &&  (theta_rad <= b))
  { phi_rad = atan((d * sin(pi - theta_rad)) / (x - d * cos(pi - theta_rad)));
    D = ((d * sin(pi - theta_rad)) / sin(phi_rad));
//    d2 = sqrt(pow((d * sin(pi - theta_rad)), 2) + pow((x - d * cos(pi - theta_rad)), 2));
  }
  else if ((theta_rad > 0) && (theta_rad <= (pi / 2)))
  { phi_rad = atan((d * sin(theta_rad)) / (x + (d * cos(theta_rad))));
    D = ((d * sin(theta_rad)) / sin(phi_rad));
//    d2 = sqrt(pow((d * sin(theta_rad)), 2) + pow((x + (d * cos(theta_rad))), 2));
  }
  phi  = (phi_rad * 57.3248);
}
