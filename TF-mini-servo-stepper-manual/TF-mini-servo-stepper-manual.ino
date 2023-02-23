#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define pulse_pin PB5   //stepper pulse   OC1A
#define limit_pin PE4   //interrupt 4 
#define dir PH6         //stepper PWM 9   OC2B
#define servo_pwm PE3   //servo PWM 5     OC3A
#define pi 3.1415

int step_t = 3200, step_c = 0 , target_step = 0 ;
uint8_t cnt = 0;
char tf_data[4];
uint16_t distance = 0, distance_2 = 0, prev_dist = 0;   //distance=instantaneous value by TFmini
int pulse = 0, prev_pos = 0, c = 0, a = 0;
unsigned int value = 0, max_span = 1511; //for 160 1422
float phi = 0, x = 34, phi_rad = 0, theta = 0, theta_rad = 0 , b = 0,deg=0;

int main() {
  DDRE &= ~(1 << limit_pin);
  DDRE |= (1 << servo_pwm);
  DDRB |= (1 << pulse_pin);
  DDRH |= (1 << dir);
  PORTE |= 1 << limit_pin;
  Serial_int_setup();
  Serial.begin(115200);
  timer_setup(500);
  while ((PINE & (1 << limit_pin)) == 0) {    //when the limit switch is open the stepper pins will be high
    PORTH &= ~(1 << dir);
    PORTB |= (1 << pulse_pin);
    _delay_us(500);
    PORTB &= ~(1 << pulse_pin);
    _delay_us(500);
  }
  prev_dist = distance;
  prev_pos = 0;
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
    Serial.println(value);
    Serial.print("    ");
//    Serial.print(prev_dist);
//    Serial.print("    ");
//    Serial.print(target_step);
//     Serial.print("    ");
//    Serial.println(b);

//    if (a == 0) {
//      if (prev_pos == 0)
//      {
//        set_pos(1400);                    //the steps from 0 to half rotation| desired steps position
//        TIMSK1 |= (1 << OCIE1A);
//        a = 1;
//        prev_dist = 10000;
//      }
//      else {
//        targetStep();                     //to return stepper to the closest distance
//        value = prev_dist;
//      }
//    }
//    else if (a == 1)
//    {
//      if (prev_pos == 1400)
//      {
//        set_pos(0);
//        TIMSK1 |= (1 << OCIE1A);
//        a = 2;
//        prev_dist = 10000;
//      }
//      else {
//        targetStep();
//        value = prev_dist;
//      }
//    }
//    else if (a == 2) {
//      if (prev_pos == 0)
//      {
//        set_pos(target_step);                    //the steps from 0 to half rotation| desired steps position
//        TIMSK1 |= (1 << OCIE1A);
//        angle_calculation(target_step, value);
//        OCR3A = (1100 + ((phi) * 20.556));
//        a = 0;
//        prev_dist = 10000;
//      }
//      else {
//        targetStep();                     //to return stepper to the closest distance
//        value = prev_dist;
//      }
//    }
  if (Serial.available()){
    deg = Serial.parseInt();
    
    set_pos(8.89*deg);
     TIMSK1 |= (1 << OCIE1A);
    angle_calculation(8.89*deg,400);
    OCR3A = (1000 + ((phi) * 20.556));
  }
  
  }
}

void targetStep()
{
  if (distance < prev_dist)
  {
    prev_dist = distance;
    target_step = prev_pos - 15;
  }
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
  ICR3 = 40000;                                           //servo
  OCR1A = pulse_width(w);                                 //variable pulse width for stepper
  TCCR3A = (1 << COM1A1) | (1 << COM1A0) | (1 << WGM11);
  TCCR3B = (1 << WGM13) | (1 << WGM12);
  TCCR3B |= (1 << CS11);
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
  int check;                //no' of value in uart array
  unsigned char uart[9];
  const int HEADER = 0x59;  // header of TFmini stepper
  if (uread() == HEADER)    // 1st header
  {
    uart[0] = HEADER;
    check += uart[0];       //next value in array

    if (uread() == HEADER)
    {
      uart[1] = HEADER;
      check += uart[1];
      for (int i = 2; i < 8; i++)// distance/strenth/temp
      {
        uart[i] = uread();
        check += uart[i];
      }
      uart[8] = uread();
      distance = uart[2] + uart[3] * 256;//distance of stepper TFmini
      //      str = uart[4] + uart[5] * 256;
    }
  }
}

void measurement_2() {                                       //for TF mini of servo
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
      distance_2 = uart[2] + uart[3] * 256;
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

char uread() {                              //to read data us of Stepper TFmini
  while (!(UCSR3A & (1 << RXC3)));
  return UDR3;
}
char uread_2() {                             //to read data bus of servo TFmini
  while (!(UCSR2A & (1 << RXC2)));
  return UDR2;
}
void angle_calculation (int steps, unsigned int d) {
  theta = (steps * 0.1125);
  theta_rad = (theta * 0.0174);
  b = pi - acos(x / d);
  if ((theta_rad > b) && (theta_rad < pi))
  { phi_rad = pi - atan((d * sin(pi - theta_rad)) / (d * cos(pi - theta_rad) - x));
  }
  else if ((theta_rad > (pi / 2)) &&  (theta_rad <= b))
  { phi_rad = atan((d * sin(pi - theta_rad)) / (x - d * cos(pi - theta_rad)));
  }
  else if ((theta_rad > 0) && (theta_rad <= (pi / 2)))
  { phi_rad = atan((d * sin(theta_rad)) / (x + (d * cos(theta_rad))));
  }
  phi = (phi_rad * 57.3248);
}
