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

int step_t = 3200, step_c = 0 , target_step1 = 0 , target_step2 = 0, i = 0, j = 0,dirf=1,set=0,tf1n=0,tf2n=0;
bool tf1=0,tf2=0;
long a2=0,b2=0;
uint8_t cnt = 0;
char tf_data1[4], tf_data2[4];
uint16_t dist1 = 0, dist2 = 0, prev_dist1 = 0, prev_dist2 = 0;
double target_dist = 0;
int pulse1 = 0, prev_pos1 = 0, c = 0, a = 0, pulse2 = 0, prev_pos2 = 0,pos1=0,pos2=0;
unsigned int value1 = 0, steps = 3200;
double phi = 0, x = 39, phi_rad = 0, theta = 0, theta_rad = 0 , b = 0;

int main() {
  init();
  DDRL &= ~((1 << limit_pin1) | (1 << limit_pin2));
  DDRB |= ((1 << pulse_pin1) | (1 << pulse_pin2));
  DDRH |= ((1 << dir1) | (1 << dir2));
  PORTL |= ((1 << limit_pin1) | (1 << limit_pin2));
 
  Serial.begin(115200);
  timer_setup(500);
 
  while ((PINL & (1 << limit_pin1))  | (PINL & (1 << limit_pin2))) { //if either of the limit pin is pressed
    PORTH &= ~((1 << dir1) | (1 << dir2));         //clockwise
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
  prev_dist1 = dist1;
  prev_pos1 = 0;
  prev_pos2 = 0;
  set_pos2(1600);
  _delay_ms(1000);
  
   Serial_int_setup();
  // a2=millis();
  //  while ((PINL & (1 << limit_pin2))) {
  //    PORTH &= ~(1 << dir2);
  //
  //    _delay_us(500);
  //  }

  while (1) {
    //delay(50);
//    angle_calculation(pos2,dist2);
//    Serial.print(((pos1)*0.1125));
  //  Serial.print(" 1  ");
    Serial.print(a2);
    Serial.print("   ");
    Serial.println(b2);
//    Serial.print("   ");
//    Serial.print(target_dist);
//    Serial.println("   ");
//        
      
    
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
      tf2n++;
       if(tf2n>99)
      {
       b2=millis();
       tf2n=0;
      }
      dist1 = tf_data1[0] + tf_data1[1] * 256;
     pos1 = prev_pos1;
        if(tf2==1){
        set_stepper();
        tf2=0;
        }
        else tf1=1;
      
      i = 0;
    }
  }
}
void set_stepper(){
  set+=8*dirf;
  set_pos1(set);
  set_pos2(1600-set);
  if(set==1000)dirf=(-1);
  if(set==0)dirf=1;
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
      tf1n++;
      if(tf1n>99)
      {
        tf1n=0;
       a2=millis();
      }
      dist2 = tf_data2[0] + tf_data2[1] * 256;
    pos2=prev_pos2;
        if(tf1==1){
        set_stepper();
        tf1=0;
        }
        else tf2=1;
      
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

void angle_calculation (float steps, float d) {
  if(!d)d=10000.0;
  theta = (steps * 0.1125);
  theta_rad = (theta * 0.0174);
  target_dist=sq(d)+sq(40.5)-(d*80.0*cos(theta_rad));
  target_dist=sqrt(target_dist);
  phi_rad=asin(((sin(theta_rad)*d)/target_dist));
  phi  = (phi_rad * 57.3248);


  
//  b = (pi - acos(x / d));
//  if ((theta_rad > b) && (theta_rad < pi)) {
//    phi_rad = pi - atan((d * sin(pi - theta_rad)) / (d * cos(pi - theta_rad) - x));
//    target_dist = ((d * sin(pi - theta_rad)) / sin(pi - phi_rad));
//  }
//  else if ((theta_rad > (pi / 2)) &&  (theta_rad <= b)) {
//    phi_rad = atan((d * sin(pi - theta_rad)) / (x - d * cos(pi - theta_rad)));
//    target_dist = ((d * sin(pi - theta_rad)) / sin(phi_rad));
//  }
//  else if ((theta_rad > 0) && (theta_rad <= (pi / 2))) {
//    phi_rad = atan((d * sin(theta_rad)) / (x + (d * cos(theta_rad))));
//    target_dist = ((d * sin(theta_rad)) / sin(phi_rad));
//  }
//  phi  = (phi_rad * 57.3248);
//  target_step2 = (phi * 8.88888); //8.88888 = 1600/180
}
