#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define pulse_pin PB5
#define limit_pin PE4
#define dir PH6
#define pwm 11
int step_t = 3200, step_c = 0 , target_step = 0 ;
uint8_t cnt = 0;
char tf_data[4];
uint16_t distance = 0, prev_dist = 0;
int pulse = 0, prev_pos = 0, c = 0, a = 0;
unsigned int value = 0;

int main() {
  // put your setup code here, to run once:
  DDRE &= ~(1 << limit_pin);
  DDRB |= (1 << pulse_pin);
  DDRH |= (1 << dir);
  PORTE |= 1 << limit_pin;
  //    pinMode(limit_pin, INPUT_PULLUP);
  //  pinMode(pwm, OUTPUT);
  //  pinMode(dir, OUTPUT);
  Serial_int_setup();
  Serial.begin(115200);
  timer_setup(500);
  while ((PINE & (1 << limit_pin)) == 0) {
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
    //        Serial.println(prev_pos);
    Serial.print(prev_dist);
    Serial.print("  ");
    Serial.print(distance);
    Serial.print(" ");
    Serial.println(value);
    if (a == 0) {
      if (prev_pos == 0)
      {

        set_pos(1400);
        TIMSK1 |= (1 << OCIE1A);
        a = 1;
        prev_dist = 10000;
      }
      else {
        targetStep();
        value = prev_dist;
      }
    }
    else if (a == 1)
    {
      if (prev_pos == 1400)
      {
        set_pos(0);
        TIMSK1 |= (1 << OCIE1A);
        a = 0;
        prev_dist = 10000;
      }
      else {
        targetStep();
        value = prev_dist;
      }
    }



    //Serial.println('a');
    //    if (prev_pos == 1400) {
    //
    //      set_pos(0);
    //      TIMSK1 |= (1 << OCIE1A);
    //      a = 0;
    //    }

    //  if (step_c < 1400) {
    //    set_pos(step_c);
    //    TIMSK1 |= (1 << OCIE1A);
    //    if (distance < prev_dist ) {
    ////      Serial.println('g');
    //      prev_dist = distance;
    //      target_step = prev_pos;
    //    }
    //    Serial.print(step_c);
    //    Serial.print("  ");
    //    Serial.println(distance);
    //    step_c++;
    //  }
    //  else {
    //    if(pulse == 0){
    //    set_pos(target_step);
    //    TIMSK1 |= (1 << OCIE1A);
    //    }
    //    Serial.println(prev_dist);
    //    Serial.println(target_step);
    //    Serial.print("  ");
    //    Serial.println(distance);
    //    if (distance != prev_dist) {
    //      step_c = 0;
    //      while(digitalRead(limit_pin)) set_pos(0);
    //    }
    //  }

    //  else if(step_c >= 1600){
    //    Serial.println('l');
    //    set_pos(target_step);
    //    while(1);
    //  }
    //  else {
    //    set_pos(target_step);
    //    Serial.print(abs(step_c - target_step) * 360 / step_t);
    //    Serial.print("  ");
    //    Serial.println(prev_dist);
    //    while (1) {
    //      //      Serial.println(distance);
    //    }
    //  }
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

void set_pos(int pos) {
  if (pos < prev_pos) {
    //      digitalWrite(dir,HIGH);
    PORTH &= ~(1 << dir);
  }
  else {
    //      digitalWrite(dir,LOW);
    PORTH |= (1 << dir);
  }
  pulse = abs(prev_pos - pos);
}

void Serial_int_setup() {
  cli();
  UBRR3H = 0x00;
  UBRR3L = 8;
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
  //TIMSK1|=(1<<OCIE1A);
  sei();
}

int pulse_width(int width) {
  return (width * 2) - 1;
}

int calculate_pulse(float a) {
  return 3200 * a / 360.0;
}

//ISR(USART3_RX_vect) {
//
//  if (cnt < 2) {
//    if (UDR3 == 0x59)  cnt++;
//    else cnt = 0;
//  }
//  else {
//    tf_data[cnt - 2] = UDR3;
//    cnt++;
//    if (cnt > 3) {
//      if ((tf_data[0] + tf_data[1] * 256) < 1500) {
//        distance = tf_data[0] + tf_data[1] * 256;
//      }
//      cnt = 0;
//    }
//  }
//}

ISR(USART3_RX_vect) {
  c++;
  measurement();
  //dist=UDR3;
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
      distance = uart[2] + uart[3] * 256;
      //      str = uart[4] + uart[5] * 256;
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  if (pulse > 0) {
    PORTB ^= (1 << pulse_pin);
    if ((PINB & 0x20) == 0) {
      pulse--;
      if ((PINH & 0x40) == 0x40) {
        //          Serial.println("hii");
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

char uread() {
  while (!(UCSR3A & (1 << RXC3)));
  return UDR3;
}
