//Stepper with resolution 3200

#include <avr/io.h>
#define F_CPU 16000000
#include <util/delay.h>
#include <avr/interrupt.h>

int counter = 0, desired_steps = 0, desired_deg = 0, current_deg = 0, deg = 0;
int change = 0, j = 1;
unsigned int final_dist = 0, a = 0, min_dist = 65535, min_deg = 0, c = 1, t=0, counts=0;
unsigned char dist[4];

void Stepper_Init() {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 600;
  TCCR1B = (1 << WGM12) | (1 << CS11); //CTC mode , No prescaler
}
void Tf_Mini_Init() {
  UCSR2C = (1 << UCSZ20) | (1 << UCSZ21);
  UBRR2L = 0x08;
  UCSR2B = (1 << RXEN2) | (1 << RXCIE2);
}
void compare() {
  if(c){
  counts++;  
  
  if ((final_dist <= min_dist)) {
    min_dist = final_dist;
    min_deg = deg;
    
  }
}
}
void stepper_deg (int desired_deg) {
  change = desired_deg - current_deg;
  current_deg = desired_deg;
  desired_steps = change * 9;
  if (desired_steps > 0) {
    PORTB |= 0x02;
    TIMSK1 = (1 << OCIE1A);
  }
  if (desired_steps < 0) {
    desired_steps *= (-1);
    PORTB &= !0x02;
    TIMSK1 = (1 << OCIE1A);
  }
}
void stepper_reset() {
  while (PIND & 0x01) {
    deg --;
    stepper_deg(deg);
    _delay_ms(10);
  }
  stepper_deg(deg + 10);
  deg = 0;
  current_deg = 0;
}
int main() {
  init();
  Serial.begin(115200);
  DDRB = 0x03; //PINB0 -> Pulse  |  PINB1 -> Direction
  DDRD = 0x00;
  PORTD = 0x01;
  Stepper_Init();
  sei();
  stepper_reset();
  t=millis();
  Tf_Mini_Init();
  while (1) {
    if (c == 1) {
      if (!counter) {
        deg++;
        stepper_deg(deg);
        if (deg > 180) {
          t=millis()-t;
          c = 0;
          //
        }
      }
    }
    else {
      if (!counter) {
        stepper_deg(min_deg);
      }
    }
    Serial.print(final_dist);
    Serial.print("      ");
    Serial.print(min_dist);
    Serial.print("      ");
    Serial.print(deg);
    Serial.print("      ");
    Serial.print(min_deg);
    Serial.print("      ");
    Serial.print(t);
    Serial.print("      ");
    Serial.println(counts);
  }
}

ISR (TIMER1_COMPA_vect) {
  if (counter == desired_steps) {
    TIMSK1 = 0x00;
    counter = 0;
    return;
  }
  PORTB ^= 0x01;
  if (PORTB & 0x01) counter++;
  
}

ISR (USART2_RX_vect) {
  if (a < 2) {
    if (UDR2 == 0x59) a++;
    else a = 0;
  }
  else {
    dist[a - 2] = UDR2;
    a++;
    if (a > 3) {
      final_dist = (dist[0] | (dist[1] << 8));
      a = 0;
      compare();
    }
  }
}
