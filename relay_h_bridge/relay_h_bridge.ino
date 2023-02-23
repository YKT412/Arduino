#include <util/delay.h>
#include <avr/io.h>
#define dir PB6
#define pwm PB5

unsigned int a = 0, c = 0, d0 = 0, d1 = 0;
int d = 0;

reset (){
  
}
void setup() {
  DDRB = (1 << dir) | (1 << pwm);
  TCCR1A = (1 << WGM11) | (1 << COM1A1); //NON-INVERTING MODE 8 PRESCALER
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); //MOTOR3
  ICR1 = 2000;
  OCR1A = 1000;
  Serial.begin (9600);
  delay(5000);
  d0 = 0;
}

void loop () {
  if (Serial.available()) {
    a = Serial.parseInt() + Serial.parseInt(); 
  }
  d1 = a;
  d = d1 - d0;
  c = d * 740;
  if ( d > 0){
    PORTB |= (1 << dir);
    while (c > 0){
      OCR1A = 1000;
      c --;
      if (c % 740 == 0){
        d0 ++;
      }
    }
  }
  else if (d < 0){
    PORTB &= ~(1 << dir);
    while (c > 0){
      OCR1A = 1000;
      c --;
      if (c % 740 == 0){
        d0 --;
      }
    }
  }
  else if ( d == 0){
    OCR1A = 0;
  }
  
  Serial.print(d1);
  Serial.print("      ");
  Serial.print(d0);
  Serial.print("      ");
   Serial.print(d0);
  Serial.print("      ");
  Serial.println(c);
  // if (a == 0) {
  //    PORTB = 0x80;
  //    _delay_us(5000);
  //    PORTB = 0xC0;
  //    _delay_us(5000);
  //  }
  //  else if (a == 1){
  //    PORTB = 0x40;
  //    _delay_us(5000);
  //    PORTB = 0xC0;
  //    _delay_us(5000);
  //  }
  //  else {
  //    PORTB = 0xC0;
  //  }
//  if (a == 0) {
//    PORTB &= ~(1 << dir);
//    OCR1A = 1000;
//  }
//  else if (a == 1) {
//    PORTB |= (1 << dir);
//    OCR1A = 1000;
//  }
//  else {
//    OCR1A = 0;
//  }
//  c = (int)(t / 0.005);
//  while (c > 0) {
//      OCR1A = 1000;
//      c--;
//    }
}
