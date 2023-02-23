#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "right_tf_mini.h"
#include "left_tf_mini.h"
#define pulse_pin1 PB4   //stepper_1 pulse pin 10
#define dir1 PH6       //stepper_1 direction pin 9
#define limit_pin1 PL3   //stepper_1 limit pin 46
#define pulse_pin2 PB5   //stepper_2 pulse pin 11
#define dir2 PH5         //stepper_2 direction pin 8
#define limit_pin2 PL1  //stepper_2 limit pin 48
#define pi 3.1415

int  dirf = 1, set = 0;
unsigned int  flag = 0 ;

int main() {

  DDRL &= ~((1 << limit_pin1) | (1 << limit_pin2));
  DDRB |= ((1 << pulse_pin1) | (1 << pulse_pin2));
  DDRH |= ((1 << dir1) | (1 << dir2));
  PORTL |= ((1 << limit_pin1) | (1 << limit_pin2));

  Serial.begin(115200);
  timer_setup(700);

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
  prev_dist1 = 65535;
  prev_dist2 = 65535;
  prev_pos1 = 0;
  prev_pos2 = 0;
  set_pos2(1600);
  _delay_ms(1000);
  Serial_int_setup();
  while (1) {
  }
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
void print_dist (){
  Serial.print(target_step2);
    Serial.print("  ");
    Serial.print(prev_dist2);
    Serial.print("  ");
    Serial.print(target_step1);
    Serial.print("  ");
    Serial.println(prev_dist1);
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




//    if (set == 0) {
//      flag = 1;
//    }
//  }
//  else {
//    angle_calculation ((target_step2), prev_dist2);
//    set_pos2(target_step2);
//    set_pos1((180- phi) * 8.8889);
    //    set_pos1((180 - phi) * 8.88889);
//  }
}
