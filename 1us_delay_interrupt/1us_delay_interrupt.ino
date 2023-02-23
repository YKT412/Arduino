#include<avr/io.h>
#include<avr/interrupt.h>
unsigned long i=0;
void timer_setup(int w)
{
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = pulse_width(w);
  TCCR1B |= (1<<WGM12);
  TCCR1B |= (1<<CS10);
  TIMSK1 |= (1<<OCIE1A);
  sei();
}
int pulse_width(int width) {
  return (width * 2) - 1;
}
int main ()
{
 
  Serial.begin(2000000);
//  TCCR1A = 0;
//  TCCR1B = 0 ;
//  OCR1A = 0;
//  TCCR1B |= (1<<WGM12)|(1<<CS11);
//  TIMSK1 |= (1<<OCIE1A);
// 
// sei();
timer_setup(1);
 while(1);
  }
ISR(TIMER1_COMPA_vect)
{
  //Serial.println("yes");
  i++;
  Serial.println(i);
}
