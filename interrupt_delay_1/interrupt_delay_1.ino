#include<avr/interrupt.h>
#include<avr/io.h>
unsigned long h = 0,l=0;

int main ()
{ 
  cli();
  Serial.begin(115200);
  DDRB = 0xFF;
  PORTB=0xFF;
 // TCNT1 = 65532; // for 16mhz
  TCCR1B = (1 << CS11);
  TIMSK1 = (1 << TOIE1);
  sei();
  while (1)
  {
   
  }
}
ISR(TIMER1_OVF_vect)
{
 
  TCNT1 = 63535;
  h++;
  Serial.println(h);
}
