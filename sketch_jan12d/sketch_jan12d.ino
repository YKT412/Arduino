#include<avr/interrupt.h>
#include<avr/io.h>
unsigned long h = 0,l=0;

int main ()
{
  Serial.begin(9600);
  DDRB = 0xFF;
  
  TCNT1 = 65534; // for 16mhz
  TCCR1B = (1 << CS11);
  TIMSK1 = (1 << TOIE1);
  sei();
  while (1)
  {
//    l = millis();
//    Serial.println(h);
//    Serial.print("  ");
//    Serial.println(l);
  }
}
ISR(TIMER1_OVF_vect)
{
//  PORTB = ~ PORTB;
  TCNT1 = 65534;
  h++;
   Serial.println(h);
}
