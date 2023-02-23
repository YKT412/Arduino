
#include <avr/io.h>
unsigned int t=0;
int dist=0;
void setup() 
{
 Serial.begin(9600);
 pinMode(21,OUTPUT);
 pinMode(49, INPUT);
// DDRL=0xFF;
// PORTL=0xFF;
  TCCR4A=0x00;
 TCCR4B=0x45;
}

void loop() 
{
 digitalWrite(21, HIGH);
 delay(0.001);
 digitalWrite(21,0);
 digitalWrite(49,0);
 t=pulseIn(49,HIGH);
 // PORTL=0x00; 
// while((TIFR4 & (1<<ICF4))==0);
// t=ICR4L;
// TIFR4=(1<<ICF4);
//Serial.println(t);
dist=(0.034*(t/2));
Serial.println(dist);
//delay(2000);

}
