#include <avr/io.h>
#include<avr/interrupt.h>
#define trigger PL3 // 46
#define echo PL1    //48
unsigned int t = 0;
int dist = 0;
#include <avr/io.h>
void setup()
{
  Serial.begin(9600);
  // pinMode(A1,OUTPUT);
  // pinMode(A0, INPUT);
  DDRL |= (1 << trigger);
  TCCR5A = 0x83; //clear
  TCCR5B = 0xCA; // mode 15 8 prescaler input capture noise canceler
}

void loop()
{
  // digitalWrite(21, HIGH);
  // delay(0.001);
  // digitalWrite(21,0);
  // digitalWrite(49,0);
  // t=pulseIn(49,HIGH);
  // PORTL=0x00;

  while ((TIFR5 & (1 << ICF5)) == 0);
  t = OCR5L;
  TIFR5 = (1 << ICF5);
  Serial.println(t);
  dist = (0.034 * (t / 2));
  Serial.println(dist);
  PCICR = 0x01;
  PCMSK0| = (1<<PCINT3);
  //delay(2000);
  sei();

}
ISR (PCINT3_vect){
  
  
}
