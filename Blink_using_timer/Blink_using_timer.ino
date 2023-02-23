#include<avr/io.h>


unsigned long t=0;

int main()
{
  init();
  Serial.begin(9600);
  DDRB = 0xff;
  PORTB = 0xff;
  TCCR2A = 0X00; 
  TCCR2B = 0X05; //1024 prescaler
  while (1)
  {
 
    PORTB ^= 0xff; // for toggling the port b
  
        Serial.print("blink");
        Serial.print("   ");
    for (int j = 0; j < 62; j++)
    {

      while ( (TIFR2 & 0x01) == 0);  // while the timer flag overflows
      TIFR2 = 0X01;             // TIFR flag set high denotes completion of one cycle
      TCNT2 = 0x00;             // generates a delay from 0 to 255
    }
    t = millis(); // returns instantaneous value of time
    if(t>5000){   // led blinks till 5 sec
      TCCR2B = 0x00;
    }
     Serial.println(t);
  }
}
