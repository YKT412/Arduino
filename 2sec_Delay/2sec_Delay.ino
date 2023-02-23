#include<avr/io.h>
#include<util/delay.h>
#define led LED_BUILTIN 
void setup()
{

  DDRB = 0xff;
 
  PORTB = 0xff;

  TCCR2A= 0X00;
  TCCR2B= 0X05; //1024 prescaler
}
    void loop() 
   {
    PORTB^= 0xff; //^ 0x04;
//
    
     for(int j =0; j<244; j++)
    { 
      
       while( (TIFR2 & 0x01)==0);
      TIFR2 = 0X01;
      TCNT2= 0x01;
    }
    }
//}
