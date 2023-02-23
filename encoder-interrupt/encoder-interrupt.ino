#include<avr/interrupt.h>
#include<avr/io.h>
 int a=0,b=0,i=0,a1=0, b1=0;
void setup()
{
  DDRB= 0x00;
  PORTB= 0x03;
  
  Serial.begin(9600);
  PCICR= 0X01;
  PCMSK0= (1<<PCINT0)|(1<<PCINT1);
  sei();
   
}
void loop()
{
  Serial.println(i);
}

ISR (PCINT0_vect)
{

  a1= ((~PINB)&0x01);                                                                                                                                                                                                                                                                                                                                                                                                                    
  b1= ((~PINB)&0X02);
  
  if(a1!=a)
  {
    a=a1;
    if(b1!=a1)
    {
      i++;
    }
    else
    {
      i--;
    }
  }
  else if(b1!=b)
  {
    b=b1;
    if(a1!=b1)
    {
      i--;
    }
    else
    {
      i++;
      
    }
  }
   
}
    
 
  
