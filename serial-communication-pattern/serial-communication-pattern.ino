/*
 * pattern-transmit.c
 *
 * Created: 22-10-2021 16:54:20
 * Author : Yukta
 */ 

#include <avr/io.h>
#include <util/delay.h>

unsigned char z=0, y=0, x=0x81,a=0, i=0,w=0x11,b=0;
int main(void)
{
   DDRB= 0x00;
   DDRC= 0xff;
   PORTB=0xff;
   
   UCSR1B= (1<<TXEN1);
   UCSR1C= (1<<UCSZ11)|(1<<UCSZ10);
   UBRR1L= 0x33;
   while (1)
   {
     
    
     z= ~(PINB)&0x01;
     if(z==0x01)
     {  y=1;
       i++;
        
     }
 
  

    if((i%2 == 1) && (y==1))
    { while(1)//a<5)
      {
        
        while(!(UCSR1A &(1<<UDRE1)));
        UDR1=(x<<a)|(x>>a);
        _delay_ms(500);
        a++;
      }
      
      a=0;
    }
    else if((i%2 == 0) && (y==1))
    { //PORTC=0xff;
      while(1)//b<4)
      {
        while(!(UCSR1A &(1<<UDRE1)));
        UDR1 =(w<<b);
        _delay_ms(500);
        b++;
        
      }
      b=0;
      
    }
  
    
   
   }
}
