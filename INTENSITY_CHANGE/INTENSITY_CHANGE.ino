/*
 * INTENSITY-CHANGING.c
 *
 * Created: 16-10-2021 13:16:44
 * Author : Yukta
 */ 

#include <avr/io.h>
#include <util/delay.h>
unsigned char r=1;
  unsigned int a=0;
int main(void)
{
    DDRB= 0xff;
  
  TCCR0A= 0x43;  // toggle mode, fast pwm
  TCCR0B= 0x01;   
  OCR0A = 0x00;
    while (1) 
    {
    if(r<256 && r>0)
    {
      if(a==0)
      {
        r++;
        OCR0B++;
        _delay_ms(10);
      }
        else if(a==1)
      {
        r--;
        OCR0B--;
        _delay_ms(10);
        
      }
    }
    else
    {
      if(a==0)r=1;
      else r= 255;
      a=!a;
    }
    
    }
}
