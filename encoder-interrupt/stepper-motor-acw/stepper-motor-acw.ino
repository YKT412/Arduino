/*
 * stepper-motor-2.c
 *
 * Created: 29-11-2021 17:13:34
 * Author : Yukta
 */ 

/*
 * stepper motor code.c
 *
 * Created: 29-11-2021 15:54:35
 * Author : Yukta
 */ 

#include <avr/io.h>
#define f_cpu 8000000UL
#include <util/delay.h>


int main(void)
{
   DDRA = 0X00;
   PORTA= 0xff;
   DDRB = 0XFF;
   int step[]={0X08, 0X02, 0X04, 0X01}, i=1, s=0; // anti clock wise
    while (i<400) 
    {
    
      PORTB = step[s] ;
      _delay_ms(10);
      s++;
      if(s==4)
      {
        s=0;
      }
     i++;
      
      
    }
}
