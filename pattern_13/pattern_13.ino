/*
 * pattern13.c
 *
 * Created: 01-10-2021 14:21:27
 * Author : Yukta
 */ 

#include <avr/io.h>


int main(void)
{

     DDRD = 0x55;
     PORTD = 0xAA;
     unsigned char z;
 
     while(1)
    {
   z=~PIND ;
  PORTD=(z<<1)|(z>>7);
  }
}
 
