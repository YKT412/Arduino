/*
 * LCD-8bit.c
 *
 * Created: 10-11-2021 17:05:41
 * Author : Yukta
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#define DPRT PORTA
#define DDDR DDRA
#define DPIN PINA
#define CPRT PORTB
#define CDDR DDRB
#define CPIN PINB
#define RS 0
#define RW 1
#define EN 2


void command( unsigned char cmnd)
{
  DPRT= cmnd;
  CPRT &= ~(1<<RS);
  CPRT &= ~(1<< RW);
  CPRT |= (1<<EN);
  _delay_us(1);
  CPRT &= ~(1<<EN);
  _delay_us(100);
  
}
void data(unsigned char data1)
{
  DPRT = data1;
  CPRT |= (1<<RS);
  CPRT &= ~(1<<RW);
  CPRT |= (1<<EN);
  _delay_us(1);
  CPRT &= ~(1<<EN);
  _delay_us(100);
}

void init()
{
  DDDR=0xff;
  CDDR= 0xff;
  
  CPRT &= ~(1<<EN);
  _delay_us(2000);
  command(0x38);
  command(0x0e);
  command(0x01);
  _delay_us(2000);
  command(0x06);
  
}
void print(char*str)
{ 
  unsigned char i=0;
  while(str[i]!= 0)
  {
    data(str[i]);
    i++;
  }
}
void gotoxy(unsigned char x, unsigned char y)
{
  unsigned char add[]={0x80,0xc0,0x94,0xd4,};
    command(add[y-1]+x-1);
    _delay_us(100);
}
int main(void)
{
    init();
    while (1)
  { gotoxy(15,1);
    command(0x18);
    _delay_ms(100);
    print("happy birthday xD ");
    
  }
    return 0;
}
// void movingprint(char*str)
// {
//  unsigned char i=0;
//  for(i=0;i<9;++i)
//  {
//    while(str[i]!= 0)
//  {
//    data(str[i]);
//    _delay_ms(10);
//    i++;
//    
//  }
//  }
// }
