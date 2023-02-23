/*
 * 12c-trial-transmit.c
 *
 * Created: 29-10-2021 15:26:33
 * Author : Yukta
 */ 

#include <avr/io.h>
 unsigned char i=0x01;
void i2c_write(unsigned char data)
{
  TWDR= data;
  TWCR= (1<< TWINT)|(1<<TWEN);
  while((TWCR & (1<<TWINT))==0);
}
void i2c_start(void)
{ TWCR=(1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
  while ((TWCR& (1<<TWINT))==0);
    
}
void i2c_stop(void)
{ 
  TWCR= (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}
void i2c_init(void)
{
  TWSR= 0x00;
  TWBR= 0x47;
  TWCR= 0x04;
  
}

int main(void)
{
 
    i2c_init();
  i2c_start();
  i2c_write(0b11010000);
  i2c_write(i);
  i2c_stop();
  
    
}
