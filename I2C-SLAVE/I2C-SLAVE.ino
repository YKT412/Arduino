/*
 * i2c-trial-reading.c
 *
 * Created: 29-10-2021 16:43:04
 * Author : Yukta
 */ 

#include <avr/io.h>
unsigned char i=0;
void i2c_write()
{
  while((TWCR & (1<<TWINT))==0);
}
void i2c_init(void)
{
  TWCR= 0x04;
  TWAR= 0b10101010;
  TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA);
  
}
unsigned char i2c_read()
{
  TWCR= (1<< TWINT)|(1<<TWEN)|(1<<TWEA);
  while((TWCR & (1<<TWINT))==0);

  return TWDR;
}
int main(void)
{
  Serial.begin(9600);
  DDRD= 0x00;

  
  i2c_init();
    PORTD = 0x00;
  // i= i2c_read();
  while(1){
    if((TWCR & (1<<TWINT))){
       if(TWSR == 0x80 ){
         Serial.println(TWDR, HEX);
       }
       TWCR|= (1<<TWINT);
        Serial.println("yes");
    }
    // Serial.println("while");
  }
  // i2c_write();  
  
  // PORTA= i;
  // Serialprintln(i, HEX)
  }
