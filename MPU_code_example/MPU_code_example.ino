#include <avr/io.h>
#include<avr/delay.h>
#include <avr/interrupt.h>
#define SLA_W 0xD0;
#define SLA_R 0xD1;//11010000

char gyro_h, gyro_l;
int gyro;
long long angle1;
float   prev_time, current_time, angle,pangle=0,offset;
long full=0,dt=0;
boolean err = false;

void  error()
{
  err = true;
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  Serial.println("error");
}
void mpu_stop ()
{
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}
void start()
{

  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  if (((TWSR & 0xF8) != 0x08)||((TWSR & 0xF8) == 0x10))error();
  Serial.println("start sent");
}
void mpu_addw()
{
  TWDR = SLA_W;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
   if ((TWSR & 0xF8) !=0x18)error();
   Serial.println("add+write sent");

}
void mpu_write(unsigned char i)
{
  TWDR = i;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  if ((TWSR & 0xF8) != 0x28)error();
  Serial.println("ra sent");
}
void mpu_addr()
{
  TWDR = SLA_R;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  if ((TWSR & 0xF8) != 0x40)error();
   Serial.println("add+receive");

}
char mpu_read()
{
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));
  if ((TWSR & 0xF8) != 0x58)error();
  Serial.println("received");
  return TWDR;
}

int main()
{
  init();
  Serial.begin(9600);
//  cli();
  TWBR = 0x03;//16Mhz/16+2( TWBR)*4
//  DDRD = 0x00;
  _delay_ms(1000);
  start();
  mpu_addw();
  mpu_write(0x6B); //power management
  mpu_write(0x00);
  mpu_stop();
  start();
  mpu_addw();
  mpu_write(0x19); //sample rate divider
  mpu_write(0x00);
  mpu_stop();// sample rate register
  start();
  mpu_addw();
  mpu_write(0x1B);//gyro config
  mpu_write(0x10); //self test acc
  mpu_stop();
  start();
  mpu_addw();
  mpu_write(0x38);//int enable
  mpu_write(0x01);//2C_SLV0_REG
  mpu_stop();
//  EIMSK =0x04;
//  sei();
  _delay_ms(100);
  for (unsigned char i = 0; i <100; i++)
  {
    start();
  mpu_addw();
  mpu_write(0x48); //gyro_z out L
  start();
  mpu_addr();
  gyro_l = mpu_read();
  mpu_stop();
    delay(10);
  
  full += gyro_l;
  
  }
offset=(-16); //full/100;
  _delay_ms(1000);
while(1)
{
//{ err=false;
//  if (err == true)
//     goto finish;
  //power management
  /*************
  */
  prev_time = millis();
  start();
  mpu_addw();
}
}
