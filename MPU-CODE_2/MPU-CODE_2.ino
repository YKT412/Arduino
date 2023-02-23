#include <avr/io.h>
#include <avr/delay.h>
int gyro_x=0, gyro_x_offset=0, c=1000, dt=0;
long int prev_millis=0;
float angle=0, gyro=0;
void i2c_error ()
{
  TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
  Serial.println("error");
}
void i2c_init()
{
  TWBR=0x03;
  // DDRD=0x00; 
}
void i2c_start()
{
  TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWSTA);
  while (!(TWCR & (1<<TWINT)));
}
void i2c_stop()
{
  TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWSTO);
}
void i2c_add_write()
{
  TWDR=0xD0;
  TWCR=(1<<TWEN)|(1<<TWINT);
  while (!(TWCR & (1<<TWINT)));
}
void i2c_add_read()
{
  TWDR=0xD1;
  TWCR=(1<<TWEN)|(1<<TWINT);
  while (!(TWCR & (1<<TWINT)));
  if ((TWSR & 0xF8) != 0x40) i2c_error();
}
void i2c_write_data(unsigned char i)
{
  TWDR=i;
  TWCR=(1<<TWEN)|(1<<TWINT);
  while (!(TWCR & (1<<TWINT)));
  if ((TWSR & 0xF8) != 0x28) i2c_error();
}
char i2c_read_data()
{
  TWCR=(1<<TWEN)|(1<<TWINT);
  while (!(TWCR &(1<<TWINT)));
  if ((TWSR & 0xF8) != 0x58) i2c_error();
  return TWDR;
}
void mpu_config(unsigned char mpu_add, unsigned char value)
{
  i2c_start();
  i2c_add_write();
  i2c_write_data(mpu_add);
  i2c_write_data(value);
  i2c_stop();
}
void setup()
{
  Serial.begin(9600);
  i2c_init();
  mpu_config(0x1B, 0x00); //Gyro Configuration
  mpu_config(0x6B, 0x00); //Power Management Reset
  mpu_config(0x19, 0x00); //Sample Rate
  prev_millis=millis();
  while (c>0)
  {
    gyro_data();   
    gyro_x_offset+=gyro_x;
    c--;
  }
  gyro_x_offset/=1000;
}
void loop()
{
  gyro_data();
  dt=millis()-prev_millis;
  gyro=((gyro_x-gyro_x_offset)*dt)/(131000.0);
  angle+=gyro;
  prev_millis=millis();
  Serial.println(gyro_x);
  _delay_ms(100);
}
void gyro_data()
{
  i2c_start();
  i2c_add_write();
  i2c_write_data(0x48);
  i2c_start();
  i2c_add_read();
  gyro_x=i2c_read_data();
  i2c_stop();
 
  i2c_start();
  i2c_add_write();
  i2c_write_data(0x47);
  i2c_start();
  i2c_add_read();  
  gyro_x|=(i2c_read_data()<<8);
  i2c_stop();
}
