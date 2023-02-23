/*
   MPU_code_1st.c

   Created: 6-9-2022 15:26:33
   Author : Yukta
*/

#include <avr/io.h>

#define Temp_h 0x41
#define Temp_l 0x42
#define config 0x1A
#define smprt 0x19
#define pwr_man 0x6B

#define START 0x08       //11
#define REP_START 0x10          //1
#define SLA_R 0xD1       
#define SLA_W 0xD0

unsigned char data = 0;
int16_t XG_test = 0, temp = 0;
boolean err = false;


void  error(){
// { Serial.println(TWSR, HEX);
  err = true;
  TWCR |= (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);  
  Serial.println("error");
}

void i2c_start(void)
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);

  //  Serial.println(TWSR, HEX);//11
  if ((TWSR & 0xF8) != START) error();
  // Serial.println("start");
}

void i2c_rep_start(void)
{
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);

  //  Serial.println(TWSR, HEX);//0x10
  if ((TWSR & 0xF0) != REP_START) error();
  // Serial.println("repeated start");
}

void mpu_write_add(unsigned char slave_add)
{
  TWDR = slave_add; // SLA+W
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  //  Serial.println(TWSR, HEX);
  if ((TWSR & 0xF8) != 0x18 ) error();
  delay(50);
  // Serial.println("mpu add + write");
}

void mpu_read_add(unsigned char slave_add)
{
  TWDR = slave_add; // SLA+W
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  //  Serial.println(TWSR, HEX);
  if ((TWSR & 0xF0) != 0x40 ) error();
  delay(50);
  // Serial.println("mpu add + read");
}

void rgst_write_add(unsigned char rgstr)
{
  TWDR = rgstr; // Register address
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  //  Serial.println(TWSR, HEX); //0x20
  if ((TWSR & 0xF0) != 0x20) error();
  delay(50);
    // Serial.println("write register add");
}

void rgst_write_data(unsigned char value)
{
  TWDR = value; // Register value
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  //  Serial.println(TWSR, HEX); //0x20
  if ((TWSR & 0xF8) != 0x28) error();
  delay(50);
    Serial.println("write register data");
}


int burst_read() {

  TWCR |= (1 << TWINT) | (1 << TWEN) | (1<<TWEA);
  while ((TWCR & (1 << TWINT)) == 0);
    // Serial.println(TWSR, HEX);
  if ((TWSR & 0xF8) != 0x50)error();
    XG_test = TWDR & 0x1F;
    // Serial.println("burst_read");
     delay(100);
  return TWDR; //XG_test;
}

int single_read() {

  TWCR |= (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
    // Serial.println(TWSR, HEX);
  if ((TWSR & 0xF8) != 0x58)error();
    XG_test = TWDR & 0x1F;
    // Serial.println("single data");
     delay(100);
     return TWDR; //XG_test;
     Serial.println(TWDR);
}


int i2c_read()
{
  TWCR |= (1 << TWINT) | (1 << TWEA);
  while ((TWCR & (1 << TWINT)) == 0);
   XG_test = TWDR & 0x1F;
  // Serial.println("read");
  return TWDR;
}


void i2c_stop(void)
{
  TWCR |= (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  // Serial.println("stop");
}


void i2c_init(void)
{
  TWSR |= 0x00;  //TWPS = 00
  TWBR = 72;
  TWCR |= (1 << TWEN);
  // Serial.println("init");
}

int register_read(uint8_t reg_add){
  i2c_start();  
  mpu_write_add(SLA_W);  
  rgst_write_add(reg_add);
  i2c_rep_start();  
  mpu_read_add(SLA_R); 
  data = single_read();
  i2c_stop();
     delay(100);
  return data;
}

int register_write(uint8_t reg_add, uint8_t value){
  i2c_start();  
  mpu_write_add(SLA_W);  
  rgst_write_add(reg_add); 
  rgst_write_data(value);
  i2c_stop();
     delay(100);
  return data;
}

int main(void)
{
  init();

  Serial.begin(115200);
  i2c_init();
   register_write(config,0x00); 
   register_write(pwr_man,0x00);
   register_write(smprt,0x00);
   while(1){   
     
     temp = register_read(Temp_l);
     temp = temp|(register_read(Temp_h)<<8);    
     temp = (temp/340)+36.53;
     Serial.println(temp);

    }   
  


}
