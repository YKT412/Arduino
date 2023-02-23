#include <avr/io.h>
#include <util/delay.h>

#define sec 0x00
#define min 0x01
#define hrs 0x02
#define days 0x03
#define alarm 0x0A

#define START 0x08
#define REP_START 0x10
#define SLA_R 0xD1
#define SLA_W 0xD0

uint8_t seconds = 0, minutes = 0, hours = 0;
unsigned char data = 0;
bool err = false;

void error() {
  // { Serial.println(TWSR, HEX);
  err = true;
  TWCR |= (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  Serial.println("error");
}

void i2c_start(void) {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  //  Serial.println(TWSR, HEX);//11
  if ((TWSR & 0xF8) != START) error();
  // Serial.println("start");
}

void i2c_rep_start(void) {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  //  Serial.println(TWSR, HEX);//0x10
  if ((TWSR & 0xF0) != REP_START) error();
  // Serial.println("repeated start");
}

void rtc_write_add(unsigned char slave_add) {
  TWDR = slave_add;  // SLA+W
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  //  Serial.println(TWSR, HEX);
  if ((TWSR & 0xF8) != 0x18) error();
  delay(50);
  // Serial.println("mpu add + write");
}

void rtc_read_add(unsigned char slave_add) {
  TWDR = slave_add;  // SLA+W
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  //  Serial.println(TWSR, HEX);
  if ((TWSR & 0xF0) != 0x40) error();
  delay(50);
  // Serial.println("mpu add + read");
}

void rgst_write_add(unsigned char rgstr) {
  TWDR = rgstr;  // Register address
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  //  Serial.println(TWSR, HEX); //0x20
  if ((TWSR & 0xF0) != 0x20) error();
  delay(50);
  // Serial.println("write register add");
}

void rgst_write_data(unsigned char value) {
  TWDR = value;  // Register value
  TWCR = (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  //  Serial.println(TWSR, HEX); //0x20
  if ((TWSR & 0xF8) != 0x28) error();
  delay(50);
  // Serial.println("write register data");
}


int burst_read() {

  TWCR |= (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while ((TWCR & (1 << TWINT)) == 0)
    ;
  // Serial.println(TWSR, HEX);
  if ((TWSR & 0xF8) != 0x50) error();
  // Serial.println("burst_read");
  delay(100);
  return TWDR;
}

int single_read() {

  TWCR |= (1 << TWINT) | (1 << TWEN);
  while ((TWCR & (1 << TWINT)) == 0);
  // Serial.println(TWSR, HEX);
  if ((TWSR & 0xF8) != 0x58) error();
  // Serial.println("single data");
  delay(100);
  return TWDR;
}


void i2c_stop(void) {
  TWCR |= (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  // Serial.println("stop");
}

void i2c_init(void) {
  TWSR |= 0x00;  //TWPS = 00
  TWBR = 72;
  TWCR |= (1 << TWEN);
  // Serial.println("init");
}

int register_read(uint8_t reg_add) {
  i2c_start();
  rtc_write_add(SLA_W);
  rgst_write_add(reg_add);
  i2c_rep_start();
  rtc_read_add(SLA_R);
  data = single_read();
  i2c_stop();
  delay(100);
  return data;
}

int register_write(uint8_t reg_add, uint8_t value) {
  i2c_start();
  rtc_write_add(SLA_W);
  rgst_write_add(reg_add);
  rgst_write_data(value);
  i2c_stop();
  delay(100);
}

int main(void) {
  init();
  Serial.begin(115200);
  i2c_init();

  while (1) {

    seconds = register_read(sec);
    seconds = ((seconds>>4)*10)+(seconds&0x0f);
    minutes = register_read(min);
    minutes = ((minutes>>4)*10)+(minutes&0x0f);
    hours = register_read(hrs);
    hours = (((hours>>4)*10)+(hours&0x0f))&0x3f;    
    Serial.print("seconds:");
    Serial.print(seconds);
    Serial.print("  ");
    Serial.print("minutes:");
    Serial.print(minutes);
    Serial.print("  ");
    Serial.print("hours:");
    Serial.println(hours);
  }
}