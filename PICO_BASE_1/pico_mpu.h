#include "address.h"

#define config 0x1B
#define smprt 0x19
#define pwr_man 0x6B
#define gyro_zl 0x48
#define gyro_zh 0x47

float z_offset = 0;
float actual = 0, desired = 0;
uint32_t j = 0, prev_micros = 0, dt = 0, dt2 = 0, prev_micros2 = 0;
int16_t rawdata = 0, zout = 0;
uint8_t datal = 0, datah = 0, c = 110;
double z = 0, angle = 0;
float zAngle = 0;
float P, D, previous_error, Z;
volatile double kp = 0, kd = 0, e = 0;

void write_reg(uint8_t reg, uint8_t value) {
  IC_DATA_CMD(1) = reg;
  IC_DATA_CMD(1) = (0x200) | (value);  //stop and data
}

void i2c_setup() {
  IC_CON(1) = 0x65;  //fast mode, master enable, slave mode disable, restart enable
  IC_FS_SCL_HCNT(1) = 4 * 133;
  IC_FS_SCL_LCNT(1) = 4.7 * 133;

  IC_TAR(1) = 0x68;    //START , ADDRESS AD
  IC_RX_TL(1) = 0x01;  // NUMBER OF ENTRIES
  IC_TX_TL(1) = 0x01;
  IC_EN(1) = 0x01;
  sleep_ms(10);
}

void read_reg(uint8_t reg) {
  while (!(IC_RAW_INTR_STAT(1) & 0x04));
  rawdata = IC_DATA_CMD(1) << 8;
  rawdata |= (IC_DATA_CMD(1));
  // data /= 255;
  IC_DATA_CMD(1) = reg;
  IC_DATA_CMD(1) = 0x100;
  IC_DATA_CMD(1) = 0x300;
}

void pid() {
  dt2 = timelr - prev_micros2;
  actual = zAngle;
  e = (desired - actual);
  D = kd * (e - previous_error) / dt2 * 1000000.0;
  P = kp * e;
  Z = P + D;
  previous_error = e;
  prev_micros2 = timelr;
}

void process_data() {
  dt = timelr - prev_micros;
  z = ((((rawdata - z_offset) / 16.4)) * (dt));  //10
  angle += z;
  zAngle = ((angle / 1000000));  //angle/10000//0.83
  prev_micros = timelr;
  if(j>4){
  pid();
Serial.println(String(zAngle)+"  "+Z+"  "+dt2+"   "+e);  
  j=0; 
  } 
  j++;
}

void read_reg1(uint8_t reg) {
  if (IC_RAW_INTR_STAT(1) & 0x04) {

    rawdata = IC_DATA_CMD(1) << 8;

    rawdata |= (IC_DATA_CMD(1));
    // data /= 255;
    IC_DATA_CMD(1) = reg;
    IC_DATA_CMD(1) = 0x100;
    IC_DATA_CMD(1) = 0x300;
    process_data();    
  }
}

void mpu_setup() {
  write_reg(pwr_man, 0x80);
  sleep_ms(50);
  write_reg(pwr_man, 0x00);
  write_reg(smprt, 0x00);
  write_reg(config, 0x18);  //18
  sleep_ms(10);
  IC_DATA_CMD(1) = gyro_zh;
  IC_DATA_CMD(1) = 0x100;
  IC_DATA_CMD(1) = 0x300;
  prev_micros = timelr;
  sleep_ms(500);
  while (c > 100) {
    read_reg(gyro_zh);
    // Serial.println(rawdata);
    c--;
  }
  while (c > 0) {
    read_reg(gyro_zh);
    // Serial.println(rawdata);
    z_offset += rawdata;
    c--;
    delay(10);
  }
  z_offset /= 100;
}

void mpu_angle() {  //loop

  read_reg1(gyro_zh);
 
  // Serial.print(rawdata);
  // Serial.print("  ");
  
  // Serial.print(dt);
  // Serial.print("  ");
  // Serial.println(dt2);
  out_clear = ((int)(0xff) << 16);
  out_set =   ((int)(zAngle) << 16)|(0x02000000);
}
