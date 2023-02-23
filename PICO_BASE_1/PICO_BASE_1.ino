#include "address.h"
#include "pico_mpu.h"

#define R 38
#define r 7.5
#define dir1 1
#define dir2 3
#define dir3 5
#define led25 25
// #define led 25
// MOTOR ORENTATION
//        M1
//       /  \
//      /    \
//     M2----M3
//
//     | MOTOR 1 |MOTOR 2  | MOTOR3
//DIR= |1        |3        |5
//PWM= |0        |2        |4
//frequency = 5K
//button|1 = up, 2 = down, 3 = right, 4 = left
// y = pwm x = rpm
//y = 48.63
volatile uint8_t i = 0, data[5], button = 0, buttonl = 0, buttonh = 0, checksum = 0, check = 0, x = 0, x2 = 0, y2 = 0;
int16_t ang[16] = { 0, 90, 270, 0, 180, 135, 225, 0, 0, 45, 315 };
volatile int d1, d2, d3;
float Dr12 = 0, Dr21 = 0, Dr31 = 0, Dr13 = 0, Dr23 = 0, Dr32 = 0;
uint8_t kpi = 0, kpf = 0, kdi =0, kdf = 0;
/*
0000  no button
0001  up      
0010  down
0011
0100  left
0101  left up
0110  left down
0111  na
1000  right
1001  right up
1010  right down
*/

//baudrate: 115200
void uart_setup() {
  clk_peri = 0x800;
  uart0_i = 72;
  uart0_f = 1;  //10
  uart0_lcr = 0x60;
  uart0_cr = 0x201;
  asm("NOP");
  irq(20) = (uint32_t)esp8266_data;
  uartimsc = 0x10;
  nvic_iser = 0x100000;
  asm("NOP");
}

void pwm_setup() {
  pwm_csr(0) = 0x00000001;  //pwm enable
  pwm_top(0) = 0x00000BB8;  //top value 3000
  pwm_div(0) = 0x0000008D;  //5000hz freq
  pwm_cc(0) = 0x00000000;

  pwm_csr(1) = 0x00000001;  //pwm enable
  pwm_top(1) = 0x00000BB8;  //top value 3000
  pwm_div(1) = 0x0000008D;  //5000hz freq
  pwm_cc(1) = 0x00000000;

  pwm_csr(2) = 0x00000001;  //pwm enable
  pwm_top(2) = 0x00000BB8;  //top value 3000
  pwm_div(2) = 0x0000008D;  //5000hz freq
  pwm_cc(2) = 0x00000000;
}

void led() {
  gpc(16) = 0x05;
  gpc(18) = 0x05;
  gpc(19) = 0x05;
  gpc(20) = 0x05;
  gpc(21) = 0x05;
  gpc(22) = 0x05;
  gpc(26) = 0x05;
}

void setup() {
  gpc(25) = 0x05;
  gpc(0) = 0x04;   //pwm
  gpc(1) = 0x05;   //dir
  gpc(2) = 0x04;   //pwm
  gpc(3) = 0x05;   //dir
  gpc(4) = 0x04;   //pwm
  gpc(5) = 0x05;   //dir
  gpc(14) = 3;     //i2c
  gpc(15) = 3;     //i2c
  gpc(17) = 0x02;  //uart
  led();
  pad_clear(17) = 0x04;
  pad_set(17) = 0x08;
  Serial.begin(115200);
  oe = 0x02ff003f;
  i2c_setup();
  mpu_setup();
  uart_setup();
  pwm_setup();
}

void loop() {
  mpu_angle();
  if (button & 0x01) motor_degree(0, 0, Z);
  else if(button & 0x02)motor_degree(0,0,50);
  else d1 = d2 = d3 = 0;
//  motor_degree(1000, 90, 0);
  motor1_value(d1);
  motor2_value(d2);
  motor3_value(d3);
  
  // Serial.print(d1);
  // Serial.print("  ");
  // Serial.print(d2);
  // Serial.print("  ");
  // Serial.println(d3);

}

void joystick() {
  if ((x2 - 108) || (y2 - 108)) motor_degree(3000, 0, 0);
  else d1 = d2 = d3 = 0;
  motor1_value(d1);
  motor2_value(d2);
  motor3_value(d3);
}

void motor_degree(float base_speed, float degree, float base_w) {
  double radian = (degree) * (0.0174532925);
  float wx = 0;//cos(radian);  // omega in degree per sec
  float wy = 0;//sin(radian);
  float wheel_w = base_speed * 7.6433;  //392.5 top base speed
  float w = base_w*R/r;
  // float vx = (x2 - 108) / 90.0;  // for joystick
  // float vy = (y2 - 108) / 90.0;  // for joystick
  d1 = wheel_w * (((-2 * wx / 3) + (0 * wy))) + w;   //0.15
  d2 = wheel_w * ((wx / 3) - (wy / (sqrt(3)))) + w;  //0.35
  d3 = wheel_w * ((wx / 3) + (wy / (sqrt(3)))) + w;  //0.48

  if ((abs(d1) > 3000) || (abs(d2) > 3000) || (abs(d3) > 3000)) {
    if (abs(d1) >= abs(d2)) {
      if (abs(d1) >= abs(d3)) {            
        Dr12 = (d1 / d2);
        Dr13 = (d1 / d3);
        d2 = 3000 / abs(Dr12)*(d2/abs(d2));
        d3 = 3000 / abs(Dr13)*(d3/abs(d3));
        d1 = 3000*(d1/abs(d1));
      } else {
        Dr31 = d3 / d1;
        Dr32 = d3 / d2;
        d2 = 3000 / abs(Dr32)*(d2/abs(d2));
        d1 = 3000 / abs(Dr31)*(d1/abs(d1));
        d3 = 3000*(d3/abs(d3));
      }
    } else {
      if (abs(d2) >= abs(d3)) {
        Dr21 = d2 / d1;
        Dr23 = d2 / d3;
        d3 = 3000 / abs(Dr23)*(d3/abs(d3));
        d1 = 3000 / abs(Dr21)*(d1/abs(d1));
        d2 = 3000*(d2/abs(d2));
        } else {
          Dr31 = d3 / d1;
          Dr32 = d3 / d2;
          d2 = 3000 / abs(Dr32)*(d2/abs(d2));
          d1 = 3000 / abs(Dr31)*(d1/abs(d1));
          d3 = 3000*(d3/abs(d3));
        }
      }
    }
}

  void motor1_value(int a) {
    if (a >= 0) {
      out_set = (1 << dir1);  //DIR FOR MOTOR1.ANTICLOCKWISE
      pwm_cc(0) = (a);        //MOTOR1
    }

    else {
      out_clear = (1 << dir1);  //DIR FOR MOTOR1.CLOCKWISE
      pwm_cc(0) = -a;           //MOTOR1
    }
  }

  void motor2_value(int a) {
    if (a >= 0) {
      out_set = (1 << dir2);  //DIR FOR MOTOR2., ANTICLOCKWISE
      pwm_cc(1) = a;          //MOTOR2
    }

    else {
      out_clear = (1 << dir2);  //DIR FOR MOTOR2. ,CLOCKWISE
      pwm_cc(1) = -a;           //MOTOR2
    }
  }
  
  void motor3_value(int a) {
    if (a >= 0) {
      out_clear = (1 << dir3);  //DIR FOR MOTOR3.  ANTICLOCKWISE
      pwm_cc(2) = a;            //MOTOR3
    }

    else {
      out_set = (1 << dir3);  //DIR FOR MOTOR3. CLOCKWISE
      pwm_cc(2) = -a;         //MOTOR3
    }
  }

  void esp8266_data() {
    //
    if (i < 2) {  //1,0
      x = uart0_dr;
      if (x != 0x59) {
        i = 0;
        Serial.println("no header");
        // sleep_ms(100);
      } else {
        i++;
        // Serial.println(x, HEX);
      }
    } else {
      if (i < 7) {  //2,3,4,5,6
        data[i - 2] = uart0_dr;
        checksum += data[i - 2];
        i++;
      } else {
        check = uart0_dr;
        if (checksum == check) {
          button = data[0];
          // buttonl = (~button)&0x0F;
          // buttonh = ((~button)&0xF0)>>4;
          kpi = data[1];
          kpf = data[2];
          kdi = data[3];
          kdf = data[4];
          kp = (kpi + (kpf/100.0))*100;
          kd = (kdi + (kdf/100.0))*100;
          // Serial.print(d1);
          // Serial.print("   ");
          // Serial.print(d2);
          // Serial.print("   ");
          // Serial.print(d3);
          // Serial.print("   ");
          // Serial.print("button:");
          // Serial.println(button);
          // Serial.print("   ");
          // Serial.print("kp:");
          // Serial.print(kpf);
          // Serial.print("   ");
          // Serial.print("kd:");
          // Serial.println(kdf);
          i = 0;
          checksum = 0;
        }
      }
    }
  }