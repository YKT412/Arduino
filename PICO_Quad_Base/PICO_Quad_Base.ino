#include "address.h"
#include "pico_mpu.h"
// diameter of wheel = 15cm
//square base side = 50cm / diagonal = 70cm
#define R 35
#define r 7.5
#define dir1 1
#define dir2 3
#define dir3 5
#define dir4 7
#define led25 25
// motor1 6600 anticlockwise
// motor2 6600 anticlockwise
// motor3 6600 anticlockwise
// motor4 6600 anticlockwise

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
volatile int d1, d2, d3, d4;
float r1 = 0, r2= 0, r3 = 0, r4 = 0;
uint8_t kpi = 0, kpf = 0, kdi = 0, kdf = 0;
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
  pwm_top(0) = 0x000019CB;  //top value 6600
  pwm_div(0) = 0x00000040;  //5000hz freq
  pwm_cc(0) = 0x00000000;

  pwm_csr(1) = 0x00000001;  //pwm enable
  pwm_top(1) = 0x000019CB;  //top value 6600
  pwm_div(1) = 0x00000040;  //5000hz freq
  pwm_cc(1) = 0x00000000;

  pwm_csr(2) = 0x00000001;  //pwm enable
  pwm_top(2) = 0x000019CB;  //top value 6600
  pwm_div(2) = 0x00000040;  //5000hz freq
  pwm_cc(2) = 0x00000000;

  pwm_csr(3) = 0x00000001;  //pwm enable
  pwm_top(3) = 0x000019CB;  //top value 6600
  pwm_div(3) = 0x00000040;  //5000hz freq
  pwm_cc(3) = 0x00000000;
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
  gpc(6) = 0x04;   //pwm
  gpc(7) = 0x05;   //dir
  gpc(14) = 3;     //i2c
  gpc(15) = 3;     //i2c
  gpc(17) = 0x02;  //uart
  led();
  pad_clear(17) = 0x04;
  pad_set(17) = 0x08;
  Serial.begin(115200);
  oe = 0x02ff00ff;
  i2c_setup();
  mpu_setup();
  uart_setup();
  pwm_setup();
}

void loop() {
  mpu_angle();
  // if(button) motor_degree(200, ang[button], 0);
  if (button & 0x01) motor_degree(0, 0, Z);
  else if (button & 0x02) motor_degree(0, 0, 90);
  else d1 = d2 = d3 = d4 = 0;
  //  motor_degree(1000, 90, 0);
  motor1_value(d1);  //1175
  motor2_value(d2);  //1080
  motor3_value(d3);  //1080
  motor4_value(d4);  //1100

  Serial.print(d1);
  Serial.print("  ");
  Serial.print(d2);
  Serial.print("  ");
  Serial.print(d3);
  Serial.print("  ");
  Serial.println(d4);
}

void joystick() {
  if ((x2 - 108) || (y2 - 108)) motor_degree(6600, 0, 0);
  else d1 = d2 = d3 = 0;
  motor1_value(d1);
  motor2_value(d2);
  motor3_value(d3);
  motor4_value(d4);
}

void motor_degree(float base_speed, float degree, float base_w) {
  double radian = (degree) * (0.0174532925);
  float wx = 0;  //cos(radian);  // omega in degree per sec
  float wy = 0;  //sin(radian);
  // float wx = cos(radian);  // omega in degree per sec
  // float wy = sin(radian);
  float wheel_w = base_speed * 7.6364;  //864.28 top base speed in cm per sec
  float w = base_w * R / r;
  // float vx = (x2 - 108) / 90.0;  // for joystick
  // float vy = (y2 - 108) / 90.0;  // for joystick
  d1 = wheel_w * (((1 * wx / sqrt(2)) - (1 * wy / sqrt(2)))) - w;   //0.15
  d2 = wheel_w * (((1 * wx / sqrt(2)) + (1 * wy / sqrt(2)))) - w;   //0.35
  d3 = wheel_w * (((-1 * wx / sqrt(2)) + (1 * wy / sqrt(2)))) - w;  //0.48
  d4 = wheel_w * (((-1 * wx / sqrt(2)) - (1 * wy / sqrt(2)))) - w;

  if ((abs(d1) > 6600) || (abs(d2) > 6600) || (abs(d3) > 6600) || (abs(d4) > 6600)) {
    if (abs(d1) >= abs(d2)) {
      if (abs(d1) >= abs(d3)) {
        if (abs(d1) >= abs(d4)) {
          r1 = 6600 / d1;
          d2 = d2 * abs(r1);
          d3 = d3 * abs(r1);
          d4 = d4 * abs(r1);
          d1 = 6600 * d1/abs(d1);
        } else {
          r4 = 6600 / d4;
          d2 = d2 * abs(r4);
          d3 = d3 * abs(r4);
          d1 = d1 * abs(r4);
          d4 = 6600 * d4/abs(d4);
        }
      } else {
        if (abs(d4) >= abs(d3)) {
          r4 = 6600 / d4;
          d2 = d2 * abs(r4);
          d3 = d3 * abs(r4);
          d1 = d1 * abs(r4);
          d4 = 6600 * d4/abs(d4);
        } else {
          r3 = 6600 / d3;
          d2 = d2 * abs(r3);
          d4 = d4 * abs(r3);
          d1 = d1 * abs(r3);
          d3 = 6600 * d3/abs(d3);
        }
      }
    } else {
      if (abs(d2) >= abs(d3)) {
        r2 = 6600 / d2;
        d4 = d4 * abs(r2);
        d3 = d3 * abs(r2);
        d1 = d1 * abs(r2);
        d2 = 6600 * d2/abs(d2);

      } else {
        if (abs(d4) >= abs(d3)) {
          r4 = 6600 / d4;
          d2 = d2 * abs(r4);
          d3 = d3 * abs(r4);
          d1 = d1 * abs(r4);
          d4 = 6600 * d4/abs(d4);
        } else {
          r3 = 6600 / d3;
          d2 = d2 * abs(r3);
          d4 = d4 * abs(r3);
          d1 = d1 * abs(r3);
          d3 = 6600 * d3/abs(d3);
        }
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

void motor4_value(int a) {
  if (a >= 0) {
    out_set = (1 << dir4);  //DIR FOR MOTOR4.ANTICLOCKWISE
    pwm_cc(3) = (a);        //MOTOR1
  }

  else {
    out_clear = (1 << dir4);  //DIR FOR MOTOR4.CLOCKWISE
    pwm_cc(3) = -a;           //MOTOR1
  }
}

void esp8266_data() {
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
        kp = (kpi + (kpf / 100.0)) * 100;
        kd = (kdi + (kdf / 100.0)) * 100;
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