#include "address.h"

uint8_t t = 1, x = 0;
float angle = 0;

int main() {

  gpc(16) = 0x04;  //pwm
  gpc(17) = 0x04;  //dir
  gpc(18) = 0x04;
  gpc(19) = 0x00000004;
  gpc(20) = 0x00000004;
  gpc(21) = 0x00000004;
  gpc(22) = 0x00000004;
  gpc(26) = 0x00000004;
  // gpc(25) = 0x00000004;
  // oe = 0x02000000;
  pwm_csr(0) = 0x00000001;  //pwm enable
  pwm_top(0) = 0x000000ff;  //top value
  pwm_div(0) = 0x00000000;
  pwm_cc(0) = 0x00000000;

  pwm_csr(1) = 0x00000001;  //pwm enable
  pwm_top(1) = 0x000000ff;  //top value
  pwm_div(1) = 0x00000000;
  pwm_cc(1) = 0x00000000;

  pwm_csr(2) = 0x00000001;  //pwm enable
  pwm_top(2) = 0x000000ff;  //top value
  pwm_div(2) = 0x00000000;
  pwm_cc(2) = 0x00000000;

  pwm_csr(3) = 0x00000001;  //pwm enable
  pwm_top(3) = 0x000000ff;  //top value
  pwm_div(3) = 0x00000000;
  pwm_cc(3) = 0x00000000;

  pwm_csr(5) = 0x00000001;  //pwm enable
  pwm_top(5) = 0x000000ff;  //top value
  pwm_div(5) = 0x00000000;
  pwm_cc(5) = 0x00000000;

  pwm_en = 0x1f;

  while (1) {
    asm("NOP");

    x = (sin((angle + 0) * 3.14 / 180) + 1) * 127;
    pwm_cc(0) = 0x00000000 | (x);
    x = (sin((angle + 45) * 3.14 / 180) + 1) * 127;
    pwm_cc(0) |= 0x00000000 | (x << 16);
    x = (sin((angle + 90) * 3.14 / 180) + 1) * 127;
    pwm_cc(1) = 0x00000000 | (x);
    x = (sin((angle + 180) * 3.14 / 180) + 1) * 127;
    pwm_cc(1) |= 0x00000000 | (x << 16);
    x = (sin((angle + 135) * 3.14 / 180) + 1) * 127;
    pwm_cc(2) = 0x00000000 | (x);
    x = (sin((angle + 270) * 3.14 / 180) + 1) * 127;
    pwm_cc(2) |= 0x00000000 | (x << 16);
    x = (sin((angle + 315) * 3.14 / 180) + 1) * 127;
    pwm_cc(3) = 0x00000000 | (x);
    x = (sin((angle + 360) * 3.14 / 180) + 1) * 127;
    pwm_cc(5) |= 0x00000000 | (x);
    x = (sin((angle + 405) * 3.14 / 180) + 1) * 127;
    delay(5);
    // pwm_cc(0) = 0x00000000;
    // pwm_cc(1) = 0x00ff00ff;
    // pwm_cc(2) = 0x00ff00ff;
    // pwm_cc(3) = 0x00ff00ff;
    // pwm_cc(4) = 0x00ff00ff;
    angle++;
    //  if(x >= 90 || x<= 0){
    //    t*=-1;
    //   }
  }
}