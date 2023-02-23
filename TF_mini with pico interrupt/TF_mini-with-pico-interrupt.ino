#include "address.h"

uint8_t i = 0, tf_data[6], check = 0, checksum = 0xB2;
uint16_t dist = 0;

void setup() {
  gpc(25) = 0x05;
  gpc(17) = 0x02;  //receive
  // gpc(19) = 0x02;  //rts
  // gpc(8) = 0x02;   //transmit
  // gpc(0)=5;
  // gpc(1)=5;
  // gpc(2)=5;
  // gpc(3)=5;
  // gpc(4)=5;
  // gpc(5)=5;
  // gpc(6)=5;
  // gpc(7)=5;
  pad_clear(17) = 0x04;  // pull down diabled
  pad_set(17) = 0x08;    // pull up enable
  oe = 0x02000000;
  clk_peri = 0x800;
  uart0_i = 72;  //115200
  uart0_f = 10;
  uart0_lcr = 0x60;  //8 bit
  uart0_cr = 0x201;  //receive enable
  // uartifls = 0b1000;  //1/4 full of 32 byte depth
  Serial.begin(115200);
  // asm("NOP");
  // asm("NOP");
  irq(20) = (uint32_t)tfmini_data;
  uartimsc = 0x10;
  nvic_iser = 0x100000;
  asm("NOP");
}
void loop() {
  asm("NOP");
  asm("NOP");
  asm("NOP");
}
void tfmini_data() {

  if (i < 2) {
    uint8_t x = uart0_dr;
    if (x == 0x59) {  //1,0
      i++;
    } else {
      i = 0;
    }
  } else {  //2,3,4,5,6, 7
    if (i < 8) {
      tf_data[i - 2] = uart0_dr;
      checksum += tf_data[i - 2];
      i++;
    } else {
      check = uart0_dr;
      if (check == checksum) {
        dist = tf_data[0];
        dist |= (tf_data[1] << 8);
        Serial.println(dist);
        i = 0;
        checksum = 0xB2;
      }
    }
  }
}