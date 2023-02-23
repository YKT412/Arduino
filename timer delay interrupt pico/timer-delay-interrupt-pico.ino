#include "address.h"

int main() {
  gpc(25) = 0x00000005;  //5+8
  // oe_clear = 0x02000000;
  oe = 0x02000000;
  // out_set
  // timelr
   irq(1) = (uint32_t)timer_delay; 
   inte |= 0x02;
   nvic_iser = 0x02;
   timer_delay();

  while (1) { 
   
  }
}

void timer_delay(){
  alarm1 = 1000000 + timelr;
  out_toggle = 0x02000000;
  intr = 0x02;
  // while(armed & 0x02);
//  asm("NOP");  
  
  
}