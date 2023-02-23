#include "address.h"

int main() {
  gpc(25) = 0x00000005;  //5+8
  // oe_clear = 0x02000000;
  oe = 0x02000000;
  // timelr
  



  while (1) {
    
    timer_delay(2000000);
    
    out_toggle = 0x02000000;  
   
  }
}

void timer_delay(uint32_t microsecond){
  alarm1 = microsecond + timelr;
  while(armed & 0x02);
 asm("NOP");  
  
  
}