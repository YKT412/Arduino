#include"address.h"
//34800
//brdi= 238  ,  brdf=55
uint8_t x = 0x11;
int main(){
  gpc(16)=2;   //transmit
  // gpc(17)=2;   //receive
  // gpc(25)=5;
  oe = 0x00010000; 
 clk_peri=0x800;
 uart0_i= 238;
 uart0_f= 55;
 uart0_lcr= 0x60;
 uart0_cr=0x101;  //transmit
//  uart0_cr=0x201;  //receive

  while(1){
  
    if(uart0_fr&0x80){     //transmit
      uart0_dr = x;
      // sleep_ms(100);  
    }
    // x = (x<<1);
    // if(uart0_fr&0x10){     //receive
    //   out = usar0_dr;
    //   sleep_ms(100);  
    // }
  }
}