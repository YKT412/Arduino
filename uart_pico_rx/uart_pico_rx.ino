#include"address.h"
uint32_t baud = 0;
//brdi= 238  ,  brdf=55
int main(){
  // gpc(16)=2;   //transmit
  gpc(17)=2;   //receive
  gpc(0)=5;
  gpc(1)=5;
  gpc(2)=5;
  gpc(3)=5;
  gpc(4)=5;
  gpc(5)=5;
  gpc(6)=5;
  gpc(7)=5;
  oe = 0x000000ff; 
 clk_peri=0x800;  //11th pin to enable
 uart0_i=238;
 uart0_f=55;
 uart0_lcr= 0x60;
//  uart0_cr=0x101;  //transmit
 uart0_cr=0x201;  //receive

  while(1){
    // if(uart0_fr&0x80){
    //   uart0_dr=0xF0;
    //   sleep_ms(100);  
    // }
    asm("NOP");
    //  out = 0xff;
    if(uart0_fr&0x40){
      out = uart0_dr;
      sleep_ms(100);  
    }
  }
}
