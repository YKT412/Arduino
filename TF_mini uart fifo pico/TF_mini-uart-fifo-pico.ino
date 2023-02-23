#include "address.h"

uint16_t i = 0,a = 0, tf_data[8], check = 0, checksum = 0;
uint16_t dist = 0;

 int main(){
  gpc(25) = 0x05;
  gpc(17) = 0x02;  //receive
  gpc(19) = 0x02;  //rts
  // gpc(8) = 0x02;   //transmit
  in = 0x020000ff;
  gpc(0)=5;
  gpc(1)=5;
  gpc(2)=5;
  gpc(3)=5;
  gpc(4)=5;
  gpc(5)=5;
  gpc(6)=5;
  gpc(7)=5;
  pad_clear(17)= 0x04;      // pull down diabled
  pad_set(17) = 0x08;  
  oe = 0x0200ffff;
  clk_peri = 0x800;
  
  uart0_cr=0x2201;   //receive rtsen  
  uart0_i = 72;     //115200
  uart0_f = 10;
  uartifls = 0b100000;  //1/4 full of 32 byte depth
  uart0_lcr= 0x70;  //fifo enable receive enable
  
  asm("NOP");
  // out=  0x02000000|(dist);
  asm("NOP");
  irq(20) =  (uint32_t)tfmini_data; 
  uartimsc = 0x10;
  nvic_iser = 0x100000;
  asm("NOP");
  
  
  while(1){
   
  //   if(uart0_fr&0x40){
    asm("NOP");  
    asm("NOP");  
    asm("NOP");         
    if(a){
      
      out =  (uint16_t)(uart0_dr>>8);
      sleep_ms(1500);
          
    }
  //  } 
  //  tfmini_data();     
  //  out= i|(0x02000000);
    // out = 0x00;
  //  out=  0x02000000|(dist);  
  } 

}     
void tfmini_data(){
  a = 1 ;
   uarticr = 0x10; 
   gpc(17) = 0x00; 
        
}       