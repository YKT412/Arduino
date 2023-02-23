#include"address.h"

#define temp_f 0x80
#define hum_f 0x00
#define stop  0xff
//brdi= 238  ,  brdf=55    //baud 34800 transmit
 uint8_t x = 0x01, checksum = 0, h1 = 7, h = 25, t = 55, f = 45, i = 0, values_send[5], temp = 0, hum = 0, data = 0;
 
int main(){
  gpc(16)=2;   //transmit
  gpc(25) = 0x05;
  gpc(0)=5;
  gpc(1)=5;
  gpc(2)=5;
  gpc(3)=5;
  gpc(4)=5;
  gpc(5)=5;
  gpc(6)=5;
  gpc(7)=5;
  oe = 0x020100ff;

 clk_peri=0x800;
 uart0_i=238;
 uart0_f=55;
 uart0_lcr= 0x60;
 uart0_cr=0x101; 
 
  checksum = h1 + h1 + h + t + f; 
   values_send[0] = h1;  
   values_send[1] = h1; 
   values_send[2] = h;
   values_send[3] = t;
   values_send[4] = f;  
   values_send[5] = checksum; 

  // irq(20) =  (uint32_t)send_data; 
  // uartimsc = 0x20;
  // nvic_iser = 0x100000;
   asm("NOP");
  //  uart0_dr = 0x00;

  while(1){
 if(uart0_fr&0x80){     //transmit
      uart0_dr = values_send[i];
      out = 0x02000000|(values_send[i]); 
      sleep_ms(100); 
      i++; 
    }
    if(i>5){
      i = 0;
    } 
   asm("NOP");
      
   }
}

// void send_data(){
//  asm("NOP");  
//   uart0_dr = values_send[i];
//    asm("NOP");
//    out = 0x02000000|(values_send[i]); 
//    sleep_ms(100); 
//  asm("NOP");            
//     i++;   
//     if(i>5){
//       i = 0;
//     }  
//  asm("NOP");     
//    uarticr = 0x20;      
// }   