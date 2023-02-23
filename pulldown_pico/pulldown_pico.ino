#include"address.h"

int main(){
  gpc(25) = 0x00000005;
  gpc(17) = 0x00000005;
  oe = 0x02000000;          //25
  oe_clear = 0x00020000;    //17
  pad_clear(17)= 0x04;      // pull down diabled
  pad_set(17) = 0x08;       // pull up enabled
  

  while(1){
    asm("NOP");
    out= (~in)<<8;
    
  }
}
