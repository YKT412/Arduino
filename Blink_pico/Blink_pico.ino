#include"address.h"

void setup(){
  gpc(18) = 0x00000005;  //funcsel GPIO5  
  oe = 0x02ff0000;  //25

  while(1){
    sleep_ms(1000);
    out_toggle = 0x02000000;
  }
}
void loop(){}
