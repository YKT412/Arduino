#include "address.h"


int main() {

  gpc(25) = 0x00000005;
  gpc(20) = 0x00000003;
  gpc(21) = 0x00000003;
  gpc(0) = 0x00000005;
  gpc(1) = 0x00000005;
  gpc(2) = 0x00000005;
  gpc(3) = 0x00000005;
  gpc(4) = 0x00000005;
  gpc(5) = 0x00000005;
  gpc(6) = 0x00000005;
  gpc(7) = 0x00000005;
  oe = 0x020000ff;

  IC_CON(0) = 0x04;  //speed
                     //  IC_TAR(0) = 0x00;
  IC_SAR(0) = 0x55;  //Slave address
                     //  IC_DATA_CMD(0) = 0x00;
                     //  IC_RX_TL(0) = 0x00;    // NUMBER OF ENTRIES
                     //  IC_FS_SCL_HCNT(0) =  4;
                     //  IC_FS_SCL_LCNT(0) =  4.7;
  IC_EN(0) = 0x01;   //Enable

  while (1) {
    sleep_ms(100);
    out_toggle = 0x02000000;
    if (IC_RAW_INTR_STAT(0) & 0x04) {
      //  Serial.println(IC_DATA_CMD(0));
      out = IC_DATA_CMD(0);
    }
  }
}