#include <avr/io.h>
#include <avr/interrupt.h>
#include "functions_mega.h"
#include <util/delay.h>
#define piSerial Serial3

int main() {
  init();
  Serial_int_setup();
  piSerial.begin(115200);
  pin_setup();
  timer_setup(700);
  PORTH |= (1 << BOH_dir);
  while (!(PINF & (1 << BOH_limit_pin))) {
    PORTB ^= (1 << BOH_pulse);
    _delay_us(500);
  }
  prev_pos = 638;
  set_pos(0);
  //    Serial.println("done!!!");

  while (1) {
    if (piSerial.available()) {
      H_pos = piSerial.parseInt();
      piSerial.read();
      H_pos = constrain(H_pos, -90, 90);
//      piSerial.println(dist);
      set_pos(H_pos*8.89);
    }
    values_send[2]=H_pos+92;
    rpm_calculation(dist);
    
     
    piSerial.print(dist);
    piSerial.print("  ");
    piSerial.print(rpm);
    
    piSerial.print("  ");
    piSerial.print(H_pos+91);
    piSerial.print("  ");
    
    piSerial.println(vertical_angle*180/pi);
    _delay_ms(10);
  }
}
