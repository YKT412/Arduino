#include <avr/io.h>
#include <avr/interrupt.h>
#include "functions.h"
#include <util/delay.h>


int main() {

  init();
  Serial_int_setup();
  Serial.begin(115200);
  pin_setup();

  timer_setup(500);

  //  while (!(PINL & (1 << BOH_limit_pin))) {
  //    if (!(PINL & (1 << BOH_limit_pin))) {
  //      PORTH &= ~(1 << BOH_dir);
  //      PORTB ^= (1 << BOH_pulse);
  //    }
  //    else {
  //      PORTH &= ~(1 << BOH_dir);
  //      PORTB &= ~(1 << BOH_pulse);
  //    }
  //    _delay_us(500);
  //  }

  while (1) {

    Serial.print(values_send[2]);// horizontal_angle
    Serial.print("    ");
    Serial.print(dist);// horizontal_angle
    Serial.print("    ");
    Serial.print(values_send[3]);//vertical_angle
    Serial.print("    ");
    Serial.print(rpm);
    Serial.print("    ");
    Serial.print(checksum2);
    Serial.println("    ");



    if (Serial.available()) {
      if (j == 0) {
        values_send[2] = Serial.parseInt();  //horizontal_angle  // 2nd , 3rd
        j = 1;
      }
      if (j == 1) {
        dist = Serial.parseInt();            // dist
        j = 0;
      }

    }



    //        steps = (horizontal_angle*8.889);
    //        set_pos(steps);
    rpm_calculation(values_send[2], dist);   // horizontal_angle, dist
    //
  }
}
