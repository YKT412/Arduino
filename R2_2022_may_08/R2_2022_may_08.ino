#include "MPU.h"
#include "motor.h"
#include "ps3.h"
#include "functions.h"

void setup()
{
  motor_init();
  Serial.begin(115200);
  Serial2.begin(74880);
  i2c_send();
  set_mpu();
  ps3_setup();


}
void loop()
{
  pid_mpu();
  ps3_data();
  control();
  lpm_pp();
}
