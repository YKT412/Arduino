#include<avr/io.h>
#include<avr/delay.h>
#include<avr/interrupt.h>
#include"motor.h"
#include"ps3.h"
#include"mpu.h"
#include"encoder.h"
#include"function.h"
int que = 0;

//           (Motor1)
//            /   \
//           /     \
//          /       \
//         /         \
//        /           \
//       /             \
// (Motor2) ---------- (Motor3)
//
//-------------------------------------
//
//
//
//
//
//
//
//
//
//
//4 - PG5
//5 - PE3
//6 - PH3
//7 - PH4
//8 - PH5
//11 - PB5
//             DIRECTION PINS
//   MOTOR 1- PG5 | MOTOR 2- PH3 | MOTOR 3- PH5
//            PWM PINS
//   MOTOR 1- PE3 | MOTOR 2- PH4 | MOTOR 3- PB5
//    OC3A     |    OC4B      |   OC1A
 void setup()
{
  Serial.begin(115200);
  set_mpu();
  motor_setup();
  ps3_setup();
  
}

void loop() 
{

  ps3_data();
  pid_mpu();

if (ps3.up)
{
  motor_degree(1010,90,0);
}
else if (ps3.down)
{
  motor_degree(1010,270,0);
}
else if (ps3.right)
{
  motor_degree(1010,0,0);
}
else if (ps3.left)
{
  motor_degree(1010,180,0);
}
else
{
  pid_set();
}
}
