#include <avr/io.h>
#include <util/delay.h>
#include "ps3.h"
#include "mpu.h"
#include "motor.h"

int d1, d2, d3;

void motor_degree(float base_speed, float degree, float wr) {
  double radian = (degree + 180) * (0.01745);
  float vx = cos(radian);
  float vy = sin(radian);
  d1 = base_speed * (((-2 * vx / 3) + (0 * vy) + (0.15 * wr)));
  d2 = base_speed * (((vx / 3) - (vy / (sqrt(3)) + (0.35 * wr))));
  d3 = base_speed * ((vx / 3) + (vy / (sqrt(3)) + (0.48 * wr)));
}

// MOTOR ORENTATION
//        M1
//       /  \
//      /    \
//     M2----M3
//
//D4 => PORTG(5)
//D5 => PORTE(3)
//D6 => PORTH(3)
//D7 => PORTH(4)
//D8 => PORTH(5)
//D11 => PORTB(5)
// offset = -6739  -1779 481 112 -78 -16


void setup() {


  DDRB = (1 << PINB5);
  DDRH = (1 << PINH3) | (1 << PINH4) | (1 << PINH5);
  DDRG = (1 << PING5);
  DDRE = (1 << PINE3);


  //     | MOTOR 1 |MOTOR 2  | MOTOR3
  //DIR= |D6       |D4      |D8
  //PWM= |D7(OC4B) |D5(OC3A)|D11(OC1A)

  set_mpu();
  TCCR4A = (1 << WGM41) | (1 << COM4B1);               //NON-INVERTING MODE 8 PRESCALER
  TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41);  //MOTOR1
  TCCR3A = (1 << WGM31) | (1 << COM3A1);               //NON-INVERTING MODE 8 PRESCALER
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);  //MOTOR2
  TCCR1A = (1 << WGM11) | (1 << COM1A1);               //NON-INVERTING MODE 8 PRESCALER
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  //MOTOR3

  //PORTG &=~ (1<<PING5);//DIR FOR MOTOR2.
  //PORTH &=~ (1<<PINH3)|(1<<PINH5);//DIR FOR MOTOR1 & MOTOR3.

  ICR1 = 2000;
  ICR3 = 2000;
  ICR4 = 2000;

  ps3_setup();
  Serial.begin(9600);
}

//************************************

void loop() {
  pid_mpu();
  ps3_data();

  if (ps3.cross) {

    OCR4B = 0;  //MOTOR1
    OCR3A = 0;  //MOTOR2
    OCR1A = 0;  //MOTOR3
    while (!ps3.start) ps3_data();
  }

  if (ps3.up) {
    if (ps3.right) {
      //for (int i =0; i < 10; ++i)
      {
        motor_degree(800, 0, -1);  //+45 degree take vx and vy negetive for positive degree.
      }

      motor1_value(d1 + P);
      motor2_value(d2 + P);
      motor3_value(d3 + P);
      reset();
      while (ps3.up && ps3.right) {
        ps3_data();
        if (ps3.cross) {

          OCR4B = 0;  //MOTOR1
          OCR3A = 0;  //MOTOR2
          OCR1A = 0;  //MOTOR3
          while (!ps3.start) ps3_data();
        }
      }
      motor1_value(0);
      motor2_value(0);
      motor3_value(0);
      set_mpu();

    }

    else if (ps3.left) {
      motor_degree(800, 0, 0);  //+45 degree take vx and vy negetive for positive degree.
      motor1_value(d1 + P);
      motor2_value(d2 + P);
      motor3_value(d3 + P);
    } else {
      motor1_value(0 + P);
      motor2_value(1000 + P);
      motor3_value(-830 + P);
    }
  }

  else if (ps3.down) {
    if (ps3.right) {
      motor_degree(1000, 315, 0);  //+45 degree take vx and vy negetive for positive degree.
      motor1_value(d1 + P);
      motor2_value(d2 + P);
      motor3_value(d3 + P);
    } else if (ps3.left) {
      motor_degree(1000, 225, 0);  //+45 degree take vx and vy negetive for positive degree.
      motor1_value(d1 + P);
      motor2_value(d2 + P);
      motor3_value(d3 + P);
    } else {
      motor1_value(0 + P);
      motor2_value(-1000 + P);
      motor3_value(1000 + P);
    }
  }

  else if (ps3.right) {
    motor1_value(990 + P);
    motor2_value(-975 + P);
    motor3_value(-875 + P);
  } else if (ps3.left) {
    motor1_value(-990 + P);
    motor2_value(900 + P);
    motor3_value(875 + P);
  }

  else if (ps3.R1) {
    {
      motor1_value(600);
      motor2_value(600);
      motor3_value(600);
      reset();
      while (ps3.R1) {
        ps3_data();
        if (ps3.cross) {

          OCR4B = 0;  //MOTOR1
          OCR3A = 0;  //MOTOR2
          OCR1A = 0;  //MOTOR3
          while (!ps3.start) ps3_data();
        }
      }
      motor1_value(0);
      motor2_value(0);
      motor3_value(0);
      set_mpu();
    }

  }  //base clockwise

  else if (ps3.L1)

  {


    motor1_value(-600);
    motor2_value(-600);
    motor3_value(-600);
    reset();
    while (ps3.L1) {
      ps3_data();
      if (ps3.cross) {

        OCR4B = 0;  //MOTOR1
        OCR3A = 0;  //MOTOR2
        OCR1A = 0;  //MOTOR3
        while (!ps3.start) ps3_data();
      }
    }
    motor1_value(0);
    motor2_value(0);
    motor3_value(0);
    set_mpu();

  }


  else if (ps3.square) {
    motor_degree(1000, 0, 0);  //+45 degree take vx and vy negetive for positive degree.
    motor1_value(d1 + P);
    motor2_value(d2 + P);
    motor3_value(d3 + P);

  }

  else {
    motor1_value(0);
    motor2_value(0);
    motor3_value(0);
  }
}