#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "ps3.h"
#include "mpu.h"
//#include <math.h>

#define Motor2_PWM PE3
#define Motor2_Dir PG5
#define Motor1_PWM PH4
#define Motor1_Dir PH3
#define Motor3_PWM PB5
#define Motor3_Dir PH5
//#define motor_speed 50

float n1 = 0, n2 = 0, n3 = 0;

void timer_config () {
  TCCR1A = (1 << WGM11) | (1 << COM1A1);
  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);

  TCCR3A = (1 << WGM31) | (1 << COM3A1);
  TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS30);

  TCCR4A = (1 << WGM41) | (1 << COM4B1);
  TCCR4B = (1 << WGM42) | (1 << WGM43) | (1 << CS40);

  ICR1 = 4000;
  ICR3 = 4000;
  ICR4 = 4000;
}

//void motor_pwm (float d1, float d2, float d3) {
//  if (d1 < 0) PORTH &= ~(1 << Motor1_Dir); //less than 0 for CW
//  else PORTH |= (1 << Motor1_Dir);
//  if (d2 < 0) PORTG &= ~(1 << Motor2_Dir);
//  else PORTG |= (1 << Motor2_Dir);
//  if (d3 < 0) PORTH &= ~(1 << Motor3_Dir);
//  else PORTH |= (1 << Motor3_Dir);
//  OCR4B = double(4000 * d1);
//  OCR3A = double(4000 * d2);
//  OCR1A = double(4000 * d3);
//  Serial.print(OCR4B);
//  Serial.print("  ");
//  Serial.print(OCR3A);
//  Serial.print("  ");
//  Serial.println(OCR1A);
//}

void motor_pwm (float d1, float d2, float d3) {
  if (d1 < 0) {
    PORTH &= ~(1 << Motor1_Dir);
    OCR4B = (-1) * d1;
  }
  else {
    PORTH |= (1 << Motor1_Dir);
    OCR4B = d1 ;
  }
  if (d2 < 0) {
    PORTG &= ~(1 << Motor2_Dir);
    OCR3A = (-1) * d2 * 1.3;
  }
  else {
    PORTG |= (1 << Motor2_Dir);
    OCR3A = d2 * 1.3;
  }
  if (d3 < 0) {
    PORTH &= ~(1 << Motor3_Dir);
    OCR1A = (-1) * d3 * 0.81;// * 0.81;
  }
  else {
    PORTH |= (1 << Motor3_Dir);
    OCR1A = d3 * 0.81;
  }
  //  Serial.print(OCR4B);
  //  Serial.print("    ");
  //  Serial.print(OCR3A);
  //  Serial.print("    ");
  //  Serial.print(OCR1A);
  //  Serial.print("    ");
}

//void motor_dir (unsigned char m1, unsigned char m2, unsigned char m3) {
//  if (m1) PORTH |= (1 << Motor1_Dir);
//  else PORTH &= ~(1 << Motor1_Dir);
//  if (m2) PORTG |= (1 << Motor2_Dir);
//  else PORTG &= ~(1 << Motor2_Dir);
//  if (m3) PORTH |= (1 << Motor3_Dir);
//  else PORTH &= ~(1 << Motor3_Dir);
//}

void base_motion_wrt_x (float deg_x, float base_speed, float w) {
  //float rad = ((deg_x * M_PI) / 180);
  float rad = deg_x * 0.01744;
  float vx = cos(rad);
  float vy = sin(rad);
  n1 = (((-2 / 3.0) * vx) * base_speed) + ((1 / 3.0) * w);
  n2 = ((((1 / 3.0) * vx) + ((-1 / sqrt(3)) * vy)) * base_speed) + ((1 / 3.0) * w);
  n3 = ((((1 / 3.0) * vx) + ((1 / sqrt(3)) * vy)) * base_speed) + ((1 / 3.0) * w);
  motor_pwm (n1-P, n2-P, n3-P);
}

void setup() {
  Serial.begin(115200);
  DDRB = (1 << Motor3_PWM);
  DDRE = (1 << Motor2_PWM);
  DDRG = (1 << Motor2_Dir);
  DDRH = (1 << Motor1_PWM) | (1 << Motor1_Dir) | (1 << Motor3_Dir);
  timer_config();
  set_mpu();
  ps3_setup();
}
void loop () {
  pid_mpu();
  ps3_data();
  Serial.println(actual);
  if (ps3.cross) {
    motor_pwm (0, 0, 0);
    while (!(ps3.start)) {
      ps3_data();
    }
  }
  else if (ps3.up) {
    //      Serial.println("UP");
    if (ps3.left) {
      base_motion_wrt_x (135, 3000, 0);
    }
    else if (ps3.right) {
      base_motion_wrt_x (45, 3000, 0);
    }
    else {
      base_motion_wrt_x (90, 3000, 0);
    }
    //      _delay_ms(1000);
  }
  else if (ps3.down) {
    if (ps3.left) {
      base_motion_wrt_x (225, 3000, 0);
    }
    else if (ps3.right) {
      base_motion_wrt_x (315, 3000, 0);
    }
    else {
      base_motion_wrt_x (270, 3000, 0);
    }
    //      Serial.println("DOWN");
    //      motor_pwm (0, 0, 0);
    //      _delay_ms(1000);
  }
  else if (ps3.right) {
    //      Serial.println("RIGHT");
    //      motor_pwm (0, 0, 0)
    base_motion_wrt_x (0, 3000, 0);
    //      _delay_ms(1000);
  }
  else if (ps3.left) {
    //      Serial.println("LEFT");
    //      motor_pwm (0, 0, 0)
    base_motion_wrt_x (180, 3000, 0);
    //      _delay_ms(1000);
  }
  else if (ps3.circle) {
    //      Serial.println("CIRCLE");
    base_motion_wrt_x (0, 0, 3000);
  }
  else if (ps3.square) {
    //      Serial.println("CIRCLE");
    base_motion_wrt_x (0, 0, -3000);
  }
  else if (ps3.tri) {
    //      Serial.println("CIRCLE and UP");
    base_motion_wrt_x (90, 1000, 4000);
  }
  else {
    //      Serial.println("Not");
    motor_pwm (-P, -P, -P);
  }
  //    motor_pwm(2000, 2000, 2000);
  //    delay(10000);
  //    motor_pwm(0, 0, 0);
}
