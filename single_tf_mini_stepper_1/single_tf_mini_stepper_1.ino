#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define pulse_pin PB4   //stepper_1 pulse pin 10
#define limit_pin PL3   //stepper_1 limit pin 46
#define limit_pin2 PL1  //stepper_2 limit pin 48
#define dir PH6       //stepper_1 direction pin 9
#define pulse_pin2 PB5   //stepper_2 pulse pin 11
#define dir2 PH5         //stepper_2 direction pin 8
#define pi 3.1415

int step_t = 3200, step_c = 0 , target_step = 0 , target_step2 = 0;
uint8_t cnt = 0;
char tf_data[4];
uint16_t distance = 0, distance_2 = 0, prev_dist = 0;
float target_dist = 0;
int pulse = 0, prev_pos = 0, c = 0, a = 0, pulse2 = 0, prev_pos2 = 0;
unsigned int value = 0, steps = 3200;
float phi = 0, x = 39, phi_rad = 0, theta = 0, theta_rad = 0 , b = 0;

int main() {
  DDRL &= ~((1 << limit_pin) | (1 << limit_pin2));
  DDRB |= ((1 << pulse_pin) | (1 << pulse_pin2));
  DDRH |= ((1 << dir) | (1 << dir2));
  PORTL |= ((1 << limit_pin) | (1 << limit_pin2));
  Serial_int_setup();
  Serial.begin(115200);
  timer_setup(500);

  while (PINL & (1 << limit_pin2)) {
    PORTH &= ~(1 << dir2);
    PORTB ^= (1 << pulse_pin2);
    PORTB &= ~(1 << pulse_pin2);
    _delay_us(500);
  }
  prev_dist = distance_2;
  value = distance_2;
  prev_pos2 = 0;
  _delay_ms(1000);

   while (1) {
    Serial.print(distance_2);
    Serial.print("    ");
    Serial.print(target_dist_2);
    Serial.print("    ");
    Serial.println(value);


if (a == 0) {
      if (prev_pos_2 == 100)
      {
        set_pos_2(1300);
        prev_dist = 10000;
      }
      else {
        set_pos2(target_step2);
        if (prev_pos == 1300) {
          a = 1;
          
           //-130
          //          set_pos(target_step);//130
        }
      }
    }
