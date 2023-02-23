#include<avr/interrupt.h>
#include<avr/io.h>
#define input_pin PE4
#define dir1 8
#define pwm1 11
#define dir2 6
#define pwm2 7

long int rpm1 = 0, rpm2 = 0, desired_rpm = 1500, error = 0;
long int prev_time1, prev_time2, duration1, duration2;
long int value1 = 0, value2 = 0;
float kp = 5, ki = 0, kd = 0;
float P = 0, I = 0, D = 0, z = 0;
int p = 0;
unsigned long h = 0;
boolean currentstate1, prevstate1, currentstate2, prevstate2;

//   Roller-1      (lagori)    Roller-2
//      6,7                      8,11
// dir1 = ph5,d8
// dir2  =ph3,d6
// pwm1 = pb5 ,d11
// pwm2 = ph4 , d7
void setup() {
  Serial.begin(115200);
  cli();
  DDRE &=~(1<<PE4) | (1<<PE5); // ir 
  DDRB |= (1<<PB5);      
  DDRH |= (1<<PH4)|(1<<PH5)|(1<<PH3);
  prev_time1 = 0;
  prev_time2 = 0;
  prevstate1 = 0x00;
  prevstate2 = 0x00;
  PORTH |= (1<<PH5)|(1<<PH3);
  
  TCCR1A = 0x82;  //roller-2 timer
  TCCR1B = 0x19;
  TCCR4B = 0x19;  // roller-1 timer
  TCCR4A = 0x22;
  TCCR3B = (1 << CS31); // timer for delay
  TIMSK3 = (1 << TOIE3);
  sei();
  ICR4 = 5700; // roller-1
  ICR1 = 5050; // roller-2
  
}

void loop() {
  if (Serial.available()) {
    p = Serial.parseInt() + Serial.parseInt();
    OCR1A=p;
    OCR4B=p;
  }
  //Serial.println(desired_rpm);
 currentstate1 = digitalRead(2);

  if (prevstate1 != currentstate1) {
    if (currentstate1 == 0x00) {
      duration1 = (h - prev_time1);
      prev_time1 = h;
      rpm1 = (243252.54 / duration1);
    }
    prevstate1 = currentstate1;
  }
 currentstate2 = digitalRead(3);
  
  if (prevstate2 != currentstate2) {
    if (currentstate2 == 0x00) {
      duration2 = (h - prev_time2);
      prev_time2 = h;
      rpm2 = (243252.54 / duration2);
    }
    Serial.print(rpm1);
    Serial.print("  ");
    Serial.println(rpm2);
    prevstate2 = currentstate2;
  }
}
ISR(TIMER3_OVF_vect)
{
  TCNT3 = 63535;
  h++;
 }
