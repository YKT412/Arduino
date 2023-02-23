#include<avr/interrupt.h>
#include<avr/io.h>
#define input_pin PE4
#define dir1 8
#define pwm1 11
#define dir2 6
#define pwm2 7

long int rpm1 = 0, rpm2 = 0, desired_rpm = 1500, d_error1=0 ,error1=0,e1 = 0,pe1=0,d_error2=0 ,error2=0,e2 = 0,pe2=0;
long int prev_time1, prev_time2, duration1, duration2;
long int value1 = 0, value2 =0;
float kp1 = 9;
float P1 = 0, z1 = 0;
float kp2 = 1;
float P2 = 0, z2 = 0;
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
  DDRC = (1<<6)|(1<<4)|(1<<2);
  DDRG = (1<<PG2)|(1<<PG0);
  DDRL = (1<<PL4);
  DDRA = (1<<PA0)|(1<<A2);
  DDRE &=~(1<<4) | (1<<5); // ir
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
  ICR4 = 5500; // roller-1
  ICR1 = 5500; // roller-2  
}

void loop() {  
  if (Serial.available()) {
    p = Serial.parseInt() + Serial.parseInt();
  }
OCR1A=p+z1;
OCR4B=p+z2;
 currentstate1 = bool(PINE & (1 << PE4));
  if (prevstate1 != currentstate1) {
    if (currentstate1 == LOW) {
      duration1 = (h - prev_time1);
      prev_time1 = h;
      rpm1 = (243252.54 / duration1);
    }
    prevstate1 = currentstate1;
  }
 currentstate2 = bool(PINE & (1 << PE5));
  if (prevstate2 != currentstate2) {
    if (currentstate2 == LOW) {
      duration2 = (h - prev_time2);
      prev_time2 = h;
      rpm2 = (243252.54 / duration2);
    }
//    Serial.print(rpm1);
//    Serial.print("  ");
//    Serial.println(rpm2);
    prevstate2 = currentstate2;
  }
  e1=p-rpm1;
  P1=kp1*e1;
  z1=P1;
  if(rpm1>p+100){
    OCR1A=0;
  }
  e2=p-rpm2;
  P2=kp2*e2;
  z2=P2;
  if(rpm2>p+100){
    OCR4B=0;
  }
//  if(PINA & (1 << PA0)){
//    PORTL |= (1<<PL4);
//    PORTG &=~ (1<<PG0);
////    Serial.println("open");
//  }
//  else if (PINA &(1<<PA2))
//  {
//    PORTG |= (1<<PG0);
//    PORTL &=~ (1<<PL4);
////     Serial.println("close");
//  }



  
//  else Serial.println("no");
  
////  pe1=e1;
  Serial.print(rpm1);
  Serial.print("  "); 
////  Serial.print(z1);
////  Serial.print("  ");
  Serial.println(rpm2);
////  Serial.print("  ");
////  Serial.println(OCR1A);
////  Serial.print("  ");
////    Serial.println(OCR4B);
////  Serial.print("  ");
}
ISR(TIMER3_OVF_vect)
{
  TCNT3 = 63535;
  h++;
 }
