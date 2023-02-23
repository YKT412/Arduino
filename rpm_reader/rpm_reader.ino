#include <avr/io.h>
#include <avr/interrupt.h>
#define get_time ovf * 65535 + TCNT1
#define p_on 50
#define i_on 5
#define d_on 0

bool flag = 0;
uint32_t ovf = 0, before = 0, ct = 0, var, c = 0;
float rpm = 0, dt = 0, dt2 = 0;
float Kp = 0, Kd = 0, Ki = i_on, P = 0, D = 0, Z = 0, I = 0;
long new_millis = 0, prev_millis = 0;
int actual = 0;
int desired = 0;
float previous_error = 0, e = 0;

/* Distance(m)   RPM 
    3         2450
    2.55      2360   
*/

void setup() {
  //COUNTER SETUP
  TCCR5A = 0X00;
  DDRL = 0x00;
  PORTL = 0X04;
  OCR5A = 600;
  TCCR5B = 0X07;
  // TIMSK5 = 0X02;
  DDRE = (1 << PE3);
  PORTE = (1 << PE3);
  // CLOCK SETUP
  Serial.begin(115200);
  TCCR1A = 0X00;
  TIMSK1 = 0X02;
  OCR1A = 20000;
  TCCR1B = 0X0A;
  //PWM SETUP
  TCCR3A = (1 << WGM31) | (1 << COM3A1);  //NON-INVERTING MODE 8 PRESCALER
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);
  ICR3 = 5500;
}

void loop() {
  
  if (Serial.available()) {
    desired = Serial.parseInt();
    while (Serial.available()) {
      Serial.read();
    }
     Ki = i_on;
     Kp = 0;   
     Kd = 0;
     ovf = 0;  
     c=0;
     flag = 0;     
  }
  // OCR3A = desired;
  // pid();
  OCR3A = constrain(Z, 0, 4000);
  Serial.print(desired);
  Serial.print("  ");
  Serial.print(rpm);
  Serial.print("  ");
  Serial.print(ct); 
  Serial.print("  ");  
  Serial.println(I);
  delay(5);
}

ISR(TIMER1_COMPA_vect) {
  ovf++;
  rpm = TCNT5 * 10;
  TCNT5 = 0;
  pid();
}

void pid() {
  
  actual = rpm;
  e = (desired - actual);
  D = Kd * (e - previous_error) * 100;
  I += (Ki * e) / 100;
  P = Kp * e;
  Z = P + D + I;
  previous_error = e;
  if ((actual > (desired-1)) && (actual < (desired+15)) && (flag ==0)) {  
    c++;
    if (c > 100) {
      ct = ovf;
      Ki = 0;
      Kp = p_on;
      Kd = d_on;
      flag = 1;  
    }
  }
}
