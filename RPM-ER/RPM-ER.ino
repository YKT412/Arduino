#include<avr/io.h>
#include<avr/delay.h>
void reset();
int ls = 0;
int main() {
  /*PB0 IR SENSOR
     PB1 PWM WHITE
     PB2 PWM BLACK
     PB3 pwm r motor
     PB4 DIR BLACK
     PB5 LIMIT SWITCH
     pD7 dir white
     pc4 dir r motor

  */
  init();
  Serial.begin(9600);
  DDRB |=  (1 << PB4) | (1 << PB1) | (1 << PB2) | (1 << PB3);
  DDRD |= (1 << PD7) ;
  DDRD &= ~(1 << PD6);
  PORTD |= (1 << PD6);
  DDRC |= (1 << PC4);
  DDRB &= ~ ((1 << PB0) | (1 << PB5));
  PORTB |= 0B00111001;
  TCCR1B = 0x19;
  //TCCR2B=0X41;
  //TCCR2A=0XC3;
  TCCR1A = 0xA2;
  //TCCR1A = 0xA2;
  // TCCR2B=0X40;
  // TCCR2A=0XC3;
  //OCR2A=190;
  //  PORTD|=(1<<PD7);
  DDRB |= (1 << PB3);
  TCCR2B = 0X41;
  TCCR2A = 0X83;


  ICR1 = 1000;

  int flag = 2;

  while (1) {
    Serial.println(!(PIND & 0X40));
    //   Serial.println(PIND&0X40);
    OCR2A = 77;
    if (!(PINB & 0X20) && flag == 2)  // limit not pressed
    {
      if ((PINB & 0X20)) {
        OCR1A = 0;
        OCR1B = 0;
        Serial.println("Stop");
      }
      reset();
    }
//    
    if (flag == 0) {
      OCR1A = 0;
      OCR1B = 0;
      OCR2A = 0;
      Serial.println("Stop");
    }
    if (Serial.available()) {
      flag = Serial.parseInt();
//      Serial.println(flag);
    }
    if (flag == 1) {
      if (~PINB & 0x01) {
        Serial.println("Sensor Detect");
        OCR1A = 0;
        OCR1B = 0;
//        flag = 0;
        _delay_ms(1000);

        while((PIND & 0X40)){
        OCR2A = 90;
        Serial.println("ROTATION");
        }
       
        //          limitswitch();
        if (!(PIND & 0X40))
        {
           OCR1A = 0;
           OCR1B = 0;
          // PORTC|=(1<<PC4);
          OCR2A = 0;
          Serial.println("STOP AT LIMIT");
          Serial.println("GRIPPER AT 160");
          delay(5000);
flag = 0;
         
        }

      }
      else {
        Serial.println("Upward direction of motor");
        PORTD &= ~(1 << PD7);
        PORTB |= (1 << PB4);
        OCR1B = 500;//169;
        OCR1A = 535;//550;
      }
    }


  }
  return 0;
}

void reset() {
  PORTD |= (1 << PD7);
  PORTB &= ~(1 << PB4);
  //PORTD &= ~(1 << PD2);
  //          PORTD |= (1 << PD4);
  OCR1B = 500;//169;
  OCR1A = 535;//550;
  Serial.println("Downward dir of motor");
}

void limitswitch()
{
  if (!(PIND & 0X40))
  {
    // PORTC|=(1<<PC4);
    OCR2A = 0;
    Serial.println("STOP AT LIMIT");
  }
  //  if (flag==3)
  //  {
  //    PORTC|=(1<<PC4);
  //    OCR2A=128;
  //  }
}