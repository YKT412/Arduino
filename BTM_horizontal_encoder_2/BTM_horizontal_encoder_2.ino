#include "encoder.h"
#define limit_switch PA2
unsigned char x_p = 0, x1 = 0,  x2 = 0, theta = 0 ;
unsigned char rx_c = 0;
int encoder_p = 0, ed = 0, spd = 1200;
void setup() {
  DDRA &= ~(1 << limit_switch); // LIMIT SWITCH PINA2
  PORTA |= (1 << limit_switch);
  DDRH |= (1 << PINH3) | (1 << PINH4);
  PORTH &= ~(1 << PINH3);  
  TCCR4A = (1 << COM4B1) | (1 << WGM41);
  TCCR4B = (1 << WGM42) | (1 << WGM43) | (1 << CS40);
  ICR4 = 2000;

  encoder_1_init();          // anticlockwise --->  -ve              clockwise --->   +ve
  Serial.begin(115200);
  while ((PINA & (1 << limit_switch))) //to set at 0 degree
  {
    OCR4B = 200;
    
  }
  OCR4B = 0;

  EncoderOne = 0;
  x_p = 0;
    Serial_init_setup();// USART COMMUNICATION

}
void loop() {
//  if (Serial.available()) {
//    theta = Serial.parseInt() + Serial.parseInt() ;
//
//  }
//  set_degree(theta);

  Serial.print(theta);
  Serial.print("    ");
  Serial.println(EncoderOne);

}
void Serial_init_setup() {
  cli();
  UBRR1L = 8;
  UBRR1H = 0x00;
  UCSR1B = 0x98;
  UCSR1C = 0x06;
  sei();
}


ISR(USART1_RX_vect) {
  theta = UDR1;
  set_degree(theta);
}


void set_degree(unsigned char x) {
  if (((x*6.67 )-10) > abs(EncoderOne))
  {
       
    PORTH |= (1 << PINH3); //direction:   anitclockwise---> high

     if((abs(EncoderOne)- (x_p*6.67)) > (6.67*(x-x_p)*0.5))
     {
      OCR4B = 200;
     }
     else 
     OCR4B = spd;
     
  }
  else if (((x*6.67)+10) < abs(EncoderOne))
  {

    PORTH &= ~(1 << PINH3);   // direction:   clockwise---> low
      if((abs(EncoderOne)- (x_p*6.67)) < (6.67*(x-x_p)*0.5))
     {
      OCR4B =  200;
     }
     else 
     OCR4B = spd;
       
        
  }
  
  else 
  {
    OCR4B = 0;
     x_p = x;
  }
}
