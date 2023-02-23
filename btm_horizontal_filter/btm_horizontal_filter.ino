#include "encoder.h"
int x=0,x_p=0;
int x_1f=0,x_1p=0;
int encoder_p=0,ed=0;
void setup() {
 DDRA=0x00; // LIMIT SWITCH PINA2
 DDRH |= (1<<PINH3) | (1<<PINH4);
 PORTH &=~(1<<PINH3);
 TCCR4A = (1<<COM4B1)|(1<<WGM41);
 TCCR4B = (1<<WGM42)|(1<<WGM43)|(1<<CS40);
 ICR4 = 2000;

  encoder_1_init();
  Serial.begin(9600);
  while(PINA&(1<<PA2))
  {
   OCR4B =200; 
  }
  OCR4B =0;
  EncoderOne=0;
 // Serial_init_setup();// USART COMMUNICATION
  //encoder_p=0;
}

//void Serial_init_setup(){
//  cli();
//  UBRR1L=8;
//  UBRR1H=0x00;
//  UCSR1B=0x98;
//  UCSR1C=0x06;
//  sei();
//}
void loop() {
    
    if(Serial.available())
    {
     
      x=Serial.parseInt()+Serial.parseInt();
    }
    
    
   if((x-x_p)>0)
   {
       if((abs(EncoderOne))>(6.67*x))
           { 
           OCR4B =0;
           x_p=x;
           }
        
        else
            {
             PORTH |=(1<<PINH3);
             OCR4B =200; 
            }
   }
   else if((x-x_p)<0)
   {
       if((abs(EncoderOne))<(6.67*x))
           { 
           OCR4B =0;
           x_p=x;
           }
        
        else
            {
             PORTH &=~(1<<PINH3);
             OCR4B =200; 
            }
   }
   else
   {
    OCR4B =0; 
   }
//  Serial.println(EncoderOne);
Serial.print(x);
Serial.print(" ");
Serial.println(x_1f);
//Serial.print(encoder_p);
//Serial.print(" ");
//Serial.println(EncoderOne);
}

//ISR(USART1_RX_vect){
//  x=UDR1;
//  x_1f=0.969*x_1f + 0.0155*x + 0.0155*x_1p;
//  x_1p=x;
//}
