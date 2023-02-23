#include "encoder.h"

unsigned char x_p=0,x1=0,x2=0, theta =0 ;
unsigned char rx_c = 0;
int encoder_p=0,ed=0, spd = 300;
void setup() {
 DDRA=0x00; // LIMIT SWITCH PINA2
 PORTA = 0XFF;
 DDRH |= (1<<PINH3) | (1<<PINH4);
 PORTH &=~(1<<PINH3);
 TCCR4A = (1<<COM4B1)|(1<<WGM41);
 TCCR4B = (1<<WGM42)|(1<<WGM43)|(1<<CS40);
 ICR4 = 2000;
//  Serial.println(digitalRead(24));
 //pinMode(19, INPUT_PULLUP);
  encoder_1_init();
  Serial.begin(115200);
  while((PINA&(1<<PA2)))
  {
   OCR4B =300;
//    Serial.println(digitalRead(24));
//   Serial.print(di); 
  }
  OCR4B =0;
//  set_degree(90);
//  _delay_ms(2000);
  EncoderOne=0;
//  Serial_init_setup();// USART COMMUNICATION
  //encoder_p=0;
}
  void loop() {
    //    Serial.println(digitalRead(24));
//  Serial.print(EncoderOne);
//Serial.println(theta);
//Serial.print(" ");
//Serial.print(encoder_p);
//Serial.print(" ");
//Serial.println(OCR4B);
    
    if(Serial.available())
    {
     
      theta=Serial.parseInt()+Serial.parseInt();
       
    }
    set_degree(theta);
//    Serial.print(x);
//  Serial.print(" ");
//  Serial.println(OCR4B);
   
}


void Serial_init_setup(){
  cli();
  UBRR1L=8;
  UBRR1H=0x00;
  UCSR1B=0x98;
  UCSR1C=0x06;
  sei();
}


ISR(USART1_RX_vect){
//  if (rx_c == 0){
//    if(UDR1==0xFF){
//      rx_c++;
//    }
//    else{
//      rx_c=0;
//    }
//    
//  }
//  else if (rx_c == 1){
//    x1 = UDR1;
//     rx_c++;
//  }
//  else if (rx_c == 2){
//    x2 = UDR1;
//    rx_c = 0;
//    if (x1 == x2){
//    x=x1;
//  }
//  }
 theta = UDR1;
// set_degree(theta);
}


void set_degree(unsigned char x){
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
             OCR4B =spd; 
             
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
             OCR4B =spd; 
            }
   }
   else
   {
    OCR4B =0; 
   }

}
