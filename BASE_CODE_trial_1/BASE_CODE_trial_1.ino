#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#define m2_dir PG5
#define m1_dir PH3
#define m3_dir PH5
#define m3_pwm PB5
#define m2_pwm PE3
#define m1_pwm PH4

// max_spd=800 DUTY CYCLE=100



int main()
{
   Serial.begin(9600);
  int b_speed=400, max_spd = 800;
  DDRB=(1<< m3_pwm);
  DDRH=(1<<m1_dir)|(1<<m3_dir)|(1<<m1_pwm);
  DDRE= (1<<m2_pwm);
  DDRG= (1<< m2_dir);
 
                      //wheel 1
    ICR1= 799;  //crystal f=16Mhz cytron freq= 20khz  
    TCCR1A= 0x82;  //non-inverted
    TCCR1B= 0x19;  //mode-14, no prescaler

   
                      //wheel 2
    ICR3= 799;  //crystal f=16Mhz cytron freq= 20khz  
    TCCR3A= 0x82;  //non-inverted
    TCCR3B= 0x19;  //mode-14, no prescaler
    
    

                       //wheel 3
    ICR4= 799;  //crystal f=16Mhz cytron freq= 20khz
    TCCR4A= 0x22;  //non-inverted
    TCCR4B= 0x19;  //mode-14, no prescaler
// TCCR1A = (1 << WGM11) | (1 << COM1A1);
//  TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);
//
//  TCCR3A = (1 << WGM31) | (1 << COM3A1);
//  TCCR3B = (1 << WGM32) | (1 << WGM33) | (1 << CS30);
//
//  TCCR4A = (1 << WGM41) | (1 << COM4B1);
//  TCCR4B = (1 << WGM42) | (1 << WGM43) | (1 << CS40);
//
//  ICR1 = 800;
//  ICR3 = 800;
//  ICR4 = 800;
   void increasing();
   
     while(b_speed < max_spd)
     {
       OCR1A= b_speed;
       OCR3A= b_speed;
       OCR4B= b_speed;
      
       
       Serial.print(OCR1A);
       Serial.print("  ");
       Serial.print(OCR3A);
       Serial.print("  ");
       Serial.println(OCR4B);
       Serial.print("  ");
       Serial.println(b_speed);
        b_speed++;
     }
     while(1);
     
   
}
void increasing()
{ 
  
  int b_speed=400;
  int max_spd=800;
  
}
