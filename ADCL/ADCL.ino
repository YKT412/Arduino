#include<avr/io.h>
#include<util/delay.h>
int main()
{
  /*
 DDRB=0XFF;
 DDRD=0XFF;*/
 Serial.begin(9600);
 ADMUX=0XC0;
 ADCSRA=0X87;
 ADCSRB=0X00;
 
 while(1)
 {
   ADCSRA=ADCSRA|(1<<ADSC);
   while((ADCSRA & (1<<ADIF))==0);
   int val=ADC;
   Serial.println(val);
//   _delay_ms(1);
 }  
 return 0;
}
