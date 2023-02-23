
#include<avr/io.h>

uint8_t u = 0, adc = 0;
void setup()
{
  Serial.begin(9600);
  ADMUX = (1<<REFS1)|(1<<REFS0)|(1<<ADLAR);
  ADCSRB=0x00;
  ADCSRA = (1<<ADEN)|(1 << ADSC)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);
//  ADCSRB=0x00;
//  DDRF = 0X00;
// ADCSRA |= (1 << ADSC);
}

void loop() {

  //ADCSRA |= (1 << ADSC);
  while (!(ADCSRA & (1 << ADIF)));
  ADCSRA|=(1 << ADIF);
 // u = ADC;
 adc = (ADCL)|(ADCH<<6);
  Serial.print(adc);
  Serial.print("   ");
  Serial.println(analogRead(A0));
  delay(500);
   ADCSRA |= (1 << ADSC);


}
