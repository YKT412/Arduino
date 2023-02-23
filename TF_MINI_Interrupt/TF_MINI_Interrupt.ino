#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#define RXD 2
//#define TXD 3 
unsigned int final_dist=0, i=0, strength=0;
unsigned char dist[4];
int main(void)
{
  
  Serial.begin(115200);
  UCSR2C=(1<<UCSZ20)|(1<<UCSZ21);
  UBRR2L=0x08;
  UCSR2B=(1<<RXEN2)|(1<<RXCIE2);
  sei();
// Serial.println("YES");
  while(1)
  {
  // if(!i){
    
  
   
    if (strength!=65536 && strength >100)
    {
   Serial.print("Distance    ");
   Serial.print(final_dist);
    Serial.print("    ");
   Serial.print("Strength   ");
   Serial.println(strength);
   
    }
     _delay_ms(10);
  }
 }

ISR(USART2_RX_vect)
{
 if (i<2)
 {
 
 if (UDR2==0x59)
 {
    i++;
 }
 else{
  i=0;
  
 }
}
else
{
  dist[i-2]=UDR2;
  i++;
 
  if (i>5)
{
  final_dist=dist[0] |(dist[1]<<8);
  strength=dist[2]|(dist[3]<<8);
  i=0;
}
}
//Serial.println(dist[0]);
//Serial.println(dist[1]);
}
