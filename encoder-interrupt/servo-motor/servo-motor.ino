#include <avr/io.h>
#include <util/delay.h>
unsigned int i=3200;
float deg=0;
unsigned char c=0;
int main (void)
{
  
  TCCR1B=0x1A; //Mode 15 with 8 as prescaler
  DDRB=0xFF;
  TCCR1A=0x33; //inverting mode
 OCR1A=39999;
Serial.begin(9600);

//for (int i=3200; i>=1600; i--)
//{
//  Serial.println("1st For Loop");
//  OCR1B=i;
// // _delay_ms(5);
//}
    while (1) 
    {
//    for (int i=1600; i<=4800; i++)
//    {
//      Serial.println("2nd For Loop");
//      OCR1B=i;
//  //   _delay_ms(5);
//    }
    
    if (i<=3200 && i>=1600 && c==0)
{
  Serial.println("1st If Block");
  OCR1B=i;
//  _delay_ms(100);
  i--;
  if (i==1600)
  {
    c=1;
  }
}
    
 else if (i<=4800 && i>=1600 && c==1)
{
  Serial.println("2nd If Block");
  OCR1B=i;
//  _delay_ms(100);
  i++;
  if (i==4800)
  {
    c=2;
  }
}
//    for (int i=4800; i>=1600; i--)
//    {
//      Serial.println("3rd For Loop");
//      OCR1B=i;
//   //   _delay_ms(5);
//    }
   else if (i<=4800 && i>=1600 && c==2)
{
  Serial.println("3rd If Block");
  OCR1B=i;
 // _delay_ms(100);
  i--;
  if (i==1600)
  {
    c=1;
  }
}
deg=(float(i)*0.056);
Serial.println(deg);
}
}
