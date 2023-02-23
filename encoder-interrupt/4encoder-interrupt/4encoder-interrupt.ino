#include<avr/interrupt.h>
#include<avr/io.h>
 int a=0,b=0,c=0,d=0,e=0,f=0,g=0,h=0,i=0,j=0,k=0,l=0,a1=0, b1=0,a2=0,b2=0,a3=0,b3=0,a4=0,b4=0;
void setup()
{
  DDRB= 0x00;
  PORTB= 0xff;
  
  Serial.begin(9600);
  PCICR= 0X01;
  PCMSK0= 0xff;
  sei();
   
}
void loop()
{
  Serial.println(i);
  Serial.print(" ");
  Serial.print(j);
  Serial.print(" ");
  Serial.print(k);
  Serial.print(" ");
  Serial.print(l);
  Serial.print(" ");
 // Serial.println('_');
  
}

ISR (PCINT0_vect)
{

  a1= ((~PINB)&0x01);
  b1= ((~PINB)&0X02);
  a2= ((~PINB)&0x04);
  b2= ((~PINB)&0X08);
  a3= ((~PINB)&0x10);
  b3= ((~PINB)&0X20);
  a4= ((~PINB)&0x40);
  b4= ((~PINB)&0X80);
  
  if(a1!=a)       //if pin A is high
  {
    a=a1;        
    if(b1!=a1)  //1st encoder   if b1 is low and a1 is high
    {
      i++;          
    }
    else
    {
      i--;
    }
  }
  else if(b1!=b)
  {
    b=b1;
    if(a1!=b1)
    {
      i--;
    }
    else
    {
      i++;
      
    }
  }
    if(a2!=c)   //2nd encoder
  {
    c=a2;
    if(b2!=a2)
    {
      j++;
    }
    else
    {
      j--;
    }
  }
  else if(b2!=d)
  {
    d=b2;
    if(a2!=b2)
    {
      j--;
    }
    else
    {
      j++;
      
    }
  }
     if(a3!=e)  //3rd encoder
  {
    e=a3;
    if(b3!=a3)
    {
      k++;
    }
    else
    {
      k--;
    }
  }
  else if(b3!=f)
  {
    f=b3;
    if(a3!=b3)
    {
      k--;
    }
    else
    {
      k++;
      
    }
  }
   if(a4!=g)  //4th encoder
  {
    g=a4;
    if(b4!=a4)
    {
      l++;
    }
    else
    {
      l--;
    }
  }
  else if(b4!=h)
  {
    h=b4;
    if(a4!=b4)
    {
      l--;
    }
    else
    {
      l++;
      
    }
  }
   
}
    
 
  
