unsigned int i,  disth, distl, dist; 
void setup()
{
   UCSR1B= (1<<RXEN1);
   UCSR1C= (1<<UCSZ11)|(1<<UCSZ10);
   UBRR1L= 0x08;
  
  Serial.begin(115200);
  
}
int read()
{  
  
  while (!(UCSR1A & (1<<RXC1)));
   i= UDR1;
   return i;
}
void loop()
{
   
   if( read() == 0x59)
   {
    if( read()== 0x59)
    {
      distl= read();
      disth= read();
      
      dist= (disth<<8) | distl;
      
    }
   }
   Serial.println(dist);
}
