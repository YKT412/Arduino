#include<avr/interrupt.h>
#include<avr/io.h>
#define dir1 8
#define pwm1 11
#define dir2 4
#define pwm2 5
// for input capture
unsigned char flag1=0;
unsigned char flag2=0;
unsigned int risingedge1=0, risingedge2=0;
float samay1=0, rpm1=0 ,rpm2=0;
float samay2=0;
int p=0;
void setup()
{   
    cli();
    DDRL &=~(1<<1); // ir1
    DDRL &=~(1<<0); // ir2
    PORTL=0xFF;
   Serial.begin(115200);
    sei();
   ///////////////////////////////////////////////////////////////////// for ir1 input capture-5////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   TCCR5A =0;
   TCCR5B |=(1<<CS51);// 8-prescaler
   TIMSK5 |=(1<<ICIE5);//enable input capture
   TCCR5B |=(1<<ICES5);//to detect rising edge

   TCNT5=0;

   /////////////////////////////////////////////////////////////////// for ir2 input capture-4//////////////////////////////////////////////////////////////////////////////////////////////
   TCCR4A =0;
   TCCR4B |=(1<<CS41);// 8-prescaler
   TIMSK4 |=(1<<ICIE4);//enable input capture
   TCCR4B |=(1<<ICES4);//to detect rising edge

   TCNT4=0;

   ///////////////////////////////////////////////////////////////// MOTORS SETUP///////////////////////////////////////////////////////////////////////////////////////////////////////////
   DDRB |= (1<<PB5);      
   DDRH |= (1<<PH5);
   DDRG |=(1<<PG5);
   DDRE |=(1<<PE3);
  
  ///////////////////////////////////////////////////////////////// MOTORS TIMERS///////////////////////////////////////////////////////////////////////////////////////////////////////////
 
   TCCR3A = 0x82; // roller-1 timer
   TCCR3B = 0x19;
   TCCR1A = 0x82; //roller-2 timer
   TCCR1B = 0x19; 

  ICR3 = 5500; // roller-1
  ICR1 = 5500; // roller-2
}

void loop() 
{ 
   if (Serial.available()) {
    p = Serial.parseInt() + Serial.parseInt();
  }

   OCR1A=p;
   OCR3A=p;
  samay1=(risingedge1)*(5);
  rpm1=(samay1);

  samay2=(risingedge2)*(0.0000005);
  rpm2=((7.5)*samay2);
  
  Serial.print(rpm1);
  Serial.print("    ");
  Serial.println(rpm2);
}

ISR(TIMER5_CAPT_vect)
{
     TCNT5=0;
    risingedge1=ICR5;  
}

ISR(TIMER4_CAPT_vect)
{   
    TCNT4=0;
    risingedge2=ICR4;
}
