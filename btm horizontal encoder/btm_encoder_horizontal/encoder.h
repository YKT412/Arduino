# include <avr/io.h>
# include <util/delay.h>
# include <avr/interrupt.h>

# define ENCODER_1_A_REGISTER DDRE // PIN 3
# define ENCODER_1_A_PORT PORTE
# define ENCODER_1_A_INPUT PINE
# define ENCODER_1_A_PIN PE5

# define ENCODER_1_B_REGISTER DDRA // PIN   22
# define ENCODER_1_B_PORT PORTA
# define ENCODER_1_B_INPUT PINA
# define ENCODER_1_B_PIN PA0     //pin 22

# define ENCODER_2_A_REGISTER DDRE  // 
# define ENCODER_2_A_PORT PORTE
# define ENCODER_2_A_INPUT PINE
# define ENCODER_2_A_PIN PE4 //pin 2

# define ENCODER_2_B_REGISTER DDRA // 
# define ENCODER_2_B_PORT PORTA
# define ENCODER_2_B_INPUT PINA
# define ENCODER_2_B_PIN PA2    // 24


int EncoderOne = 0;
int EncoderTwo = 0;

int encoder_1_init();
int encoder_2_init();

int encoder_1_init()
{
  ENCODER_1_A_REGISTER |= (0 << ENCODER_1_A_PIN) ;  //DDRA|=(0<<PE5);
  ENCODER_1_B_REGISTER |= (0 << ENCODER_1_B_PIN);   //DDRB|=(0<<PA0);


  ENCODER_1_A_PORT |= (1 << ENCODER_1_A_PIN); //PORTA|=(1<<PE5);
  ENCODER_1_B_PORT |= (1 << ENCODER_1_B_PIN); //PORTB|=(1<<PA0);

  EIMSK |= (1 << INT5);
  EICRB |= (1 << ISC50) | (1 << ISC51) ; // INT5 : RISING EDGE WILL GENRATE INTURRUPT
         
  sei();
}


int encoder_2_init()
{
  ENCODER_2_A_REGISTER |= (0 << ENCODER_2_A_PIN); //DDRA|=(0<<PE4);
  ENCODER_2_B_REGISTER |= (0 << ENCODER_2_B_PIN); //DDRA|=(0<<PA2);

  ENCODER_2_A_PORT |= (1 << ENCODER_2_A_PIN); //PORTA|=(1<<PE4);
  ENCODER_2_B_PORT |= (1 << ENCODER_2_B_PIN); //PORTB|=(1<<PA2);
  EIMSK |= (1 << INT4);
  EICRB |= (1 << ISC40) // INT5 : RISING EDGE WILL GENRATE INTURRUPT
         | (1 << ISC41) ;
  sei();
}

ISR (INT5_vect)
{
  if (ENCODER_1_A_INPUT & (1 << ENCODER_1_A_PIN)) // if( PINE & (1<<PE5))
  {
    if (ENCODER_1_B_INPUT & (1 << ENCODER_1_B_PIN))  // if( PINA & (1<<PA0))
    {
      EncoderOne--;        //B_5
    }
    else
    {
      EncoderOne++;        //A_1
    }
  }
  else if (~(ENCODER_1_A_INPUT & (1 << ENCODER_1_A_PIN))) //if( ~(PINE & (1<<PE5)))
  {
    if (ENCODER_1_B_INPUT & (1 << ENCODER_1_B_PIN))  //if( PINA & (1<<PA0))
    {
      EncoderOne++;    //A_2
    }
    else    //C2_LOW
    {
      EncoderOne--;    //B_6
    }
  }

  else if (ENCODER_1_A_INPUT & (1 << ENCODER_1_A_PIN)) //if(PINE & (1<<PE5)) 
  {
    if (ENCODER_1_B_INPUT & (1 << ENCODER_1_B_PIN))  //if(PINA &(1<<PA0))
    {
      EncoderOne++;    //A_3
    }
    else    //C1_LOW
    {
      EncoderOne--;    //B_7
    }
  }
  else    //C2_LOW
  {
    if (ENCODER_1_B_INPUT & (1 << ENCODER_1_B_PIN))  //C1_HIGH
    {
      EncoderOne--;    //B_8
    }
    else    //C1_LOW
    {
      EncoderOne++;    //A_4
    }
  }
}




ISR (INT4_vect)
{

  if (ENCODER_2_A_INPUT & (1 << ENCODER_2_A_PIN))  //C1_HIGH
  {
    if (ENCODER_2_B_INPUT & (1 << ENCODER_2_B_PIN))  //C2_HIGH
    {
      EncoderTwo--;        //B_5
    }
    else    //C2_LOW
    {
      EncoderTwo++;        //A_1
    }
  }
  else if (~(ENCODER_2_A_INPUT & (1 << ENCODER_2_A_PIN))) //C1_LOW
  {
    if (ENCODER_2_B_INPUT & (1 << ENCODER_2_B_PIN))  //C2_HIGH
    {
      EncoderTwo++;    //A_2
    }
    else    //C2_LOW
    {
      EncoderTwo--;    //B_6
    }
  }

  else if (ENCODER_2_A_INPUT & (1 << ENCODER_2_A_PIN))  //C2_HIGH
  {
    if (ENCODER_2_B_INPUT & (1 << ENCODER_2_B_PIN))  //C1_HIGH
    {
      EncoderTwo++;    //A_3
    }
    else    //C1_LOW
    {
      EncoderTwo--;    //B_7
    }
  }
  else    //C2_LOW
  {
    if (ENCODER_2_B_INPUT & (1 << ENCODER_2_B_PIN))  //C1_HIGH
    {
      EncoderTwo--;    //B_8
    }
    else    //C1_LOW
    {
      EncoderTwo++;    //A_4
    }
  }
}
