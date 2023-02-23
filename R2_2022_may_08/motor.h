
int memory_flag=0, flag_servo=0, flag_sahi=0 , flag_galat=0 , flag_detect=0;
int value_1 = 0;
int value_2 = 0;
int value_3 = 0;
void motor_1_(int value_1 );
void motor_2_(int value_2 );
void motor_3_(int value_3 );

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void motor_init()
{   
  //for gripper motor
  pinMode(2,OUTPUT);
    DDRH|=(1<<PH6);//ph6 = pwm for ocr2b //
    DDRB|=(1<<PB6)|(1<<PB7)|(1<<PB4);//pb6 = pwm for OCR1B//pb7 = dir for ocr1b// pb4 = dir for ocr2b
  //    // oc1b BOTTOM
  //  TCCR1A|=(1<<COM1B1)|(1<<WGM10);
  //  TCCR1B|=(1<<WGM12)|(1<<CS10);// already declared in ICR mode
  //OC2B UP
    TCCR2A|=(1<<WGM20)|(1<<COM2B1)|(1<<WGM21);
    TCCR2B|=(1<<CS20);
    
    DDRK = 0x00;
   PORTK = (1<<PK3)|0x40;//upper limit pk6
    DDRG|= (1<<PG0)|(1<<PG2);//upper gripper pnematic
    DDRC |= (1<<PC0)|(1<<PC2)|(1<<PC4);// bottom gripper pnematic
    DDRC |= (1<<PC6)|(1<<PC4);
    DDRL |= (1 << PL0) | (1 << PL4)|(1<<PL3)|(1<<PL5)|(1<<PL2)|(1<<PL6);// PL3=SERVO SIGNAL PL5=PP GRIPPER SERVO 
    DDRL &=~ (1<<PL7)|(1<<PL1);
    
    //for base motors
  DDRG |= (1 << PING5);     // dir for ocr3a                                                                                                                                                                                                           
  DDRH |= (1 << PINH3) | (1 << PINH4) | (1 << PINH5);  //ph3 = dir for ocr4b // ph5 = dir for ocr1a // ph4 = pwm for OCR4B 
  DDRE |= (1<<PE3)|(1<<PE4)|(1<<PE5);// pwm for OCR3A , DIR AND PWM FOR SERVO GRIPPER
  DDRB |= (1<<PB5);//pwm for OCR1A   
  PORTC|=(1<<PC4);     
                                                                                                                                                                           
  TCCR3A = 0XAA;
  TCCR3B = 0X19;
  ICR3 = 2019;
  TCCR4A = 0X22;
  TCCR4B = 0X19;
  ICR4 = 2019;
  TCCR1A = 0XA2;
  TCCR1B = 0X19;
  ICR1 = 2019;
  TCCR5A=0x8A;
  TCCR5B=0x1A;
  ICR5=40000;
  // 1000- FOR -90
  //2800 FOR 0
  //4800 FOR+90
}                                                       //11-8(motor3),4-5(motor2),6-7(motor 1)


 void motor_2_(int value_2)
{
  if (value_2 > 0)  PORTG |= (1 << PG5);     // 4 //
  if (value_2 < 0)  PORTG &= (~(1 << PG5));  // 4 //
  OCR3A = constrain(abs(value_2), 0, 1500);  // 5 //
}
void motor_1_(int value_1)
{
  if (value_1 > 0)  PORTH |= (1 << PH3);     // 6 //
  if (value_1 < 0)  PORTH &= (~(1 << PH3));  // 6 //
  OCR4B = constrain(abs(value_1), 0, 1500);  // 7 //
}
void motor_3_(int value_3)
{
  if (value_3 > 0)  PORTH |= (1 << PH5);    // 8 //
  if (value_3 < 0)  PORTH &= ~(1 << PH5);   // 8 //
  OCR1A = constrain(abs(value_3), 0, 1500); // 11 //
}
void motor_up_(int value_up)
{
  if (value_up > 0)     PORTB|=(1<<PB4);    // 10 //
  if (value_up < 0)    PORTB &= ~(1 << PB4);// 10 //
  OCR2B = constrain(abs(value_up), 0, 90); // 9 //
}
void motor_bottom_(int value_bottom)
{
  if (value_bottom > 0)   PORTB &= ~(1 << PB7);  // 13 //
  if (value_bottom < 0)   PORTB|=(1<<PB7);   // 13 //
  
  OCR1B = constrain(abs(value_bottom), 0, 900); // 12  //
}
void motor_servo_(int value_servo)
{
  if (value_servo > 0)   digitalWrite(2,LOW);  // 2 //
  if (value_servo < 0)   digitalWrite(2,HIGH);   // 2 //
  OCR3C = constrain(abs(value_servo), 0, 900); // 3  //
}
// 45, 47 FOR SERVO GRIPPER PNEMATIC
//333 CLOSE
//444 OPEN
