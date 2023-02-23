/* BTM - ADK Pin Description */
/* BTM - Stepper */
#define BTM_limit_pin PL1 // 48
#define BTM_pulse PB4 // 10
#define BTM_dir PH6 // 9


/* Linear Actuator */
#define actuator_limit_pin PL0 // 49  
                                        
#define actuator_pwm PL4 // 45 (OC5B)
#define actuator_dir PL6 // 43

/* Motor 1 */
#define M1_pwm PH3 // 6 (OC4A) //IR pin 3
#define M1_dir PG5 // 4

/* Motor 2 */
#define M2_pwm PH4 // 7 (OC4B) //IR pin 2
#define M2_dir PE3 // 5 
///* Potentiometer */
//#define pot_pin PF0 // A0  // for potentiometer -> UNUSED

/* IR Signal Pins */
#define IR1_pin PE5 // 3 
#define IR2_pin PE4 // 2

/* Encoder Pins */
#define encoder_signal_1 PB0 // 53 (PCINT0)
#define encoder_signal_2 PB2 // 51 (PCINT2)


/* Pneumatic */
#define pneumatic_pin PA0 // 22 

/* Serial Communication Description */
/* Mega2560 to ADK -> Serial 1 (Rx1) */

uint16_t set_ocr = 0; //to set OCR value according to the duty cycle

/* Pin Setup */
void pin_setup() {
  DDRA |= (1 << pneumatic_pin);
  DDRB &= ~((1 << encoder_signal_1) | (1 << encoder_signal_2));
  DDRB |= (1 << BTM_pulse);
  DDRE &= ~((1 << IR1_pin) | (1 << IR2_pin));
  DDRE |= (1 << M2_dir);
  DDRG |= (1 << M1_dir);
  DDRH |= (1 << BTM_dir) | (1 << M2_pwm) | (1 << M1_pwm);
  DDRL &= ~((1 << BTM_limit_pin) | (1 << actuator_limit_pin));
  DDRL |= (1 << actuator_pwm) | (1 << actuator_dir);
  PORTA &= ~(1 << pneumatic_pin);
  PORTB |= (1 << encoder_signal_1) | (1 << encoder_signal_2);
  PORTE |= (1 << IR1_pin) | (1 << IR2_pin) | (1 << M2_dir);
  PORTG |= (1 << M1_dir);
  PORTL |= (1 << BTM_limit_pin) | (1 << actuator_limit_pin);
}

int pulse_width(int width) {
  return (width * 2) - 1;
}

/* Mega2560 (BOH) to ADK (BTM)*/
void Serial_int_setup() {
  cli();
  UBRR1L = 0x08;
  UBRR1H = 0x00;
  UCSR1B = (1 << RXCIE1) | (1 << RXEN1);
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
  sei();
}

/* Motor 1 -> Timer 4
   Motor 2 -> Timer 4
   Linear Actuator -> Timer 5
   IR RPM Calculation -> Timer 1
   BTM Stepper -> Timer 3
   Motors : Mode 14 (Fast PWM), No prescaler
   Linear Actuator : Mode 14 (Fast PWM), 8 - bit Prescaler
   BTM Stepper : Mode 4 (CTC Timer), 8 - bit Prescaler*/
void timer_setup(int w, uint8_t duty_cycle) {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1C = 0;
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3C = 0;
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4C = 0;
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5C = 0;
  TCNT1 = 0; //Timer counter for Custom Millis function
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS10);
  TCCR4A |= (1 << WGM41) | (1 << COM4A1) | (1 << COM4B1);
  TCCR4B |= (1 << WGM43) | (1 << WGM42);
  TCCR4B |= (1 << CS40);
  TCNT3 = 0; //Timer counter for BTM - Stepper
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  TCCR5A |= (1 << WGM51) | (1 << COM5B1);
  TCCR5B |= (1 << WGM53) | (1 << WGM52);
  TCCR5B |= (1 << CS51);
  ICR4 = 5800; //top value -> M1 and M2
  ICR5 = 500; //top value -> Linear actuator
  OCR1A = 0;
  OCR4A = 0;
  OCR4B = 0;
  OCR5B = 0;
  set_ocr = (5 * duty_cycle); //pwm with duty cycle of linear actuator
  OCR3A = pulse_width(w);
  OCR1A = 159; //top value -> Custom Millis function
  TIMSK1 |= (1 << OCIE1A);
  TIMSK3 &= ~(1 << OCIE3A);
  sei();
}

/* Encoder Interrupt (PCINT) setup */
void encoder_setup () {
  cli();
  PCICR = (1 << PCIE0);
  PCMSK0 = (1 << PCINT0) | (1 << PCINT2);
  sei();
}

/* IR signal pins Interrupt setup
   Falling edge generates interrupt
*/
void IR_setup() {
  cli();
  EIMSK |= (1 << INT5) | (1 << INT4);
  EICRB |= (1 << ISC51) | (1 << ISC41);
  sei();
}
