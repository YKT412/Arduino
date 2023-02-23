/* BTM - Stepper */
#define BTM_limit_pin PH6 // 9
#define BTM_pulse PB5 // 11
#define BTM_dir PB4 // 10

/* Linear Actuator */
#define actuator_limit_pin PH5 // 8  
#define actuator_pwm PB6 // 12 (OC1B)
#define actuator_dir PB7 // 13

/* Motor 1 */
#define M1_pwm PH3 // 6 (OC4A) 
#define M1_dir PG5 // 4

/* Motor 2 */
#define M2_pwm PH4 // 7 (OC4B) 
#define M2_dir PE3 // 5

/* IR Signal Pins */
#define IR1_pin PE5 // 3 
#define IR2_pin PE4 // 2

/* Encoder Pins */
#define actuator_encoder_1 PJ0 // 15 (PCINT10)
#define actuator_encoder_2 PJ1 // 14 (PCINT9)

/* Encoder stepper*/
#define stepper_encoder_1 PK1 // A9 (PCINT 17)
#define stepper_encoder_2 PK2 // A10 (PCINT 18)

/* Pneumatic */
#define pneumatic_pin PA2 // 24 

/* Serial Communication with Node MCU*/
#define RX2 PH0 // 18

unsigned int set_ocr = 0; //to set OCR value according to the duty cycle

/* Pin Setup */
void pin_setup() {
  DDRA |= (1 << pneumatic_pin);
  DDRB |= (1 << BTM_pulse) | (1 << BTM_dir) | (1 << actuator_pwm) | (1 << actuator_dir);
  DDRE &= ~((1 << IR1_pin) | (1 << IR2_pin));
  DDRE |= (1 << M2_dir);
  DDRH |= (1 << M1_pwm) | (1 << M2_pwm);
  DDRH &= ~((1 << BTM_limit_pin) | (1 << actuator_limit_pin) || (1 << RX2));
  DDRG |= (1 << M1_dir);
  DDRJ &= ~((1 << actuator_encoder_1) | (1 << actuator_encoder_2));
  DDRK &= ~((1 << stepper_encoder_1) | (1 << stepper_encoder_2));
  PORTA &= ~(1 << pneumatic_pin);
  PORTJ |= (1 << actuator_encoder_1) | (1 << actuator_encoder_2);
  PORTK |= (1 << stepper_encoder_1) | (1 << stepper_encoder_2);
  PORTE &= ~(1 << M2_dir);
  PORTE |= (1 << IR1_pin) | (1 << IR2_pin);
  PORTG &= ~(1 << M1_dir);
  PORTH |= (1 << BTM_limit_pin) | (1 << actuator_limit_pin) | (1 << RX2);
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

/* Motor 1 -> Timer 4 (OCR4A)
   Motor 2 -> Timer 4 (OCR4B)
   Linear Actuator -> Timer 1
   IR RPM Calculation -> Timer 5
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
  TCNT5 = 0; //Timer counter for Custom Millis function
  TCCR5B |= (1 << WGM52); //CTC mode
  TCCR5B |= (1 << CS50); // no prescaler
  TCCR1A |= (1 << WGM11) | (1 << COM1B1) | (1 << COM1A1); //linear actuator
  TCCR1B |= (1 << WGM13) | (1 << WGM12);
  TCCR1B |= (1 << CS10);
//  TCNT3 = 0; //Timer counter for BTM - Stepper
//  TCCR3B |= (1 << WGM32);
//  TCCR3B |= (1 << CS31);
  TCCR4A |= (1 << WGM41) | (1 << COM4A1) | (1 << COM4B1); //M1 M2
  TCCR4B |= (1 << WGM43) | (1 << WGM42);
  TCCR4B |= (1 << CS41);
  ICR4 = 5300; //top value -> M1 and M2 //4800 -> 18 V battery
  ICR1 = 16000; //top value -> Linear actuator
  OCR1A = 0;
  OCR1B = 0;
  OCR5A = 0;
  OCR4A = 0;
  OCR4B = 0;
  set_ocr = (160 * duty_cycle); //pwm with duty cycle of linear actuator
//  OCR3A = pulse_width(w);
  OCR5A = 159; //top value -> Custom Millis function
  TIMSK5 |= (1 << OCIE5A);
//  TIMSK3 &= ~(1 << OCIE3A);
  sei();
}

/* Encoder Interrupt (PCINT) setup */
void encoder_setup () {
  cli();
  PCICR = (1 << PCIE1)|(1<< PCIE2);
  PCMSK1 = (1 << PCINT9) | (1 << PCINT10);
  PCMSK2 = (1 << PCINT17) | (1<< PCINT18);
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
