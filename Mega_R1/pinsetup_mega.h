/* RPM calculation constants */
#define pi 3.14159
#define g 980.665
#define roller_radius 7.5

/* BOH - Mega2560 Pin Description */
/* BOH - Stepper */
#define BOH_limit_pin PF0 // A0
#define BOH_pulse PB4 // 10
#define BOH_dir PH6 // 9

/* Serial Communication Description */
/*
  Mega2560 to RaspberryPi -> Serial 3 (Tx3)
  RaspberryPi to Mega2560 -> Serial 3 (Rx3)
  Mega2560 to ADK -> Serial 1 (TX1)
  Tf_Mini to Mega2560 -> Serial 2 (RX2)
*/

/* Pin Setup */
void pin_setup() {
  DDRB |= (1 << BOH_pulse);
  DDRH |= (1 << BOH_dir);
  DDRF &= ~(1 << BOH_limit_pin);
  PORTF |= (1 << BOH_limit_pin);
}

/* Mega2560 (BOH) to ADK (BTM)
   Mega2560 <-> RaspberryPi
   Tfmini to Mega2560
*/
void Serial_int_setup() {
  cli();
  UBRR3L = 0x08;   // Rpi to mega
  UBRR3H = 0x00;
  UCSR3B = (1 << UDRIE3) | (1 << RXEN3) | (1 << RXCIE3) | (1 << TXEN3);
  UCSR3C = (1 << UCSZ31) | (1 << UCSZ30);
  UBRR1L = 0x08;   // mega to ADK
  UBRR1H = 0x00;
  UCSR1B &= ~((1 << UDRIE1));
  UCSR1B |= ((1 << TXEN1));
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
  UBRR2L = 0x08;   // TF mini to mega
  UBRR2H = 0x00;
  UCSR2B = (1 << RXEN2) | (1 << RXCIE2);
  UCSR2C = (1 << UCSZ21) | (1 << UCSZ20);
  sei();
}

/* To set pulse width of timer ONLY FOR 8 - Prescaler */
int pulse_width(int width) {
  return (width * 2) - 1;
}

/* BOH Stepper -> Timer 3
   BOH Stepper : Mode 4 (CTC Timer), 8 - bit Prescaler
*/
void timer_setup(int w) {
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3C = 0;
  TCNT3 = 0;
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS31);
  OCR3A = pulse_width(w);
  sei();                           
}
