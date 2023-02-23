#include <util/delay.h>
#include <avr/interrupt.h>
#define dir PL6
#define pwm PL4

uint16_t adc_reading = 0, prev_deg = 0, desired_deg = 0, counter = 0;
uint16_t max_deg = 360, max_analog = 1024, set_ocr = 0;
int16_t diff_deg = 0;
//unsigned int adc_reading = 0;
int main(){
  init();
  Serial.begin(115200);
  DDRL = (1 << dir) | (1 << pwm);
  PORTL &= ~(1 << dir);
  pwm_config(30);
  adc_setup();
  sei();
//  while(adc_reading != 0){
//    OCR5B = set_ocr;
//  }
  OCR5B = 0;
  prev_deg = 0;
  _delay_ms(1000);
  while(1){
    if (Serial.available()){
      desired_deg = Serial.parseInt();
    }
    diff_deg = (desired_deg - prev_deg);
    actuator_dir(diff_deg);
    if (deg_to_analog(abs(diff_deg)) == adc_reading){
      OCR5B = 0;
    }
    else if (deg_to_analog(abs(diff_deg)) != adc_reading){
      OCR5B = set_ocr;
    }
Serial.print(OCR5B);
Serial.print("    ");
Serial.print(deg_to_analog(abs(diff_deg)));
Serial.print("    ");
Serial.println(adc_reading);
  }
}

uint16_t deg_to_analog (float deg){
  uint16_t analog;
//  analog = (uint16_t)((max_analog * deg) / max_deg);
  analog = (uint16_t)(3 * deg);
  return analog;
}

void actuator_dir(int d){
  if (d > 0){
    PORTB |= (1 << dir);
  }
  if (d < 0){
    PORTB &= ~(1 << dir);
  }
}

void adc_setup() {
  cli();
  ADCSRB = 0x00;
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | (1 << ADSC) | (1 << ADPS1) | (1 << ADPS0); // div factor 8
//  DIDR0 = (1 << ADC0D);
  ADMUX = (1 << REFS0) | (1 << ADLAR);
  sei();
}

void pwm_config (unsigned int duty_cycle) {
  cli();
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5C = 0;
  TCCR5A |= (1 << WGM51) | (1 << COM5B1);
  TCCR5B |= (1 << WGM53) | (1 << WGM52);
  TCCR5B |= (1 << CS50);
  ICR5 = 5000;
  OCR5B = 0;
  set_ocr = (50 * duty_cycle);
  sei();
}

ISR (ADC_vect) {
  adc_reading = ((ADCL >> 6) | ( ADCH << 2));
  ADCSRA |= (1 << ADSC);
}
