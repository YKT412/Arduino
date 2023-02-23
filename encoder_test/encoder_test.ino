
/* Encoder stepper*/
#define stepper_encoder_1 PK1 // A9 (PCINT 17)
#define stepper_encoder_2 PK2 // A10 (PCINT 18)

uint16_t c0, d0, c1, d1;
int stepper_encoder_counts =0; 
int main()
{ Serial.begin(9600);
  DDRK &= ~(1<< stepper_encoder_2)|(1<< stepper_encoder_1);
  PORTK |= (1<< stepper_encoder_2)|(1<< stepper_encoder_1);
  encoder_setup();
  
  while(1){
    Serial.println(stepper_encoder_counts);
  }
}
void encoder_setup () {
  cli();
  PCICR = (1 << PCIE1)|(1<< PCIE2);
  PCMSK1 = (1 << PCINT9) | (1 << PCINT10);
  PCMSK2 = (1 << PCINT17) | (1<< PCINT18);
  sei();
}
ISR (PCINT2_vect)
{
  c0 = ~PINK & 0x02; //encoder_signal_1 (Pk1)
  d0 = ~PINK & 0x04; //encoder_signal_2 (Pk2)
  if (c0 != c1) {
    if (c0) {
      if (d0) stepper_encoder_counts++;
      else stepper_encoder_counts--;
    }
    else {
      if (d0) stepper_encoder_counts--;
      else stepper_encoder_counts++;
    }
  }
  else if (d0 != d1) {
    if (d0) {
      if (c0) stepper_encoder_counts--;
      else stepper_encoder_counts++;
    }
    else {
      if (c0) stepper_encoder_counts++;
      else stepper_encoder_counts--;
    }
  }
  c1 = c0;
  d1 = d0;
}
