
#define pulse_pin PB5
#define limit_pin 2
#define dir 9
#define pwm 11
int step_t = 3200, step_c = 0 , target_step = 0 ;
uint8_t cnt = 0;
char tf_data[4];
uint16_t distance = 0, prev_dist = 0;
int pulse = 0, prev_pos = 0, c = 0, a = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(limit_pin, INPUT_PULLUP);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  Serial_int_setup();
  Serial.begin(115200);
  timer_setup(500);
  while (digitalRead(limit_pin)) {
    //    Serial.println(digitalRead(limit_pin));
    digitalWrite(dir, LOW);  // low=down and HIGH=up
    digitalWrite(pwm, HIGH);
    delayMicroseconds(500);
    digitalWrite(pwm, LOW);
    delayMicroseconds(500);
  }
  prev_dist = distance;
  prev_pos = 0;
  delay(1000);
}

void loop() {
  Serial.print(prev_dist);
  Serial.print("  ");
  Serial.print(target_step);
  Serial.print("  ");
  Serial.println(distance);
  if (a == 0) {
    if (prev_pos == 0) {
      set_pos(1600);
      TIMSK1 |= (1 << OCIE1A);
      //    Serial.println("loop");
      a = 1;
    }
  }
  else if (a == 1) {
    if (prev_pos == 1600) {
      set_pos(target_step);
      TIMSK1 |= (1 << OCIE1A);
      a = 2;
    }
    else {
      if (distance < prev_dist) {
        prev_dist = distance;
        target_step = prev_pos - 15;
      }
    }
  }
}

void set_pos(int pos) {
  if (pos > prev_pos) {
    digitalWrite(dir, HIGH);
  }
  else {
    digitalWrite(dir, LOW);
  }
  pulse = abs(prev_pos - pos);
}

void Serial_int_setup() {
  cli();
  UBRR3H = 0x00;
  UBRR3L = 8;
  UCSR3B = 0x98;
  UCSR3C = 0x06;
  sei();
}

void timer_setup(int w) {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = pulse_width(w);
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  //TIMSK1|=(1<<OCIE1A);
  sei();
}

int pulse_width(int width) {
  return (width * 2) - 1;
}

int calculate_pulse(float a) {
  return 3200 * a / 360.0;
}

ISR(USART3_RX_vect) {
  c++;
  measurement();
  //dist=UDR3;
}

void measurement() {
  int check;
  unsigned char uart[9];
  const int HEADER = 0x59;
  if (uread() == HEADER)
  {
    uart[0] = HEADER;
    check += uart[0];

    if (uread() == HEADER)
    {
      uart[1] = HEADER;
      check += uart[1];
      for (int i = 2; i < 8; i++)
      {
        uart[i] = uread();
        check += uart[i];
      }
      uart[8] = uread();
      distance = uart[2] + uart[3] * 256;
      //      str = uart[4] + uart[5] * 256;
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  if (pulse > 0) {
    PORTB ^= (1 << pulse_pin);
    if (digitalRead(pwm) == 0) {
      pulse--;
      if (digitalRead(dir) == 1)  prev_pos++;
      else prev_pos--;
    }
  }
  else if (pulse == 0)
  {
    TIMSK1 &= ~(1 << OCIE1A);
  }
}

char uread() {
  while (!(UCSR3A & (1 << RXC3)));
  return UDR3;
}
