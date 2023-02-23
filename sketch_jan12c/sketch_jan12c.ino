#define input_pin PE4
#define dir1 8
#define pwm1 11
#define dir2 6
#define pwm2 7

long int rpm1 = 0, rpm2 = 0, desired_rpm = 1500, error = 0;
long int prev_time1, prev_time2, duration1, duration2;
long int value1 = 0, value2 = 0;
float kp = 5, ki = 0, kd = 0;
float P = 0, I = 0, D = 0, z = 0;
int p = 0;
boolean currentstate1, prevstate1, currentstate2, prevstate2;

void setup() {
  //cli();
  //  DDRE &= ~ (1 << input_pin);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  Serial.begin(9600);
  pinMode(dir1, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(pwm2, OUTPUT);
  //  EICRB |= (1 << ISC41) | (1 << ISC40);
  //  EIMSK |= (1 << INT4);
  //  sei();
  prev_time1 = 0;
  prev_time2 = 0;
  prevstate1 = LOW;
  prevstate2 = LOW;
  digitalWrite(dir1, 1);
  digitalWrite(dir2, 1);
  TCCR1A = 0x82;
  TCCR1B = 0x19;
  TCCR4B = 0x19;
  TCCR4A = 0x22;
  ICR1 = 5050;
  ICR4 = 5700;
}

void loop() {
  if (Serial.available()) {
    p = Serial.parseInt() + Serial.parseInt();
    //    desired_rpm = Serial.parseInt() + Serial.parseInt();
//    analogWrite(pwm1, p+9);
//    analogWrite(pwm2, p);
OCR1A=p;
OCR4B=p;
  }
  //Serial.println(desired_rpm);
  currentstate1 = digitalRead(2);
  if (prevstate1 != currentstate1) {
    if (currentstate1 == LOW) {
      duration1 = (micros() - prev_time1);
      prev_time1 = micros();
      rpm1 = (60000000 / duration1);
    }
    prevstate1 = currentstate1;
  }
  currentstate2 = digitalRead(3);
  if (prevstate2 != currentstate2) {
    if (currentstate2 == LOW) {
      duration2 = (micros() - prev_time2);
      prev_time2 = micros();
      rpm2 = (60000000 / duration2);
    }
    Serial.print(rpm1);
    Serial.print("  ");
    Serial.println(rpm2);
    prevstate2 = currentstate2;
  }
}
