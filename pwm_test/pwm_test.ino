void setup() {
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);

}

void loop() {
  analogWrite(9, 150);
  digitalWrite(10, 0);

}
