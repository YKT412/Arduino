int a = 0;  //where a is the min value for motor1



void motor1_value(int a) {
  if (a >= 0) {
    PORTH &= ~(1 << PINH3);  //DIR FOR MOTOR1.CLOCKWISE
    OCR4B = a;               //MOTOR1
  }

  else if (a < 0) {
    PORTH |= (1 << PINH3);  //DIR FOR MOTOR1. ANTICLOCKWISE
    OCR4B = a * (-1);       //MOTOR1
  }
}
void motor2_value(int a) {
  if (a >= 0) {
    PORTG &= ~(1 << PING5);  //DIR FOR MOTOR2., CLOCKWISE
    OCR3A = a;               //MOTOR1
  }

  else if (a < 0) {
    PORTG |= (1 << PING5);  //DIR FOR MOTOR2. , ANTICLOCKWISE
    OCR3A = a * (-1);       //MOTOR1
  }
}
void motor3_value(int a) {
  if (a >= 0) {
    PORTH &= ~(1 << PINH5);  //DIR FOR MOTOR3.  CLOCKWISE
    OCR1A = a;               //MOTOR1
  }

  else if (a < 0) {
    PORTH |= (1 << PINH5);  //DIR FOR MOTOR3. ANTICLOCKWISE
    OCR1A = a * (-1);       //MOTOR1
  }
}