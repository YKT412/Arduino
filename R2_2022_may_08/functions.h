#include<ArduinoJson.h>


StaticJsonDocument<200> json;
unsigned char i = 0;
unsigned int data = 0;
unsigned char c1 = 0, c2 = 0;
char string[10];
int zz = 0, zz1;
#define addr 0xC0
short HEIGHT = 0;
float PRESSURE = 0.0;
int motor_speed = 650;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct Direction
{
  float d1;
  float d2;
  float d3;
};
//
void get_direction(int raftaar, float Vx, float Vy, float Wr)
{
  struct Direction D;
  D.d1 = -0.66 * Vx + 0.33 * Wr;
  D.d2 = 0.33 * Vx - 0.577 * Vy + 0.33 * Wr;
  D.d3 = 0.33 * Vx + 0.577 * Vy + 0.33 * Wr;
  motor_1_((raftaar * D.d1) - z);
  motor_2_((raftaar * D.d2) - z);
  motor_3_((raftaar * D.d3) - z);
}
////
////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////
////
void mpu_aavyu()
{
  motor_1_(-z);
  motor_2_(-z);
  motor_3_(-z);
}
////
////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////
////
////
void slow_clockwise()
{
  int spd = motor_speed / 2.7;
  motor_1_(-spd);
  motor_2_(-spd);
  motor_3_(-spd);
  reset_para();
  while (ps3.L1) ps3_data();
  motor_1_(0);
  motor_2_(0);
  motor_3_(0);
  delay(40);
  set_mpu();
}
////
////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
////
////
void slow_anti_clockwise()
{
  int spd = motor_speed / 2.7;
  motor_1_(spd);
  motor_2_(spd);
  motor_3_(spd);
  reset_para();
  while (ps3.R1) ps3_data();
  motor_1_(0);
  motor_2_(0);
  motor_3_(0);
  delay(40);
  set_mpu();
}
////
////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ruko()
{
  motor_1_(0);
  motor_2_(0);
  motor_3_(0);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////
void control()
{
  if (ps3.select) motor_speed = 1200;
  if (ps3.start) motor_speed = 650;
  if (ps3.up)
  {
//    if (ps3.square) get_direction(motor_speed, -0.86, -0.5, 0);
//    else if (ps3.circle) get_direction(motor_speed, 0.86, -0.5, 0);
////    else if (ps3.tri) get_direction(motor_speed, 0.707, 0.707, 0);
     get_direction(motor_speed, 0, -1, 0);
  }
  else if (ps3.down)
  {
//    if (ps3.square) get_direction(motor_speed, -0.86, 0.5, 0);
//    else if (ps3.circle) get_direction(motor_speed, 0.86, 0.5, 0);
////    else if (ps3.tri) get_direction(motor_speed, -0.707, -0.707, 0);
      get_direction(motor_speed, 0, 1, 0); //180
  }
  else if (ps3.right)   get_direction(motor_speed, 1, 0, 0);
  else if (ps3.left)  get_direction(motor_speed, -1, 0, 0);
  else if (ps3.tri) 
  {
     motor_up_(70);//up gripper => upwards
  }
  else if (ps3.cross)
  {
    motor_up_(-50);//up gripper => downwards
  }
  else if (ps3.circle)
  {
     motor_bottom_(800);// bottom gripper => upwards
  }
  else if (ps3.square)
  {
    motor_bottom_(-500);// bottom gripper => downwards
  }
  else if (ps3.R1)  slow_anti_clockwise();
  else if (ps3.L1)  slow_clockwise();  //  else if (ps3.circle)
 
  else
  { 
     motor_up_(10);
      motor_bottom_(50);
    mpu_aavyu();
    //    prev_state = 0;
  }
}
//////////////////////////////////////////////////////////////////
void i2c_stop() {
  TWCR = 0x94;
}
void i2c_write(unsigned char d) {
  TWDR = d;
  TWCR = 0x84;
  while (!(TWCR & 0x80));
  if (!(TWSR & 0x28)) {
    i2c_stop();
    Serial.println("write error+");
  }
}
void i2c_start() {
  TWCR = 0xA4;
  while (!(TWCR & 0x80));
  if (!(TWSR & 0x08)) {
    i2c_stop();
    Serial.println("start error");
  }
}

void i2c_addr() {
  TWDR = addr;
  TWCR = 0x84;
  while (!(TWCR & 0x80));
  if (!(TWSR & 0x18)) {
    i2c_stop();
    //    Serial.println("addr error");
  }
}
void i2c_send() {
  TWBR = 0x03;

  c1 = (data >> 8) & 0x0F;
  c2 = data & 0xFF;
  i2c_start();
  // Serial.println("start");
  i2c_addr();
  // Serial.println("addr");
  i2c_write(c1);
  // Serial.println("c1");
  i2c_write(c2);
  // Serial.println("c2");
  i2c_stop();
  // Serial.println(data/807);
}
///////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void lpm_pp_setup() {
  //
  //  while (!(PINK & (1 << PK5))) {
  //
  //    motor_up_(-70);
  //    Serial.println("upppppp");
  //  }
  //  motor_up_(20);
  //  Serial.println("downnnnnnn");
//  while ((PINK & (1 << PK0))) //UP
//last code for gripper down start from next line
//  while ((PINK & (1 << PK0))) //BOTTOM
//  {
//    ps3_data();
//    if (ps3.up) {
//      Serial.println("upppppp");
//      motor_up_(-70);
//      zz1 = 1;
//    }
//    else {
//      if (zz1 == 1) {
//        Serial.println("downnnnnnn");
//        motor_up_(20);
//        zz1=0;
//      }
//    }
//    motor_bottom_(60); // ICR1=2019 , top value 900
//  }
//
//}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void neeche()
{
  if ((PINK & (1 << PK1))) //BOTTOM
  {
    motor_bottom_(-800);       // ICR1=2019 , top value 900
  }
  else
  {
    OCR1B = 0;
    //      motor_bottom_(0);
    if ((PINK & (1 << PK0))) //UP
    {
      motor_up_(-40);   // top value 90
    }
    else
    {
      //        motor_up_(0);
      OCR2B = 0;
    }

  }
}
////////////////////////////////////////////////////////////////////////////////////////////////
//void collision()
//{
//  if (((PINK & (1 << PK1))) &&(!(PINK & (1 << PK0))))
//  {
//    Serial.println("collision detected");
//    OCR2B = 0;
//    OCR1B = 0;
//  }
//}
///////////////////////////////////////////////////////////////////////////////
void getDataFromApp()
{
  String str = "";
  while (Serial2.available())
  {
    char c = Serial2.read();
    str += c;
  }
  deserializeJson(json, str.c_str());
  int digit = json["digit"].as<int>();
  float pressure = json["pressure"].as<float>();

  // short localData = 0;
  short LOCALHEIGHT = 0;
  float LOCALPRESSURE = 0.0;

  if (digit) {
    LOCALHEIGHT = digit;
    HEIGHT = LOCALHEIGHT;
  } else {
    LOCALHEIGHT = HEIGHT;
  }


  if (pressure) {
    LOCALPRESSURE = pressure;
    PRESSURE = LOCALPRESSURE;
  } else {
    LOCALPRESSURE = PRESSURE;
  }

  //  Serial.print("HEIGHT: ");
  Serial.print(HEIGHT);
  //  Serial.print("  ");
  //  Serial.print("PRESSURE: ");
  //  Serial.println(PRESSURE);
}

//
void lpm_pp()
{
  //A11(PK3) ,13(PK5) ,10(PK2) ,12(PK4)
  // 10(BIG) ,15 ,20 ,25(SMALL)

  PORTL |= (1 << PL6);// pick-pass constant in one-state

  getDataFromApp();

  if (HEIGHT == 100)
  {
    motor_up_(50);
    Serial.println("upp");
  }
  
 else  if (HEIGHT == 10)
  {
    //    int bottom_lagori = 1;
    if ((PINK & (1 << PK3))) //11(PK3)
    {
      if (((PINK & (1 << PK1))) && (!(PINK & (1 << PK0))))
      {
        Serial.println("collision detected");
        OCR2B = 0;
        OCR1B = 0;
      }
      else {
        Serial.println("5th");
        motor_up_(20);
        motor_bottom_(800);     // ICR1=2019

      }
    }
    else
    {
      motor_up_(20);
      motor_bottom_(0);
      //      Serial.println("no");

    }
  }

  else if (HEIGHT == 15)
  {
    if ((PINK & (1 << PK5))) //13(PK5)
    {
      if (PINK & 0x40)motor_up_(70);
      else motor_up_(20);// top value 90
      //      motor_bottom_(0);
      Serial.println("Speed 2,4th");

    }
    else
    {
      motor_up_(20);

      //      OCR1B = 0;
      //      OCR2B = 0;
      //      Serial.println("no");

    }
  }
  else if (HEIGHT == 20)
  {
    if ((PINK & (1 << PK2))) //10(PK2)
    {

      if (((PINK & (1 << PK1))) && (!(PINK & (1 << PK0))))
      {
        Serial.println("collision detected");
        OCR2B = 0;
        OCR1B = 0;
      }
      else {
//        OCR2B = 0;
        motor_up_(20);
        motor_bottom_(800);     // ICR1=2019

        Serial.println("3rd");
      }
    }
    else
    {
      motor_up_(20);
      //        motor_bottom_(0);
     motor_bottom_(60);

      //      Serial.println("no");

    }
  }
  else if (HEIGHT == 25)
  {
    if ((PINK & (1 << PK4))) //12(PK4)
    {
      if (PINK & 0x40)motor_up_(80);
      else motor_up_(20);   // top value 90
      //      motor_bottom_(0);
      OCR1B = 0;
      Serial.println("Speed 2,2nd");

    }
    else
    {
      OCR1B = 0;
      motor_up_(20);
    }
  }
  else if (HEIGHT == 30)
  {
    if (PINL & (1 << PL7)) //42
    {
      Serial.println("1st ir");

      motor_servo_(500);
    }
    else
    {
      motor_servo_(30);
    }
  }
  else if (HEIGHT == 31)
  {

    if (PINL & (1 << PL1)) //48
    {
      Serial.println("2nd ir");
      motor_servo_(-700);
    }
    else
    {
      motor_servo_(-30);
    }
  }
  else if (HEIGHT == 152)

  {
    Serial.println("90 degree");
    OCR5A = 4800;// 90 DEGREE
  }
  else if (HEIGHT == 151)
  {
    Serial.println("0 degree");
    OCR5A = 2800;// 0 DEGREE
  }
  else if (HEIGHT == 150)
  {
    Serial.println("-90 degree");
    OCR5A = 1000;//-90
  }
  else if (HEIGHT == 78)
  {
    PORTL |= (1 << PL4);
    PORTL &= ~ (1 << PL2);
    Serial.println(" grip close");

  }
  else if (HEIGHT == 189)
  {
    PORTL |= (1 << PL2);
    PORTL &= ~ (1 << PL4);
    Serial.println(" grip open");
  }

  else if (HEIGHT == 60)
  {
    PORTC &= ~ (1 << PC2);
    PORTC |= (1 << PC0);
    //    Serial.println("bottom grip open");
  }
  else if (HEIGHT == 51)
  {
    //    Serial.println("up grip open");
    PORTC &= ~ (1 << PC6);   //41->31 KARYU ,PG0->PC6 KARYU
    PORTG |= (1 << PG2);
  }
  else if (HEIGHT == 50)
  {
    PORTC |= (1 << PC2);
    PORTC &= ~ (1 << PC0);
    //    Serial.println("bottom grip close");
  }
  else if (HEIGHT == 61)
  {
    motor_up_(-30);
    PORTC |= (1 << PC6);
    PORTG &= ~ (1 << PG2);
    //    Serial.println("up grip close");

  }

  else if (HEIGHT == 199)
  {

    PORTC |= (1 << PC4);

    Serial.println("HOLD");
  }
  else if (HEIGHT == 99)
  {

    PORTC &= ~ (1 << PC4);
    Serial.println("THROW");
  }
  else if (HEIGHT == 7)
  {
    // for ground position
    neeche();
    Serial.println("ground");
  }
  else if (HEIGHT == 87)
  {
    OCR5C = 4500; // PP SERVO GRIPPER
    Serial.println("servo OPEN");
  }
  else if (HEIGHT == 243)
  {
    OCR5C = 5000; // PP SERVO GRIPPER
    Serial.println("servo CLOSE");
  }

  else
  {
//    motor_up_(0);
//    motor_bottom_(0);
    PORTL |= (1 << PL6);// pick-pass constant in one-state
    //Serial.println("no data");
  }
  data = PRESSURE * 912.03;
  if (data > 4095)data = 4095;
  i2c_send();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
