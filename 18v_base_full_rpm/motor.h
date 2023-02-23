
//           (Motor1)
//            /   \
//           /     \
//          /       \
//         /         \
//        /           \
//       /             \
// (Motor2) ---------- (Motor3)
//
//4 - PG5
//5 - PE3
//6 - PH3
//7 - PH4
//8 - PH5
//11 - PB5
//             DIRECTION PINS
//   MOTOR 1- PG5 | MOTOR 2- PH3 | MOTOR 3- PH5
//            PWM PINS
//   MOTOR 1- PE3 | MOTOR 2- PH4 | MOTOR 3- PB5
//    OC3A     |    OC4B      |   OC1A
int a = 0, d1, d2, d3;
int k = 0;
float z = 0;
int value_1 = 0;
int value_2 = 0;
int value_3 = 0;

void motor_setup() {

  DDRG = (1 << PING5);
  DDRE = (1 << PINE3);
  DDRH = (1 << PINH3) | (1 << PINH5) | (1 << PINH4);
  DDRB = (1 << PINB5);

  // ALL DIRECTION PINS ARE SET HIGH ==> Base Anti-clock
  PORTG |= (1 << PING5);
  PORTH |= (1 << PINH3) | (1 << PINH5);

  TCCR3A = (1 << COM3A1) | (1 << WGM31);
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31); // 1/8 PRESCALER , NON INVERTING MODE
  TCCR4A = (1 << COM4B1) | (1 << WGM41);
  TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41); // 1/8 PRESCALER , NON INVERTING MODE
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // 1/8 PRESCALER , NON INVERTING MODE

  ICR1 = 2019;
  ICR3 = 2019;
  ICR4 = 2019;
}

float e1=0,e2=0,e3=0;
float h1=0,h2=0,h3=0;
float ratio(float n1,float n2)
{
  return (n1/n2);
}


void motor_2_(int value_2)
{
  if (value_2 > 0)  PORTG |= (1 << PG5);     // 4 //
  if (value_2 < 0)  PORTG &= (~(1 << PG5));  // 4 //
  OCR3A = constrain(abs(value_2), 0, 2000);  // 5 //
}
void motor_1_(int value_1)
{
  if (value_1 > 0)  PORTH |= (1 << PH3);     // 6 //
  if (value_1 < 0)  PORTH &= (~(1 << PH3));  // 6 //
  OCR4B = constrain(abs(value_1), 0, 2000);  // 7 //
}
void motor_3_(int value_3)
{
  if (value_3 > 0)  PORTH |= (1 << PH5);    // 8 //
  if (value_3 < 0)  PORTH &= ~(1 << PH5);   // 8 //
  OCR1A = constrain(abs(value_3), 0, 2000); // 11 //
}

void motor_speed(float b,float r1,float r2,float r3)
{
     d2=(b*(r2*h2)); 
     d1=(b*(r1*h1)); 
     d3=(b*(r3*h3));

      motor_1_(d1 + z);
      motor_2_(d2 + z);
      motor_3_(d3 + z);
}

void motor_degree(float base__speed , float degree , float wr)
{

   float radian =(degree+180)*(0.01745);
   float vx=cos(radian);
   float vy=sin(radian);

   e1=(((-2*(vx/3)+(0.33*wr))));
   e2=((vx/3)-(vy/sqrt(3))+(0.33*wr));
   e3=((vx/3)+(vy/sqrt(3))+(0.33*wr));

   h2=e2/abs(e2);
   h1=e1/abs(e1);
   h3=e3/abs(e3);

    if (e1>=e2 && e1>=e3)
    {
      if(e1==e2)
      {
        if(e1==e3)
        {
          e1=e2=e3=1;
          motor_speed(base_speed,e1,e2,e3);
        }
        else
        {
          e1=e2=1;
          e3=ratio(e1,e3);
          motor_speed(base_speed,e1,e2,e3);
        }
      }
      else if(e1==e3)
      {
        if(e1==e2)
        {
          e1=e2=e3=1;
          motor_speed(base_speed,e1,e2,e3);
        }
        else
        {
          e1=e3=1;
          e2=ratio(e1,e2);
          motor_speed(base_speed,e1,e2,e3);
        }
      }
      else
      {
        e1=1;
        e2=ratio(e1,e2);
        e3=ratio(e1,e3);
        motor_speed(base_speed,e1,e2,e3);
      }
    }

/////////////////////////////////////////////////////////
  else if (e2>=e1 && e2>=e3)
    {
      if(e2==e1)
      {
        if(e2==e3)
        {
          e1=e2=e3=1;
          motor_speed(base_speed,e1,e2,e3);
        }
        else
        {
          e1=e2=1;
          e3=ratio(e2,e3);
          motor_speed(base_speed,e1,e2,e3);
        }
      }
      else if(e2==e3)
      {
        if(e2==e1)
        {
          e1=e2=e3=1;
          motor_speed(base_speed,e1,e2,e3);
        }
        else
        {
          e2=e3=1;
          e1=ratio(e2,e1);
          motor_speed(base_speed,e1,e2,e3);
        }
      }
      else
      {
        e2=1;
        e1=ratio(e2,e1);
        e3=ratio(e2,e3);
        motor_speed(base_speed,e1,e2,e3);
      }
    }
/////////////////////////////////////////////////////
   else
    {
      if(e3==e2)
      {
        if(e3==e1)
        {
          e1=e2=e3=1;
          motor_speed(base_speed,e1,e2,e3);
        }
        else
        {
          e3=e2=1;
          e1=ratio(e3,e1);
          motor_speed(base_speed,e1,e2,e3);
        }
      }
      else if(e3==e1)
      {
        if(e3==e2)
        {
          e1=e2=e3=1;
          motor_speed(base_speed,e1,e2,e3);
        }
        else
        {
          e1=e3=1;
          e2=ratio(e3,e2);
          motor_speed(base_speed,e1,e2,e3);
        }
      }
      else
      {
        e3=1;
        e1=ratio(e3,e1);
        e2=ratio(e3,e2);
        motor_speed(base_speed,e1,e2,e3);
      }
    }      
}
