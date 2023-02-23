#include"tf-mini-stepper-half-rotation-3"

#define pulse_pin1 PB4   //stepper_1 pulse pin 10
#define dir1 PH6       //stepper_1 direction pin 9
#define limit_pin1 PL3   //stepper_1 limit pin 46


int target_step1 = 0 , i = 0;
bool tf1 = 0;
unsigned char tf_data1[7], checksum1 = 0xB2, check1 = 0;
uint16_t dist1 = 0, prev_dist1 = 0;
double target_dist1 = 0;
int pulse1 = 0, prev_pos1 = 0,  pos1 = 0;

double phi1 = 0, phi_rad1 = 0, theta1 = 0, theta_rad1 = 0 ;


void set_stepper2() {
//  if (!flag) {
    set2 += 8 * dirf2;
    set_pos2(1600 - set2);
    if (set2 == 1000){
    dirf = (-1);
    print_dist();
    prev_dist2 = 65535;
    }
    if (set2 == 0){
      dirf2 = 1;
      print_dist();
     prev_dist2 = 65535;
    }

void min_dist1()
{
  if (dist1 == 0)dist1 = 65535;
  if (dist1 < prev_dist1)
  {
    prev_dist1 = dist1;
    target_step1 = prev_pos1;
  }
}


void set_pos1(int pos) {
  if (pos < prev_pos1) {
    PORTH &= ~(1 << dir1);
  }
  else {
    PORTH |= (1 << dir1);
  }
  pulse1 = abs(prev_pos1 - pos);
  TIMSK1 |= (1 << OCIE1A);
}

ISR(USART3_RX_vect) {
  if (i < 2) {
    if (UDR3 == 0x59) {
      i++;
    }
    else {
      i = 0;
    }
  }
  else {
    if (i < 8) {
      tf_data1[i - 2] = UDR3;
      checksum1 += tf_data1[i - 2];
      i++;
    }
    else {
      check1 = UDR3;
      if (checksum1 == check1) {
        dist1 = tf_data1[0] + tf_data1[1] * 256;
        pos1 = prev_pos1;
        checksum1 = 0xB2;
        tf1 = 1;
        min_dist1();
      }
      i = 0;
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  if (pulse1 > 0) {
    PORTB ^= (1 << pulse_pin1);
    if ((PINB & 0x10) == 0) {
      pulse1--;
      if ((PINH & 0x40) == 0x40) {
        prev_pos1++;
      }
      else prev_pos1--;
    }
  }
  else if (pulse1 == 0)
  {
    TIMSK1 &= ~(1 << OCIE1A);
  }
}
char uread() {
  while (!(UCSR3A & (1 << RXC3)));
  return UDR3;
}

void angle_calculation1 (float steps1, float d1) {
  //  if (!d)d = 10000.0;
  theta1 = (steps1 * 0.1125);
  theta_rad1 = (theta1 * 0.0174);
  target_dist1 = sq(d1) + sq(40.5) - (d1 * 81.0 * cos(theta_rad1));
  target_dist1 = sqrt(target_dist1);
  phi_rad1 = asin(((sin(theta_rad1) * d1) / target_dist1));
  phi1  = (phi_rad1 * 57.3248);
}
