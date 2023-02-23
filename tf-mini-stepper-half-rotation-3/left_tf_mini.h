#include"tf-mini-stepper-half-rotation-3"

#define pulse_pin2 PB5   //stepper_2 pulse pin 11
#define dir2 PH5         //stepper_2 direction pin 8
#define limit_pin2 PL1  //stepper_2 limit pin 48
//#define pi 3.1415


int target_step2 = 0,  j = 0, set2 = 0, flag2 = 0, dirf2 = 0;
bool  tf2 = 0;
unsigned char  tf_data2[7], checksum2 = 0xB2, check2 = 0;
uint16_t  dist2 = 0,  prev_dist2 = 0;
double target_dist2 = 0;
int   pulse2 = 0, prev_pos2 = 0,  pos2 = 0;
double phi2 = 0, phi_rad2 = 0, theta2 = 0, theta_rad2 = 0 ;


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


void min_dist2()
{
  if (dist2 == 0)dist2 = 65535;
  if (dist2 < prev_dist2)
  {
    prev_dist2 = dist2;
    target_step2 = prev_pos2;
  }
}




void set_pos2(int pos) {
  if (pos < prev_pos2) {
    PORTH &= ~(1 << dir2);
  }
  else {
    PORTH |= (1 << dir2);
  }
  pulse2 = abs(prev_pos2 - pos);
  TIMSK3 |= (1 << OCIE3A);
}




ISR(USART2_RX_vect) {
  if (j < 2) {
    if (UDR2 == 0x59) {
      j++;
    }
    else{
      j=0;
    }
    }
    else {
      if (j < 8) {
        tf_data2[j - 2] = UDR2;
        checksum2 += tf_data2[j - 2];
        j++;
      }
      else {
        check2 = UDR2;
        if (checksum2 == check2) {
          dist2 = tf_data2[0] + tf_data2[1] * 256;
          pos2 = prev_pos2;
          checksum2 = 0xB2;
          if (tf1 == 1){
          set_stepper2();
          tf1 = 0;
          min_dist2();
          }
          
        }
        j = 0;
      }
    }
  }



ISR(TIMER3_COMPA_vect) {
  if (pulse2 > 0) {
    PORTB ^= (1 << pulse_pin2);
    if ((PINB & 0x20) == 0) {
      pulse2--;
      if ((PINH & 0x20) == 0x20) {
        prev_pos2++;
      }
      else prev_pos2--;
    }
  }
  else if (pulse2 == 0)
  {
    TIMSK3 &= ~(1 << OCIE3A);
  }
}



char uread_2() {
  while (!(UCSR2A & (1 << RXC2)));
  return UDR2;
}


void angle_calculation2 (float steps2, float d2) {
  //  if (!d)d = 10000.0;
  theta2 = (steps2 * 0.1125);
  theta_rad2 = (theta2 * 0.0174);
  target_dist2 = sq(d2) + sq(20.25) - (d2 * 40.5 * cos(theta_rad2));
  target_dist2 = sqrt(target_dist2);
  phi_rad2 = asin(((sin(theta_rad2) * d2) / target_dist2));
  phi2  = (phi_rad2 * 57.3248);
}
