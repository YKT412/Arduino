#include "address.h"
#define s1 2
#define s2 3
#define dir 1
#define AL 18
#define AH 19
#define BL 22
#define BH 23
#define A 17
#define B 21
uint8_t pid_flag = 0;
int16_t i = 0, d_count = 0, pwm = 0, deg = 0, encoder_cnt = 0, desired_count = 0;
unsigned long long h = 0, prev_h = 0;
float kp = 1, kd = 0, P = 0, D = 0, z = 0, e = 0, prev_e = 0;

void timer_int() {
  irq(1) = (uint32_t)milliseconds;
  inte |= 0x02;
}

void counter_int() {
  irq(4) = (uint32_t)counter;  //IRQ PWM WRAP
  pwm_inte = 0x02;
}

void encoder_int() {
  irq(13) = (uint32_t)encoder;                                  //IO IRQ BANK
  proc0_inte0 = (1 << AH) | (1 << AL) | (1 << BL) | (1 << BH);  // interrupt enable
}

void led() {

  gpc(16) = 0x05;
  gpc(17) = 0x05;
  gpc(18) = 0x05;
  gpc(19) = 0x05;
  gpc(20) = 0x05;
  gpc(21) = 0x05;
  gpc(22) = 0x05;
  gpc(26) = 0x05;
}

int main() {
  gpc(0) = 0x04;  //pwm  0A channel
  gpc(1) = 0x05;  //dir
  gpc(2) = 0x05;  //pneumatic
  gpc(3) = 0x05;  //pneumatic
  gpc(4) = 0x05;  // encoder input
  gpc(5) = 0x05;  // encoder input counter 3A channel
  // Serial.begin(115200);
  led();

  oe = 0x00ff000F;
  // out = 0;
  pad_clear(4) = 0x04;  // pull down diabled
  pad_set(4) = 0x08;    // pull up enabled also sets the bit high
  pad_clear(5) = 0x04;  // pull down diabled
  pad_set(5) = 0x08;    // pull up enabled also sets the bit high

  pwm_en = 0x01;
  pwm_csr(0) = 0x00000001;  //pwm enable motor
  pwm_top(0) = 0x000003E8;  //top value 1000
  pwm_div(0) = 0x000000D5;  // int:13, frac:5
  pwm_cc(0) = 0;

  // pwm_csr(3) = 0x00000031;  //pwm enable counter
  // pwm_top(3) = 1;  //top value 1000
  // pwm_div(3) = 0x00000000;  // int:13, frac:5 D5
  // pwm_cc(3) = 0;

  // d_count = 0;

  // counter_int();

  encoder_int();
  nvic_iser = 0x2000;  //13th bit and 4th
                       // milliseconds();
                       // out_set = (1 << s2);
                       // out_clear = (1 << s1);
  deg = 50;
  // out = 0xff<<16;
  while (1) {
    asm("nop");
    // desired_count = (deg / 360) * (640*2);
    // pid();
    // if (z > 0) {
    //   out_clear = (1 << dir); //ACW
    // } else {
    //   out_set = (1 << dir); //CW
    // }
    // pwm_cc(0) = ab+s(z);
    // pwm_cc(0) = 200;
    out = (encoder_cnt)<<16;
    sleep_ms(100);
  }
}

void milliseconds() {
  alarm1 = timelr;
  i++;
  // h = i / 1000;
  intr = 0x02;
  asm("NOP");
}

void pid() {
  h = timelr;
  e = desired_count - encoder_cnt;
  out = int(e) << 16;
  P = kp * e;
  D = kd * (e - prev_e) / (h - prev_h);
  D = isnan(D) ? 0 : D;
  z = P + D;
  prev_e = e;
  prev_h = h;
}



void encoder() {
  // if (proc0_ints0 & (1 << AH)) {  //falling
  //   // encoder_cnt += (intr0 &(1<<B)) ? 1 : (-1);
  //   if (intr0 & (1 << B)) {
  //     encoder_cnt++;
  //     intr0_set = (1 << AH);
  //   } else {
  //     encoder_cnt--;
  //   }
  //   intr0_set = (1 << AH);
  // } else if (proc0_ints0 & (1 << AL)) {
  //   // encoder_cnt += (intr0 &(1<<A)) ? 1 : (-1);
  //   if (intr0 & (1 << B)) {
  //     encoder_cnt--;
  //     intr0_set = (1 << B);
  //   } else {
  //     encoder_cnt++;
  //   }
  //   intr0_set = (1 << AL);
  // } else if (proc0_ints0 & (1 << BL)) {
  //   if (intr0 & (1 << A)) {
  //     encoder_cnt++;
  //     intr0_set = (1 << A);
  //   } else {
  //     encoder_cnt--;
  //   }
  //   intr0_set = (1 << BL);
  // } else {
  //   if (intr0 & (1 << A)) {
  //     encoder_cnt--;
  //     intr0_set = (1 << A);
  //   } else {
  //     encoder_cnt++;
  //   }
  //   intr0_set = (1 << BH);
  // }
  // encoder_cnt += (intr0 & (1 << B)) ? (-1) : 1;t
  
asm("NOP");
if(proc0_ints0 & (1<<AL))       //pin A Low Edge
  {
    // a=a1;        
    if(proc0_ints0 & (1<<BH))  //pin B High edge
    { 
      asm("NOP");
      encoder_cnt++;                 //Clockwise
       intr0 |= (1<<BH);    
    }
    else if(proc0_ints0 & (1<<BL))// pin B Low edge
    {
      asm("NOP");
      encoder_cnt--;
      intr0 |= (1<<BL);               ////Anti-Clockwise
    }
    intr0 |= (1<<AL);
  }
  // asm("NOP");
  else if(proc0_ints0 & (1<<AH))  // pin A High Edge
  {
    // b=b1;
    if(proc0_ints0 & (1<<BH))    //pin B High Edge
    {
      asm("NOP");
      encoder_cnt--;
      intr0 |= (1<<BH);
    }
    else if(proc0_ints0 & (1<<BL))  // pin B Low Edge
    {
      asm("NOP");
      encoder_cnt++;
      intr0 |= (1<<BL);
      
    }
    intr0 |= (1<<AH);
  }
}


void counter() {
  // pwm_cc(0) = 0x00;
  // out_set = (1<<s1);
  // out_clear = (1<<s2);
  // // pid_flag = 1;
  // encoder_count = 0;
  // // prev_h = h;
  // d_count = 0;
  // pwm_intr = 0x02;
}

// #define encoder_signal_1 PJ0 // 15 (PCINT0)
// #define encoder_signal_2 PJ1 // 14 (PCINT2)
// #define mtr_pwm PB5 // 11
// #define mtr_dir PH5 // 8
// #define s1 PK1 //A10
// #define s2 PK2 //A9

// int encoder_cnt = 0, dir = 0, desired_cnt = 0;
// unsigned char a0, b0, a1, b1, pid_flag = 0, deg = 0;
// unsigned long long h = 0, prev_h = 0;
// float kp = 158, kd = 640, P = 0, D = 0, z = 0, e = 0, prev_e = 0;  //70, 324.847
// //158, 640
// unsigned long  prev_h2 = 0, prev_h3 = 0;

// void encoder_setup () {
//   cli();
//   PCICR = (1 << PCIE1);
//   PCMSK1 &= ~((1 << PCINT9) | (1 << PCINT10));
//   sei();
// }

// void counter_config() {
//   TCCR5A = 0x00;
//   TCCR5B |= 0x07;
//   TIMSK5 = 0x02;
//   sei();
//   TCNT5 = 0;
// }

// void deg_to_cnt(unsigned char dg) {
//   OCR5A = dg * 0.8889;
//   OCR5A = constrain(OCR5A, 0, 160);
// }

// void motor_dir (int d) {
//   if (d > 0) {
//     PORTH |= (1 << mtr_dir);
//   }
//   else {
//     PORTH &= ~(1 << mtr_dir);
//   }
// }

// void pid () {
//   e = desired_cnt - encoder_cnt;
//   P = kp * e;
//   D = kd * (e - prev_e) / (h - prev_h);
//   D = isnan(D) ? 0 : D;
//   z = P + D;
//   prev_e = e;
//   prev_h = h;
// }


// void pwm_config() {
//   TCCR1A = 0x82;
//   TCCR1B = 0x19;
//   TCCR1C = 0x00;
//   ICR1 = 1600;
//   OCR1A = 0;
// }

// void timer_config() {
//   cli();
//   TCNT3 = 0; //Timer counter for Custom Millis function
//   TCCR3B |= (1 << WGM32);
//   TCCR3B |= (1 << CS30);
//   OCR3A = 159;
//   TIMSK3 |= (1 << OCIE3A);
//   sei();
// }

// int main() {
//   init();
//   DDRB |= (1 << mtr_pwm);
//   DDRJ &= ~((1 << encoder_signal_1) | (1 << encoder_signal_2));
//   PORTJ |= (1 << encoder_signal_1) | (1 << encoder_signal_2);
//   DDRH |= (1 << mtr_dir);
//   DDRK |= (1 << s1) | (1 << s2);
//   DDRL = 0x00;
//   PORTL |= 0x02;
//   counter_config();
//   encoder_setup();
//   timer_config();
//   pwm_config();
//   Serial.begin(115200);
//   PORTK |= (1 << s2);
//   PORTK &= ~(1 << s1);
//   while (1) {
//     if (Serial.available()) {
//       //      PORTK |= (1 << s2);
//       //      PORTK &= ~(1 << s1);
//       deg  = Serial.parseInt();
//       deg = constrain(deg, 0, 180);
//       if ((deg > 180) || (deg < 10)) {
//         deg = 0;
//         OCR5A = 0;
//         OCR1A = 0;
//       }
//       else {
//         deg_to_cnt(deg);
//         TIMSK5 |= 0x02;
//         PCMSK1 &= ~((1 << PCINT9) | (1 << PCINT10));
//         TCNT5 = 0;
//         OCR1A = 1600;
//         //        prev_h2 = h;
//       }
//       encoder_cnt = 0;
//       pid_flag = 0;
//     }
//     //5, 10, 12
//     if (TCNT5 >= 12) {
//       PORTK |= (1 << s1);
//       PORTK &= ~(1 << s2);
//     }
//     Serial.print(deg);
//     Serial.print("    ");
//     Serial.print(TCNT5);
//     Serial.print("    ");
//     Serial.print(e);
//     Serial.print("    ");
//     Serial.print(OCR1A);
//     Serial.print("    ");
//     Serial.print(OCR5A);
//     Serial.print("    ");
//     Serial.print("    ");
//     Serial.println(encoder_cnt);
//     //    Serial.print(long(h));
//     //    Serial.print("    ");
//     //    Serial.print(long(prev_h2));
//     //    Serial.print("    ");
//     //    Serial.print(long(prev_h3));
//     //    Serial.print("    ");
//     //    Serial.print(long(prev_h3 - prev_h2));
//     //    Serial.print("    ");
//     //    Serial.println(long(h - prev_h2));

//     if (pid_flag) {
//       //      if (long(h - prev_h2) >= 200) {
//       //        PORTK |= (1 << s1);
//       //        PORTK &= ~(1 << s2);
//       //        prev_h2 = 0;
//       //        prev_h3 = 0;
//       //      }
//       pid();
//       motor_dir(z);
//       OCR1A = abs(z);
//       OCR1A = constrain(OCR1A, 0, 1600);
//     }
//   }
// }

// ISR(PCINT1_vect)
// {
//   a0 = ~PINJ & 0x01; //encoder_signal_1 (PJ0)
//   b0 = ~PINJ & 0x02; //encoder_signal_2 (PJ1)
//   if (a0 != a1) {
//     if (a0) {
//       if (b0) encoder_cnt++;
//       else encoder_cnt--;
//     }
//     else {
//       if (b0) encoder_cnt--;
//       else encoder_cnt++;
//     }
//   }
//   else if (b0 != b1) {
//     if (b0) {
//       if (a0) encoder_cnt--;
//       else encoder_cnt++;
//     }
//     else {
//       if (a0) encoder_cnt++;
//       else encoder_cnt--;
//     }
//   }
//   a1 = a0;
//   b1 = b0;
// }

// ISR (TIMER3_COMPA_vect) {
//   h++;
// }

// ISR (TIMER5_COMPA_vect) {
//   //    PORTK |= (1 << s1);
//   //    PORTK &= ~(1 << s2);
//   OCR1A = 0;
//   TIMSK5 &= ~(0x02);
//   PCMSK1 |= (1 << PCINT9) | (1 << PCINT10);
//   pid_flag = 1;
//   encoder_cnt = 0;
//   //  prev_h3 = h;
//   OCR5A = 0;
//   TCNT5 = 0;
//   //  TIMSK5 &= ~(0x02);
//   //  PCMSK1 |= (1 << PCINT9) | (1 << PCINT10);
//   //  encoder_cnt = 0;
//   //  pid_flag = 1;
// }