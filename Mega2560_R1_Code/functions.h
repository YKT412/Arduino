#include "pin_setup.h"
/* To set pulse width of timer ONLY FOR 8 - Prescaler */
float  horizontal_angle = 0,vertical_angle = 0, velocity = 0, height = 85;
uint8_t c = 0, p =0;
uint16_t prev_pos = 0,  pulse = 0;
uint16_t dist = 0 , rpm = 0;
unsigned char i = 0, tf_data[7], values_send[7] = {0x57, 0x57 , 0, 0, 0, 0, 0}, check = 0,checksum = 0xB2, j = 0,checksum2 = 0xAE;
uint16_t steps = 0; 





void set_pos(int pos) {
  if (pos > prev_pos) {
    PORTH |= (1 << BOH_dir);           //clockwise
  }
  else {
    PORTH &= ~(1 << BOH_dir);          //anticlockwise
  }
  p = prev_pos - pos;
  pulse = abs(p);
  TIMSK3 =(1 << OCIE3A);
}

void rpm_calculation( int angle, uint16_t distance){
  vertical_angle = atan((2*height)/dist);
  velocity = sqrt((2*height*9.8)/ pow(sin(vertical_angle),2));
  rpm = ((60*velocity)/(2*PI*7.5));
  values_send[3] = vertical_angle;
  values_send[4] = (unsigned char)((rpm)&0x00ff);
  values_send[5] = (unsigned char)((rpm)>>8);
  checksum2 += values_send[2]+values_send[3]+values_send[4]+values_send[5];
  UCSR1B |= ((1 << UDRIE1) | (1 << TXEN1));
}




ISR(USART3_RX_vect) {      //Rpi to mega
  
  horizontal_angle = UDR3;
//   values_send[0] = horizontal_angle; 
}


ISR(USART1_UDRE_vect){   // mega to ADK
//  if (c < 4) {
//    UDR1 = values_send[c];
//  }
//  c++;
//  if (c > 3) {
//    c = 0;
//
//  }
 
 if (c < 7) {  
    UDR1 = values_send[c];
   
  }
  c++;
  if (c > 6) {
    c = 0;
//    UCSR1B = 0x00;
 UCSR1B &= ~((1 << UDRIE1)| (1 << TXEN1));
 checksum2 = 0xAE;
  }
}


ISR(USART2_RX_vect) {         //tfmini 1 to mega
  if (i < 2) {
    if (UDR2 == 0x59) {
      i++;
    }
    else {
      i = 0;
    }
  }
  else {
    if (i < 8) {
      tf_data[i - 2] = UDR2;
      checksum += tf_data[i - 2];
      i++;
    }
    else {
      check = UDR2;
      if (checksum == check) {
        dist = tf_data[0] + tf_data[1] * 256;
        checksum = 0xB2;
        }
      }
      i = 0;
    }
 }


ISR(TIMER3_COMPA_vect) { // for stepper
  
  if (pulse > 0) {
    PORTB ^= (1 << BOH_pulse);
    if ((PINB & (1 << BOH_pulse)) == 0) {
      pulse--;
      if ((PINH & (1 << BOH_dir)) == 0x20) {
        prev_pos++;
      }
      else prev_pos--;
    }
  }
  else if (pulse == 0)
  {
    TIMSK3 &= ~(1 << OCIE3A);
  }
}
  
