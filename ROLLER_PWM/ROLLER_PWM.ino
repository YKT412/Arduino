#include<avr/io.h>
#include<util/delay.h>
#define m2_dir PG5
#define m1_dir PH3
#define m2_pwm PL3  //46
#define m1_pwm PL5  //44

int main(){
  DDRL = (1<< m1_pwm)|(1<< m2_pwm);
                    
    ICR5= 799;  //crystal f=16Mhz   
    TCCR5A= 0x8A;  //non-inverted
    TCCR5B= 0x19;  //mode-14, no prescaler

    while(1){
    OCR5A= 400;  //46
    OCR5C= 400;  //44
//    PORTL = (1<< m1_pwm)|(1<< m2_pwm);

}
}
