#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


unsigned char c = 0, values_send[7],i = 0, checksum = 0, h1 = 0x57;
uint16_t  rpm = 0  ;

int main(){
    init();
  Serial_int_setup();
  Serial.begin(115200);

  while (1) {
        Serial.print(i);
        Serial.print("    ");        
        Serial.print(values_send[2]);
        Serial.print("    ");
        Serial.print(values_send[3]);
        Serial.print("    ");
        Serial.print(rpm); 
        Serial.print("    ");
        Serial.print(checksum);               
        Serial.println("    ");

    
           if(Serial.available()){
            
            if(i<2){                      // 0th , 1st
             
              values_send[i] = h1;
              checksum += values_send[i];
             
            }
           if(i>1 && i<4){               // 2nd , 3rd
              values_send[i] = Serial.parseInt();
               checksum += values_send[i];
               
            }
             if(i > 3 && i<5){          // 4th , 5th , 6th
                rpm = Serial.parseInt();
                values_send[4] = (unsigned char)((rpm)&0x00ff);
                values_send[5] = (unsigned char)((rpm)>>8);
                 checksum += values_send[4]+values_send[5];
                 values_send[6] = checksum;
              }
              
              
           i++;
           if(i>4){
           i = 0;
            UCSR1B = (1 << UDRIE1) | (1 << TXEN1);
           checksum = 0;
           }
           
         }
//                 
            
          
       

  }
}


void Serial_int_setup() {
  cli();
  UBRR1L = 0x08;
  UBRR1H = 0x00;
  UCSR1B &= ~((1 << UDRIE1) | (1 << TXEN1));
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
  sei();
}

ISR(USART1_UDRE_vect) {  // mega to ADK


  
  if (c < 7) {  
    UDR1 = values_send[c];
  }
  c++;
  if (c > 6) {
    c = 0;
    UCSR1B = 0x00;
  }

}
