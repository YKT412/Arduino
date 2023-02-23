#include <avr/io.h>
#include "sensor1.h"
#include "sensor2.h"

/*parameters*/
#define min_temp 40
#define max_temp 60
#define min_hum  30
#define max_hum  90

/*parity bit*/
#define temp_f 0x80
#define hum_f 0x00
#define stop  0xff

/*sensor pin*/
#define DHTPIN 2    
#define DHTTYPE DHT22   // DHT 22  (AM2302)

// transmit_pin PD3    // TX1
// receive_pin PH0    // RX2

/* pins for components*/
#define fan PB7
#define heater PB6

DHT dht(DHTPIN, DHTTYPE);

unsigned char c = 0, i = 0, h1 = 0x01, h2 = 0x02, desired_temp = 0, desired_hum = 0;
uint8_t  values_send[5],  checksum = 0, value_received = 0;

int main() {
  init();
  // Serial_init_setup();
  Serial.begin(115200);
  // DDRB= (1<<fan)|(1<<heater);
  // PORTB&= ~((1<<fan)|(1<<heater));
 
  UBRR0L = 0x08;                                     //transmit
  UBRR0H = 0x00;
  UCSR0B &= (1 << TXEN0);  
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

  // UBRR2L = 0x08;                                    //receive
  // UBRR2H = 0x00;
  // UCSR2B &= (1 << RXEN2);  
  // UCSR2C = (1 << UCSZ21) | (1 << UCSZ20);  

  
  
  // Serial.println("AM2302!");

  dht.begin();


  while(1){
   // Wait a few seconds between measurements.
   delay(2000);

   float h = dht.readHumidity();
   float t = dht.readTemperature();
   float f = dht.readTemperature(true);
  
   if (isnan(h) || isnan(t) || isnan(f)) {
     Serial.println("Failed to read from DHT sensor!");
     return;
   }

   // Compute heat index in Fahrenheit 
   float hif = dht.computeHeatIndex(f, h);
   // Compute heat index in Celsius
   float hic = dht.computeHeatIndex(t, h, false);
  
  //  Serial.print(checksum);
  //  Serial.print("  ");
  //  Serial.print("Humidity: ");
  //  Serial.print(h);
  //  Serial.print(" %\t");
  //  Serial.print("Temperature: ");
  //  Serial.print(t);
  //  Serial.print(" *C ");
  //  Serial.print(f);
  //  Serial.println(" *F\t");
   
  
   checksum = h1 + h2 + h + t + f; 
   values_send[0] = h1;  
   values_send[1] = h2; 
   values_send[2] = h;
   values_send[3] = t;
   values_send[4] = f;  
   values_send[5] = checksum;     
  
    while(c<6){                      //transmitting 6 values
      if(!(UCSR0A &(1<<UDRE0))){  
        UDR0 = values_send[c];
        c++;
      }    
    }
     if (c > 5) {
       c = 0;
      } 


  //  while(!(UCSR2A &(1<<RXC2)));   //receiving command
  //     value_received = UDR2;
  
    // if(value_received & temp_f){     //temperature
    //   value_received^= temp_f;
    //   if((value_received < max_temp) && (value_received > min_temp)){
    //     desired_temp = value_received;
    //     if(t < desired_temp){
    //       PORTB&= (1<<heater);
    //     }
    //     else if(t > desired_temp){
    //       PORTB&= (1<<fan);                
    //     }
    //     else{
    //       PORTB&= ~((1<<fan)|(1<<heater));                
    //     }       
    //   }
    // } 
    // else if(!(value_received & temp_f)){    //humidity            
    //   if((value_received < max_hum) && (value_received > min_hum)){
    //     desired_hum = value_received;
    //     if(h < desired_hum){
    //       PORTB&= ~(1<<fan);
    //     }
    //     else if(h > desired_hum){
    //       PORTB|= (1<<fan);                
    //     }
    //     else{
    //       PORTB&= ~((1<<fan)|(1<<heater));                
    //     }
    //   }        
    // }    
    // else if((value_received & stop) == 0xff){    //stop
    //   PORTB &= ~((1 << heater)|(1 << fan));
    // }
     



 
   // Serial.print("Heat index: ");
   // Serial.print(hic);
   // Serial.print(" *C ");
   // Serial.print(hif);
   // Serial.println(" *F");
  }
}

// void Serial_init_setup() {          
//   cli();
//   UBRR1L = 0x08;                                   //transmit
//   UBRR1H = 0x00;
//   UCSR1B &= ((1 << UDRIE1) | (1 << TXEN1));
//   UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
//   UBRR2L = 0x08;                                    //receive
//   UBRR2H = 0x00;
//   UCSR2B &= (1 << RXEN2)|(1<<UDRIE1);  
//   UCSR2C = (1 << UCSZ21) | (1 << UCSZ20);    
//   sei();
// }

// ISR(USART1_UDRE_vect) {  
  
//   if (c < 6) {  
//     UDR1 = values_send[c];
//   }
//   c++;
//   if (c > 5) {
//     c = 0;
//     UCSR1B = 0x00;
//   }

// }

// ISR(USART2_RXC_vect){
//   value_received = UDR2;     
// }