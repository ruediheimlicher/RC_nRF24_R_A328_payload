#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

#include "lcd.h"
#include "expo.h"

#include "defines.h"

#include "MS5611.h"

/*
 RC_nRF_Receiver A328 payload
 
 PCB: RC_nRF24_A8_1
 
 */

uint16_t loopcounter = 0;

uint8_t impulscounter = 0;
uint16_t resetcounter = 0;
uint16_t radiocounter = 1;

uint8_t radiostatus = 0;

// ms5611
float temperature = 0;
float temperaturmittel = 0;
 uint16_t temperature_int = 0;
const float seaLevelPressure = 1013.25;
float pressuremittel = 0;
float pressurediff = 0;
float startpressuremittel = 0;
float faktor = 0.05;
float pressure = 0;
uint16_t pressureint = 0;

float altitude = 0;
float startaltitude = 0;


uint16_t aktpressure = 0;
uint16_t pressuredelaycounter = 0;
float startpressure = 0;


//
// ack

// ********************
// ACK Payload ********
bool newData = false;
uint8_t ackData[4] = {31,32,33,34};
// ********************
// ********************




uint16_t firsttimecounter = 0;


int ch_width_1 = 127;
int ch_width_2 = 127;
int ch_width_3 = 127;
int ch_width_4 = 127;
int ch_width_5 = 127;
int ch_width_6 = 127;


Servo ch1;
Servo ch2;
Servo ch3;
Servo ch4;
Servo ch5;
Servo ch6;

struct Signal 
{
   
   byte throttle;
   byte pitch;  
   byte roll;
   byte yaw;
   byte aux1;
   byte aux2;
   
};

Signal data;



void initADC()
{
   ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS0);    // Frequenzvorteiler auf 32 setzen und ADC aktivieren 
   
   
   ADMUX |= (1<<REFS1) | (1<<REFS0); // interne Referenzspannung nutzen 
   //ADMUX |= (1<<REFS0); // VCC als Referenzspannung nutzen 
   
   /* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
    also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
   ADCSRA |= (1<<ADSC);              // eine ADC-Wandlung (Der ADC setzt dieses Bit ja wieder auf 0 nach dem Wandeln)
   while ( ADCSRA & (1<<ADSC) ) {
      ;     // auf Abschluss der Wandlung warten 
   }
}

uint16_t readKanal(uint8_t derKanal) //Unsere Funktion zum ADC-Channel aus lesen
{
   uint8_t i;
   uint16_t result = 0;         //Initialisieren wichtig, da lokale Variablen
   //nicht automatisch initialisiert werden und
   //zufällige Werte haben. Sonst kann Quatsch rauskommen
   ADMUX &= 0XF0;         //clearing channels
   ADMUX |= derKanal; 
   // Eigentliche Messung - Mittelwert aus 4 aufeinanderfolgenden Wandlungen
   for(i=0;i<4;i++)
   {
      ADCSRA |= (1<<ADSC);            // eine Wandlung
      while ( ADCSRA & (1<<ADSC) ) {
         ;     // auf Abschluss der Wandlung warten 
      }
      result += ADCW;            // Wandlungsergebnisse aufaddieren
   }
   //  ADCSRA &= ~(1<<ADEN);             // ADC deaktivieren ("Enable-Bit" auf LOW setzen)
   
   result /= 4;                     // Summe durch vier teilen = arithm. Mittelwert
   
   return result;
}

const uint64_t pipeIn = 0xABCDABCD71LL;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);

MS5611 ms5611(0x77);

void ResetData()
{
   
   data.throttle = 0;   // Define the initial value of each data input. 
   data.roll = MITTE;
   data.pitch = MITTE;
   data.yaw = MITTE+10;
   data.aux1 = 0;                                              
   data.aux2 = 0;
   resetcounter++;                                               
}

uint8_t initradio(void)
{
   ResetData();                   // Configure the NRF24 module  | NRF24 Modül konfigürasyonu
   radio.begin();
   radio.openReadingPipe(1,pipeIn);
   //radio.setChannel(100);
   radio.setChannel(124);
   
   // ********************
   // ACK Payload ********
   //radio.setAutoAck(false);
   // ********************
   // ********************
   
   //radio.setDataRate(RF24_250KBPS);    // The lowest data rate value for more stable communication  | Daha kararlı iletişim için en düşük veri hızı.
   radio.setDataRate(RF24_2MBPS); // Set the speed of the transmission to the quickest available
   radio.setPALevel(RF24_PA_MAX);                           // Output power is set for maximum |  Çıkış gücü maksimum için ayarlanıyor.
   radio.setPALevel(RF24_PA_MIN); 
   radio.setPALevel(RF24_PA_MAX); 
   
   
   // ********************
   // ACK Payload ********
   radio.enableAckPayload();
   // ********************
   
   radio.startListening(); 
   if (radio.failureDetected) 
   {
      radio.failureDetected = false;
      delay(250);
      lcd_gotoxy(18,0);
      lcd_puts("-");
      
      return 0;
   }
   else
   {
      ResetData();
      lcd_gotoxy(18,0);
      lcd_puts("+");
      return 1;
      
   }
   // Start the radio comunication for receiver | Alıcı için sinyal iletişimini başlatır.
   
}


void readStartpressure()
{
  startpressure = ms5611.getPressure();
}

uint16_t readSensor()
{
   ms5611.read();    
   temperature = ms5611.getTemperature();
   
    if(temperature == 0)
    {
      temperaturmittel = temperature;
    }
    else
    {
      temperaturmittel = temperaturmittel + faktor * (temperature - temperaturmittel);
    }
   
   pressure = ms5611.getPressure() * 10; // 
   
   // Filter
    if (pressuremittel == 0)
    {
      pressuremittel = pressure;
    }
    else
    {
      pressuremittel = pressuremittel + faktor * ( pressure - pressuremittel);
    }

   return pressuremittel ;
}

void setup() 
{
   
   
   switch (BOARD)
   {
      case R_SMD:
      {
         
      }break;
      case R_DIL:
      {
         
      }break;
   }
   LCD_DDR |= (1<<LCD_RSDS_PIN);
   LCD_DDR |= (1<<LCD_ENABLE_PIN);
   LCD_DDR |= (1<<LCD_CLOCK_PIN);
   
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   delay(5);
   lcd_puts("Guten Tag\0");
   delay(1000);
   lcd_clr_line(0);
   
   LOOPLED_DDR |= (1<<LOOPLED);

   BATT_DDR &= ~(1<<BATT_PIN); // Batt
   
   BUZZER_DDR |= (1<<BUZZER_PIN); // Buzzer 
   

   OSZIA_DDR |= (1<<OSZIA_PIN); // OSZIA
   OSZIA_PORT |= (1<<OSZIA_PIN); // OSZIA

   TEST_DDR &= ~(1<<TEST_PIN); // TEST  INPUT
   TEST_PORT |= (1<<TEST_PIN); // TEST  PULLUP
   
   // Set the pins for each PWM signal | Her bir PWM sinyal için pinler belirleniyor.
   ch1.attach(S0); // YAW
   ch2.attach(S1); // PITCH
   ch3.attach(S2); // ROLL
   ch3.attach(S3); // THROTTLE
   
   //ch5.attach(IO0]);
   //ch6.attach(IO1);
   
   ResetData();                                            
   
   if(initradio())
   {
      radiostatus |= (1<<RADIOSTARTED);
      lcd_gotoxy(19,0);
      lcd_puts("+");
   }
   initADC();
   
   Wire.begin();
   if (ms5611.begin() == true)
   {
      lcd_gotoxy(0,3);
      lcd_puts("ms5611 found: ");
      lcd_putint12(ms5611.getAddress());
   }
   else
   {
      lcd_gotoxy(0,3);
      lcd_puts("ms5611 not found: ");
   }
   
   ms5611.reset(0);
   
   ms5611.setOversampling(OSR_STANDARD);


   
   _delay_ms(1000);
   lcd_clr_line(3);

 
   
}
unsigned long lastRecvTime = 0;

void recvData()
{
   if ( radio.available() ) 
   {
      radiocounter++;
      radio.read(&data, sizeof(Signal));
      lastRecvTime = millis();   // Receive the data | Data alınıyor
      
      // ********************
      // ACK Payload ********
      radio.writeAckPayload(1, &ackData, sizeof(ackData));
      // ********************
      // ********************
   }
}




void loop() 
{
   
   pressuredelaycounter++;
   if(pressuredelaycounter > 0xFF)
   {
      pressuredelaycounter = 0;
      OSZIALO;
      float pressurenew = readSensor(); // temperaturmittel, pressuremittel*10
      OSZIAHI;
      temperature_int = uint8_t(temperaturmittel *5); // 3 Stellen <255
      ackData[0] = temperature_int;

      pressureint = (pressuremittel); // 
      ackData[1] = (pressureint & 0xFF00)>>8;
      ackData[2] = (pressureint & 0x00FF);


   }
   
   loopcounter++;
   
   if(loopcounter >= BLINKRATE)
   {
      if(!(PIND & (1<<TEST_PIN)))
      {
         
         //uint16_t diff = altitude - startaltitude +1;
        
        lcd_gotoxy(0,0);
         lcd_putint(ackData[1]);
         lcd_gotoxy(4,0);
         lcd_putint(ackData[2]);
         //uint16_t pressureint2 = (ackData[1] <<8) | ackData[2];
         //lcd_gotoxy(8,0);
         //lcd_putint16(pressureint2);

         lcd_gotoxy(0,2);
         lcd_putint12(temperature);
         lcd_gotoxy(6,2);
         lcd_putint(temperature_int);

         lcd_gotoxy(12,2);
         lcd_putint12(temperaturmittel);
         lcd_putc(' ');
         //(diff);
         
         lcd_gotoxy(0,3);
         lcd_putint12(pressuremittel);
         /*
         lcd_putc(' ');
         lcd_putint12(pressuremittel);
         lcd_putc(' ');
         lcd_putint12(pressureint);
         */
         //lcd_putc(' ');
         //lcd_putint12(altitudeint);
         
         //lcd_putc(' ');
         //lcd_putint(altitudeint);

         //uint16_t diff = startpressure - aktpressure ;
         
         
         
         //lcd_putint12(startaltitude);
         //lcd_putc(' ');
         //lcd_gotoxy(10,2);490,0,255);
         //lcd_putint12(altitudeint);
         // lcd_putc(' ');
      }
      else
      {
         
      }
      
      uint16_t batt = readKanal(BATT_PIN);// BATT 8.4V: 998    6.4V: 748  5.0: 700
     // lcd_gotoxy(0,0);
     // lcd_putint12(batt);

     /*
    map() begrenzt nicht.
    Werte unter in_min → Ergebnis unter out_min.
    Speicherung in byte → Unterlauf → scheinbar 0–255.
    */
      batt = constrain(batt, 600, 1000);
      {
      ackData[3] = map(batt,600,1000,0,255); // BATT 8.4V: 240   6.4V: 94   6.0: 65
      }
     
     // lcd_putc(' ');
     // lcd_putint(ackData[3]);

      LOOPLED_PORT ^= (1<<LOOPLED); // LOOPLED
      
      loopcounter = 0;
      impulscounter++;
      

      if(!(PIND & (1<<TEST_PIN)))
      {
       
         //lcd_gotoxy(4,0);
         //lcd_putint12(resetcounter);
         
         lcd_gotoxy(16,0);
         lcd_putint12(radiocounter);
         
         
         lcd_gotoxy(0,1);
         lcd_putint(data.yaw);
         lcd_putc(' ');
         //lcd_putint12(ch_width_1);
         //lcd_putc(' ');
         lcd_putint(data.pitch);
         //lcd_putc(' ');
         //lcd_putint12(ch_width_2);
         
         lcd_gotoxy(12,1);
         lcd_putint(ackData[2]);// alt
         lcd_gotoxy(16,1);
         lcd_putint(ackData[3]); // Batt
         
      } // if TEST
         
      
   }
   
   /*
    if ((firsttimecounter < FIRSTTIMEDELAY ) && !(radiostatus & (1<<RADIOSTARTED)))
    {
    firsttimecounter++;
    }
    else if  (firsttimecounter ==  FIRSTTIMEDELAY )
    {
    if(initradio())
    {
    radiostatus |= (1<<RADIOSTARTED);
    }
    
    }
    */
   
   if( radiostatus & (1<<RADIOSTARTED))
   {
      
      
      //ackData[0] = data.yaw;
      //ackData[1] = data.pitch;
      
      recvData();
      unsigned long now = millis();
      if ( now - lastRecvTime > 1000 ) 
      {
         ResetData();  // Signal lost.. Reset data
      }
   } 
   
   //data.yaw = (impulscounter & 0xFF );//& 0xFF00) >> 8;
   
   // map: 
   // map(value, fromLow, fromHigh, toLow, toHigh)
   
   ch_width_1 = map(data.yaw, 0, 255, 1000, 2000);       // YAW
   ch_width_2 = map(data.pitch, 0, 255, 1000, 2000);     // PITCH
   
   ch_width_3 = map(data.roll, 0, 255, 1000, 2000);      // ROLL
   ch_width_4 = map(data.throttle, 0, 255, 1000, 2000);  // THROTTLE
   
   // ON/OFF
   ch_width_5 = map(data.aux1, 0, 1, 1000, 2000); 
   //ch_width_6 = map(data.aux2, 0, 1, 1000, 2000); 
   //ch_width_6 = map((impulscounter & 0xFF ), 0, 255, 1000, 2000);
   
   //ch_width_1 = 100;
   
   ch1.writeMicroseconds(ch_width_1);           // Write the PWM signal
   ch2.writeMicroseconds(ch_width_2);
   ch3.writeMicroseconds(ch_width_3);
   ch4.writeMicroseconds(ch_width_4);
   ch5.writeMicroseconds(ch_width_5);
   //ch6.writeMicroseconds(ch_width_6); 
}

