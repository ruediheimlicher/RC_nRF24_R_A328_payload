#include <Arduino.h>
#include <SPI.h>
//#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#include "lcd.h"
#include "expo.h"

#define TEST    0
/*
RC_nRF_Receiver A328 payload

PCB: RC_nRF24_A8_1

*/

//#define LOOPLED A3 // PC3

#define LOOPLED PB0

#define BLINKRATE 0x02FF

uint16_t loopcounter = 0;

uint8_t impulscounter = 0;
uint16_t resetcounter = 0;
uint16_t radiocounter = 1;

uint8_t radiostatus = 0;

// ack

// ********************
// ACK Payload ********
bool newData = false;
uint8_t ackData[2] = {11,12};
// ********************
// ********************



#define FIRSTTIMEDELAY  0x0FF
#define RADIOSTARTED    1
#define RADIORUNNING    2
uint16_t firsttimecounter = 0;


int ch_width_1 = 127;
int ch_width_2 = 127;
int ch_width_3 = 0;
int ch_width_4 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;


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

#define MITTE 170

/*
// SMD
#define S0  PD0     // PD0 // YAW
#define S1  PD1     // PD1 // PITCH
#define S2  PD2     // PD2 // ROLL
#define S3  PD3     // PD3 // THROTTLE
#define IO0 PD4     // PD4 // AUX
//#define IO1 A0    // PD1

#define CE_PIN 10   // PB2
#define CSN_PIN 9  // PB1
*/




// RC_NRF_REC_1
#define S0  A0     // PD0 // YAW
#define S1  A1    // PD1 // PITCH
#define S2  A2     // PD2 // ROLL
#define S3  A3     // PD3 // THROTTLE

#define IO0 PD4     // PD4 // AUX
#define IO1 A0    // PD1

#define CE_PIN 10   // PB2
#define CSN_PIN 9  // PB1



const uint64_t pipeIn = 0xABCDABCD71LL;

  // instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN);


void ResetData()
{

data.throttle = 0;   // Define the initial value of each data input. 
data.roll = MITTE;
data.pitch = MITTE;
data.yaw = MITTE+30;
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
  radio.enableDynamicPayloads();
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

void setup() 
{
  
  LCD_DDR |= (1<<LCD_RSDS_PIN);
  LCD_DDR |= (1<<LCD_ENABLE_PIN);
  LCD_DDR |= (1<<LCD_CLOCK_PIN);

	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
  delay(5);
	lcd_puts("Guten Tag\0");
  delay(1000);
  lcd_clr_line(0);
  
  DDRB |= (1<<PB0); // LED
  DDRC &= ~(1<<PC4); // Batt
  DDRC |= (1<<PC5); // Buzzer

  // Set the pins for each PWM signal | Her bir PWM sinyal için pinler belirleniyor.
  ch1.attach(S0); // YAW
  ch2.attach(S1); // PITCH
  ch3.attach(S2); // ROLL
  ch4.attach(S3); // THROTTLE
  ch5.attach(IO0);
  //ch6.attach(IO1);
                                                       
  ResetData();                                            
  
  if(initradio())
  {
    radiostatus |= (1<<RADIOSTARTED);
    lcd_gotoxy(19,0);
    lcd_puts("+");
  }
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
  loopcounter++;
 
  if(loopcounter >= BLINKRATE)
  {
    
    PORTB ^= (1<<0);
    
    loopcounter = 0;
    impulscounter++;
    
    //digitalWrite(LOOPLED, ! digitalRead(LOOPLED));
    //digitalWrite(A0, ! digitalRead(A0))
    //Serial.println(data.yaw);
    if(TEST)
    {
      /*
      lcd_gotoxy(0,0);
      lcd_putint(impulscounter);
      */
      lcd_gotoxy(4,0);
      lcd_putint12(resetcounter);
      
      lcd_gotoxy(10,0);
      lcd_putint12(radiocounter);
      
      

      lcd_gotoxy(0,1);
      lcd_putint(data.yaw);
      lcd_putc(' ');
      lcd_putint12(ch_width_1);
      lcd_putc(' ');
      lcd_putint(data.pitch);
      lcd_putc(' ');
      lcd_putint12(ch_width_2);
  
      /*
      lcd_putint(data.roll);
      lcd_putc(' ');
      lcd_putint(data.throttle);
      */
      //lcd_gotoxy(16,1);
      //lcd_putint(yawraw);

      //lcd_putc(' ');
      //lcd_gotoxy(10,2);
      //lcd_putint(ch_width_2);
      //lcd_putc(' ');
      //lcd_putint(data.pitch);
      //lcd_putc(' ');
      
      /*
      lcd_putint(ch_width_3);
      lcd_putc(' ');
      lcd_gotoxy(0,3);
      lcd_putint(ch_width_4);
      lcd_putc(' ');
      lcd_putint(ch_width_5);
      lcd_putc(' ');
      lcd_putint(ch_width_6);
      */
    

      if( radiostatus & (1<<RADIOSTARTED))
      {
        recvData();
        lcd_gotoxy(16,3);
        lcd_puts("strt");
      }
      else
      {
        lcd_gotoxy(16,3);
        lcd_puts("xxxx");
      }
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

    ackData[0] = data.yaw;
    ackData[1] = data.pitch;
    
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

