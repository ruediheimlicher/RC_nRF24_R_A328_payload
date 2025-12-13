//
//  expo.h
//  DOG_LCD
//
//  Created by Ruedi Heimlicher on 22.11.2013.
//
//

#include <inttypes.h>


#ifndef DEFINES_h
#define DEFINES_h

#define TEST    1
#define R_SMD 0
#define  R_DIL 1
#define BOARD 1

#define LOOPLED PB0

#define BATT_PIN   PC3

#define BLINKRATE 0x01FF

#define FIRSTTIMEDELAY  0x0FF
#define RADIOSTARTED    1
#define RADIORUNNING    2

#define MITTE 170

// RC_NRF_REC_1
#define S0  A0      // YAW
#define S1  A1      //PITCH
#define S2  A2      // ROLL
#define S3  PD0     // THROTTLE

#define IO0 PD3     // AUX
#define IO1 PD2    // AUX2

/*
 // SMD
 #define S0  PD0     // PD0 // YAW
 #define S1  PD1     // PD1 // PITCH
 #define S2  PD2     // PD2 // ROLL
 #define S3  PD3     // PD3 // THROTTLE
 #define IO0 PD4     // PD4 // AUX
 //#define IO1 A0    // PD1
 
 */


#define TEST_PIN  PD2



#define OSZIA_PIN  PD3
#define OSZIAHI PORTD |= (1<<PD3)
#define OSZIALO PORTD &= ~(1<<PD3)


#define CE_PIN 10   // PB2
#define CSN_PIN 9  // PB1

#endif


