/*      Project: WhackaMole 18F25K22
 *      adapted from DualControl for Rev 6.1 boards
 * 
 *      9-8-16: Copied over Dual Rev 6.1 project
 *      Deleted, removed everything unnecessary
 *      9-9-16: Got Parallax encoder working complete with rotation direction interrupts on change.
 *      9-12-16: Modified on NUC. Uploaded to GIT. 
 *      Downloaded from GIT to laptop. Modified on laptop, uploaded to GIT.
 */
 

#include <XC.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "DELAY16.h"

#define FALSE 0
#define TRUE !FALSE
#define true TRUE
#define false FALSE

#define MICROSWITCH_A   PORTbbits.RB1
#define SYNC_IN         PORTBbits.RB6

#define ENCODER_DIRECTION_IN PORTBbits.RB4
#define ENCODER_COUNTER_IN 

#define ACCELEROMETER_ENABLE LATCbits.LATC5

// Watchdog enabled for 2.1 second timeout: 1/(31,250 Hz/128/512) = 2.1 seconds approx, so WDTPS = 512
// WhackaMole: added CCP3MX = PORTB5, WATCHDOG OFF
#pragma config FOSC = HSMP, PLLCFG = ON, IESO = OFF, FCMEN = OFF, BOREN = ON, BORV = 250, PWRTEN = ON, WDTEN = OFF, WDTPS = 512, CCP2MX = PORTB3, T3CMX = PORTB5, PBADEN = OFF, MCLRE = INTMCLR, DEBUG = OFF, STVREN = ON, XINST = OFF, LVP = OFF, CP0 = ON, CP1 = ON, CP2 = ON, CP3 = ON, CPB = ON, CPD = OFF

void InitializePIC(void);
void setDutyPWMOne(unsigned short dutyCycle);
void setDutyPWMTwo(unsigned short dutyCycle);

short readEncoderA(void);
void resetEncoderA(void);

#define ENCODER_A_MASK 0b00100000
unsigned char EncoderADirection = 0;
unsigned short lastCountA = 0;
unsigned short DelayCounterA = 0; 
short EncoderAPosition = 0;
unsigned char Timer2Flag = false;

#define RUN_PWM 800 //  was 300
#define PWM_MAX 800
#define TOP_A 800

#define DOWN 0
#define UP 1

unsigned char encoderDirection = DOWN;
unsigned char motorDirection = DOWN;

void main() {        
    InitializePIC();
    DelayMs(100);
    printf("\rSTARTING WHACKAMOLE PIC18F25K22");
    TMR3 = 0;
    while(1){
        DelayMs(100);
        EncoderAPosition = EncoderAPosition + readEncoderA();
        printf ("\rPOS = %d", EncoderAPosition);
    }
} // end main()			

void InitializePIC(void) {
    GIE = 0; // Global interrupts disabled

    ANSELA = 0b00000011; // RA0 & RA1 are analog, the rest is digital
    ANSELB = 0b00000000; // Ports B and C are all digital  
    ANSELC = 0b00011100; // RC2, RC3, RC4 are accelerometer analog inputs for X,Y,Z axes

    TRISA = 0b00001111; // RA0 - RA3 are inputs, the rest are outputs   
    PORTBbits.RB0 = PORTBbits.RB3 = 0; // Initialize PWM outputs as IO pins to start with and set them low
    RBPU = 0; // Enable Port B pullups        
    TRISB = 0b11110110;    // Outputs: RB0 = PWM 1, RB3 = PWM 2, RB4 = ENCODER_DIRECTION_IN, RB6 = SYNC IN
    IOCB = 0b00100000;     // Interrupt on change enable

    RBPU = 0; // Global pullup enable
    WPUB = 0b11111111; // Individual PORTB pullup enable      

    TRISC = 0b10011110; // RC0 = TESTOUT, RC5 = ACCELROMETER_ENABLE, and RC6 = TX are outputs, rest is input 
    // Set up UART #1
    TXSTAbits.BRGH = 1; // High speed baud rate
    SPBRG = 207; // 19200 baud @ 64 mhz   
    TXSTAbits.SYNC = 0; // asynchronous
    RCSTAbits.SPEN = 1; // enable serial port pins
    RCSTAbits.CREN = 1; // enable reception
    RCSTAbits.SREN = 0; // no effect
    TXSTAbits.TX9 = 0; // 8-bit transmission
    RCSTAbits.RX9 = 0; // 8-bit reception
    TXSTAbits.TXEN = 1; // enable the transmitter

    ACCELEROMETER_ENABLE = 1; // Accelerometer is ON.

    // A/D Initialization - AD converter not currently used
    ADCON0 = 0x00; // A/D Off
    //ADCON1 = 0x00; // Use Vdd and Vss as ref, no trigger        
    // #define AD_NORMAL_POWER 0b10111110  // Right justified result, 20Tad acquisition time, FOSC/64 clock source  `A
    // ADCON2 = AD_NORMAL_POWER;    


    // INITIALIZE PWMs:
    T2CON = 0;
    CCPTMRS0 = 0x00; // C2TSEL<1:0>: CCP2 Timer Selection bits:
    CCPTMRS1 = 0x00; // C4TSEL<1:0>: CCP4 Timer Selection bits: 00 = CCP4 ? PWM mode uses Timer2 THIS IS THE PWM #1 OUTPUT
    CCP4CON = CCP2CON = 0x00; // At power up, make sure PWM outputs are disabled so that RB0 and RB3 are IO outputs
    
                            // Set up Timer 3 as a counter for Encoder A
    T3CON = 0x00;           // Clear Timer control register
    T3CONbits.T3RD16 = 1;   // Allow 16 bit reads
    T3CONbits.T3SOSCEN = 0; // Secondary OSC disabled
    T3CONbits.TMR3CS1 = 1;  // Use T3CKI counter input
    T3CONbits.TMR3CS0 = 0;  
    T3CONbits.T3CKPS1 = 0;  // Prescale = 1:1
    T3CONbits.T3CKPS0 = 0;
    T3CONbits.nT3SYNC = 0;  // Sync with Fosc
    TMR3 = 0x0000;
    TMR3ON = 1;             // Enable counter.	
    

    // Initialize timer 2 to create 20 kHz PWM clock:
    T2CKPS0 = 1; // 1:4 prescaler
    T2CKPS1 = 0;
    T2OUTPS0 = 0; // 1:1 postscaler
    T2OUTPS1 = 0;
    T2OUTPS2 = 0;
    T2OUTPS3 = 0;
    PR2 = 199; // PWM Period = [PRx+1] * 4 * (Tosc) * (TMR2 prescaler)
    // [199+1] * 4 * (16khz * 4) * 4 = 1/20 KHZ clock period for PWMs
    TMR2ON = 1; // Let her rip

    // Initialize Timer 4 to create 1 kHz interrupts for general timing purposes:
    T4CKPS0 = 1; // 1:16 prescaler
    T4CKPS1 = 1;
    T4OUTPS0 = 1; // 1:4 postscaler
    T4OUTPS1 = 1;
    T4OUTPS2 = 0;
    T4OUTPS3 = 0;
    PR4 = 250; // For 16 Mhz crystal: 16 * 4 / 4 / 16 / 4 / 250 = 1/1 KHZ period interrupts  
    TMR4ON = 1; // Let her rip

    INTCON = 0x00; // First, clear all interrupts
    PIE1 = 0; // Clear all peripheral interrupts
    PIE1bits.ADIE = 0; // Make sure AD interrupts are disabled 
    SSPIE = 0; // Disable SSP interrupts

    RBIE = 1; // Enable interrupt on change 
    RCIE = 0; // Disable RX interrupts
    TXIE = 0; // Disable UART #1 TX interrupts         
    TX2IE = 0; // Disable UART #2 TX interrupts 
    RC2IE = 0; // Disable RX interrupts

    PEIE = 1; // Enable peripheral interrupts.
    TMR0IE = 0;
    TMR1IE = 0;
    TMR2IE = 0;
    TMR3IE = 0;
    TMR4IE = 1;  // Enable Timer 4 interrupts
    TMR5IE = 0;
    TMR5IE = 0;
    
    INTCONbits.GIE = 1; // Enable global interrupts
}


// This accepts a ten bit positive value 0 to PWM_MAX 
// and writes it to PWM One
void setDutyPWMOne(unsigned short dutyCycle){
    unsigned short lowBits;
    unsigned short ndutyCycle;
    unsigned char maskReg = 0x00;

    if (dutyCycle > PWM_MAX) ndutyCycle = 0;
    else ndutyCycle = PWM_MAX - dutyCycle;

    CCPR4L = (ndutyCycle >> 2) & 0xFF; // Shift duty cycle down to get 8 MSBs and write to CCPxL register 
    lowBits = (ndutyCycle << 4) & 0b00110000; // Shift up two LSBs and mask off other bits 
    maskReg = CCP4CON & 0b11001111; // Mask off bits 4 and 5 of CCPCON register
    CCP4CON = maskReg | lowBits; // OR in LSBs     
}

// This accepts a ten bit positive value 0 to PWM_MAX 
// and writes it to PWM Two
void setDutyPWMTwo(unsigned short dutyCycle) {
    unsigned short lowBits;
    unsigned short ndutyCycle;
    unsigned char maskReg = 0x00;

    if (dutyCycle > PWM_MAX) ndutyCycle = 0;
    else ndutyCycle = PWM_MAX - dutyCycle;

    CCPR2L = (ndutyCycle >> 2) & 0xFF; // Shift duty cycle down to get 8 MSBs and write to CCPxL register
    lowBits = (ndutyCycle << 4) & 0b00110000; // Shift up two LSBs and mask off other bits  
    maskReg = CCP2CON & 0b11001111; // Mask off bits 4 and 5 of CCPCON register
    CCP2CON = maskReg | lowBits; // OR in LSBs 
}

// This routine transmits the input byte ch from UART #1. 
// It is used by printf() to send formatted strings.
void putch(char ch) {   
    while (TXIF == 0); // If TX buffer is busy, wait for previous character to be sent
    TXREG = ch; // Send new character	    
}

// This routine returns the number of TICs the encoder
// has moved since the last read. This is the VELOCITY.
// The lastCount variable stores the last read.
// This is subtracted from the new read,
// and the difference is returned.
// Timer 3 is used here to count encoder TICs.
//
// If encoder changes direction, that is detected
// in the PORTB interrupts on change.
// The encoderDirection flag is set in that interrupt.
// This flag determines whether the motor is rotating 
// clockwise or counter clockwise, 
// and sets the sign of diffcount accordingly.
// UP is positive, DOWN is negative.
short readEncoderA(void) {
    unsigned short newCount, tempCount;
    short diffCount;

    newCount = TMR3;

    if (newCount == lastCountA)                         // If motor hasn't moved, return 0
        return (0);
    else if (newCount > lastCountA)
        diffCount = (short) (newCount - lastCountA);
    else {                                              // If new count is LESS than last count, then timer must have rolled over.		
        tempCount = ~lastCountA + 1;                    // If Timer 3 has rolled over since last read, 
                                                        // get difference before rollover and add it to new count
        diffCount = (short) (newCount + tempCount);
    }

    lastCountA = newCount;    
    if (encoderDirection == UP) return (diffCount);
    else return (0 - diffCount);
}

void resetEncoderA(void) {
    TMR3 = 0x0000;
    lastCountA = 0;
}

static void interrupt
isr(void) {
    static unsigned int T2interruptCounter = 0;    
    static unsigned char previousEncoderDirection = DOWN;
    unsigned char PORTBreg;
    
    if (RBIF){
        RBIF = 0;
        RBIE = 0;
        PORTBreg = PORTB & 0b00110000;        
        if (PORTBreg == 0x10 || PORTBreg == 0x20) encoderDirection = UP;
        else if (PORTBreg == 0x00 || PORTBreg == 0x30) encoderDirection = DOWN;
        if (previousEncoderDirection != encoderDirection){
            previousEncoderDirection = encoderDirection;
            TMR3 = 0x00;    
            lastCountA = 0;
        }
    }

    if (TMR2IF) {
        TMR2IF = 0;
        if (DelayCounterA) DelayCounterA--;
        T2interruptCounter++;
        if (T2interruptCounter >= 11) {
            T2interruptCounter = 0;
            Timer2Flag = true;
        }
    }
    
    if (TMR4IF){
        TMR4IF = 0;
        RBIE = 1;
    }
}



