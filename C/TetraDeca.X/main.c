//
// PIC16F1705 code for the Tetradeca project
// located at github.com/SmallRoomLabs/Tetradeca
// Copyright (c) 2016 Mats Engstrom SmallRoomLabs
// Released under the MIT license
//

//
// This is the hardware test and development code written 
// in C to debug the hardware and algorithms before converting
// the code into ASM to keep the sie below 1024 bytes for the
// Hackaday.io 1KB competition
//

//            16F1705                  '595
//            +--u--+                 +--u--+
//          - |1  14| +             QB|1  16| +
//         A5 |2  13| RA0/ISCPDAT   QC|2  15| QA
//         A4 |3  12| RA1/ICSPCLK   QD|3  14| SerIn
//     RES/A3 |4  11| RA2           QE|4  13| /OE
//         C5 |5  10| C0            QF|5  12| LATCH
//         C4 |6   9| C1            QG|6  11| CLOCK
//         C3 |7   8| C2            QH|7  10| /CLEAR
//            +-----+               - |8   9| SerOut
//                                    +-----+
//
//    A0 i ISCP-DAT
//    A1 i ICSP-CLK
//    A2 i Quadrature Button
//    A3 i ICSP-MCLR
//    A4 o 595-Latch
//    A5 i
//    C0 i
//    C1 i Quadrature-A
//    C2 i Quadrature-B
//    C3 i Buttons
//    C4 o 595-DataIn
//    C5 o 595-Clock 
//
//
//    aaaaaaaaa
//   ih   g   bc
//   i h  g  b c
//   i  h g b  c
//   i   hgb   c
//    jjjj dddd
//   k   lnf   e
//   k  l n f  e 
//   k l  n  f e
//   kl   n   fe
//    mmmmmmmmm
//   
//  
// Character map
//
//        abcdefghijklmn
//  
//  0 0   abc-e---i-klm-
//  1 1   -bc-e---------
//  2 2   a-cd-----jk-m-
//  3 3   a-cde-------m-
//  4 4   --cde---ij----
//  5 5   a----f--ij--m-
//  6 6   a--de---ijk-m-
//  7 7   ab-----------n
//  8 8   a-cde---ijk-m-
//  9 9   a-cde---ij--m- 
// 10 sp  --------------
// 11 A   a-cde---ijk---
// 12 B   a-cde-g-----mn
// 13 C   a-------i-k-m-
// 14 D   a-c-e-g-----mn
// 15 E   a-------ijk-m-
// 16 F   a-------ijk---  
// 17 G   a--de---i-k-m-
// 18 H   --cde---ijk---
// 19 I   a-----g-----mn
// 20 J   --c-e-----k-m-
// 21 K   -b---f--ijk---  
// 22 L   --------i-k-m-
// 23 M   -bc-e--hi-k---
// 24 N   --c-ef-hi-k---
// 25 O   a-c-e---i-k-m-
// 26 P   a-cd----ijk---
// 27 Q   a-c-ef--i-k-m- 
// 28 R   a-cd-f--ijk---
// 29 S   a--de---ij--m-
// 30 T   a-----g------n
// 31 U   --c-e---i-k-m-
// 32 V   -b------i-kl-- 
// 33 W   --c-ef--i-kl--
// 34 X   -b---f-h---l-- 
// 35 Y   -b-----h-----n
// 36 Z   ab---------lm- 
// 37 -   ---d-----j----
// 38 +   ---d--g--j---n
// 39 =   ---d-----j--m-
//

#define _XTAL_FREQ 32000000

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <xc.h>

#define T0_DIV256 0b0111
#define T0_DIV128 0b0110
#define T0_DIV64  0b0101
#define T0_DIV32  0b0100
#define T0_DIV16  0b0011
#define T0_DIV8   0b0010
#define T0_DIV4   0b0001
#define T0_DIV2   0b0000
#define T0_DIV1   0b1000

// PIC16F1705 Configuration Bit Settings
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit cannot be cleared once it is set by software)
#pragma config ZCDDIS = ON      // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR)
#pragma config PLLEN = ON       // Phase Lock Loop enable (4x PLL is always enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

// #pragma config IDLOC0 = 0x01	// Set the User ID data to some
// #pragma config IDLOC1 = 0x23    // values for testing
// #pragma config IDLOC2 = 0x45
// #pragma config IDLOC3 = 0x56

#define QUAD_BUT_a      (1<<2)      // A2
#define QUAD_A_c        (1<<1)      // C1
#define QUAD_B_c        (1<<2)      // C2
#define BUTTON_c        (1<<3)      // C3
#define SHIFT_LATCH_a   (1<<4)      // A4
#define SHIFT_DATA_c    (1<<4)      // C4
#define SHIFT_CLOCK_c   (1<<5)      // C5

volatile uint8_t tick=0;            // Incremented by IRQ at 0.1s rate
volatile uint8_t button=0xFF;
uint8_t pattern=0x18;

void ShiftOut(uint8_t v) {
    for (uint8_t i=0; i<8; i++) {
        if (v&1) LATC|=SHIFT_DATA_c; else LATC&=~SHIFT_DATA_c;
        v/=2;
        LATC|=SHIFT_CLOCK_c;
        NOP();
        LATC&=~SHIFT_CLOCK_c;
    }
}



//
//
//
void main() {
    OSCCON=0b11110000;      // Use internal oscillator set at 32MHz

    INTCON=0;               // Disable all interrupts
    PIE1=0;                 // Disable peripheral interrupts
    PIE2=0;                 // Disable more peripheral interrupts
    PIE3=0;                 // Disable even more peripheral interrupts
    IOCAN=0x00;             // Disable pin-change interrupts
    IOCAP=0x00;
    IOCCN=0x00;
    IOCCP=0x00;

    ANSELA=0;               // All GPIOs are digital
    ANSELC=0;
    WPUA=0xFF;              // Turn on all pullups
    WPUC=0xFF;	
    TRISA=~(SHIFT_LATCH_a); // Set output mode
    TRISC=~(SHIFT_DATA_c|SHIFT_CLOCK_c);
    LATA&=~SHIFT_LATCH_a;   // Low LATCH
    LATC&=~SHIFT_CLOCK_c;   // Low CLOCK
    
  //  
  //   Setup Timer0 to generate interrupts at about 488 Hz for refreshing
  //   the displays at a relatively flicker-free 61 Hz.
  //  
  //     fOsc/4      Prescale     8bit counter   8 displays 
  //   32000000/4 -> 8000000/64 -> 125000/256 -> 488/8 -> 61 Hz Refresh 
  //  
  OPTION_REG=0x00 | T0_DIV64;   // Wpu enabled, Prescale T0 from Fosc/4 with 64 
  INTCONbits.T0IE=1;            // Enable interrupt on TMR0 overflow
  INTCONbits.GIE=1;             // Global interrupt enable
  uint8_t tic=0;

  for (;;) {
//      tic=tick; while (tic==tick){};
  }
}


//
// Interrupt handler. 
//
void interrupt ISR(void) {
    static uint8_t disp=0;          // Index into the current display
    static uint8_t lastButton=0x01; //
    static uint16_t localTick=0;    // Internal counter keeping track of the 0.1s tick

    if(INTCONbits.T0IF) {           // If timer flag is set
        INTCONbits.T0IF = 0;        // Clear the interrupt flag 

        if (localTick++>49) {       // Locally count up to 49 before incrementing
            localTick=0;            //   the external 0.1s tick counter
            tick++;
        }
    }

    uint8_t ch=39;
    
    uint8_t u=ch/8;
    uint8_t b=1<<(ch&7);
    uint8_t v;
    
    v=0; if (u==0) v=b;             // Shiftout data for the five '595
    ShiftOut(v);                    // that goes to the character ROM.
    v=0; if (u==1) v=b;             // The data is all 0 bits except for
    ShiftOut(v);                    // a single 1 bit that selects the
    v=0; if (u==2) v=b;             // character data.
    ShiftOut(v);
    v=0; if (u==3) v=b;
    ShiftOut(v);
    v=0; if (u==4) v=b;
    ShiftOut(v);
    
    ShiftOut(~((1<<disp)));         // Select the current display
    
    NOP();
    LATA|=SHIFT_LATCH_a;            // Now latch data in all six '595
    NOP();
    LATA&=~SHIFT_LATCH_a;

    
    
    //    ShiftOut(~((1<<disp)&pattern));

    if (!(PORTC&BUTTON_c)) { //pressed
        lastButton++;
//        lastButton|=(1<<disp);
    } else {
//        lastButton&=~(1<<disp);
    }
    if (disp++==8) {    // All digits refreshed, all button read
        disp=0;          
        pattern=lastButton;
    }

}  



 