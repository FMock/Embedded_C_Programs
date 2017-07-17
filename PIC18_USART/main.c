/***********************************************
 * Project Name: PIC18-USART
 * Author: Frank Mock July 2017
 * File Name: main.c
 * Microcontroller used: PIC18F45K22
 * Compiler used is XC8 VERSION 1.35
 * Description: The PIC will receive and transmit via it's serial port
 * Serial Port Settings: Baud rate 115200, 8 bit, no flow control, no stop bit
 ****************************/
#include<xc.h>
#include<plib\usart.h> // provides serial port related functions

// Oscillator Selection bits (Internal oscillator block)
#pragma config FOSC = INTIO67
// MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)
#pragma config MCLRE = EXTMCLR
// Watchdog Timer Enable bits (Watchdog timer is always disabled)
#pragma config WDTEN = OFF

#define _XTAL_FREQ 16000000  //speed of the internal oscillator

unsigned char c; // will hold rx and tx value

// Function prototypes
void SetUpClock();// setup internal oscillator
void interrupt isr(void); //UART receive interrupt handler
void delay_seconds(unsigned char s); //Creates delay in seconds
void FlashLEDs(); //Flashes an LED at one second intervals

int main(){
    SetUpClock(); //internal clock set to 16MHz

    ANSELD = 0; //Configure PORTD as digital
    TRISD = 0; //Configure PORTD as output
    PORTD = 0b00000000; //initial state - PORTD all off
    
     /*
     * According to page 268 of the data sheet, TRIS control bits of
     * RX and TX should be set to 1. USART will automatically reconfigure
     * them from input to output as necessary
     */
    TRISCbits.RC6 = 1;
    TRISCbits.RC7 = 1;
    
    //analog PORTC ports may interfere with serial port receive
    ANSELC = 0X00; //Make all PORTC pins digital
    RCONbits.IPEN = 0; //Disable using interrupt priority
    INTCONbits.GIE = 1; //Enable all unmasked interrupts
    INTCONbits.PEIE = 1; //Enable all unmasked peripheral interrupts
    
    Close1USART();  //turn off USART if was previously on
    
    //configure USART
    Open1USART(USART_TX_INT_OFF & 
               USART_RX_INT_ON &
               USART_ASYNCH_MODE &
               USART_EIGHT_BIT &
               USART_CONT_RX &
               USART_BRGH_HIGH,
               8);//8 = x. Calculated x using 16(x + 1) = Fosc / Baud Rate
                 // This gave me a 115,200 baud rate
        
    for(;;){   
        FlashLEDs();
    }
} // END main

/*IRCF<2:0> Set Up Internal RC Oscillator Frequency Select bits
 * 111 = HFINTOSC - (16 MHz) SEE PAGE 32 OF DATASHEET
 */
void SetUpClock(){
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
}

// interrupt service routine
void interrupt isr(void){
    //The serial port has received a byte
    if((PIR1bits.RC1IF == 1)&&(PIE1bits.RC1IE == 1)){
        // The serial port has received a byte  
        c = Read1USART();
        // Transmit what was received out the serial port
        Write1USART(c);
        //Clear ISR flag
        PIR1bits.RC1IF = 0;      
    }
}

/* Creates delay in seconds
 * parameter s is the number of seconds */
void delay_seconds(unsigned char s){
    unsigned char i,j;
    
    for(i = 0; i < s; i++){
        for(j = 0; j < 100; j++)
            __delay_ms(10);
    }
}

//Flashes an LED at one second intervals
void FlashLEDs(){
    for(int i = 0;i < 2; i++){
        PORTDbits.RD1 = 1;  //LED 1 ON
        delay_seconds(1);
        PORTDbits.RD1 = 0;  //LED 1 OFF
        delay_seconds(1);
    }
}