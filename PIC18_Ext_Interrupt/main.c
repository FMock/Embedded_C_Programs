/***********************************************
 * Project Name: PIC18_Ext_INT
 * Author: Frank Mock, August 2017
 * File Name: main.c
 * Microcontroller used: PIC18F45K22
 * Compiler used is XC8 VERSION 1.35
 * Description: The main loop simply reads the value of the ADC output, when an 
 *              external interrupt occurs, the code in the ISR will be executed.
 *              The ISR sends the current voltage value out the serial port 
 *              (USART1).
 * Serial Port Settings: Baud rate 115200, 8 bit, no flow control, no stop bit
 ****************************/
#include<xc.h>
#include<stdio.h> // For sprintf to convert int to string
#include<plib\usart.h>
#include<plib\adc.h>
// Oscillator Selection bits (Internal oscillator block)
#pragma config FOSC = INTIO67
// MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)
#pragma config MCLRE = EXTMCLR
// Watchdog Timer Enable bits (Watchdog timer is always disabled)
#pragma config WDTEN = OFF

#define _XTAL_FREQ 16000000  //speed of the internal oscillator

unsigned char c; // will hold rx value
const float VOLTAGE = 3.291; // system voltage
float volts; // to hold the voltage
char str[256]; //buffer to hold string

// Function prototypes
void SetUpClock();// setup internal oscillator
void interrupt isr(void); //UART receive interrupt handler
void delay_seconds(unsigned char s); //Creates delay in seconds
void FlashLEDs(); //Flashes an LED at one second intervals
float readVoltage(); // Read the ADC, convert and assign to volts
void txVoltage(); // Transmit voltage out the serial port

int main(){
    SetUpClock(); //internal clock set to 16MHz
    
    /*Port setup for the LED*/
    ANSELD = 0; //Configure PORTD as digital
    TRISD = 0; //Configure PORTD as output
    PORTD = 0b00000000; //initial state - PORTD all off
    
    /*Port setup for the ADC*/
    ANSELB = 0b00000001; // Configure pin33 RB0 (AN12) as analog
    TRISB = 0b00000011; // Set RB0 and RB1 as an input
    
    /*Configure the A/D converter*/
    OpenADC(ADC_FOSC_2 & ADC_RIGHT_JUST & ADC_12_TAD,
            ADC_CH12 & ADC_INT_OFF,
            ADC_TRIG_CTMU & ADC_REF_VDD_VDD & ADC_REF_VDD_VSS);
     /*
     * According to page 268 of the data sheet, TRIS control bits of
     * RX and TX should be set to 1. USART will automatically reconfigure
     * them from input to output as necessary */
    TRISCbits.RC6 = 1;
    TRISCbits.RC7 = 1;
    
    //analog PORTC ports may interfere with serial port receive
    ANSELC = 0X00; //Make all PORTC pins digital
    RCONbits.IPEN = 0; //Disable using interrupt priority
        
    /* Enable Global Interrupts GIE, bit 7
     * Enable Peripheral Interrupts, bit 6 */
    INTCON = 0b11000000;
    
    /*Set INT1 to trigger on rising edge*/
    INTCON2bits.INTEDG1 = 1;
    
    /*Clear INT1 interrupt flag*/
    INTCON3bits.INT1IF = 0;
    
    /*Enable external interrupt INT1*/
    INTCON3bits.INT1E = 1;
        
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
        volts = readVoltage(); // ADC
    }
}

/*IRCF<2:0> Set Up Internal RC Oscillator Frequency Select bits
 * 111 = HFINTOSC - (16 MHz) SEE PAGE 32 OF DATASHEET
 */
void SetUpClock(){
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
}

//UART receive interrupt handler
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
    
    // If external interrupt has occurred
    if(INTCON3bits.INT1IF == 1){
        txVoltage(); // transmit voltage out serial port
        INTCON3bits.INT1IF = 0; // clear INT1 interrupt flag
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

float readVoltage(){
    SelChanConvADC(ADC_CH12); // Select CH12 and start the conversion
    while(BusyADC()); // Wait for A/D conversion to complete
    int v = ReadADC(); // Read converted data
    return v * VOLTAGE/1024; // convert to volts
}

void txVoltage(){
    sprintf(str, "Volts = %f\r\n", volts); // convert to string
    putrs1USART(str); // write the string out the serial port
}