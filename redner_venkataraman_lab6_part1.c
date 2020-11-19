/****** ASEN 4/5067 Lab 6 ******************************************************
 * Author: Lewis Redner, Srikanth Venkataraman
 * Date  : 10/11/20
 *
 * Updated for XC8
 * 
 * Description
 * On power up execute the following sequence:
 *      RD5 ON for 0.5s +/- 10ms then off
 *      RD6 ON for 0.5s +/- 10ms then off
 *      RD7 ON for 0.5s +/- 10ms then off
 * The following then occurs forever:
 *      RD4 blinks: 100ms +/- 10ms ON, then 900ms +/- 10ms OFF
 *      LCD Displays the following lines:
 *          'T=xx.x C'
 *          'PT=x.xxV'
 *      Where the 'x' is replaced by a digit in the measurement.
 *          Temperature data must be calculated / displayed with one digit to
 *          the right of the decimal as shown.  The sensor itself can have
 *          errors up to +/- 5 degrees Celsius.
 *          Potentiometer data must be calculated / displayed with two digits
 *          to the right of the decimal as shown.
 *          These measurements must be refreshed at LEAST at a frequency of 5Hz.
 *      USART Commands are read / executed properly. '\n' is a Line Feed char (0x0A)
 *          ASEN 4067:
 *              'TEMP\n'     - Transmits temperature data in format: 'XX.XC'
 *              'POT\n'      - Transmits potentiometer data in format: X.XXV'
 *          ASEN 5067: Same as ASEN 4067, plus two additional commands
 *              'CONT_ON\n'  - Begins continuous transmission of data over USART
 *              'CONT_OFF\n' - Ends continuous transmission of data over USART
 *
 *              Continuous transmission should output in the following format:
 *                  'T=XX.XC; PT = X.XXV\n'
 *      DAC is used to output analog signal onto RA5 with jumper cables. 
 *          ASEN 4067:
 *              Potentiometer voltage is converted from a digital value to analog 
 *              and output on the DAC. 
 *          ASEN 5067: 
 *              A 0.5 Hz 0-3.3V triangle wave is output on the DAC. 
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/

#include <xc.h>
#include "LCDroutinesEasyPic.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
//const char LCDRow1[] = {0x80,'T','=',0x00};
char LCDRow1[] = {0x80,'T','=','X','X','.','X','C',0x00};
char LCDRow2[] = {0xC0,'P','T','=','X','.','X','X','V',0x00}; //add V???
char cmd_rx[10];
char rx_count = 0;
char status = 0; //0 = non cont
char cmd = 0; 
unsigned int Alive_count = 0;
//unsigned int led_max = 2;
//unsigned int toggle = 0xFF;
//unsigned char alive_times[] = {0x24, 0x46,0xE7, 0x8C};

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void TMR0handler(void);     // Interrupt handler for TMR0, typo in main
void uart_rx_handler(void);
unsigned short get_temperature(void);
unsigned short get_pot(void);
void tx_temp(char prefix);
void tx_pot(char prefix);
void update_display(unsigned short temperature, unsigned short pot);
void parse_cmd(void);


/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
     Initial();                 // Initialize everything
     unsigned short temp;
    unsigned short pot;
     
      while(1) {
        // Sit here for ever
          temp = get_temperature();
          pot = get_pot();
          update_display(temp,pot);
          
        
     }
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR0 and sets CCP1 for compare, and enables LoPri interrupts for
 * both.
 ******************************************************************************/
void Initial() {
    // Configure the IO ports
    TRISD  = 0b00001111;
    LATD = 0;
    TRISC  = 0b10010011;
    LATC = 0;
    
    // Configure the LCD pins for output. Defined in LCDRoutines.h
    LCD_RS_TRIS   = 0;              // 
    LCD_E_TRIS    = 0;
    LCD_DATA_TRIS = 0b11000000;     // Note the LCD is only on the upper nibble
                                    // The lower nibble is all inputs
    LCD_DATA_LAT = 0;           // Initialize LCD data LAT to zero


    // Initialize the LCD and print to it
    InitLCD();
    DisplayC(LCDRow1);
    DisplayC(LCDRow2);
    
    LATDbits.LATD5 = 1;
    __delay_ms(500);
    LATDbits.LATD5 = 0;
    
    LATDbits.LATD6 = 1;
    __delay_ms(500);
    LATDbits.LATD6 = 0;
    
    LATDbits.LATD7 = 1;
    __delay_ms(500);
    LATDbits.LATD7 = 0;

    // Initializing TMR0
    T0CON = 0b00000101;             // 16-bit, 64x prescaler
    TMR0L = 0x8C;
    TMR0H = 0xE7;

    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCON2bits.TMR0IP = 0;         // Assign low priority to TMR0 interrupt
    IPR1bits.RC1IP = 0; //rx interrupts
    PIE1bits.RC1IE = 1;

    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

    T0CONbits.TMR0ON = 1;           // Turning on TMR0
    ADCON0 = 0x01;
    
    ADCON1 = 0x00;
    ADCON2 = 0b10111101;
    
    ANCON0 = 0x05;
    TRISA = 0b00001111;
    
    //_delay_ms(1);
    //unsigned char t = get_temperature();
    
    RCSTAbits.SPEN = 1;
    BAUDCON1 = 0b00000000;
    TXSTA1 = 0b00100100;
    RCSTA1 = 0b10010000;
    SPBRG1 = 51; //19200
    
    
    
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
    
}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR0IF and CCP1IF are clear.
 ******************************************************************************/

void __interrupt(low_priority) LoPriISR(void) 
{
    // Save temp copies of WREG, STATUS and BSR if needed.
    while(1) {
        if( INTCONbits.TMR0IF ) {
            TMR0handler();
            continue;
        }
        if( PIR1bits.RC1IF ) {
            uart_rx_handler();
            continue;
        }
        // Save temp copies of WREG, STATUS and BSR if needed.
        break;      // Supports RETFIE automatically
    }
}


/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter
 ******************************************************************************/
void TMR0handler() {
        LATD = LATD^0x10;

        if(PORTDbits.RD4){
            TMR0L = 0x46; //900 ms
            TMR0H =  0x24;
            INTCONbits.TMR0IF = 0;
            return;
        }
        TMR0L = 0x8C; //100 ms
        TMR0H = 0xE7;
        if(status==1){
            tx_temp(1);
            tx_pot(1);
        }

           
    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}

unsigned short get_temperature(void){
    unsigned short adcl;
    unsigned short adch;
    unsigned short adc_12bit;
    ADCON0bits.CHS0 =1;
    ADCON0bits.CHS1 =1;
    //throw away
    __delay_ms(1);
    ADCON0bits.GO  = 1;

    while(1){
        if(ADCON0bits.DONE ==0){
            
            break;
        }
    }
    __delay_ms(1);
    ADCON0bits.GO  = 1;
    while(1){
        if(ADCON0bits.DONE ==0){
                adcl = ADRESL;
                adch = ADRESH & 0x0F; //0b00001111
                adc_12bit =  adcl | (adch<<8)  ;
                
            break;
        }
    }    
    
    return adc_12bit;
    
}
    
unsigned short get_pot(void){
    
    unsigned short adcl;
    unsigned short adch;
    unsigned short adc_12bit;
    ADCON0bits.CHS0 =0;
    ADCON0bits.CHS1 =0;
    //throw away
    __delay_ms(1);
    ADCON0bits.GO  = 1;

    while(1){
        if(ADCON0bits.DONE ==0){
            
            break;
        }
    }
    __delay_ms(1);
    ADCON0bits.GO  = 1;
    while(1){
        if(ADCON0bits.DONE ==0){
                adcl = ADRESL;
                adch = ADRESH & 0x0F; //0b00001111
                adc_12bit =  adcl | (adch<<8)  ;
                
            break;
        }
    }    
    
    return adc_12bit;

}
    
void update_display(unsigned short temp_adc, unsigned short pot){
    //806 uv /bin
     float voltage = temp_adc*0.806; //mv
     //voltage = 0;
     float temp_c = voltage/10; 
     // check this code
    temp_c = temp_c*10;
    int temp2;
    temp2 = (int)temp_c;
    // convert to integer - typecast
    
    char temp_str[5];
    sprintf(temp_str, "%i", temp2);
    
    // before convert to string try converting to integer

    LCDRow1[3]= temp_str[0];
   LCDRow1[4]= temp_str[1];
   LCDRow1[6]= temp_str[2];
    DisplayC(LCDRow1);
    
    voltage = pot*0.000806;
    temp_c = voltage;
    temp2 = (int)temp_c;
    // convert to integer - typecast
    
    //char temp_str[5];
    sprintf(temp_str, "%.2f", temp_c);
    
    // before convert to string try converting to integer

    LCDRow2[4]= temp_str[0];
   LCDRow2[5]= temp_str[1];
   LCDRow2[6]= temp_str[2];
   LCDRow2[7] = temp_str[3];
    DisplayC(LCDRow2);
    
    __delay_ms(100);
    
    
}
void parse_cmd(void){
    cmd_rx[rx_count] = '\0';
    if(!strcmp(cmd_rx,"TEMP")){
        cmd =1;

    }else if(!strcmp(cmd_rx,"POT")){
        cmd =2;

    }else if(!strcmp(cmd_rx,"CONT_ON")){
        cmd = 3;
    }else if(!strcmp(cmd_rx,"CONT_OFF")){
        cmd =4;
    }
    
    rx_count = 0;
    memset(cmd_rx, 0, 10);
    return;
}
void uart_rx_handler(){
    //from lecture slide
    char temp_rx;
    if( RCSTA1bits.FERR == 1 ) {
        temp_rx = RCREG1; // Clear out RCREG2 to temp register & ignore
        return;
    }
    if( RCSTA1bits.OERR == 1) {
        RCSTA1bits.CREN = 0;
        RCSTA1bits.CREN = 1; // Reset CREN will reset USART2
        return;
    }

    temp_rx = RCREG1;
    if(temp_rx == '\n'){
        parse_cmd();
    }else{
        cmd_rx[rx_count]  = temp_rx;
        rx_count++;
    }

    
    if(cmd ==0){
        return;
    }
    if(cmd==1){
    tx_temp(0);
    }else if(cmd==2){
       tx_pot(0);
    }else if(cmd == 3){
        status = 1;
    }else if(cmd==4){
        status = 0;
    }
    cmd = 0;
  //tx_temp(temp);
    PIR1bits.RC1IF = 0;

}
void tx_temp(char prefix){
    char counter = 0;
    char end =5;
    char offset = 3;
    if(prefix){
        end  = 7;
        offset=1;
    }

    while(1){
    if(PIR1bits.TX1IF !=0 ){
        TXREG1 =LCDRow1[counter+offset];
        counter+=1;
    }
    if(counter ==end){
        while(1){
            if(PIR1bits.TX1IF !=0 ){
            TXREG1 ='\n';
            break;
            }
        }
        
        break;
    }
    }
    //__delay_ms(10);
}
void tx_pot(char prefix){
    char counter = 0;
    char end =5;
    char offset = 4;
    if(prefix){
        end  = 8;
        offset=1;
    }

    while(1){
    if(PIR1bits.TX1IF !=0 ){
        TXREG1 =LCDRow2[counter+offset];
        counter+=1;
    }
    if(counter ==end){
        while(1){
            if(PIR1bits.TX1IF !=0 ){
            TXREG1 ='\n';
            break;
            }
        }
        
        break;
    }
    }
    //__delay_ms(10);
}