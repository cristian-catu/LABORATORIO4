 /*
 * File            : I2C.c
 * Author          : Ligo George
 * Company         : electroSome
 * Project         : I2C Library for MPLAB XC8
 * Microcontroller : PIC 16F877A
 * Created on April 15, 2017, 5:59 PM
 * Link: https://electrosome.com/i2c-pic-microcontroller-mplab-xc8/
 * Modificada por: Pablo Mazariegos con la ayuda del auxiliar Gustavo Ordoñez 
 * Basado en Link: http://microcontroladores-mrelberni.com/i2c-pic-comunicacion-serial/
 */
#include "I2C.h"

void I2C_Master_Init(const unsigned long c)
{
    SSPCON = 0b00101000;
    SSPCON2 = 0;
    SSPADD = (_XTAL_FREQ/(4*c))-1;
    SSPSTAT = 0;
    TRISCbits.TRISC3 = 1;
    TRISCbits.TRISC4 = 1;
}

void I2C_Master_Wait()
{
    while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Master_Start()
{
    I2C_Master_Wait();
    SSPCON2bits.SEN = 1;
}

void I2C_Master_RepeatedStart()
{
    I2C_Master_Wait();
    SSPCON2bits.RSEN = 1;
}

void I2C_Master_Stop()
{
    I2C_Master_Wait();
    SSPCON2bits.PEN = 1;
}

void I2C_Master_Write(unsigned d)
{
    I2C_Master_Wait();
    SSPBUF = d;             
}

unsigned short I2C_Master_Read(unsigned short a)
{
    unsigned short temp;
    I2C_Master_Wait();
    SSPCON2bits.RCEN = 1;
    I2C_Master_Wait();
    temp = SSPBUF;
    I2C_Master_Wait();
    if(a == 1){
        SSPCON2bits.ACKDT = 0;
    }else{
        SSPCON2bits.ACKDT = 1;
    }
    SSPCON2bits.ACKEN = 1;
    return temp;
}

void I2C_Slave_Init(uint8_t address)
{ 
    SSPADD = address;
    SSPCON = 0x36;
    SSPSTAT = 0x80;
    SSPCON2 = 0x01;
    TRISC3 = 1;
    TRISC4 = 1;
    GIE = 1;
    PEIE = 1;
    SSPIF = 0;
    SSPIE = 1;
}
//*****************************************************************************

/*

Ejemplo de interrupción para el dispositivo Periférico. Note que no todas las
variables están definidas dentro de la interrupción.

void __interrupt() isr(void){
   if(PIR1bits.SSPIF == 1){ 

        SSPCONbits.CKP = 0;
       
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL))
        {
            z = SSPBUF;
            SSPCONbits.SSPOV = 0;
            SSPCONbits.WCOL = 0;
            SSPCONbits.CKP = 1;
        }

        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW)
        {
            //__delay_us(7);
            z = SSPBUF;
            //__delay_us(2);
            PIR1bits.SSPIF = 0;
            SSPCONbits.CKP = 1;
            while(!SSPSTATbits.BF);
            PORTD = SSPBUF;
            __delay_us(250);
            
        }
        else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW)
        {
            z = SSPBUF;
            BF = 0;
            SSPBUF = PORTB;
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
       
        PIR1bits.SSPIF = 0;    
    }
}

*/

