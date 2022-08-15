    /*
 * File:
 * Author: Cristian Catú
 * LAB 3
 *
 * Created on 7 august 2022, 19:04
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic16f887.h>
#include <stdio.h>
#include <stdint.h>
#define _XTAL_FREQ 8000000

#include "oscilador.h"
#include "tmr0.h"
#include "I2C.h"
#include "adc.h"
#include "LCD.h"
#include "I2C_librery.h" 

uint8_t contador = 0;
uint8_t z = 0;
uint8_t dato = 0;
uint8_t push = 0;
uint8_t init_POT_0 = 0;
uint8_t dec_POT_0 = 0;
uint8_t VAL_POT_0 = 0;

uint8_t hours = 9;                             // Variable de 8 bits para las horas.
uint8_t minutes = 59;                           // Variable de 8 bits para los minutos.
uint8_t seconds = 50;                           // Variable de 8 bits para los segundos.

unsigned short VOLTAJE_0 = 0;
char s[];

uint8_t BCD_a_Decimal (uint8_t numero);
uint8_t Decimal_a_BCD (uint8_t numero);    // Función que convierte un número decimal a BCD.
void setup(void);
void setup2(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, //Función del mapeo
            unsigned short out_min, unsigned short out_max);

void __interrupt() isr (void){
    if (PORTCbits.RC0 == 0){
        if(PIR1bits.SSPIF == 1){
            SSPCONbits.CKP = 0;
            if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){ //POR SEGURIDAD
                z = SSPBUF;
                SSPCONbits.SSPOV = 0;
                SSPCONbits.WCOL = 0;
                SSPCONbits.CKP = 1;
            }
            if (!SSPSTATbits.D_nA && !SSPSTATbits.R_nW){ //LECTURA DEL MASTER
                z = SSPBUF;
                PIR1bits.SSPIF = 0;
                SSPCONbits.CKP = 1;        
                while(!SSPSTATbits.BF);
                PORTD = ~SSPBUF;
                __delay_us(250);
                
            }
            else if (!SSPSTATbits.D_nA && SSPSTATbits.R_nW){ //ESCRITURA DEL MASTER
                z = SSPBUF;
                BF = 0;
                SSPBUF = PORTB;
                SSPCONbits.CKP = 1;
                __delay_us(250);
                while(SSPSTATbits.BF);
            }
            
            PIR1bits.SSPIF = 0;
        }
        if(PIR1bits.ADIF){
            PORTB = adc_read();
        }
    }
        
}

void main(void) {
    TRISCbits.TRISC0 = 1;
    if (PORTCbits.RC0 == 1){
        setup();
        Lcd_Init(); //inicialización
        Lcd_Clear(); //limpiamos LCD
        
        seconds = Decimal_a_BCD(seconds);
        minutes = Decimal_a_BCD(minutes);
        hours = Decimal_a_BCD(hours);
        
        I2C_Start();                // Llamamos a la función Start.
        I2C_Write(0xD0);            // Escribimos en SSPBUF la dirección de DS1307 1101000 + 0 de escritura.
        I2C_Write(0x00);            // Dirección de segundos.
        I2C_Write(seconds);            // Reiniciamos los segundos.
        I2C_Write(minutes);         // Cargamos el valor de minutos en la dirección de minutos.
        I2C_Write(hours);           // Cargamos el valor de horas en la dirección de horas.
        I2C_Stop();                 // Llamamos a la función Stop.
        __delay_ms(200);            // Retardo de 200 ms.
        
        while(1){
            I2C_Master_Start(); //Iniciamos el master
            I2C_Master_Write(0x50); //escribir en PIC2
            I2C_Master_Write(PORTB);//escribimos lo del purto b
            I2C_Master_Stop();
            __delay_ms(10);
            
            I2C_Master_Start();
            I2C_Master_Write(0x51); //Leemos desde el PIC2, ponemos address
            VAL_POT_0 = I2C_Master_Read(0);//leemos valor del POT
            I2C_Master_Stop(); //STOP
            __delay_ms(10);
            
            I2C_Start();                        // Llamamos a la función Start.
            I2C_Write(0xD0);                    // Escribimos en SSPBUF la dirección de DS1307 1101000 + 0 de escritura.
            I2C_Write(0);                       // Dirección de segundos.
            I2C_ReStart();                      // Llamamos a la función ReStart.
            I2C_Write(0xD1);                    // Escribimos en SSPBUF la dirección de DS1307 1101000 +1 de lectura.
            seconds=I2C_Read();                 // Cargamos la variable "seconds" con el valor de SSPBUF.
            I2C_Ack();                          // ACK.
            minutes=I2C_Read();                 // Cargamos la variable "minutes" con el valor de SSPBUF.
            I2C_Ack();                          // ACK.
            hours=I2C_Read();                   // Cargamos la variable "hours" con el valor de SSPBUF.
            I2C_NO_Ack();                       // NO ACK.
            I2C_Stop();                         // Llamamos a la función Stop.

            __delay_ms(50);                     // Retardo de 50 ms.            
            
            seconds = BCD_a_Decimal(seconds);
            minutes = BCD_a_Decimal(minutes);
            hours = BCD_a_Decimal(hours);
            
            VOLTAJE_0 = map(VAL_POT_0, 0, 255, 0, 500); //mapeamos el voltaje de 0 a 500
            init_POT_0 = VOLTAJE_0/100; //guardo el entero
            dec_POT_0 = VOLTAJE_0-init_POT_0*100; //guardo el decimal
            
            //Mostramos en LCD
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("   HORA     POT");
            Lcd_Set_Cursor(2,1);
            sprintf(s, " %d%d:%d%d:%d%d  %d.%d%dV ", hours/10 ,hours%10, minutes/10, minutes%10, seconds/10, seconds%10, init_POT_0, dec_POT_0/10, dec_POT_0%10); //guardamos el string que vamos a mostrar
            Lcd_Set_Cursor(2,1);
            
            Lcd_Write_String(s);
        }
        return;
    }
    else if (PORTCbits.RC0 == 0){
        setup2();
        while(1){
            adc_start(0); //iniciamos lectura ADC
        }
        return;
    }
}

void setup(void){
    ANSEL = 0;
    TRISA = 0;
    PORTA = 0;
    TRISD = 0;
    PORTD = 0;
    ANSELH = 0;
    TRISB = 0b00001111; 
    init_osc_MHz(3);
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    

    I2C_Master_Init(100000);
}

void setup2(void){
    ANSELH = 0;
    TRISB = 0;
    PORTB = 0;
    TRISD = 0;
    PORTD = 0;
    init_osc_MHz(3);
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    adc_init(1,0,0);
    I2C_Slave_Init(0x50);
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
uint8_t BCD_a_Decimal (uint8_t numero)            // Función que convierte un número BCD a decimal.
{
  return ((numero >> 4) * 10 + (numero & 0x0F));  // Retornamos la variable "numero" desplazado 4 posiciones a la izquierda, multiplicado por 10 más "numero" &&  1111.
}
uint8_t Decimal_a_BCD (uint8_t numero)            // Función que convierte un número decimal a BCD. 
{
    return (((numero / 10) << 4) + (numero % 10));// Retornamos la decena de la variable "numero" desplazado 4 bits a la derecha y las unidades de la variable "numero" en el nibble menos significativo
}