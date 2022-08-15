#ifndef XC_I2C_LIBRERY_H
#define	XC_I2C_LIBRERY_H
#include <stdint.h>
#include <xc.h>                // include processor files - each processor file is guarded.4
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 8000000


#endif

/*==========================================================================================================
 ===========================================================================================================*/
void I2C_Init (uint32_t frecuencia);//Función de inicialización del periferico I2C.
void I2C_Start (void);              // Función que inicia la comunicación I2C.
void I2C_ReStart (void);            // Función que reinicia la comuncación I2C.
void I2C_Stop (void);               // Función que detiene la comunicación I2c.
void I2C_Ack (void);                // Función para transmitir Acknowledge.
void I2C_NO_Ack (void);             // Función para transmitir NO Acknowledge.
void I2C_Write (uint8_t data);      // Función para escribir el SSPBUF.
uint8_t I2C_Read (void);            // Función para leer el SSPBUF.

#endif	/* XC_I2C_LIBRERY_H */