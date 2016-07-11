// This file contains board compilation constants
#ifndef BOARD_H
#define	BOARD_H

////////////////////////////////////////////////////////////////////////////////
// Hardware settings

#define FCY 16000000L // Clock frequency, MANDATORY definition for __delay_us and __delay_ms to work properly

// Digital output port lines Direction controls
#define YELLOWLED_DIR TRISEbits.TRISE7    // LED D3 is YELLOW LED
#define GREENLED_DIR TRISEbits.TRISE6    // LED D2 is GREEN LED
#define REDLED_DIR TRISEbits.TRISE5    // LED D1 is RED LED

#define TEMPSENSOR_PWRCTRL_DIR TRISGbits.TRISG6

#define IDIO_DIR TRISEbits.TRISE4

#define RFCS_DIR TRISFbits.TRISF1
#define RFWK_DIR TRISDbits.TRISD7
#define RFRST_DIR TRISFbits.TRISF0

#define ACCCS_DIR TRISEbits.TRISE3
#define ACCRST_DIR TRISEbits.TRISE2

// Digital output port lines
#define YELLOWLED LATEbits.LATE7
#define GREENLED LATEbits.LATE6
#define REDLED LATEbits.LATE5

#define TEMPSENSOR_PWRCTRL LATGbits.LATG6

#define IDIO LATEbits.LATE4 

#define RFCS LATFbits.LATF1
#define RFWK LATDbits.LATD7
#define RFRST LATFbits.LATF0

#define ACCCS LATEbits.LATE3
#define ACCRST LATEbits.LATE2

// digital inputs
#define IDIO_READ PORTEbits.RE4
#define RFINT PORTDbits.RD6
#define ACCINT1 PORTEbits.RE0
#define ACCINT2 PORTEbits.RE1
#define YELLOWLED_READ PORTEbits.RE7
#define GREENLED_READ PORTEbits.RE6
#define REDLED_READ PORTEbits.RE5

// Analog Inputs
#define TEMP_SENSOR_READ PORTBbits.RB6

// UART
#define UART_TX_DIR TRISDbits.TRISD1
#define UART_RX_DIR TRISDbits.TRISD2
#define UART_TX PORTDbits.RD1
#define UART_RX PORTDbits.RD2

// Sensor related constants
#define TEMPERATURE_SENSOR_ADC_CHANNEL 6 // RB6/AN6 of the microcontroller is used for monitoring temperature

#endif /* BOARD_H */

