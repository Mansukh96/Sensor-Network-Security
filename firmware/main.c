// This file contains main function for base station in non beacon mode
//#include <stdio.h>
#include <stdlib.h>
#include "board.h"
#include "mrf24j40.h"
#include "fxls8471q.h"
#include "ieee802154.h"
#include "config.h"
#include <libpic30.h>
#include "p24FJ256GA106.h"

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
// HARDWARE
#pragma config POSCMOD = 1              // Oscillator selection right after RESET is XTPLL
#pragma config FNOSC = 3
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer is enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx3               // Comm Channel Select (Emulator functions are shared with PGEC3/PGED3)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)
////////////////////////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
// HARDWARE
int CurrentCPUIPL; // this variable is used for saving current IPL level
unsigned int BatteryVoltage; // Binary representation of Battery Voltagein millivolt
unsigned char Temperature; // Binary representation of temperature
unsigned char Timer1IntOccurred;

unsigned char U1TxBuf[256];
unsigned char U1TxBufLen;
unsigned char U1TxBufIndex;
unsigned char U1RxBuf[256];
unsigned char U1RxBufIndex; // Index in U1RxBuf into which next received character will be written into by the RxISR
unsigned char U1TxCompleted;

// PHY LAYER
// PHY PIB attributes
unsigned char PhyCurrentChannel;
unsigned long PhyChannelsSupported;
unsigned char PhyTransmitPower;
unsigned char PhyCCAMode;
// PHY variables
volatile unsigned char Psdu[aMaxPHYPacketSize];
volatile unsigned char PsduLength;
volatile unsigned char PpduLinkQuality;
volatile unsigned char PhyState;
volatile unsigned char PhyPktReceived;
volatile unsigned char TxCompleted;

// MAC LAYER
unsigned int MacPANID; // PAN ID 
unsigned char EUID48[6]; // read from Microchip 11AA02E48 IC, LSByte in EUID48[0]
unsigned char MacExtendedAddress[8]; // formed from EUID48 to get unique IEEE address, LSByte in MacExtendedAddress[0]
unsigned int MacShortAddress;
unsigned char MacCoordExtendedAddress[8]; // IEEE 64 bit address of co-ordinator
unsigned int MacCoordShortAddress;
unsigned char MacDSN; // Sequence number to be added to the transmitted data/MAC command frame
ACLEntry MacACLEntryDescriptorSet[MAX_NODES_PER_PAN];
unsigned char MacHeaderLength;
unsigned char MacFrameLength;
unsigned char Msdu[aMaxMACFrameSize];
unsigned char MsduLength;

// NWK LAYER

// APPLICATION
unsigned char NumNodes;
////////////////////////////////////////////////////////////////////////////////
// Function Prototypes
unsigned char MRF24J40_ReadShortRAMAddr(unsigned char Addr);
void MRF24J40_WriteShortRAMAddr(unsigned char Addr, unsigned char Dat);
void MRF24J40_GoToSleep(void);
void MRF24J40_SetChannel(unsigned char Channel);
unsigned char MRF24J40_ReadLongRAMAddr(unsigned int Addr);
void MRF24J40_HardwareReset(void);
void BinToAscii(unsigned int Binary, unsigned char *Buf, unsigned char NumChars, unsigned char DecHex);
////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
// HARDWARE

void EnterProtectedSection(void) {
    // enter protected section of code by the mechanism of raising IPL to 7
    SET_AND_SAVE_CPU_IPL(CurrentCPUIPL, 7);
}

void LeaveProtectedSection(void) {
    // Leave protected code section and restore IPL
    RESTORE_CPU_IPL(CurrentCPUIPL);
}

void SPI1Init(void) {
    SPI1CON1bits.MSTEN = 1;
    SPI1CON1bits.SPRE = 7; // SPI clock is scaled down from FCY by 64. So SPI Clock = 500 KHz
    SPI1CON1bits.PPRE = 0;
    SPI1CON1bits.CKE = 1;
    SPI1CON1bits.SMP = 1;
    SPI1CON1bits.CKP = 0;
    SPI1STATbits.SPIROV = 0;
    SPI1STATbits.SPIEN = 1;
    IFS0bits.SPI1IF = 0;
}

unsigned char SPI1SendByte(unsigned char Dat) {
    IFS0bits.SPI1IF = 0;
    SPI1BUF = Dat; // send the byte
    while (!IFS0bits.SPI1IF); // wait for bits to be sent out
    IFS0bits.SPI1IF = 0;
    return SPI1BUF;
}

unsigned char SPI1ReadByte(void) {
    SPI1BUF = 0x00; // send the dummy byte
    while (!IFS0bits.SPI1IF); // wait for bits to be sent out
    IFS0bits.SPI1IF = 0;
    return SPI1BUF;
}

void RTCCWREN(void) {
    asm volatile("disi #5");
    __builtin_write_RTCWEN();
}

// This function has to be passed all values in BCD format.
// It enables Alarm every second.  Since the PIC microcontroller disables ALARM after the alarm occurs, in the RTCC ISR we have to enable the ALARM again.

void RTCCInit(unsigned char Year, unsigned char Month, unsigned char Date, unsigned char DayOfWeek, unsigned char Hour, unsigned char Minute, unsigned char Second) {

    PMD3 &= 0xfdff;

    // Enable RTCC
    RTCCWREN();
    RCFGCALbits.RTCEN = 0;

    // Write RTC values
    RCFGCALbits.RTCPTR = 3; // point to year
    RTCVAL = Year; // none,year
    RTCVAL = (unsigned int) Month << 8 | Date; // month,date
    RTCVAL = (unsigned int) DayOfWeek << 8 | Hour; // weekday,hours
    RTCVAL = (unsigned int) Minute << 8 | Second; // minutes,seconds
    RCFGCALbits.RTCEN = 1;
    RCFGCALbits.RTCWREN = 0;

    // Set ALARM for every 1 second
    ALCFGRPTbits.AMASK = 1;
    ALCFGRPTbits.ALRMEN = 1;

    // Enable RTCC ALARM interrupts
    IFS3bits.RTCIF = 0;
    IEC3bits.RTCIE = 1;
}

void BoardInit(void) {
    // Pin selection for peripherals.
    // SPI1 pin selection
    __builtin_write_OSCCONL(0x00); // Unlock IOLOCK
    RPINR20bits.SDI1R = 22; // SDI1 is assigned to RP22
    RPOR12bits.RP25R = 7; // SDO1 is assigned to RP25
    RPOR10bits.RP20R = 8; // SCK1 is assigned to RP20
    RPINR18bits.U1RXR = 23; // UART1 RX is assigned to RP23
    RPOR12bits.RP24R = 3; // UART1 TX is assigned to RP24
    __builtin_write_OSCCONL(0x42); // Lock IOLOCK. Enable SOSC 32.768 KHz oscillator

    // Make all unused Port lines as digital outputs and turn them LOW
    AD1PCFGL = 0xffbf; // First turn all default Analog pins to digital except RB6/AN6 which is used for TEMPERATURE. Leave RB4 and RB5 untouched since they are programming pins
    TRISB &= 0x0070; // Turn unused port lines as Digital outputs
    TRISD &= 0xf0fe;
    TRISE &= 0xfff0;
    TRISF &= 0xff83;
    TRISG &= 0xfc73;

    // Turn unused port lines LOW
    LATB &= 0x0070;
    LATD &= 0xf0fe;
    LATE &= 0xfff0;
    LATF &= 0xff83;
    LATG &= 0xfc73;

    // LED init
    REDLED_DIR = 0;
    GREENLED_DIR = 0;
    YELLOWLED_DIR = 0;
    REDLED = 0;
    GREENLED = 0;
    YELLOWLED = 0;

    //Deselect CS lines to both FXLS8471 and MRF24J40MA . Initialize SPI1
    RFCS_DIR = 0;
    RFCS = 1;
    SPI1Init();

    // Timer1 init for system tick
    T1CONbits.TCS = 1;
    TMR1 = 0;
    PR1 = 32768;
    T1CONbits.TON = 1;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;

    // UART1 init. 38400 baud rate
    U1RxBufIndex = 0;
    U1BRG = 25; // Baud rate = FCY/16 divided by U1BRG
    U1STAbits.UTXISEL0 = 1; // Tx Interrupt AFTER char is transmitted; Rx interrupt on receipt of character
    IFS0bits.U1RXIF = 0; // Clear UART1 RX interrupt flag
    IFS0bits.U1TXIF = 0; // Clear UART1 TX interrupt flag
    IEC0bits.U1RXIE = 1; // enable UART1 RX interrupts
    IEC0bits.U1TXIE = 1; // enable UART1 TX interrupts
    U1MODEbits.UARTEN = 1; // enable UART1

    // Interrupt system init. Enable ACCelerator Interrupts, Timer1 and RTCC interrupts. Enable UART1 interrupts if it is Base station
    INTCON1bits.NSTDIS = 1; // disable nested interrupts

    // Disable all modules except SPI1, Timer1, UART1.  The other modules are enabled as needed during operation
    PMD1 = 0xf7d7;
    PMD2 = 0xffff;
    PMD3 = 0xffff;
    PMD4 = 0xffff;
    PMD5 = 0xffff;
    PMD6 = 0xffff;

    // MRF24J40MA Reset lines is made an output
    RFRST_DIR = 0;
}

// converts the specified ADC channel.
// The reference for conversion is always AVDD.  Since in the battery operated application AVDD will be
// varying, the first task in the conversion is to measure AVDD.  AVDD is measured by doing a BandGap
// reference measurement using AVDD as the reference.  Since the BG Reference is always constant (1.2V), AVDD
// can be calcuated.
// The next task is the measurement of the specified ADC channel.  This is done again with AVDD as
// reference. Since AVDD is now known, the specified ADC channel can be measured.
// Since this function always does a AVDD measurement first, it writes this value into the global
// variable BatteryVoltage.

unsigned int ADCConvert(unsigned char ADCChannel) {
    unsigned int T16;
    unsigned long T32;

    PMD1bits.ADC1MD = 0; // Enable the ADC
    AD1CON1bits.ADON = 1; // Turn ON ADC module

    AD1CON3bits.SAMC = 31; // 31 TAD is the sampling period
    AD1CON3bits.ADCS = 31; // TAD = 32*TCY
    AD1PCFGHbits.PCFG17 = 1; // Internal Band Gap reference is enabled for input scan

    AD1CHSbits.CH0SA = 17; // Band Gap reference is the analog input to convert.
    AD1CON1bits.SAMP = 1; // Start SAMPling
    __delay_us(50);
    AD1CON1bits.SAMP = 0; // End SAMPling
    while (AD1CON1bits.DONE == 0); // Wait for end of conversion
    T16 = ADC1BUF0;

    T32 = ((unsigned long) 1200 * 0x3ff) / T16; // Calculate BatteryVoltage in millivolt
    BatteryVoltage = T32;

    AD1CHSbits.CH0SA = ADCChannel; // select analog channel to convert.
    AD1CON1bits.SAMP = 1; // Start SAMPling
    __delay_us(50);
    AD1CON1bits.SAMP = 0; // End SAMPling
    while (AD1CON1bits.DONE == 0); // Wait for end of conversion
    T16 = ADC1BUF0;

    AD1CON1bits.ADON = 0; // Turn OFF ADC module
    PMD1bits.ADC1MD = 1; // Disable the ADC

    return T16;
}

// This function converts ADCValue from MCP9700A into a temperature value in degree celcius
// It uses BatteryVoltage value. MCP9700A produces 500mV at 0 celcius, and this voltage increases
// at 10mV per degree celcius.

unsigned char ConvertADCValToTemperature(unsigned int ADCValue) {
    unsigned long T32;
    unsigned char T8;

    T32 = (ADCValue * BatteryVoltage) / 0x3ff;

    T32 -= 500;
    T8 = T32 / 10;
    if ((T32 % 10) >= 5)
        T8 += 1;
    return T8;
}

// Sets up Timer1 to generate an interrupt after DurationinSymbolPeriods symbol periods (ie after DurationinSymbolPeriods * 16 microseconds)
// Timer1 runs on FOSC/2 with 1:256 prescale, ie at 62500 Hz

void SetTimer1(unsigned int DurationInSymbolPeriods) {
    T1CONbits.TON = 0; // First turn OFF Timer1
    Timer1IntOccurred = 0;
    PR1 = DurationInSymbolPeriods;
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt flag
    T1CONbits.TON = 1; // Turn ON Timer1
}

// Initiates transmission of a character sequence in U1TxBuf[128], length in U1TxBufLen
// Should be called after filling in U1TxBuf, U1TxBufLen, U1TxBufIndex = 0.
// The transmission is completed by the _U1TxInterrupt ISR.

void U1Tx(void) {
    U1TxCompleted = 0;
    U1TXREG = U1TxBuf[U1TxBufIndex++];
    U1STAbits.UTXEN = 1;
}

// Functions to read EUID48 from the 11AA02E48 IC. Right now a dummy function which simply fills zeros in EUID48
// Bit period is 50us (corresponds to 20KHz)
// Initializtion of IDIO.

void EUID48Init(void) {
    // First Make the SCIO line as output and turn it LOW
    IDIO_DIR = 0;
    IDIO = 0;
    __delay_ms(1);
    // Generate the LOW to HIGH transition required after POR
    IDIO = 1;
    __delay_ms(1);
    // Turn SCIO back to LOW in preparation for subsequenty STBY pulse
    IDIO = 0;
}

// Sends HIGH pulse for minimum TSTBY (600us)

void EUID48SendStandbyPulse(void) {
    IDIO = 1;
    __delay_ms(1);
    IDIO = 0;
}

// Maintains LOW for minimum THDR (5 us)

void EUID48SendHeaderLowPeriod(void) {
    __delay_ms(1);
}

// sends LOW to HIGH pulse for DataBit=1 and HIGH To LOW pulse for DataBit=0

void EUID48SendEncodedBit(unsigned char DataBit) {
    IDIO_DIR = 0;
    if (DataBit == 0) {
        IDIO = 1;
        __delay_us(25);
        IDIO = 0;
        __delay_us(25);
    } else {
        IDIO = 0;
        __delay_us(25);
        IDIO = 1;
        __delay_us(25);
    }
}

void EUID48SendEncodedByte(unsigned char DataByte) {
    unsigned char T8;
    unsigned char Mask;

    Mask = 0x80;
    for (T8 = 0; T8 < 8; T8++) {
        if ((DataByte & Mask) != 0)
            EUID48SendEncodedBit(1);
        else
            EUID48SendEncodedBit(0);
        Mask = Mask >> 1;
    }
}

// returns 1 if LOW to HIGH is detected within bit period, and 0 if HIGH to LOW is detected.
// If no transition is detected it returns -1.

char EUID48ReadEncodedBit(void) {
    unsigned char i;
    unsigned char j;
    unsigned int T16;
    char RetVal;

    IDIO_DIR = 1;
    __delay_us(10);
    i = IDIO_READ;
    T16 = 0;
    do {
        j = IDIO_READ;
    } while ((i == j) && (T16++ < 1000));
    if (T16 != 1000) {
        if ((i == 0) && (j == 1))
            RetVal = 1;
        else
            RetVal = 0;
    } else
        RetVal = -1;
    __delay_us(25);
    return RetVal;
}

unsigned char EUID48ReadEncodedByte(void) {
    unsigned char T8;
    unsigned char Val;

    Val = EUID48ReadEncodedBit();
    for (T8 = 0; T8 < 7; T8++) {
        Val = Val << 1;
        Val |= EUID48ReadEncodedBit();
    }
    return Val;
}

void ReadEUID48From11AA02E48(void) {
    unsigned char T8;

    // Send STBY pulse
    EUID48SendStandbyPulse();
    // Send THDR
    EUID48SendHeaderLowPeriod();
    // Send Header of 01010101
    EUID48SendEncodedByte(0x55);
    // Send MAK bit
    EUID48SendEncodedBit(1);
    // Wait for NoSAK period
    __delay_us(50);
    // Send Device address of 10100000
    EUID48SendEncodedByte(0xA0);
    // Send MAK bit
    EUID48SendEncodedBit(1);
    // Wait for SAK
    EUID48ReadEncodedBit();
    // Send Read Command 00000011
    EUID48SendEncodedByte(0x03);
    // Send MAK bit
    EUID48SendEncodedBit(1);
    // Wait for SAK
    EUID48ReadEncodedBit();
    // Send Word Address MSB (0x00)
    EUID48SendEncodedByte(0x00);
    // Send MAK
    EUID48SendEncodedBit(1);
    // Wait for SAK
    EUID48ReadEncodedBit();
    // Send Word Address LSB (0xFA)
    EUID48SendEncodedByte(0xFA);
    // Send MAK
    EUID48SendEncodedBit(1);
    // Wait for SAK
    EUID48ReadEncodedBit();

    // Read Values from locations 0xFA through 0xFE
    for (T8 = 0; T8 < 5; T8++) {
        // Read Value
        EUID48[T8] = EUID48ReadEncodedByte();
        // Send MAK
        EUID48SendEncodedBit(1);
        // Wait for SAK
        EUID48ReadEncodedBit();
    }
    // Read Value at location 0xFF
    EUID48[T8] = EUID48ReadEncodedByte();
    // Send NoMAK
    EUID48SendEncodedBit(0);
    // Wait for SAK
    EUID48ReadEncodedBit();

    // At exit, the SCIO line should be an output, turned HIGH. See Section 3.1 of datasheet, Standby Pulse - first paragraph.  It states
    // that a HIGH to LOW transition on SCIO will return device to ACTIVE state.  So leave the SCIO line HIGH.
    __delay_ms(10); // Wait for completion of any Slave driving the SCIO line
    IDIO_DIR = 0;
    IDIO = 0;
    __delay_ms(10);
    IDIO = 1;
}

void FormMacExtendedAddressFromEUID48(void) {
    unsigned char T8;

    for (T8 = 0; T8 < 3; T8++)
        MacExtendedAddress[T8] = EUID48[5 - T8];
    MacExtendedAddress[3] = 0xfe;
    MacExtendedAddress[4] = 0xff;
    for (T8 = 3; T8 < 6; T8++)
        MacExtendedAddress[T8 + 2] = EUID48[5 - T8];
}

// Timer1 interrupt

void __attribute__((interrupt, auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; // Clear Timer1 Interrupt flag
    Timer1IntOccurred = 1;
    if (YELLOWLED_READ == 1)
        YELLOWLED = 0;
    else
        YELLOWLED = 1;
}

// UART1 RX interrupt
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void) {
    unsigned char RxChar;

    IFS0bits.U1RXIF = 0; // Clear UART1 Rx Interrupt flag
    RxChar = U1RXREG;
    if (U1RxBufIndex != 128) {
        U1RxBuf[U1RxBufIndex++] = RxChar;
    }
}

// UART1 TX interrupt

void __attribute__((interrupt, auto_psv)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0; // Clear UART1 TX interrupt flag
    if (U1TxBufLen > U1TxBufIndex)
        U1TXREG = U1TxBuf[U1TxBufIndex++];
    else {
        U1STAbits.UTXEN = 0;
        U1TxCompleted = 1;
    }
}

// Change Notification Interrupts for ACCINT1, ACCINT2, RFINT

void __attribute__((interrupt, auto_psv)) _CNInterrupt(void) {
    unsigned char T8;

    // Clear CNIF Flag
    IFS1bits.CNIF = 0;

    // Check if RFINT interrupt occurred, this is active LOW
    if (PORTDbits.RD6 == 0) {
        T8 = MRF24J40_ReadShortRAMAddr(INTSTAT);
        if (T8 & 0x08) // if RX interrupt has been raised
            PhyPktReceived = 1;
        if (T8 & 0x01) // if TX complete interrupt has been raised
            TxCompleted = 1;
    }

    // Check if ACCINT1 falling edge occurred, this is also active low
    if (PORTEbits.RE0 == 0) {
        if (REDLED_READ == 1)
            REDLED = 0;
        else
            REDLED = 1;
    }
}

// RTC interrupt

void __attribute__((interrupt, auto_psv)) _RTCCInterrupt(void) {
    IFS3bits.RTCIF = 0; // Clear RTC Interrupt flag
    ALCFGRPTbits.ALRMEN = 1;
}

unsigned char MRF24J40_ReadShortRAMAddr(unsigned char Addr) {
    unsigned char T8 = 0;

    RFCS = 0;
    __delay_us(1);
    SPI1SendByte(Addr << 1);
    T8 = SPI1ReadByte();
    RFCS = 1;
    return (T8);
}

void MRF24J40_WriteShortRAMAddr(unsigned char Addr, unsigned char Dat) {
    RFCS = 0;
    __delay_us(1);
    SPI1SendByte((Addr << 1) | 0x1);
    SPI1SendByte(Dat);
    RFCS = 1;
}

unsigned char MRF24J40_ReadLongRAMAddr(unsigned int Addr) {
    unsigned char T8;

    T8 = 0;
    RFCS = 0;
    SPI1SendByte(((Addr >> 3)&0x7F) | 0x80);
    SPI1SendByte(((Addr << 5)&0xE0));
    T8 = SPI1ReadByte();
    RFCS = 1;
    return (T8);
}

void MRF24J40_WriteLongRAMAddr(unsigned int Addr, unsigned char Dat) {
    RFCS = 0;
    SPI1SendByte((((Addr >> 3))&0x7F) | 0x80);
    SPI1SendByte((((Addr << 5))&0xE0) | 0x10);
    SPI1SendByte(Dat);
    RFCS = 1;
}

unsigned long MRF24J40_GetSupportedChannels(void) {
    return 0x07fff800; // return bitmap showing that channels 11 to 26 are supported 
}

unsigned char MRF24J40_CCA(void) {
    MRF24J40_WriteShortRAMAddr(BBREG6, 0x80);
    while ((MRF24J40_ReadShortRAMAddr(BBREG6) & 0x01) == 0x00);
    return (MRF24J40_ReadLongRAMAddr(RSSI));
}

void MRF24J40_HardwareReset(void) {
    RFRST = 0;
    __delay_us(500); // Minimum 500 us RESET LOW pulse
    RFRST = 1;
    __delay_ms(2); // as per datasheet, wait at least 2ms after RESET pulse
}

/*
1. SOFTRST (0x2A) = 0x07 ? Perform a software Reset. The bits will be automatically cleared to ?0? by hardware.
2. PACON2 (0x18) = 0x98 ? Initialize FIFOEN = 1 and TXONTS = 0x6.
3. TXSTBL (0x2E) = 0x95 ? Initialize RFSTBL = 0x9.
4. RFCON0 (0x200) = 0x03 ? Initialize RFOPT = 0x03.
5. RFCON1 (0x201) = 0x01 ? Initialize VCOOPT = 0x02.
6. RFCON2 (0x202) = 0x80 ? Enable PLL (PLLEN = 1).
7. RFCON6 (0x206) = 0x90 ? Initialize TXFIL = 1 and 20MRECVR = 1.
8. RFCON7 (0x207) = 0x80 ? Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
9. RFCON8 (0x208) = 0x10 ? Initialize RFVCO = 1.
10. SLPCON1 (0x220) = 0x21 ? Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.
Configuration for nonbeacon-enabled devices (see Section 3.8 ?Beacon-Enabled and Nonbeacon-Enabled Networks?):
11. BBREG2 (0x3A) = 0x80 ? Set CCA mode to ED.
12. CCAEDTH = PHY_CCA_ED_THRESHOLD ? Set CCA ED threshold.
13. BBREG6 (0x3E) = 0x40 ? Set appended RSSI value to RXFIFO.
14. Enable interrupts ? See Section 3.3 ?Interrupts?.
15. Set channel ? See Section 3.4 ?Channel Selection?.
16. Set transmitter power - See ?REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)?.
17. RFCTL (0x36) = 0x04 ? Reset RF state machine.
18. RFCTL (0x36) = 0x00.
19. Delay at least 192 µs.
 */
void MRF24J40_Init(void) {
    MRF24J40_WriteShortRAMAddr(PACON2, 0x98);
    MRF24J40_WriteShortRAMAddr(TXSTBL, 0x95);
    MRF24J40_WriteLongRAMAddr(RFCON0, 0x03);
    MRF24J40_WriteLongRAMAddr(RFCON1, 0x02);
    MRF24J40_WriteLongRAMAddr(RFCON2, 0x80);
    MRF24J40_WriteLongRAMAddr(RFCON6, 0x90);
    MRF24J40_WriteLongRAMAddr(RFCON7, 0x80);
    MRF24J40_WriteLongRAMAddr(RFCON8, 0x10);
    MRF24J40_WriteLongRAMAddr(SLPCON1, 0x21);
    MRF24J40_WriteShortRAMAddr(BBREG2, 0x80);
    MRF24J40_WriteShortRAMAddr(CCAEDTH, PHY_CCA_ED_THRESHOLD);
    MRF24J40_WriteShortRAMAddr(BBREG6, 0x40);
    MRF24J40_WriteShortRAMAddr(RXFLUSH, 0x0c); // Enable reception of COMMAND and DATA frames ONLY.
    // Code for channel selection, if not default channel 11, should come here
    MRF24J40_SetChannel(PHY_PIB_CURRENT_CHANNEL);
    // Code for setting Tx Power level, if it is different from 0 dB, should come here
    MRF24J40_ReadShortRAMAddr(INTSTAT); // Clear Interrupt flags
    MRF24J40_WriteShortRAMAddr(INTCON, 0xf6); // Enable Tx and Rx interrupts
    CNEN1bits.CN15IE = 1; // enable Change Notification interrupt for RFINT (which is on RD6)
}

/* This function does the necessary configuration for a non-beacon network.
   It checks the SRAMNodeShortAddress variable to see if the node is the PAN coordinator or not, and accordingly does the initialization.*/
void MRF24J40_Configure_NonBeaconNetwork(void) {
    unsigned char T8;

    // Write PANID into PANIDL and PANIDH registers
    MRF24J40_WriteShortRAMAddr(PANIDL, MacPANID);
    MRF24J40_WriteShortRAMAddr(PANIDH, MacPANID >> 8);

    // Write NodeShortAddress into SADRL and SADRH registers
    MRF24J40_WriteShortRAMAddr(SADRL, MacShortAddress);
    MRF24J40_WriteShortRAMAddr(SADRH, MacShortAddress >> 8);

    // Write MacExtendedAddress into EADRn registers
    MRF24J40_WriteShortRAMAddr(EADR0, MacExtendedAddress[0]);
    MRF24J40_WriteShortRAMAddr(EADR1, MacExtendedAddress[1]);
    MRF24J40_WriteShortRAMAddr(EADR2, MacExtendedAddress[2]);
    MRF24J40_WriteShortRAMAddr(EADR3, MacExtendedAddress[3]);
    MRF24J40_WriteShortRAMAddr(EADR4, MacExtendedAddress[4]);
    MRF24J40_WriteShortRAMAddr(EADR5, MacExtendedAddress[5]);
    MRF24J40_WriteShortRAMAddr(EADR6, MacExtendedAddress[6]);
    MRF24J40_WriteShortRAMAddr(EADR7, MacExtendedAddress[7]);

    // Set the PANCOORD (RXMCR 0x00<3>) bit = 1 to configure as PAN co-ordinator.
    T8 = MRF24J40_ReadShortRAMAddr(RXMCR);
    T8 |= 0b00001000;
    MRF24J40_WriteShortRAMAddr(RXMCR, T8);

    // Clear the SLOTTED (TXMCR 0x11<5>) bit = 0 to use Unslotted CSMA-CA mode.
    T8 = MRF24J40_ReadShortRAMAddr(TXMCR);
    T8 &= ~0b00100000;
    MRF24J40_WriteShortRAMAddr(TXMCR, T8);

    // For PAN coordinator nodes, set BO and SO to 0x0f
    // Configure BO (ORDER 0x10<7:4>) value = 0xF. Configure SO (ORDER 0x10<3:0>) value = 0xF.
    MRF24J40_WriteShortRAMAddr(ORDER, 0xff);
}

/* this function transmits the packet passed in PktPtr. It blocks until the packet is transmitted */
void MRF24J40_Tx(volatile unsigned char *PktPtr, unsigned char Length) {
    unsigned char T8;

    // MHR length field
    MRF24J40_WriteLongRAMAddr(TX_NORMAL_FIFO_BASE_ADDR, MacHeaderLength);

    // Frame Length Field
    MRF24J40_WriteLongRAMAddr(TX_NORMAL_FIFO_BASE_ADDR + 1, MacFrameLength);

    // Copy MHR and MSDU into Normal TXFIFO
    for (T8 = 0; T8 < Length; T8++)
        MRF24J40_WriteLongRAMAddr(TX_NORMAL_FIFO_BASE_ADDR + 2 + T8, *PktPtr++);

    TxCompleted = 0;
    MRF24J40_ReadShortRAMAddr(INTSTAT); // Clear Interrupt flags
    REDLED = 1;

    // Start the transmission of the packet by setting the TXNTRIG (TXNCON 0x1B<0>) bit = 1. Set TXNACKREQ if ACK is needed
    if (*PktPtr & 0x20)
        T8 = 0x05;
    else
        T8 = 0x01;
    MRF24J40_WriteShortRAMAddr(TXNCON, T8);

    // Poll for end of transmission procedure, with timeout
    T8 = 0;
    do {
        __delay_ms(1);
    } while ((TxCompleted == 0) && (++T8 < 50));
    REDLED = 0;
}

// Side effect of this function is that the RF Module comes out of Sleep Mode, if it were in Sleep Mode

void MRF24J40_SetChannel(unsigned char Channel) {
    unsigned char T8;

    // Change RF Channel
    T8 = (Channel - 11) << 4;
    T8 |= 0x03;
    MRF24J40_WriteLongRAMAddr(RFCON0, T8);
    // RF Reset for Channel change to take effect
    MRF24J40_WriteShortRAMAddr(RFCTL, 0x04);
    MRF24J40_WriteShortRAMAddr(RFCTL, 0x00);
    // After RF Reset, wait at least 192 us
    __delay_us(200);
}

void MRF24J40_SetCCAMode(unsigned char CCAMode) {
    // Update CCA Mode - not implemented now
}

void MRF24J40_SetTxPowerLevel(unsigned char TxPowerLevel) {
    // Update Tx Power Level - not implemented now
}

// Should be called by application layer to initialize the PHY layer. The PHY
// layer is initialized and the PhyStatus is set to PHY_TRX_OFF
void PHYInit(void) {
    PhyCurrentChannel = PHY_PIB_CURRENT_CHANNEL;
    PhyChannelsSupported = PHY_PIB_SUPPORTED_CHANNELS;
    PhyTransmitPower = PHY_PIB_TX_POWER;
    PhyCCAMode = PHY_PIB_CCA_MODE;
    MRF24J40_HardwareReset();
    MRF24J40_Init();
    PhyState = PHY_TRX_OFF;
    PhyPktReceived = 0;
}

// Called for checking if channel is clear for transmission
unsigned char PLME_CCA_REQUEST() {
    if (PhyState == PHY_TX_ON)
        return PHY_TX_ON;
    if (PhyState == PHY_TRX_OFF)
        return PHY_TRX_OFF;
    if (PhyState == PHY_RX_ON) {
        if (MRF24J40_CCA() > PHY_CCA_ED_THRESHOLD)
            return PHY_BUSY;
        else
            return PHY_IDLE;
    }
    return PHY_INVALID_PARAMETER;
}

// Called for Energy Detection in selected channel
unsigned char PLME_ED_REQUEST(unsigned char *EnergyLevel) {
    if (PhyState == PHY_TX_ON)
        return PHY_TX_ON;
    if (PhyState == PHY_TRX_OFF)
        return PHY_TRX_OFF;
    if (PhyState == PHY_RX_ON) {
        *EnergyLevel = MRF24J40_CCA();
        return PHY_SUCCESS;
    }
    return PHY_INVALID_PARAMETER;
}

// Called for Reading a PHY PIB parameter
unsigned char PLME_GET_REQUEST(unsigned char Attr, void *AttrPtr) {
    if (Attr == PHY_CURRENT_CHANNEL) {
        *(unsigned char *) AttrPtr = PhyCurrentChannel;
        return PHY_SUCCESS;
    } else if (Attr == PHY_CHANNELS_SUPPORTED) {
        *(unsigned long *) AttrPtr = PhyChannelsSupported;
        return PHY_SUCCESS;
    } else if (Attr == PHY_TRANSMIT_POWER) {
        *(unsigned char *) AttrPtr = PhyTransmitPower;
        return PHY_SUCCESS;
    } else if (Attr == PHY_CCA_MODE) {
        *(unsigned char *) AttrPtr = PhyCCAMode;
        return PHY_SUCCESS;
    } else
        return PHY_UNSUPPORTED_ATTRIBUTE;
}

// In this primitive, the CCA mode, current channel and TX power levels are updated to the PIB values
// before the transceiver state change is made
unsigned char PLME_SET_TRX_STATE_REQUEST(unsigned char State) {
    // if PHY is already in requested state
    if (State == PhyState)
        return State;

    // if request is to go into RX state
    if (State == PHY_RX_ON) {
        if (PhyState == PHY_TRX_OFF) {
            PhyState = PHY_RX_ON;
            return PHY_SUCCESS;
        }
        // If transmission is in progress, return PHY_BUSY_TX.  
        if (PhyState == PHY_TX_ON) {
            return PHY_BUSY_TX;
        }
    }
        // if request is to go into TX State
    else if (State == PHY_TX_ON) {
        if (PhyState == PHY_TX_ON)
            return PHY_BUSY_TX;
        // If not already transmitting, return SUCCESS status, but there is no need to
        // put the MRF24J40 into "TX" mode, since it will start transmission once the TX is triggered.
        return PHY_SUCCESS;
    }
        // if request is to go into OFF state
    else if (State == PHY_TRX_OFF) {
        if (PhyState == PHY_RX_ON) {
            PhyState = PHY_TRX_OFF;
            return PHY_SUCCESS;
        }
    }
        // if request is to FORCE go into OFF state
    else if (State == PHY_FORCE_TRX_OFF) {
        PhyState = PHY_TRX_OFF;
        return PHY_SUCCESS;
    }
    return PHY_INVALID_PARAMETER;
}

// The REQUEST primitive is issued by calling PLME_SET_REQUEST. 
unsigned char PLME_SET_REQUEST(unsigned char Attr, void *AttrPtr) {
    unsigned char T8;
    unsigned long T32;

    if (Attr == PHY_CURRENT_CHANNEL) {
        T8 = *(unsigned char *) AttrPtr;
        if (PhyChannelsSupported & (0x1 << T8)) {
            PhyCurrentChannel = T8;
            MRF24J40_SetChannel(T8);
            return PHY_SUCCESS;
        } else
            return PHY_INVALID_PARAMETER;
    }
    else if (Attr == PHY_CHANNELS_SUPPORTED) {
        T32 = *(unsigned long *) AttrPtr;
        if ((T32 & ~(MRF24J40_GetSupportedChannels())) == 0) {
            PhyChannelsSupported = *((unsigned long *) AttrPtr);
            return PHY_SUCCESS;
        } else
            return PHY_INVALID_PARAMETER;
    }
    else if (Attr == PHY_TRANSMIT_POWER) {
        T8 = *(unsigned char *) AttrPtr;
        if (T8 <= 0xbf) {
            PhyTransmitPower = T8;
            MRF24J40_SetTxPowerLevel(T8);
            return PHY_SUCCESS;
        } else
            return PHY_INVALID_PARAMETER;
    }
    else if (Attr == PHY_CCA_MODE) {
        T8 = *(unsigned char *) AttrPtr;
        if ((T8 == 1) || (T8 == 2) || (T8 == 3)) {
            PhyCCAMode = T8;
            MRF24J40_SetCCAMode(T8);
            return PHY_SUCCESS;
        } else
            return PHY_INVALID_PARAMETER;
    } else
        return PHY_UNSUPPORTED_ATTRIBUTE;
}

// MAC LAYER
// Non beacon networks only.  Intra PAN only. No Indirect transmissions. No security.
/*
 List of functions of MAC layer as defined in the IEEE standard are :
 Generating network beacons if the device is a coordinator.
 Synchronizing to the beacons.
 Supporting PAN association and disassociation.
 Supporting device security.
 Employing the CSMA-CA mechanism for channel access.
 Handling and maintaining the GTS mechanism.
 Providing a reliable link between two peer MAC entities.
 */

/*
 The functions which are implemented here are :
 Supporting PAN association and disassociation
 Employing CSMA-CA for channel access
 Providing a reliable link between two peer MAC entities
 */

void MACInit(void) {
    EUID48Init();
    ReadEUID48From11AA02E48();
    FormMacExtendedAddressFromEUID48();
    MacCoordExtendedAddress[7] = MAC_COORD_EUID_BYTE7; // IEEE 64 bit address MSByte
    MacCoordExtendedAddress[6] = MAC_COORD_EUID_BYTE6; // IEEE 64 bit address
    MacCoordExtendedAddress[5] = MAC_COORD_EUID_BYTE5; // IEEE 64 bit address
    MacCoordExtendedAddress[4] = MAC_COORD_EUID_BYTE4; // IEEE 64 bit address
    MacCoordExtendedAddress[3] = MAC_COORD_EUID_BYTE3; // IEEE 64 bit address
    MacCoordExtendedAddress[2] = MAC_COORD_EUID_BYTE2; // IEEE 64 bit address
    MacCoordExtendedAddress[1] = MAC_COORD_EUID_BYTE1; // IEEE 64 bit address
    MacCoordExtendedAddress[0] = MAC_COORD_EUID_BYTE0; // IEEE 64 bit address
    MacCoordShortAddress = MAC_COORD_SHORT_ADDRESS_VALUE;
    MacDSN = rand(); // Sequence number to be added to the transmitted data/MAC command frame. Initialize with call to rand
    MacPANID = MAC_PANID_VALUE;
    MacShortAddress = MAC_COORD_SHORT_ADDRESS_VALUE;
    MRF24J40_Configure_NonBeaconNetwork();
}

// MCPS_DATA REQUEST and CONFIRM

unsigned char MCPS_DATA_REQUEST(unsigned char SrcAddrMode, unsigned int SrcPANID, unsigned char *SrcAddr,
        unsigned char DstAddrMode, unsigned int DstPANID, unsigned char *DstAddr,
        unsigned char MsduLength, unsigned char* MsduPtr, unsigned char MsduHandle, unsigned char TxOptions) {
    unsigned char T8;
    volatile unsigned char *PsduPtr;

    // Parameter validation
    if ((SrcAddrMode != MAC_ADDR_MODE_SHORT) && (SrcAddrMode != MAC_ADDR_MODE_EXTENDED))
        return MAC_INVALID_PARAMETER;
    if (SrcPANID != MacPANID)
        return MAC_INVALID_PARAMETER;
    if ((DstAddrMode != MAC_ADDR_MODE_SHORT) && (DstAddrMode != MAC_ADDR_MODE_EXTENDED))
        return MAC_INVALID_PARAMETER;
    if (MsduLength > aMaxMACFrameSize)
        return MAC_FRAME_TOO_LONG;
    if (TxOptions & 0xfe) // Only Acknowledged Transmissions option is supported. GTS, Security, Indirect transmissions NOT supported.
        return MAC_INVALID_PARAMETER;

    // Calculate MHR Length
    T8 = 5; // 2 bytes FCS, 1 byte sequence number, 2 bytes Destination PANID
    SrcAddrMode == MAC_ADDR_MODE_SHORT ? (T8 += 2) : (T8 += 8); // if short address, 2 bytes more; else extended address means 8 bytes more
    DstAddrMode == MAC_ADDR_MODE_SHORT ? (T8 += 2) : (T8 += 8); // if short address, 2 bytes more; else extended address means 8 bytes more
    MacHeaderLength = T8; // MHR Length.
    // Calculate Frame Length (equal to MHR + MSDULength)
    MacFrameLength = MacHeaderLength + MsduLength; // Frame Length

    // Fill Psdu with MHR and MSDU
    PsduPtr = &Psdu[0];
    // Write FCS LSByte
    T8 = 0;
    T8 |= 0x40; // IntraPAN ONLY
    if (TxOptions & 0x1) // Set ACK REQUEST bit depending on TxOptions
        T8 |= 0x20;
    T8 |= MAC_FC_DATA_FRAME; // Frame Type field is DATA frame
    *PsduPtr++ = T8;
    // Write FCS MSByte
    T8 = 0;
    T8 |= SrcAddrMode << 6; // Source Address Mode field
    T8 |= DstAddrMode << 2; // Destination Address Mode field
    *PsduPtr++ = T8;
    // Sequence number field
    *PsduPtr++ = MacDSN++;
    // Destination PAN Identifier
    *PsduPtr++ = MacPANID; // LSByte of Destination PANID (Destination PANID is same as source PANID since all packets are intraPAN)
    *PsduPtr++ = MacPANID >> 8; // MSByte of Destination PANID
    // Destination address (stored LSByte first)
    if (DstAddrMode == MAC_ADDR_MODE_SHORT) {
        for (T8 = 0; T8 < 2; T8++)
            *PsduPtr++ = *DstAddr++;
    } else {
        for (T8 = 0; T8 < 8; T8++)
            *PsduPtr++ = *DstAddr++;
    }
    // Source Address (stored LSByte first)
    if (SrcAddrMode == MAC_ADDR_MODE_SHORT) {
        for (T8 = 0; T8 < 2; T8++)
            *PsduPtr++ = *SrcAddr++;
    } else {
        for (T8 = 0; T8 < 8; T8++)
            *PsduPtr++ = *SrcAddr++;
    }
    // MSDU
    for (T8 = 0; T8 < MsduLength; T8++)
        *PsduPtr++ = *MsduPtr++;

    // Transmision.
    // Check TXNSTAT bit for transmission status
    MRF24J40_Tx(&Psdu[0], MacFrameLength);
    T8 = MRF24J40_ReadShortRAMAddr(TXSTAT);
    if (T8 & 0x20)
        return MAC_CHANNEL_ACCESS_FAILURE;
    if (T8 & 0x01)
        return MAC_NO_ACK;
    else
        return MAC_SUCCESS;
}

// MLME_ASSOCIATE REQUEST, CONFIRM. INDICATION and RESPONSE also for FFD.
// MLME-SAP association primitives define how a device becomes associated with a PAN.
// If this primitive is successful, it returns MAC_ASSOCIATE_REQUEST_SUCCESS and writes the
// allocated short address into the integer pointed to by ResultPtr
// MLME_ASSOCIATE_RESPONSE. Sends Response command frame for MLME_ASSOCIATE_REQUEST.  This is used only in PAN Coordinators

unsigned char MLME_ASSOCIATE_RESPONSE() {
    unsigned char T8, i, SrcAddrIndex;
    volatile unsigned char *PsduPtr;
    unsigned int NodeShortAddress;

    // If maximum number of nodes per network have already been added, respond with failure code
    if (NumNodes == MAX_NODES_PER_PAN)
        T8 = MAC_ASSOCIATE_REQUEST_PAN_FULL;

        // otherwise add the node into the ACL List.
        // NOTE : Not Implemented : check if a node with same extended address is already in the list
    else {
        (((Psdu[1] & 0x0c) >> 2) == MAC_ADDR_MODE_SHORT) ? (SrcAddrIndex = 9) : (SrcAddrIndex = 15);
        for (i = 0; i < 8; i++) {
            MacACLEntryDescriptorSet[NumNodes].ACLExtendedAddress[i] = Psdu[SrcAddrIndex + i];
        }
        NodeShortAddress = NumNodes + 1;
        MacACLEntryDescriptorSet[i].ACLShortAddress = NodeShortAddress;
        MacACLEntryDescriptorSet[i].ACLPANID = MacPANID;
        NumNodes += 1;
        T8 = MAC_ASSOCIATE_REQUEST_SUCCESS;
    }

    // Calculate MHR Length
    T8 = 23; // 2 bytes FCS, 1 byte sequence number, 2 bytes Destination PANID, 2 bytes Source PANID, 8 bytes each of Source and Dest Extended addresses
    MacHeaderLength = T8; // MHR Length.
    // Calculate Frame Length (equal to MHR + MSDULength)
    MacFrameLength = MacHeaderLength + 4; // Frame Length

    // Fill Psdu with MHR and MSDU
    PsduPtr = &Psdu[0];
    // Write FCS LSByte
    T8 = 0;
    T8 |= 0x40; // IntraPAN ONLY
    T8 |= 0x20; // Set ACK REQUEST bit
    T8 |= MAC_FC_COMMAND_FRAME; // Frame Type field is COMMAND frame
    *PsduPtr++ = T8;
    // Write FCS MSByte
    T8 = 0;
    T8 |= MAC_ADDR_MODE_EXTENDED << 6; // Source Address Mode field is Extended address
    T8 |= MAC_ADDR_MODE_EXTENDED << 2; // Destination Address Mode field
    *PsduPtr++ = T8;
    // Sequence number field
    *PsduPtr++ = MacDSN++;
    // Destination PAN Identifier
    *PsduPtr++ = MacPANID; // LSByte of PANID (Destination PANID is same as source PANID since all packets are intraPAN)
    *PsduPtr++ = MacPANID >> 8; // MSByte of PANID
    // Destination Extended Address (LSbyte first)
    for (i = 0; i < 8; i++)
        *PsduPtr++ = MacACLEntryDescriptorSet[NumNodes - 1].ACLExtendedAddress[i];
    // Source PANID 
    *PsduPtr++ = MacPANID;
    *PsduPtr++ = MacPANID;
    // Source Extended Address (stored LSByte first)
    for (T8 = 0; T8 < 8; T8++)
        *PsduPtr++ = MacExtendedAddress[T8];
    // MSDU
    *PsduPtr++ = MAC_ASSOCIATION_RESPONSE;
    *PsduPtr++ = NodeShortAddress;
    *PsduPtr++ = NodeShortAddress >> 8;
    *PsduPtr++ = T8;

    // Transmision.
    MRF24J40_Tx(&Psdu[0], MacFrameLength);
    // Check TXNSTAT bit for transmission status
    T8 = MRF24J40_ReadShortRAMAddr(TXSTAT);
    if ((T8 & 0x01) == 0) {
        if (T8 & 0x20)
            return MAC_CHANNEL_ACCESS_FAILURE;
        if ((T8 & 0xc0) == aMaxFrameRetries)
            return MAC_NO_ACK;
    }
    return MAC_SUCCESS;
}

// MLME_GET REQUEST. The MLME-SAP get primitives define how to read values from the MAC PIB.
// NOTE : Implemented only for PIB attributes held in MRF24J40 registers.  Rest of the PIB attributes can be accessed as global variables.

unsigned char MLME_GET_REQUEST(unsigned int PIBAttribute, unsigned char *ValuePtr) {
    if (PIBAttribute == MAC_ACK_WAIT_DURATION) {
        // Return the value in the MAWD field of ACKTMOUT register (Register address 0x12, bits <6:0>)
        *ValuePtr = MRF24J40_ReadShortRAMAddr(ACKTMOUT) & 0x7f; // Max ACK wait duration in symbol periods
        return MAC_SUCCESS;
    } else if (PIBAttribute == MAC_MAX_CSMA_BACKOFFS) {
        // Return the value in CSMABF field of TXMCR register (Register address 0x11, bits <2:0>)
        *ValuePtr = MRF24J40_ReadShortRAMAddr(TXMCR) & 0x07; // Max number of CSMA backoffs
        return MAC_SUCCESS;
    } else if (PIBAttribute == MAC_MIN_BE) {
        // Return value in MACMINBE field of Register TXMCR (Register address 0x11, bits <4:3>)
        *ValuePtr = (MRF24J40_ReadShortRAMAddr(TXMCR) >> 3) & 0x03; // Min value of Backoff exponent used in CSMA-CA algorithm
        return MAC_SUCCESS;
    } else
        return MAC_UNSUPPORTED_ATTRIBUTE;
}
// MLME_SET REQUEST.  MLME-SAP set primitives define how MAC PIB attributes may be written.
// NOTE : Implemented only for PIB attributes held in MRF24J40 registers.  Rest of the PIB attributes can be accessed as global variables.

unsigned char MLME_SET_REQUEST(unsigned int PIBAttribute, unsigned char *ValuePtr) {
    unsigned char T8;

    if (PIBAttribute == MAC_ACK_WAIT_DURATION) {
        // Write the value into the MAWD field of ACKTMOUT register (Register address 0x12, bits <6:0>)
        T8 = MRF24J40_ReadShortRAMAddr(ACKTMOUT) & ~0x7f;
        T8 |= (*ValuePtr & 0x7f);
        MRF24J40_WriteShortRAMAddr(ACKTMOUT, T8); // Max ACK wait duration in symbol periods
        return MAC_SUCCESS;
    } else if (PIBAttribute == MAC_MAX_CSMA_BACKOFFS) {
        // Write the value into CSMABF field of TXMCR register (Register address 0x11, bits <2:0>)
        T8 = MRF24J40_ReadShortRAMAddr(TXMCR) & ~0x07;
        T8 |= (*ValuePtr & 0x07);
        MRF24J40_WriteShortRAMAddr(TXMCR, T8); // Max number of CSMA backoffs
        return MAC_SUCCESS;
    } else if (PIBAttribute == MAC_MIN_BE) {
        // Write value into MACMINBE field of Register TXMCR (Register address 0x11, bits <4:3>)
        T8 = MRF24J40_ReadShortRAMAddr(TXMCR) & ~0x18;
        T8 |= ((*ValuePtr & 0x03) << 3);
        MRF24J40_WriteShortRAMAddr(TXMCR, T8); // Max number of CSMA backoffs
        return MAC_SUCCESS;
    } else
        return MAC_UNSUPPORTED_ATTRIBUTE;
}

// MLME_RESET REQUEST. MLME-SAP reset primitives specify how to reset the MAC sublayer to its default values.

unsigned char MLME_RESET_REQUEST(unsigned char SetDefaultPIB) {
    unsigned char val1, val2, val3;

    /* If SetDefaultPIB is TRUE, the MAC sublayer is reset and all MAC PIB attributes are set to their default values. If FALSE, the
       MAC sublayer is reset but all MAC PIB attributes retain their values prior to the generation of the MLME-RESET.request primitive.
     */
    // if SetDefaultPIB is true, do a hardware RESET by pulsing the RESET pin of the MRF24J40 Low
    if (SetDefaultPIB == 1) {
        MRF24J40_HardwareReset();
    } else
        // Save the PIB attributes in the MRF24J40 registers. Then reinitialize the MRF24J40. Finally write back the PIB attributes.
    {
        MLME_GET_REQUEST(MAC_ACK_WAIT_DURATION, &val1);
        MLME_GET_REQUEST(MAC_MAX_CSMA_BACKOFFS, &val2);
        MLME_GET_REQUEST(MAC_MIN_BE, &val3);
        MRF24J40_Init();
        MLME_SET_REQUEST(MAC_ACK_WAIT_DURATION, &val1);
        MLME_SET_REQUEST(MAC_MAX_CSMA_BACKOFFS, &val2);
        MLME_SET_REQUEST(MAC_MIN_BE, &val3);
    }
    return MAC_SUCCESS;
}
// MLME_RX_ENABLE REQUEST, CONFIRM. MLME-SAP receiver state primitives define how a device can enable or disable the receiver at a given time.

unsigned char MLME_RX_ENABLE(unsigned char DeferPermit, unsigned int RxOnTime, unsigned int RxOnDuration) {
    // Only the RxOnDuration parameter is considered in non-beacon enabled networks.
    // RxOnDuration specifies the number of symbol periods for which the receiver should be enabled.
    if (RxOnDuration == 0)
        PLME_SET_TRX_STATE_REQUEST(PHY_TRX_OFF);
    else {
        SetTimer1(RxOnDuration);
        PhyPktReceived = 0;
        PLME_SET_TRX_STATE_REQUEST(PHY_RX_ON);
        while (Timer1IntOccurred == 0);
        PLME_SET_TRX_STATE_REQUEST(PHY_TRX_OFF);
    }
    return MAC_SUCCESS;
}

// APPLICATION
// This function acquires the data from the sensors.

void AcquireData(void) {
    // Take temperature sensor readings (this will always cause BatteryVoltage variable to be updated)
    TEMPSENSOR_PWRCTRL_DIR = 0;
    TEMPSENSOR_PWRCTRL = 1;
    __delay_ms(1); // wait for MCP9700A to stabilize
    Temperature = ConvertADCValToTemperature(ADCConvert(TEMPERATURE_SENSOR_ADC_CHANNEL));
    TEMPSENSOR_PWRCTRL = 0;
}

// This function reads sensors and forms the Data packet which is sent periodically over the U1 TX link. This includes BatteryVoltage and Temperature.

void FormDataPkt(void) {
    unsigned char i;

    i = 0;
    Msdu[i++] = NODE_TYPE;
    Msdu[i++] = BatteryVoltage;
    Msdu[i++] = BatteryVoltage >> 8;
    Msdu[i++] = Temperature;
    MsduLength = i;
}


//functions for decrypting AES

const unsigned char sbox[16*16] = {	//declare static
   0x63,0x7C,0x77,0x7B,0xF2,0x6B,0x6F,0xC5,0x30,0x01,0x67,0x2B,0xFE,0xD7,0xAB,0x76,
   0xCA,0x82,0xC9,0x7D,0xFA,0x59,0x47,0xF0,0xAD,0xD4,0xA2,0xAF,0x9C,0xA4,0x72,0xC0,
   0xB7,0xFD,0x93,0x26,0x36,0x3F,0xF7,0xCC,0x34,0xA5,0xE5,0xF1,0x71,0xD8,0x31,0x15,
   0x04,0xC7,0x23,0xC3,0x18,0x96,0x05,0x9A,0x07,0x12,0x80,0xE2,0xEB,0x27,0xB2,0x75,
   0x09,0x83,0x2C,0x1A,0x1B,0x6E,0x5A,0xA0,0x52,0x3B,0xD6,0xB3,0x29,0xE3,0x2F,0x84,
   0x53,0xD1,0x00,0xED,0x20,0xFC,0xB1,0x5B,0x6A,0xCB,0xBE,0x39,0x4A,0x4C,0x58,0xCF,
   0xD0,0xEF,0xAA,0xFB,0x43,0x4D,0x33,0x85,0x45,0xF9,0x02,0x7F,0x50,0x3C,0x9F,0xA8,
   0x51,0xA3,0x40,0x8F,0x92,0x9D,0x38,0xF5,0xBC,0xB6,0xDA,0x21,0x10,0xFF,0xF3,0xD2,
   0xCD,0x0C,0x13,0xEC,0x5F,0x97,0x44,0x17,0xC4,0xA7,0x7E,0x3D,0x64,0x5D,0x19,0x73,
   0x60,0x81,0x4F,0xDC,0x22,0x2A,0x90,0x88,0x46,0xEE,0xB8,0x14,0xDE,0x5E,0x0B,0xDB,
   0xE0,0x32,0x3A,0x0A,0x49,0x06,0x24,0x5C,0xC2,0xD3,0xAC,0x62,0x91,0x95,0xE4,0x79,
   0xE7,0xC8,0x37,0x6D,0x8D,0xD5,0x4E,0xA9,0x6C,0x56,0xF4,0xEA,0x65,0x7A,0xAE,0x08,
   0xBA,0x78,0x25,0x2E,0x1C,0xA6,0xB4,0xC6,0xE8,0xDD,0x74,0x1F,0x4B,0xBD,0x8B,0x8A,
   0x70,0x3E,0xB5,0x66,0x48,0x03,0xF6,0x0E,0x61,0x35,0x57,0xB9,0x86,0xC1,0x1D,0x9E,
   0xE1,0xF8,0x98,0x11,0x69,0xD9,0x8E,0x94,0x9B,0x1E,0x87,0xE9,0xCE,0x55,0x28,0xDF,
   0x8C,0xA1,0x89,0x0D,0xBF,0xE6,0x42,0x68,0x41,0x99,0x2D,0x0F,0xB0,0x54,0xBB,0x16
};


const unsigned char invsbox[16*16] = {	//declare static
   0x52,0x09,0x6A,0xD5,0x30,0x36,0xA5,0x38,0xBF,0x40,0xA3,0x9E,0x81,0xF3,0xD7,0xFB,
   0x7C,0xE3,0x39,0x82,0x9B,0x2F,0xFF,0x87,0x34,0x8E,0x43,0x44,0xC4,0xDE,0xE9,0xCB,
   0x54,0x7B,0x94,0x32,0xA6,0xC2,0x23,0x3D,0xEE,0x4C,0x95,0x0B,0x42,0xFA,0xC3,0x4E,
   0x08,0x2E,0xA1,0x66,0x28,0xD9,0x24,0xB2,0x76,0x5B,0xA2,0x49,0x6D,0x8B,0xD1,0x25,
   0x72,0xF8,0xF6,0x64,0x86,0x68,0x98,0x16,0xD4,0xA4,0x5C,0xCC,0x5D,0x65,0xB6,0x92,
   0x6C,0x70,0x48,0x50,0xFD,0xED,0xB9,0xDA,0x5E,0x15,0x46,0x57,0xA7,0x8D,0x9D,0x84,
   0x90,0xD8,0xAB,0x00,0x8C,0xBC,0xD3,0x0A,0xF7,0xE4,0x58,0x05,0xB8,0xB3,0x45,0x06,
   0xD0,0x2C,0x1E,0x8F,0xCA,0x3F,0x0F,0x02,0xC1,0xAF,0xBD,0x03,0x01,0x13,0x8A,0x6B,
   0x3A,0x91,0x11,0x41,0x4F,0x67,0xDC,0xEA,0x97,0xF2,0xCF,0xCE,0xF0,0xB4,0xE6,0x73,
   0x96,0xAC,0x74,0x22,0xE7,0xAD,0x35,0x85,0xE2,0xF9,0x37,0xE8,0x1C,0x75,0xDF,0x6E,
   0x47,0xF1,0x1A,0x71,0x1D,0x29,0xC5,0x89,0x6F,0xB7,0x62,0x0E,0xAA,0x18,0xBE,0x1B,
   0xFC,0x56,0x3E,0x4B,0xC6,0xD2,0x79,0x20,0x9A,0xDB,0xC0,0xFE,0x78,0xCD,0x5A,0xF4,
   0x1F,0xDD,0xA8,0x33,0x88,0x07,0xC7,0x31,0xB1,0x12,0x10,0x59,0x27,0x80,0xEC,0x5F,
   0x60,0x51,0x7F,0xA9,0x19,0xB5,0x4A,0x0D,0x2D,0xE5,0x7A,0x9F,0x93,0xC9,0x9C,0xEF,
   0xA0,0xE0,0x3B,0x4D,0xAE,0x2A,0xF5,0xB0,0xC8,0xEB,0xBB,0x3C,0x83,0x53,0x99,0x61,
   0x17,0x2B,0x04,0x7E,0xBA,0x77,0xD6,0x26,0xE1,0x69,0x14,0x63,0x55,0x21,0x0C,0x7D
};

unsigned char rcon[10] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36};

typedef unsigned char state_t[4][4];
state_t matrix;
state_t* state = &matrix;	//declare static


unsigned char RoundKey[176];	//declare static

//Encryption key
const unsigned char key[16] = {0x0f, 0x15, 0x71, 0xc9, 0x47, 0xd9, 0xe8, 0x59, 0x0c, 0xb7, 0xad, 0xd6, 0xaf, 0x7f, 0x67, 0x98};

//initializing variable random (input to encryption using cbc mode of operation) with some arbitrary initial vector
unsigned char random[16] = {0x6f, 0x72, 0x61, 0x63, 0x6c, 0x65, 0x2f, 0x61, 0x72, 0x63, 0x68, 0x69, 0x74, 0x65, 0x63, 0x74};

void keyExpansion(void) {	//declare static
	unsigned char i,j,k, temp[4];

	for (i=0; i<4; i++) {
		RoundKey[i*4 + 0] = key[i*4 +0];
		RoundKey[i*4 + 1] = key[i*4 +1];
		RoundKey[i*4 + 2] = key[i*4 +2];
		RoundKey[i*4 + 3] = key[i*4 +3];
	//instead of loop
	}

	for (; i<44; i++) {
		for (j=0; j<4; j++)
			temp[j] = RoundKey[(i-1)*4 + j];


		if (i%4==0) {

			//function rotWord()
			{
			k=temp[0];
			temp[0]=temp[1];
			temp[1]=temp[2];
			temp[2]=temp[3];
			temp[3]=k;
			//instead of loop
			}

			//function subWord()
			{
			temp[0] = sbox[temp[0]];
			temp[1] = sbox[temp[1]];
			temp[2] = sbox[temp[2]];
			temp[3] = sbox[temp[3]];
			//instead of loop
			}


			temp[0] = temp[0] ^ rcon[i/4 - 1];
		}


		{
		RoundKey[i*4 + 0] = RoundKey[(i-4)*4 + 0] ^ temp[0];	//printf("%u,%x\t", i*4+0, RoundKey[i*4 + 0]);
		RoundKey[i*4 + 1] = RoundKey[(i-4)*4 + 1] ^ temp[1];	//printf("%u,%x\t", i*4+1, RoundKey[i*4 + 1]);
		RoundKey[i*4 + 2] = RoundKey[(i-4)*4 + 2] ^ temp[2];	//printf("%u,%x\t", i*4+2, RoundKey[i*4 + 2]);
		RoundKey[i*4 + 3] = RoundKey[(i-4)*4 + 3] ^ temp[3];	//printf("%u,%x\t", i*4+3, RoundKey[i*4 + 3]);
		//instead of loop
		}
		//printf("\n");

	}
}



void AddRoundKey(unsigned char round) {	//declare static
	unsigned char i,j;
	for (i=0; i<4; i++)
		for (j=0; j<4; j++)
			(*state)[j][i] ^= RoundKey[round*16 + i*4 +j];
}



void InvSubBytes(void) {
	unsigned char i,j;
	for (i=0; i<4; i++)
	for (j=0; j<4; j++)
		(*state)[i][j] = invsbox[(*state)[i][j]];
}



void InvShiftRows(void) {
	unsigned char temp;

	temp=(*state)[1][0];
	(*state)[1][0] = (*state)[1][3];
	(*state)[1][3] = (*state)[1][2];
	(*state)[1][2] = (*state)[1][1];
	(*state)[1][1] = temp;

	temp = (*state)[2][0];
	(*state)[2][0] = (*state)[2][2];
	(*state)[2][2] = temp;
	temp = (*state)[2][1];
	(*state)[2][1] = (*state)[2][3];
	(*state)[2][3] = temp;

	temp = (*state)[3][0];
	(*state)[3][0] = (*state)[3][1];
	(*state)[3][1] = (*state)[3][2];
	(*state)[3][2] = (*state)[3][3];
	(*state)[3][3] = temp;


	//instead of loop
}


unsigned char xtime(unsigned char x)	//declare static
{
  return ((x<<1) ^ (((x>>7) & 1) * 0x1b));
}


// Multiply is used to multiply numbers in the field GF(2^8)
unsigned char Multiply(unsigned char x, unsigned char y)	//declare static
{
  return (((y & 1) * x) ^
       ((y>>1 & 1) * xtime(x)) ^
       ((y>>2 & 1) * xtime(xtime(x))) ^
       ((y>>3 & 1) * xtime(xtime(xtime(x)))) ^
       ((y>>4 & 1) * xtime(xtime(xtime(xtime(x))))));
  }


// MixColumns function mixes the columns of the state matrix.
// The method used to multiply may be difficult to understand for the inexperienced.
// Please use the references to gain more information.
void InvMixColumns(void)	//declare static
{
  int i;
  unsigned char a,b,c,d;
  for(i=0;i<4;++i)
  { 
    a = (*state)[0][i];
    b = (*state)[1][i];
    c = (*state)[2][i];
    d = (*state)[3][i];

    (*state)[0][i] = Multiply(a, 0x0e) ^ Multiply(b, 0x0b) ^ Multiply(c, 0x0d) ^ Multiply(d, 0x09);
    (*state)[1][i] = Multiply(a, 0x09) ^ Multiply(b, 0x0e) ^ Multiply(c, 0x0b) ^ Multiply(d, 0x0d);
    (*state)[2][i] = Multiply(a, 0x0d) ^ Multiply(b, 0x09) ^ Multiply(c, 0x0e) ^ Multiply(d, 0x0b);
    (*state)[3][i] = Multiply(a, 0x0b) ^ Multiply(b, 0x0d) ^ Multiply(c, 0x09) ^ Multiply(d, 0x0e);
  }
}




void decrypt(void) {
	unsigned char round;

	AddRoundKey(10);

	for (round=9; round>0; round--) {
		InvShiftRows();
		InvSubBytes();
		AddRoundKey(round);
		InvMixColumns();
	}

	InvShiftRows();
	InvSubBytes();
	AddRoundKey(0);
}




int main(void) {
    unsigned char i, j;

    // Power ON delay
    __delay_ms(100);

    // Initialization of NodeType - should be done first
    if (NODE_TYPE == BASESTATION) {
        NumNodes = 0;
        for (i = 0; i < MAX_NODES_PER_PAN; i++) {
            for (j = 0; j < 8; j++) {
                MacACLEntryDescriptorSet[i].ACLExtendedAddress[j] = 0;
            }
            MacACLEntryDescriptorSet[i].ACLShortAddress = 0;
              MacACLEntryDescriptorSet[i].ACLPANID = 0;
        }
    }

    // Hardware platform initialization
    BoardInit();

    // Power ON signature
    REDLED = 1;
    __delay_ms(100);
    REDLED = 0;
    __delay_ms(100);
    GREENLED = 1;
    __delay_ms(100);
    GREENLED = 0;
    __delay_ms(100);
    YELLOWLED = 1;
    __delay_ms(100);
    YELLOWLED = 0;
    __delay_ms(100);

    //RTCCInit(0x15,0x02,0x02,0x01,0x14,0x20,0x00); // 2015 Feb 2, Monday, 14:20:00 hours

    // Initialization of PHY layer
    PHYInit();

    // Initialization of MAC layer
    MACInit();

    // Events which are monitored are MRF24J40MA interrupt, and the ACCELEROMETER interrupt. Both these are sensed through CHANGE NOTIFICATION interrupts of the PIC.
    IFS1bits.CNIF = 0; // Clear Change Notification interrupt Flag
    IEC1bits.CNIE = 1; // enable Change Notification interrupts in general

    // welcome message
    i = 0;
    U1TxBuf[i++] = 'W';
    U1TxBuf[i++] = 'e';
    U1TxBuf[i++] = 'l';
    U1TxBuf[i++] = 'c';
    U1TxBuf[i++] = 'o';
    U1TxBuf[i++] = 'm';
    U1TxBuf[i++] = 'e';
    U1TxBuf[i++] = ' ';
    U1TxBuf[i++] = 't';
    U1TxBuf[i++] = 'o';
    U1TxBuf[i++] = ' ';
    U1TxBuf[i++] = 'P';
    U1TxBuf[i++] = 'A';
    U1TxBuf[i++] = 'p';
    U1TxBuf[i++] = ' ';
    BinToAscii(MAC_PANID_VALUE, (unsigned char *) &U1TxBuf[i], 4, 0);
    i += 4;
    U1TxBuf[i++] = '\n';
    U1TxBuf[i++] = '\r';
    U1TxBufLen = i;
    U1TxBufIndex = 0;
    U1Tx();
    while (U1TxCompleted == 0); // wait for transmission of welcome message
    

    while (1) {
        // Basestation waits for packets from nodes
        if (PhyPktReceived == 1) {
            PhyPktReceived = 0;
            GREENLED = 1;

            // Set RXDECINV = 1; disable receiving packets off air.
            MRF24J40_WriteShortRAMAddr(BBREG1, 0x04);
            // Read address 0x300 in RXFIFO to get frame length value.
            PsduLength = MRF24J40_ReadLongRAMAddr(RX_FIFO_BASE_ADDR);
            // Read RXFIFO - MHR, MSDU, FCS, LQI, RSSI
            for (j = 0; j < PsduLength + 2; j++)
                Psdu[j] = MRF24J40_ReadLongRAMAddr(RX_FIFO_BASE_ADDR + 1 + j);
            // Clear RXDECINV = 0; enable receiving packets.
            MRF24J40_WriteShortRAMAddr(BBREG1, 0x00);
            
            
            
            
            //my naive attempt to display the encrypted data (ie. before decryption algorithm)
            i = 0;
            BinToAscii(Psdu[11], &U1TxBuf[i], 2, 01);
            i += 2; // NodeType MSByte
            BinToAscii(Psdu[10], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // NodeType LSByte
            BinToAscii(Psdu[8], &U1TxBuf[i], 2, 01);
            i += 2; // NodeID MSByte
            BinToAscii(Psdu[7], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // NodeID LSByte
            BinToAscii(Psdu[2], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // Sequence Number
            BinToAscii(Psdu[9], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // Node Status
            BinToAscii(Psdu[12], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // Packet Type
            BinToAscii(Psdu[14], &U1TxBuf[i], 2, 01);
            i += 2; // Battery Voltage MSByte
            BinToAscii(Psdu[13], &U1TxBuf[i], 2, 1);
            i += 2;
            U1TxBuf[i++] = ' '; // Battery Voltage LSByte
            BinToAscii(Psdu[15], &U1TxBuf[i], 2, 0);
            i += 2;
            U1TxBuf[i++] = ' '; // Temperature
            BinToAscii(Psdu[17], &U1TxBuf[i], 2, 1);
            i += 2; // AccX LSByte
            BinToAscii(Psdu[16], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // AccX MSByte
            BinToAscii(Psdu[19], &U1TxBuf[i], 2, 01);
            i += 2; // AccY LSByte
            BinToAscii(Psdu[18], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // AccY MSByte
            BinToAscii(Psdu[21], &U1TxBuf[i], 2, 01);
            i += 2; // AccZ LSByte
            BinToAscii(Psdu[20], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // AccZ MSByte
            BinToAscii(Psdu[24], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // LQI
            BinToAscii(Psdu[25], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' ';// RSSI (see table 3-8 in MRF24J40 datasheet)
            BinToAscii(Psdu[22], &U1TxBuf[i], 2, 01);
            i += 2;
            BinToAscii(Psdu[23], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' ';
            U1TxBuf[i++] = '\n';
            U1TxBuf[i++] = '\r';
            U1TxBufIndex = 0;
            U1TxBufLen = i;
            U1Tx();
            while (U1TxCompleted == 0);
            //end of the attempt
            
            
            
            
            //AES decryption algorithm     
            keyExpansion();
            
            unsigned char a, b, c;
            
            /*
            unsigned char random_for_round[16];
            
            for (a=0; a<16; a++)
              //  random_for_round[a] = random[a];
            
            //random for the next round
            //for (a=0, c=9; a<16; a++, c++)
              //  random[a] = Psdu[c];
            */
            
            
            c=9;
            
            for (b=0; b<4; b++) {
                for (a=0; a<4; a++)
                    (*state)[a][b] = Psdu[c++];
            }
                                
            
            decrypt();
            
            c=9;
            
            for (b=0; b<4; b++) {
                for (a=0; a<4; a++)
                        Psdu[c++] = (*state)[a][b];
            }
            
            
            /*
            for (a=0, c=9; a<16; a++, c++)
                Psdu[c] ^= random_for_round[a];
            */
            //end of AES algorithm
                    

            // Send out the received packet (selected information only) on the UART1
            i = 0;
            BinToAscii(Psdu[11], &U1TxBuf[i], 2, 01);
            i += 2; // NodeType MSByte
            BinToAscii(Psdu[10], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // NodeType LSByte
            BinToAscii(Psdu[8], &U1TxBuf[i], 2, 01);
            i += 2; // NodeID MSByte
            BinToAscii(Psdu[7], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // NodeID LSByte
            BinToAscii(Psdu[2], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // Sequence Number
            BinToAscii(Psdu[9], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // Node Status
            BinToAscii(Psdu[12], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // Packet Type
            BinToAscii(Psdu[14], &U1TxBuf[i], 2, 01);
            i += 2; // Battery Voltage MSByte
            BinToAscii(Psdu[13], &U1TxBuf[i], 2, 1);
            i += 2;
            U1TxBuf[i++] = ' '; // Battery Voltage LSByte
            BinToAscii(Psdu[15], &U1TxBuf[i], 2, 0);
            i += 2;
            U1TxBuf[i++] = ' '; // Temperature
            BinToAscii(Psdu[17], &U1TxBuf[i], 2, 1);
            i += 2; // AccX LSByte
            BinToAscii(Psdu[16], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // AccX MSByte
            BinToAscii(Psdu[19], &U1TxBuf[i], 2, 01);
            i += 2; // AccY LSByte
            BinToAscii(Psdu[18], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // AccY MSByte
            BinToAscii(Psdu[21], &U1TxBuf[i], 2, 01);
            i += 2; // AccZ LSByte
            BinToAscii(Psdu[20], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // AccZ MSByte
            BinToAscii(Psdu[24], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' '; // LQI
            BinToAscii(Psdu[25], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' ';// RSSI (see table 3-8 in MRF24J40 datasheet)
            BinToAscii(Psdu[22], &U1TxBuf[i], 2, 01);
            i += 2;
            BinToAscii(Psdu[23], &U1TxBuf[i], 2, 01);
            i += 2;
            U1TxBuf[i++] = ' ';
            U1TxBuf[i++] = '\n';
            
            //my naive attempt to organize the data
            
            U1TxBuf[i++] = '\n';
            U1TxBuf[i++] = '\n';
            U1TxBuf[i++] = '\n';
            //end of modification (the three lines above) to original code
            
            
            U1TxBuf[i++] = '\r';
            U1TxBufIndex = 0;
            U1TxBufLen = i;
            U1Tx();
            while (U1TxCompleted == 0); // wait for transmission of welcome message

            // If a DATA frame has been received
            if ((Psdu[0] & 0x07) == MAC_FC_DATA_FRAME) {
            }// END of DATA frame processing

                // If a COMMAND frame has been received
            else if ((Psdu[0] & 0x07) == MAC_FC_COMMAND_FRAME) {
                // if ASSOCIATE request is received from a node, add the node to ACL list & send ASSOCIATE RESPONSE with short address
                if (((Psdu[1] & 0x0c) >> 2) == MAC_ADDR_MODE_SHORT) {
                    if (Psdu[17] == MAC_ASSOCIATION_REQUEST)
                        MLME_ASSOCIATE_RESPONSE();
                } else if (((Psdu[1] & 0x0c) >> 2) == MAC_ADDR_MODE_EXTENDED) {
                    if (Psdu[23] == MAC_ASSOCIATION_REQUEST)
                        MLME_ASSOCIATE_RESPONSE();
                }
            } // END of COMMAND frame processing
            GREENLED = 0;
        }
    }
    return 0;
}

/* This function converts the binary value passed to it into an ASCII string which it stores at the array pointed to by Buf.
   If DecHex is 1, it converts into Decimal string, else Hexadecimal string.  It converts to NumChars significant digits.
 */
void BinToAscii(unsigned int Binary, unsigned char *Buf, unsigned char NumChars, unsigned char DecHex) {
    if (DecHex) {
        switch (NumChars) {
            case 4:
                *Buf++ = (Binary / 1000) + 0x30;
            case 3:
                *Buf++ = ((Binary % 1000) / 100) + 0x30;
            case 2:
                *Buf++ = ((Binary % 100) / 10) + 0x30;
            case 1:
                *Buf++ = (Binary % 10) + 0x30;
                break;
            default:
                break;
        }
    } else {
        switch (NumChars) {
            case 4:
                if (((Binary >> 12) & 0x0f) <= 9)
                    *Buf++ = ((Binary >> 12) & 0x0f) + 0x30;
                else
                    *Buf++ = ((Binary >> 12) & 0x0f) + 0x37;
            case 3:
                if (((Binary >> 8) & 0x0f) <= 9)
                    *Buf++ = ((Binary >> 8) & 0x0f) + 0x30;
                else
                    *Buf++ = ((Binary >> 8) & 0x0f) + 0x37;
            case 2:
                if (((Binary >> 4) & 0x0f) <= 9)
                    *Buf++ = ((Binary >> 4) & 0x0f) + 0x30;
                else
                    *Buf++ = ((Binary >> 4) & 0x0f) + 0x37;
            case 1:
                if (((Binary) & 0x0f) <= 9)
                    *Buf++ = ((Binary) & 0x0f) + 0x30;
                else
                    *Buf++ = ((Binary) & 0x0f) + 0x37;
                break;
            default:
                break;
        }
    }
} /* End of BinToAscii() */
