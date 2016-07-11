// This file contains compilation constants for the Accelerometer IC FXLS8471Q
#ifndef FXLS8471Q_H
#define	FXLS8471Q_H

// FXLS8471Q internal register addresses
#define FXLS8471Q_STATUS 0x00
#define FXLS8471Q_WHOAMI 0x0D
#define FXLS8471Q_XYZ_DATA_CFG 0x0E
#define FXLS8471Q_CTRL_REG1 0x2A
#define FXLS8471Q_WHOAMI_VAL 0x6A

// number of bytes to be read from FXLS8471Q
#define FXLS8471Q_READ_LEN 7// status plus 3 accelerometer channels

// The high and low bytes of the three accelerometer are placed into a structure of type SRAWDATA containing three signed short integers
typedef struct
{
    int x;
    int y;
    int z;
} SRAWDATA;

#endif	/* FXLS8471Q_H */


