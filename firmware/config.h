// This file contains configuration settings
#ifndef CONFIG_H
#define	CONFIG_H

// PHY configuration
#define PHY_CCA_ED_THRESHOLD 0x60 // this corresponds to -68dBm
#define PHY_PIB_CURRENT_CHANNEL 11 // first channel in 2.4GHz band at 2.405 GHz
#define PHY_PIB_SUPPORTED_CHANNELS 0x07fff800 // Bitmap indicating channels 11 to 26 are supported in 2.4GHz band
#define PHY_PIB_TX_POWER 0x00 // nominal transmit power level of MRF24J40 is 1mW (0dbm)
#define PHY_PIB_CCA_MODE 1  // CCA reports a busy medium upon detecting energy above the Energy Detection (ED) threshold

// MAC Configuration
#define MAC_PANID_VALUE 0x0001 // Also referred to as PANID
#define MAC_COORD_SHORT_ADDRESS_VALUE 0x0000 // Coordinator short address is always 0x0000
#define MAC_SHORT_ADDRESS_VALUE 0x0004 // Also referred to as NodeID
#define MAC_COORD_EUID_BYTE7 0x00
#define MAC_COORD_EUID_BYTE6 0x04
#define MAC_COORD_EUID_BYTE5 0xA3
#define MAC_COORD_EUID_BYTE4 0xFF
#define MAC_COORD_EUID_BYTE3 0xFE
#define MAC_COORD_EUID_BYTE2 0xDC
#define MAC_COORD_EUID_BYTE1 0x79       
#define MAC_COORD_EUID_BYTE0 0x36

// APPLICATION related constants
// Node types
#define BASESTATION 0
#define TEMP_SENSOR 1
#define ACC_SENSOR 2
#define SWITCH_SENSOR 3
#define TEMP_ACC_SENSOR 4
#define MEDIATOR 5

// Battery types
#define COIN_CELL 1
#define AA_X_2_CELLS 2
#define MAINS 3

// Network Sizing information
#define MAX_NODES_PER_PAN 20

// Packet types
#define PERIODIC_DATA_PKT 1
#define EVENT_DATA_PKT 2

// APP configuration
#define NODE_TYPE MEDIATOR // Also referred to as TypeID
#define DATA_PERIODICITY 120 // Periodicity in seconds for sending sensor data
#define BATTERY_TYPE AA_X_2_CELLS
////////////////////////////////////////////////////////////////////////////////
#endif	/* CONFIG_H */

