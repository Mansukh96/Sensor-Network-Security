// This file contains compilation constants for the IEEE 802.15.4 standard
#ifndef IEEE802154_H
#define	IEEE802154_H

////////////////////////// PHY LAYER  ///////////////////////////////////////
// PHY constants
#define aMaxPHYPacketSize 127
#define aTurnaroundTime 12      // In Symbol Period units

// PHY PIB attribute enumeration values
#define PHY_CURRENT_CHANNEL 0
#define PHY_CHANNELS_SUPPORTED 1
#define PHY_TRANSMIT_POWER 2
#define PHY_CCA_MODE 3

// PHY status related constants
#define PHY_BUSY 0
#define PHY_BUSY_RX 1
#define PHY_BUSY_TX 2
#define PHY_FORCE_TRX_OFF 3
#define PHY_IDLE 4
#define PHY_INVALID_PARAMETER 5
#define PHY_RX_ON 6
#define PHY_SUCCESS 7
#define PHY_TRX_OFF 8
#define PHY_TX_ON 9
#define PHY_UNSUPPORTED_ATTRIBUTE 10

////////////////////////// MAC LAYER ///////////////////////////////////////
// MAC Constants.   Symbol period = 4 bit periods = 16us for 250KHz operation
#define aMaxBE 5 // maximum value of Backoff Exponent in the CSMA-CA algorithm
#define aMaxFrameOverhead 25 // maximum number of bytes added by MAC layer to a frame without security
#define aMaxFrameResponseTime 1220 // max number of symbols to
#define aMaxFrameRetries 3
#define aMaxMACFrameSize (aMaxPHYPacketSize - aMaxFrameOverhead)
#define aUnitBackoffPeriod 20 // Number of symbols forming the basic time period used by the CSMA-CA algorithm
#define aBaseSlotDuration 60 // The number of symbols forming a superframe slot when the superframe order is equal to 0.
#define aNumSuperframeSlots 16 // The number of slots contained in any superframe.
#define aBaseSuperFrameDuration (aBaseSlotDuration * aNumSuperframeSlots) // The number of symbols forming a superframe when the superframe order is equal to 0.
#define aResponseWaitTime (32 * aBaseSuperFrameDuration) // Max number of symbol periods a device will wait for Response command after sending a Request command

// MAC PIB attribute enumeration values
#define MAC_ACK_WAIT_DURATION 0x40
#define MAC_COORD_EXTENDED_ADDRESS 0x4a
#define MAC_COORD_SHORT_ADDRESS 0x4b
#define MAC_DSN 0x4c
#define MAC_MAX_CSMA_BACKOFFS 0x4e
#define MAC_MIN_BE 0x4f
#define MAC_PANID 0x50
#define MAC_SHORT_ADDRESS 0x53
#define MAC_ACL_ENTRY_DESCRIPTOR_SET 0x70
#define MAC_ACL_ENTRY_DESCRIPTOR_SET_SIZE 0x71
#define MAC_SECURITY_MODE 0x76

// MAC status constants
#define MAC_SUCCESS 0 // MAC REQUEST successful
#define MAC_CHANNEL_ACCESS_FAILURE 0xe1 // A transmission could not take place due to activity on the channel, i.e., the CSMA-CA mechanism has failed.
#define MAC_FRAME_TOO_LONG 0xe5 // The frame resulting from secure processing has a length that is greater than aMACMaxFrameSize.
#define MAC_INVALID_PARAMETER 0xe8 // A parameter in the primitive is out of the valid range.
#define MAC_NO_ACK 0xe9 // No acknowledgment was received after aMaxFrameRetries.
#define MAC_NO_DATA 0xeb // No response data were available following a request.
#define MAC_NO_SHORT_ADDRESS 0xec // The operation failed because a short address was not allocated
#define MAC_UNSUPPORTED_ATTRIBUTE 0xf4 // A SET/GET request was issued with the identifier of a PIB attribute that is not supported.

// MAC Frame Control - Frame Type values
#define MAC_FC_DATA_FRAME 1
#define MAC_FC_ACK_FRAME 2
#define MAC_FC_COMMAND_FRAME 3

// MAC COMMAND identifiers
#define MAC_ASSOCIATION_REQUEST 0x01
#define MAC_ASSOCIATION_RESPONSE 0x02
#define MAC_DISASSOCIATION_NOTIFICATION 0x03
#define MAC_DATA_REQUEST 0x04
#define MAC_PANID_CONFLICT_NOTIFICATION 0x05
#define MAC_ORPHAN_NOTIFICATION 0x06
#define MAC_BEACON_REQUEST 0x07

// MAC typedefs
typedef struct
{
    unsigned char ACLExtendedAddress[8];
    unsigned int ACLShortAddress;
    unsigned int ACLPANID;
} ACLEntry;

// MAC REQUEST primitive status
#define MAC_ASSOCIATE_REQUEST_SUCCESS 0x00
#define MAC_ASSOCIATE_REQUEST_PAN_FULL 0x01

// MAC Addressing Modes
#define MAC_ADDR_MODE_SHORT 0x02
#define MAC_ADDR_MODE_EXTENDED 0x03

#endif	/* IEEE802154_H */

