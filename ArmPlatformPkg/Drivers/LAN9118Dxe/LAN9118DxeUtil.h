/** @file
*
*  Copyright (c) 2012-2013, ARM Limited. All rights reserved.
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#ifndef __LAN9118_DXE_UTIL_H__
#define __LAN9118_DXE_UTIL_H__

// Most common CRC32 Polynomial for little endian machines
#define CRC_POLYNOMIAL               0xEDB88320

/**
  This internal function reverses bits for 32bit data.

  @param  Value                 The data to be reversed.

  @return                       Data reversed.

**/
UINT32
ReverseBits (
  UINT32  Value
  );

// Create an Ethernet CRC
UINT32
GenEtherCrc32 (
  IN    EFI_MAC_ADDRESS *Mac,
  IN    UINT32 AddrLen
  );

/* --------------------- Structures -------------------- */

// Flags
#define TX_BUFFER_OFFSET                  1
#define TX_BUFFER_PADDING                 1

// Buffer structure: create static instance
typedef struct {
  UINT32 CommandA;
  UINT32 CommandB;
  UINT32 Addrs[3];      // dest and src addresses
  UINT16 Protocol;
  UINT8 Payload[1508];
  UINT8 Padding[6];     // preserves 4, 16 and 32 byte alignment
} TX_BUFFER;


// Initialise a buffer structure with args
EFI_STATUS
NewBuffer (
  IN  OUT TX_BUFFER *TxBuffer,
  IN      UINT32 HeaderSize,
  IN      EFI_MAC_ADDRESS *DestAddr,
  IN      EFI_MAC_ADDRESS *SrcAddr,
  IN      UINT16 Protocol,
  IN      VOID *Data,
  IN      UINT32 DataLength
  );

// Packet tag: 16-bit Id
typedef UINT16 PACKET_TAG;

//

/* ------------------ Debug Functions ------------------ */

// Debug funtion to print the binary representation of registers
CHAR16*
ToBinary (
  IN    UINT32 data
  );


// Flags for printing register values
#define PRINT_REGISTERS_MAC               BIT0
#define PRINT_REGISTERS_PHY               BIT1
#define PRINT_REGISTERS_ALL               (BIT0 | BIT1)
#define PRINT_REGISTERS_SEL_MAC           BIT12
#define PRINT_REGISTERS_SEL_PHY           BIT13


// DEBUG: Print all register values
UINT32
PrintRegisters (
  IN  UINT32 Flags,
  IN  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );


/* ------------------ MAC CSR Access ------------------- */

// Read from MAC indirect registers
UINT32
IndirectMACRead32 (
  UINT32 Index
  );


// Write to indirect registers
UINT32
IndirectMACWrite32 (
  UINT32 Index,
  UINT32 Value
  );


/* --------------- PHY Registers Access ---------------- */

// Read from MII register (PHY Access)
UINT32
IndirectPHYRead32(
  UINT32 Index
  );


// Write to the MII register (PHY Access)
UINT32
IndirectPHYWrite32(
  UINT32 Index,
  UINT32 Value
  );

/* ---------------- EEPROM Operations ------------------ */

// Read from EEPROM memory
UINT32
IndirectEEPROMRead32 (
  UINT32 Index
  );

// Write to EEPROM memory
UINT32
IndirectEEPROMWrite32 (
  UINT32 Index,
  UINT32 Value
  );

/* ---------------- General Operations ----------------- */

// Initialise the LAN9118
EFI_STATUS
Initialize (
  IN  UINT32 Flags,
  IN  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Flags for software reset
#define SOFT_RESET_CHECK_MAC_ADDR_LOAD                  BIT0
#define SOFT_RESET_CLEAR_INT                            BIT1
#define SOFT_RESET_SELF_TEST                            BIT2

// Perform software reset on the LAN9118
EFI_STATUS
SoftReset (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Flags for PHY reset
#define PHY_RESET_PMT                                   BIT0
#define PHY_RESET_BCR                                   BIT1
#define PHY_RESET_CHECK_LINK                            BIT2
#define PHY_SOFT_RESET_CLEAR_INT                        BIT3

// Perform PHY software reset
INT32
PhySoftReset (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Flags for Hardware configuration
#define HW_CONF_USE_LEDS                                BIT0

// Configure hardware for LAN9118
EFI_STATUS
ConfigureHardware (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Configure flow control
EFI_STATUS
ConfigureFlow (
  UINT32 Flags,
  UINT32 HighTrig,
  UINT32 LowTrig,
  UINT32 BPDuration,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Flags for auto negotiation
#define AUTO_NEGOTIATE_COLLISION_TEST         BIT0
#define AUTO_NEGOTIATE_ADVERTISE_ALL          BIT1

// Do auto-negotiation
EFI_STATUS
AutoNegotiate (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Check the Link Status and take appropriate action
EFI_STATUS
CheckLinkStatus (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Stop transmitter flags
#define STOP_TX_MAC                       BIT0
#define STOP_TX_CFG                       BIT1
#define STOP_TX_CLEAR                     BIT2

// Stop the transmitter
EFI_STATUS
StopTx (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Stop receiver flags
#define STOP_RX_CLEAR                     BIT0

// Stop the receiver
EFI_STATUS
StopRx (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Start transmitter flags
#define START_TX_MAC                      BIT0
#define START_TX_CFG                      BIT1
#define START_TX_CLEAR                    BIT2

// Start the transmitter
EFI_STATUS
StartTx (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Stop receiver flags
#define START_RX_CLEAR                     BIT0

// Start the receiver
EFI_STATUS
StartRx (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Check Tx Data available space
UINT32
TxDataFreeSpace (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Check Tx Status used space
UINT32
TxStatusUsedSpace (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Check Rx Data used space
UINT32
RxDataUsedSpace (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

// Check Rx Status used space
UINT32
RxStatusUsedSpace (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );


// Flags for FIFO allocation
#define ALLOC_USE_DEFAULT                 0x00000001
#define ALLOC_USE_FIFOS                   0x00000002
#define ALLOC_USE_DMA                     0x00000004

// FIFO min and max sizes
#define TX_FIFO_MIN_SIZE            0x00000600
#define TX_FIFO_MAX_SIZE            0x00003600
//#define RX_FIFO_MIN_SIZE
//#define RX_FIFO_MAX_SIZE

// Change the allocation of FIFOs
EFI_STATUS
ChangeFifoAllocation (
  IN      UINT32 Flags,
  IN  OUT UINTN  *TxDataSize    OPTIONAL,
  IN  OUT UINTN  *RxDataSize    OPTIONAL,
  IN  OUT UINT32 *TxStatusSize  OPTIONAL,
  IN  OUT UINT32 *RxStatusSize  OPTIONAL,
  IN  OUT EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

#endif // __LAN9118_DXE_UTIL_H__
