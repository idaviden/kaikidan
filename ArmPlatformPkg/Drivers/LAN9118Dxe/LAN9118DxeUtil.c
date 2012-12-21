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

#include "LAN9118Dxe.h"

/**
  This internal function reverses bits for 32bit data.

  @param  Value                 The data to be reversed.

  @return                       Data reversed.

**/
UINT32
ReverseBits (
  UINT32  Value
  )
{
  UINTN   Index;
  UINT32  NewValue;

  NewValue = 0;
  for (Index = 0; Index < 32; Index++) {
    if ((Value & (1 << Index)) != 0) {
      NewValue = NewValue | (1 << (31 - Index));
    }
  }

  return NewValue;
}

/*
**  Create Ethernet CRC
**
**  INFO USED:
**    1: http://en.wikipedia.org/wiki/Cyclic_redundancy_check
**
**    2: http://www.erg.abdn.ac.uk/~gorry/eg3567/dl-pages/crc.html
**
**    3: http://en.wikipedia.org/wiki/Computation_of_CRC
*/
UINT32
GenEtherCrc32 (
  IN    EFI_MAC_ADDRESS *Mac,
  IN    UINT32 AddrLen
  )
{
  INT32 Iter;
  UINT32 Remainder;
  UINT8 *Ptr;

  Iter = 0;
  Remainder = 0xFFFFFFFF;    // 0xFFFFFFFF is standard seed for Ethernet

  // Convert Mac Address to array of bytes
  Ptr = (UINT8*)Mac;

  // Generate the Crc bit-by-bit (LSB first)
  while (AddrLen--) {
    Remainder ^= *Ptr++;
    for (Iter = 0;Iter < 8;Iter++) {
      // Check if exponent is set
      if (Remainder & 1) {
        Remainder = (Remainder >> 1) ^ CRC_POLYNOMIAL;
      } else {
        Remainder = (Remainder >> 1) ^ 0;
      }
    }
  }

  // Reverse the bits before returning (to Big Endian)
  return ReverseBits (Remainder);
}



/*
**  ToBinary ()
*/
CHAR16*
ToBinary (
  IN    UINT32 data
  )
{
  UINT32 size;
  CHAR16* ret;
  UINT32 ldata;
  UINT32 bit;

  // Initialize variables
  size = 8 * sizeof(UINT32);
  ret = AllocateZeroPool ((size+1) * sizeof(CHAR16));
  ldata = data;

  // Extract each bit from the argument
  INT32 i,j = 0;for(i = size - 1;i >= 0;i--)
  {
    if ( ((j+1) % 9) == 0 )
    {
      ret[j] = (CHAR16) (45);
      j += 1;
    }

    bit = (ldata >> i) & 1;
    ret[j] = (CHAR16) (bit + 48); // Convert to ascii 0 or 1
    j++;
  }
  ret[j] = L'\0';
  return ret;
}



// Initialise a buffer structure with args
EFI_STATUS
NewBuffer (
  IN  TX_BUFFER* TxBuffer,
  IN  UINT32 HeaderSize,
  IN  EFI_MAC_ADDRESS *DstAddr,
  IN  EFI_MAC_ADDRESS *SrcAddr,
  IN  UINT16 Protocol,
  IN  VOID *Data,
  IN  UINT32 DataLength
  )
{
  UINT8 *DwordData;
  UINT32 Iter = 0;

  // Error checking
  if ((TxBuffer == NULL) || (DstAddr == NULL) || (SrcAddr == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  if (DataLength > 1508) {
    return EFI_UNSUPPORTED;
  }

  // Clear the buffer first
  ZeroMem(TxBuffer,sizeof(TX_BUFFER));

  // Assign the data first
  DwordData = (UINT8*)Data;

  if (HeaderSize) {
    // Copy the Destination address
    TxBuffer->Addrs[0] = (DstAddr->Addr[0]) |
                         (DstAddr->Addr[1] << 8) |
                         (DstAddr->Addr[2] << 16) |
                         (DstAddr->Addr[3] << 24);
    //TxBuffer->Addrs[0] = HTONL(TxBuffer->Addrs[0]);

    TxBuffer->Addrs[1] = (DstAddr->Addr[4]) |
                         (DstAddr->Addr[5] << 8) |

    // Copy the Source Address
                         (SrcAddr->Addr[0] << 16) |
                         (SrcAddr->Addr[1] << 24);
    //TxBuffer->Addrs[1] = HTONL(TxBuffer->Addrs[1]);

    TxBuffer->Addrs[2] = (SrcAddr->Addr[2]) |
                         (SrcAddr->Addr[3] << 8) |
                         (SrcAddr->Addr[4] << 16) |
                         (SrcAddr->Addr[5] << 24);
    //TxBuffer->Addrs[2] = HTONL(TxBuffer->Addrs[2]);

    // Copy the protocol
    TxBuffer->Protocol = HTONS(Protocol);

  } else {

    // Copy the Destination address
    TxBuffer->Addrs[0] = (DwordData[0]) |
                         (DwordData[1] << 8) |
                         (DwordData[2] << 16) |
                         (DwordData[3] << 24);
    //TxBuffer->Addrs[0] = HTONL(TxBuffer->Addrs[0]);

    TxBuffer->Addrs[1] = (DwordData[4]) |
                         (DwordData[5] << 8) |

    // Copy the Source Address
                         (DwordData[6] << 16) |
                         (DwordData[7] << 24);
    //TxBuffer->Addrs[1] = HTONL(TxBuffer->Addrs[1]);

    TxBuffer->Addrs[2] = (DwordData[8]) |
                         (DwordData[9] << 8) |
                         (DwordData[10] << 16) |
                         (DwordData[11] << 24);
    //TxBuffer->Addrs[2] = HTONL(TxBuffer->Addrs[2]);

    // Copy the protocol
    TxBuffer->Protocol = DwordData[12] | (DwordData[13] << 8);
  }

  for (Iter = 0;Iter < DataLength;Iter++) {
    TxBuffer->Payload[Iter] = DwordData[Iter + 14];
  }

  return EFI_SUCCESS;
}


// DEBUG: Print all register values
UINT32
PrintRegisters (
  IN  UINT32 Flags,
  IN  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 Count;
  UINT32 RegValue;

  // Select MAC register if defined (5 LSBs select register)
  if (Flags & PRINT_REGISTERS_SEL_MAC) {
    RegValue = IndirectMACRead32 (Flags & 0x1FF);
    DEBUG((EFI_D_ERROR, "MAC Register %02x:\thex: 0x%08x\tbin: %s\n",Flags & 0x1FF, RegValue, ToBinary (RegValue)));
    return 0;
  }

  // Select PHY register if defined (5 LSBs select register)
  if (Flags & PRINT_REGISTERS_SEL_PHY) {
    RegValue = IndirectPHYRead32 (Flags & 0x1FF);
    DEBUG((EFI_D_ERROR, "PHY Register %d:\thex: 0x%08x\tbin: %s\n",Flags & 0x1FF, RegValue, ToBinary (RegValue)));
    return 0;
  }

  // Loop through all MAC registers
  if (Flags & PRINT_REGISTERS_MAC) {
    for (Count = 1;Count <= 0xC;Count++) {
      RegValue = IndirectMACRead32 (Count);
      DEBUG((EFI_D_ERROR, "MAC Register %02x:\thex: 0x%08x\tbin: %s\n", Count, RegValue, ToBinary (RegValue)));
    }
  }

  // Print PHY registers
  if (Flags & PRINT_REGISTERS_PHY) {
    for (Count = 0;Count <= 6;Count ++) {
      RegValue = IndirectPHYRead32 (Count);
      DEBUG((EFI_D_ERROR, "PHY Register %d:\thex: 0x%08x\tbin: %s\n", Count, RegValue, ToBinary (RegValue)));
    }

    Count = 17;
    RegValue = IndirectPHYRead32 (Count);
    DEBUG((EFI_D_ERROR, "PHY Register %d:\thex: 0x%08x\tbin: %s\n", Count, RegValue, ToBinary (RegValue)));

    Count = 18;
    RegValue = IndirectPHYRead32 (Count);
    DEBUG((EFI_D_ERROR, "PHY Register %d:\thex: 0x%08x\tbin: %s\n", Count, RegValue, ToBinary (RegValue)));

    Count = 27;
    RegValue = IndirectPHYRead32 (Count);
    DEBUG((EFI_D_ERROR, "PHY Register %d:\thex: 0x%08x\tbin: %s\n", Count, RegValue, ToBinary (RegValue)));

    Count = 29;
    RegValue = IndirectPHYRead32 (Count);
    DEBUG((EFI_D_ERROR, "PHY Register %d:\thex: 0x%08x\tbin: %s\n", Count, RegValue, ToBinary (RegValue)));

    Count = 30;
    RegValue = IndirectPHYRead32 (Count);
    DEBUG((EFI_D_ERROR, "PHY Register %d:\thex: 0x%08x\tbin: %s\n", Count, RegValue, ToBinary (RegValue)));

    Count = 31;
    RegValue = IndirectPHYRead32 (Count);
    DEBUG((EFI_D_ERROR, "PHY Register %d:\thex: 0x%08x\tbin: %s\n", Count, RegValue, ToBinary (RegValue)));
  }

  return 0;
}


// Function to read from MAC indirect registers
UINT32
IndirectMACRead32 (
  UINT32 Index
  )
{
  UINT32 MacCSR;

  // Check index is in the range
  ASSERT(Index <= 12);

  // Wait until CSR busy bit is cleared
  while ((MmioRead32 (LAN9118_MAC_CSR_CMD) & MAC_CSR_BUSY) == MAC_CSR_BUSY);

  // Set CSR busy bit to ensure read will occur
  // Set the R/W bit to indicate we are reading
  // Set the index of CSR Address to access desired register
  MacCSR = MAC_CSR_BUSY | MAC_CSR_READ | MAC_CSR_ADDR(Index);

  // Write to the register
  MmioWrite32 (LAN9118_MAC_CSR_CMD, MacCSR);

  // Wait until CSR busy bit is cleared
  while ((MmioRead32 (LAN9118_MAC_CSR_CMD) & MAC_CSR_BUSY) == MAC_CSR_BUSY);

  // Now read from data register to get read value
  return MmioRead32 (LAN9118_MAC_CSR_DATA);
}

// Function to write to MAC indirect registers
UINT32
IndirectMACWrite32 (
  UINT32 Index,
  UINT32 Value
  )
{
  UINT32 ValueWritten;
  UINT32 MacCSR;

  // Check index is in the range
  ASSERT(Index <= 12);

  // Wait until CSR busy bit is cleared
  while ((MmioRead32 (LAN9118_MAC_CSR_CMD) & MAC_CSR_BUSY) == MAC_CSR_BUSY);

  // Set CSR busy bit to ensure read will occur
  // Set the R/W bit to indicate we are writing
  // Set the index of CSR Address to access desired register
  MacCSR = MAC_CSR_BUSY | MAC_CSR_WRITE | MAC_CSR_ADDR(Index);

  // Now write the value to the register before issuing the write command
  ValueWritten = MmioWrite32 (LAN9118_MAC_CSR_DATA, Value);

  // Write the config to the register
  MmioWrite32 (LAN9118_MAC_CSR_CMD, MacCSR);

  // Wait until CSR busy bit is cleared
  while ((MmioRead32 (LAN9118_MAC_CSR_CMD) & MAC_CSR_BUSY) == MAC_CSR_BUSY);

  return ValueWritten;
}

// Function to read from MII register (PHY Access)
UINT32
IndirectPHYRead32 (
  UINT32 Index
  )
{
  UINT32 ValueRead;
  UINT32 MiiAcc;

  // Check it is a valid index
  ASSERT(Index < 31);

  // Wait for busy bit to clear
  while ((IndirectMACRead32 (INDIRECT_MAC_INDEX_MII_ACC) & MII_ACC_MII_BUSY) == MII_ACC_MII_BUSY);

  // Clear the R/W bit to indicate we are reading
  // Set the index of the MII register
  // Set the PHY Address
  // Set the MII busy bit to allow read
  MiiAcc = MII_ACC_MII_READ | MII_ACC_MII_REG_INDEX(Index) | MII_ACC_PHY_VALUE | MII_ACC_MII_BUSY;

  // Now write this config to register
  IndirectMACWrite32 (INDIRECT_MAC_INDEX_MII_ACC, MiiAcc & 0xFFFF);

  // Wait for busy bit to clear
  while ((IndirectMACRead32 (INDIRECT_MAC_INDEX_MII_ACC) & MII_ACC_MII_BUSY) == MII_ACC_MII_BUSY);

  // Now read the value of the register
  ValueRead = (IndirectMACRead32 (INDIRECT_MAC_INDEX_MII_DATA) & 0xFFFF); // only lower 16 bits are valid for any PHY register

  return ValueRead;
}


// Function to write to the MII register (PHY Access)
UINT32
IndirectPHYWrite32 (
  UINT32 Index,
  UINT32 Value
  )
{
  UINT32 MiiAcc;
  UINT32 ValueWritten;

  // Check it is a valid index
  ASSERT(Index < 31);

  // Wait for busy bit to clear
  while ((IndirectMACRead32 (INDIRECT_MAC_INDEX_MII_ACC) & MII_ACC_MII_BUSY) == MII_ACC_MII_BUSY);

  // Clear the R/W bit to indicate we are reading
  // Set the index of the MII register
  // Set the PHY Address
  // Set the MII busy bit to allow read
  MiiAcc = MII_ACC_MII_WRITE | MII_ACC_MII_REG_INDEX(Index) | MII_ACC_PHY_VALUE | MII_ACC_MII_BUSY;

  // Write the desired value to the register first
  ValueWritten = IndirectMACWrite32 (INDIRECT_MAC_INDEX_MII_DATA, (Value & 0xFFFF));

  // Now write the config to register
  IndirectMACWrite32 (INDIRECT_MAC_INDEX_MII_ACC, MiiAcc & 0xFFFF);

  // Wait for operation to terminate
  while ((IndirectMACRead32 (INDIRECT_MAC_INDEX_MII_ACC) & MII_ACC_MII_BUSY) == MII_ACC_MII_BUSY);

  return ValueWritten;
}


/* ---------------- EEPROM Operations ------------------ */


// Function to read from EEPROM memory
UINT32
IndirectEEPROMRead32 (
  UINT32 Index
)
{
  // Eeprom command
  UINT32 EepromCmd = 0;//= MmioRead32 (LAN9118_E2P_CMD);

  // Set the busy bit to ensure read will occur
  EepromCmd |= ((UINT32)1 << 31);

  // Set the EEPROM command to read(0b000)
  EepromCmd &= ~(0x70000000); // Clear the command first
  EepromCmd |= (0 << 28);     // Not necessary, but here for clarity

  // Set the index to access desired EEPROM memory location
  EepromCmd |= (Index & 0xF);

  // Write to Eeprom command register
  MmioWrite32 (LAN9118_E2P_CMD, EepromCmd);
  gBS->Stall (LAN9118_STALL);

  // Wait until operation has completed
  while (MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_BUSY);

  // Check that operation didn't time out
  if (MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_TIMEOUT) {
    DEBUG((EFI_D_ERROR, "EEPROM Operation Timed out: Read command on index %x\n",Index));
    return 0;
  }

  // Wait until operation has completed
  while (MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_BUSY);

  // Finally read the value
  return MmioRead32 (LAN9118_E2P_DATA);
}

// Function to write to EEPROM memory
UINT32
IndirectEEPROMWrite32 (
  UINT32 Index,
  UINT32 Value
  )
{
  UINT32 ValueWritten;
  UINT32 EepromCmd;

  ValueWritten = 0;

  // Read the EEPROM Command register
  EepromCmd = MmioRead32 (LAN9118_E2P_CMD);

  // Set the busy bit to ensure read will occur
  EepromCmd |= ((UINT32)1 << 31);

  // Set the EEPROM command to write(0b011)
  EepromCmd &= ~(0x70000000); // Clear the command first
  EepromCmd |= (3 << 28);     // Write 011

  // Set the index to access desired EEPROM memory location
  EepromCmd |= (Index & 0xF);

  // Write the value to the data register first
  ValueWritten = MmioWrite32 (LAN9118_E2P_DATA, Value);

  // Write to Eeprom command register
  MmioWrite32 (LAN9118_E2P_CMD, EepromCmd);
  gBS->Stall (LAN9118_STALL);

  // Wait until operation has completed
  while (MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_BUSY);

  // Check that operation didn't time out
  if (MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_TIMEOUT) {
    DEBUG((EFI_D_ERROR, "EEPROM Operation Timed out: Write command at memloc 0x%x, with value 0x%x\n",Index, Value));
    return 0;
  }

  // Wait until operation has completed
  while (MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_BUSY);

  return ValueWritten;
}

/* ---------------- General Operations ----------------- */


// Initialise the LAN9118
/*
**  Initialize()
**
**  This operation can be carried out when the LAN9118 is in any power state
**
**
*/
EFI_STATUS
Initialize (
  IN  UINT32 Flags,
  IN  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  // Attempt to wake-up the device if it is in a lower power state
  if (((MmioRead32 (LAN9118_PMT_CTRL) & MPTCTRL_PM_MODE_MASK) >> 12) != 0) {
    DEBUG((DEBUG_NET, "Waking from reduced power state.\n"));
    MmioWrite32 (LAN9118_BYTE_TEST, 0xFFFFFFFF);
    gBS->Stall (LAN9118_STALL);
  }

  // Check that device is active
  while ((MmioRead32 (LAN9118_PMT_CTRL) & MPTCTRL_READY) == 0);

  // Check that EEPROM isn't active
  while (MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_BUSY);

  // Check that MAC Address loaded successfully
  if ((MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_MAC_ADDRESS_LOADED) == 0) {
    DEBUG((EFI_D_ERROR, "Warning: There was an error detecting EEPROM or loading the MAC Address:\n"));

    // See if there is a cached address first
    if (&Snp->Mode->PermanentAddress != NULL) {
      EFI_MAC_ADDRESS Mac = Snp->Mode->PermanentAddress;
      IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRL,
                          (Mac.Addr[0] & 0xFF) |
                          ((Mac.Addr[1] & 0xFF) << 8) |
                          ((Mac.Addr[2] & 0xFF) << 16) |
                          ((Mac.Addr[3] & 0xFF) << 24)
                        );

      IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRH,
                          (UINT32)(Mac.Addr[4] & 0xFF) |
                          ((Mac.Addr[5] & 0xFF) << 8)
                        );

      goto ON_EXIT;
    }

    if (&Snp->Mode->CurrentAddress != NULL) {
      EFI_MAC_ADDRESS Mac = Snp->Mode->CurrentAddress;
      IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRL,
                          (Mac.Addr[0] & 0xFF) |
                          ((Mac.Addr[1] & 0xFF) << 8) |
                          ((Mac.Addr[2] & 0xFF) << 16) |
                          ((Mac.Addr[3] & 0xFF) << 24)
                        );

      IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRH,
                          (UINT32)(Mac.Addr[4] & 0xFF) |
                          ((Mac.Addr[5] & 0xFF) << 8)
                        );

      goto ON_EXIT;
    }

    // If there are no cached addresses, then create our own
    IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRL, 0x00F70200);
    IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRH, 0x00009040);

    //return -1;
  }

  // Check that EEPROM isn't active
  while (MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_BUSY);

ON_EXIT:
  // Clear and acknowledge interrupts
  MmioWrite32 (LAN9118_INT_EN, 0);
  MmioWrite32 (LAN9118_IRQ_CFG, 0);
  MmioWrite32 (LAN9118_INT_STS, 0xFFFFFFFF);

  // Do self tests here?

  return EFI_SUCCESS;
}


// Perform software reset on the LAN9118
// Return 0 on success, -1 on error
EFI_STATUS
SoftReset (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 HwConf;
  UINT32 ResetTime;

  // Initialize variable
  ResetTime = 0;

  // Stop Rx and Tx
  StopTx (STOP_TX_MAC | STOP_TX_CFG | STOP_TX_CLEAR, Snp);
  StopRx (STOP_RX_CLEAR, Snp); // Clear receiver FIFO

  // Issue the reset
  HwConf = MmioRead32 (LAN9118_HW_CFG);
  HwConf |= 1;

  // Set the Must Be One (MBO) bit
  if (((HwConf & HWCFG_MBO) >> 20) == 0) {
    HwConf |= HWCFG_MBO;
  }

  // Check that EEPROM isn't active
  while (MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_BUSY);

  // Write the configuration
  MmioWrite32 (LAN9118_HW_CFG, HwConf);
  gBS->Stall (LAN9118_STALL);

  // Wait for reset to complete
  while (MmioRead32 (LAN9118_HW_CFG) & HWCFG_SRST) {

    gBS->Stall (LAN9118_STALL);
    ResetTime += 1;

    // If time taken exceeds 100us, then there was an error condition
    if (ResetTime > 1000) {
      Snp->Mode->State = EfiSimpleNetworkStopped;
      return EFI_TIMEOUT;
    }
  }

  // Check that MAC Address loaded successfully (if required)
  if (Flags & SOFT_RESET_CHECK_MAC_ADDR_LOAD) {
    if ((MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_MAC_ADDRESS_LOADED) == 0) {
      DEBUG((EFI_D_ERROR, "Warning: There was an error detecting EEPROM or loading the MAC Address:\n"
                          "         Using a spare MAC Address.\n"));

      // See if there is a cached address first
      if (&Snp->Mode->PermanentAddress != NULL) {
        EFI_MAC_ADDRESS Mac = Snp->Mode->PermanentAddress;
        IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRL,
                            (Mac.Addr[0] & 0xFF) |
                            ((Mac.Addr[1] & 0xFF) << 8) |
                            ((Mac.Addr[2] & 0xFF) << 16) |
                            ((Mac.Addr[3] & 0xFF) << 24)
                          );

        IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRH,
                            (UINT32)(Mac.Addr[4] & 0xFF) |
                            ((Mac.Addr[5] & 0xFF) << 8)
                          );

        goto ON_EXIT;
      }

      if (&Snp->Mode->CurrentAddress != NULL) {
        EFI_MAC_ADDRESS Mac = Snp->Mode->CurrentAddress;
        IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRL,
                            (Mac.Addr[0] & 0xFF) |
                            ((Mac.Addr[1] & 0xFF) << 8) |
                            ((Mac.Addr[2] & 0xFF) << 16) |
                            ((Mac.Addr[3] & 0xFF) << 24)
                          );

        IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRH,
                            (UINT32)(Mac.Addr[4] & 0xFF) |
                            ((Mac.Addr[5] & 0xFF) << 8)
                          );

        goto ON_EXIT;
      }

      // If there are no cached addresses, then create our own
      IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRL, 0x00F70200);
      IndirectMACWrite32 (INDIRECT_MAC_INDEX_ADDRH, 0x00009040);

      //return -1;
    }
  }

  // Check that EEPROM isn't active
  while (MmioRead32 (LAN9118_E2P_CMD) & E2P_EPC_BUSY);

ON_EXIT:
  // Clear and acknowledge all interrupts
  if (Flags & SOFT_RESET_CLEAR_INT) {
    MmioWrite32 (LAN9118_INT_EN, 0);
    MmioWrite32 (LAN9118_IRQ_CFG, 0);
    MmioWrite32 (LAN9118_INT_STS, 0xFFFFFFFF);
  }

  // Do self tests here?
  if (Flags & SOFT_RESET_SELF_TEST) {

  }

  return EFI_SUCCESS;
}


// Perform PHY software reset
INT32
PhySoftReset (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 PmtCtrl = 0;
  UINT32 LinkTo = 0;

  // PMT PHY reset takes precedence over BCR
  if (Flags & PHY_RESET_PMT) {
    PmtCtrl = MmioRead32 (LAN9118_PMT_CTRL);
    PmtCtrl |= MPTCTRL_PHY_RST;
    MmioWrite32 (LAN9118_PMT_CTRL,PmtCtrl);

    // Wait for completion
    while (MmioRead32 (LAN9118_PMT_CTRL) & MPTCTRL_PHY_RST) {
      gBS->Stall (LAN9118_STALL);
    }
  // PHY Basic Control Register reset
  } else if (Flags & PHY_RESET_PMT) {
    IndirectPHYWrite32 (PHY_INDEX_BASIC_CTRL, PHYCR_RESET);

    // Wait for completion
    while (IndirectPHYRead32 (PHY_INDEX_BASIC_CTRL) & PHYCR_RESET) {
      gBS->Stall (LAN9118_STALL);
    }
  }

  // Check the link status
  if (Flags & PHY_RESET_CHECK_LINK) {
    LinkTo = 100000; // 2 second (could be 50% more)
    while (EFI_ERROR(CheckLinkStatus(0, Snp)) && (LinkTo > 0)) {
      gBS->Stall (LAN9118_STALL);
      LinkTo--;
    }

    // Timed out
    if (LinkTo <= 0) {
      return -1;
    }
  }

  // Clear and acknowledge all interrupts
  if (Flags & PHY_SOFT_RESET_CLEAR_INT) {
    MmioWrite32 (LAN9118_INT_EN, 0);
    MmioWrite32 (LAN9118_IRQ_CFG, 0);
    MmioWrite32 (LAN9118_INT_STS, 0xFFFFFFFF);
  }

  return 0;
}


// Configure hardware for LAN9118
EFI_STATUS
ConfigureHardware (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 GpioConf;

  // Check if we want to use LEDs on GPIO
  if (Flags & HW_CONF_USE_LEDS) {
    GpioConf = MmioRead32 (LAN9118_GPIO_CFG);

    // Enable GPIO as LEDs and Config as Push-Pull driver
    GpioConf |= GPIO_GPIO0_PUSH_PULL | GPIO_GPIO1_PUSH_PULL | GPIO_GPIO2_PUSH_PULL |
                GPIO_LED1_ENABLE | GPIO_LED2_ENABLE | GPIO_LED3_ENABLE;

    // Write the configuration
    MmioWrite32 (LAN9118_GPIO_CFG, GpioConf);
    gBS->Stall (LAN9118_STALL);
  }

  return EFI_SUCCESS;
}

// Configure flow control
EFI_STATUS
ConfigureFlow (
  UINT32 Flags,
  UINT32 HighTrig,
  UINT32 LowTrig,
  UINT32 BPDuration,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  return EFI_SUCCESS;
}

// Do auto-negotiation
EFI_STATUS
AutoNegotiate (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 PhyControl;
  UINT32 PhyStatus;
  UINT32 Features;
  UINT32 TimeOut;

  // First check that auto-negotiation is supported
  PhyStatus = IndirectPHYRead32 (PHY_INDEX_BASIC_STATUS);
  if ((PhyStatus & PHYSTS_AUTO_CAP) == 0) {
    return EFI_DEVICE_ERROR;
  }

  // Check that link is up first
  if ((PhyStatus & PHYSTS_LINK_STS) == 0) {
    // Wait until it is up or until Time Out
    TimeOut = 2000;
    while ((IndirectPHYRead32 (PHY_INDEX_BASIC_STATUS) & PHYSTS_LINK_STS) == 0) {
      gBS->Stall (LAN9118_STALL);
      TimeOut--;
      if (!TimeOut) {
        return EFI_TIMEOUT;
      }
    }
  }

  // Configure features to advertise
  Features = IndirectPHYRead32 (PHY_INDEX_AUTO_NEG_ADVERT);

  if ((Flags & AUTO_NEGOTIATE_ADVERTISE_ALL) > 0) {
    // Link speed capabilities
    Features |= (PHYANA_10BASET | PHYANA_10BASETFD | PHYANA_100BASETX | PHYANA_100BASETXFD);

    // Pause frame capabilities
    Features &= ~(PHYANA_PAUSE_OP_MASK);
    Features |= 3 << 10;
  }

  // Write the features
  IndirectPHYWrite32 (PHY_INDEX_AUTO_NEG_ADVERT, Features);

  // Read control register
  PhyControl = IndirectPHYRead32 (PHY_INDEX_BASIC_CTRL);

  // Enable Auto-Negotiation
  if ((PhyControl & PHYCR_AUTO_EN) == 0) {
    PhyControl |= PHYCR_AUTO_EN;
  }

  // Restart auto-negotiation
  PhyControl |= PHYCR_RST_AUTO;

  // Enable collision test if required to do so
  if (Flags & AUTO_NEGOTIATE_COLLISION_TEST) {
    PhyControl |= PHYCR_COLL_TEST;
  } else {
    PhyControl &= ~ PHYCR_COLL_TEST;
  }

  // Write this configuration
  IndirectPHYWrite32 (PHY_INDEX_BASIC_CTRL, PhyControl);

  // Wait until process has completed
  while ((IndirectPHYRead32 (PHY_INDEX_BASIC_STATUS) & PHYSTS_AUTO_COMP) == 0);

  return EFI_SUCCESS;
}

// Check the Link Status and take appropriate action
EFI_STATUS
CheckLinkStatus (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  // Get the PHY Status
  UINT32 PhyBStatus = IndirectPHYRead32 (PHY_INDEX_BASIC_STATUS);

  if (PhyBStatus & PHYSTS_LINK_STS) {
    return EFI_SUCCESS;
  } else {
    return EFI_DEVICE_ERROR;
  }
}

// Stop the transmitter
EFI_STATUS
StopTx (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 MacCsr;
  UINT32 TxCfg;

  MacCsr = 0;
  TxCfg = 0;

  // Check if we want to clear tx
  if (Flags & STOP_TX_CLEAR) {
    TxCfg = MmioRead32 (LAN9118_TX_CFG);
    TxCfg |= TXCFG_TXS_DUMP | TXCFG_TXD_DUMP;
    MmioWrite32 (LAN9118_TX_CFG, TxCfg);
    gBS->Stall (LAN9118_STALL);
  }

  // Check if already stopped
  if (Flags & STOP_TX_MAC) {
    MacCsr = IndirectMACRead32 (INDIRECT_MAC_INDEX_CR);

    if (MacCsr & MACCR_TX_EN) {
      MacCsr &= ~MACCR_TX_EN;
      IndirectMACWrite32 (INDIRECT_MAC_INDEX_CR, MacCsr);
    }
  }

  if (Flags & STOP_TX_CFG) {
    TxCfg = MmioRead32 (LAN9118_TX_CFG);

    if (TxCfg & TXCFG_TX_ON) {
      TxCfg |= TXCFG_STOP_TX;
      MmioWrite32 (LAN9118_TX_CFG, TxCfg);
      gBS->Stall (LAN9118_STALL);

      // Wait for Tx to finish transmitting
      while (MmioRead32 (LAN9118_TX_CFG) & TXCFG_STOP_TX);
    }
  }

  return EFI_SUCCESS;
}

// Stop the receiver
EFI_STATUS
StopRx (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 MacCsr;
  UINT32 RxCfg;

  RxCfg = 0;

  // Check if already stopped
  MacCsr = IndirectMACRead32 (INDIRECT_MAC_INDEX_CR);

  if (MacCsr & MACCR_RX_EN) {
    MacCsr &= ~ MACCR_RX_EN;
    IndirectMACWrite32 (INDIRECT_MAC_INDEX_CR, MacCsr);
  }

  // Check if we want to clear receiver FIFOs
  if (Flags & STOP_RX_CLEAR) {
    RxCfg = MmioRead32 (LAN9118_RX_CFG);
    RxCfg |= RXCFG_RX_DUMP;
    MmioWrite32 (LAN9118_RX_CFG, RxCfg);
    gBS->Stall (LAN9118_STALL);

    while (MmioRead32 (LAN9118_RX_CFG) & RXCFG_RX_DUMP);
  }

  return EFI_SUCCESS;
}

// Start the transmitter
EFI_STATUS
StartTx (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 MacCsr;
  UINT32 TxCfg;

  MacCsr = 0;
  TxCfg = 0;

  // Check if we want to clear tx
  if (Flags & START_TX_CLEAR) {
    TxCfg = MmioRead32 (LAN9118_TX_CFG);
    TxCfg |= TXCFG_TXS_DUMP | TXCFG_TXD_DUMP;
    MmioWrite32 (LAN9118_TX_CFG, TxCfg);
    gBS->Stall (LAN9118_STALL);
  }

  // Check if tx was started from MAC and enable if not
  if (Flags & START_TX_MAC) {
    MacCsr = IndirectMACRead32 (INDIRECT_MAC_INDEX_CR);
    gBS->Stall (LAN9118_STALL);
    if ((MacCsr & MACCR_TX_EN) == 0) {
      MacCsr |= MACCR_TX_EN;
      IndirectMACWrite32 (INDIRECT_MAC_INDEX_CR, MacCsr);
      gBS->Stall (LAN9118_STALL);
    }
  }

  // Check if tx was started from TX_CFG and enable if not
  if (Flags & START_TX_CFG) {
    TxCfg = MmioRead32 (LAN9118_TX_CFG);
    gBS->Stall (LAN9118_STALL);
    if ((TxCfg & TXCFG_TX_ON) == 0) {
      TxCfg |= TXCFG_TX_ON;
      MmioWrite32 (LAN9118_TX_CFG, TxCfg);
      gBS->Stall (LAN9118_STALL);
    }
  }

  // Set the tx data trigger level

  return EFI_SUCCESS;
}


// Start the receiver
EFI_STATUS
StartRx (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 MacCsr;
  UINT32 RxCfg;

  RxCfg = 0;

  // Check if already started
  MacCsr = IndirectMACRead32 (INDIRECT_MAC_INDEX_CR);

  if ((MacCsr & MACCR_RX_EN) == 0) {
    // Check if we want to clear receiver FIFOs before starting
    if (Flags & START_RX_CLEAR) {
      RxCfg = MmioRead32 (LAN9118_RX_CFG);
      RxCfg |= RXCFG_RX_DUMP;
      MmioWrite32 (LAN9118_RX_CFG, RxCfg);
      gBS->Stall (LAN9118_STALL);

      while (MmioRead32 (LAN9118_RX_CFG) & RXCFG_RX_DUMP);
    }

    MacCsr |= MACCR_RX_EN;
    IndirectMACWrite32 (INDIRECT_MAC_INDEX_CR, MacCsr);
    gBS->Stall (LAN9118_STALL);
  }

  return EFI_SUCCESS;
}

// Check Tx Data available space
UINT32
TxDataFreeSpace (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 TxInf;
  UINT32 FreeSpace;

  // Get the amount of free space from information register
  TxInf = MmioRead32 (LAN9118_TX_FIFO_INF);
  FreeSpace = (TxInf & TXFIFOINF_TDFREE_MASK);

  return FreeSpace; // Value in bytes
}

// Check Tx Status used space
UINT32
TxStatusUsedSpace (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 TxInf;
  UINT32 UsedSpace;

  // Get the amount of used space from information register
  TxInf = MmioRead32 (LAN9118_TX_FIFO_INF);
  UsedSpace = (TxInf & TXFIFOINF_TXSUSED_MASK) >> 16;

  return UsedSpace << 2; // Value in bytes
}

// Check Rx Data used space
UINT32
RxDataUsedSpace (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 RxInf;
  UINT32 UsedSpace;

  // Get the amount of used space from information register
  RxInf = MmioRead32 (LAN9118_RX_FIFO_INF);
  UsedSpace = (RxInf & RXFIFOINF_RXDUSED_MASK);

  return UsedSpace; // Value in bytes (rounded up to nearest DWORD)
}

// Check Rx Status used space
UINT32
RxStatusUsedSpace (
  UINT32 Flags,
  EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 RxInf;
  UINT32 UsedSpace;

  // Get the amount of used space from information register
  RxInf = MmioRead32 (LAN9118_RX_FIFO_INF);
  UsedSpace = (RxInf & RXFIFOINF_RXSUSED_MASK) >> 16;

  return UsedSpace << 2; // Value in bytes
}


// Change the allocation of FIFOs
EFI_STATUS
ChangeFifoAllocation (
  IN      UINT32 Flags,
  IN  OUT UINTN  *TxDataSize    OPTIONAL,
  IN  OUT UINTN  *RxDataSize    OPTIONAL,
  IN  OUT UINT32 *TxStatusSize  OPTIONAL,
  IN  OUT UINT32 *RxStatusSize  OPTIONAL,
  IN  OUT EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  UINT32 HwConf;
  UINT32 TxFifoOption;

  // Check that desired sizes don't exceed limits
  if (*TxDataSize > TX_FIFO_MAX_SIZE)
    return EFI_INVALID_PARAMETER;

#if defined(RX_FIFO_MIN_SIZE) && defined(RX_FIFO_MAX_SIZE)
  if (*RxDataSize > RX_FIFO_MAX_SIZE) {
    return EFI_INVALID_PARAMETER;
  }
#endif

  if (Flags & ALLOC_USE_DEFAULT) {
    return EFI_SUCCESS;
  }

  // If we use the FIFOs (always use this first)
  if (Flags & ALLOC_USE_FIFOS) {
    // Read the current value of allocation
    HwConf = MmioRead32 (LAN9118_HW_CFG);
    TxFifoOption = (HwConf >> 16) & 0xF;

    // Choose the correct size (always use larger than requested if possible)
    if (*TxDataSize < TX_FIFO_MIN_SIZE) {
      *TxDataSize = TX_FIFO_MIN_SIZE;
      *RxDataSize = 13440;
      *RxStatusSize = 896;
      TxFifoOption = 2;
    } else if ((*TxDataSize > TX_FIFO_MIN_SIZE) && (*TxDataSize <= 2560)) {
      *TxDataSize = 2560;
      *RxDataSize = 12480;
      *RxStatusSize = 832;
      TxFifoOption = 3;
    } else if ((*TxDataSize > 2560) && (*TxDataSize <= 3584)) {
      *TxDataSize = 3584;
      *RxDataSize = 11520;
      *RxStatusSize = 768;
      TxFifoOption = 4;
    } else if ((*TxDataSize > 3584) && (*TxDataSize <= 4608)) { // default option
      *TxDataSize = 4608;
      *RxDataSize = 10560;
      *RxStatusSize = 704;
      TxFifoOption = 5;
    } else if ((*TxDataSize > 4608) && (*TxDataSize <= 5632)) {
      *TxDataSize = 5632;
      *RxDataSize = 9600;
      *RxStatusSize = 640;
      TxFifoOption = 6;
    } else if ((*TxDataSize > 5632) && (*TxDataSize <= 6656)) {
      *TxDataSize = 6656;
      *RxDataSize = 8640;
      *RxStatusSize = 576;
      TxFifoOption = 7;
    } else if ((*TxDataSize > 6656) && (*TxDataSize <= 7680)) {
      *TxDataSize = 7680;
      *RxDataSize = 7680;
      *RxStatusSize = 512;
      TxFifoOption = 8;
    } else if ((*TxDataSize > 7680) && (*TxDataSize <= 8704)) {
      *TxDataSize = 8704;
      *RxDataSize = 6720;
      *RxStatusSize = 448;
      TxFifoOption = 9;
    } else if ((*TxDataSize > 8704) && (*TxDataSize <= 9728)) {
      *TxDataSize = 9728;
      *RxDataSize = 5760;
      *RxStatusSize = 384;
      TxFifoOption = 10;
    } else if ((*TxDataSize > 9728) && (*TxDataSize <= 10752)) {
      *TxDataSize = 10752;
      *RxDataSize = 4800;
      *RxStatusSize = 320;
      TxFifoOption = 11;
    } else if ((*TxDataSize > 10752) && (*TxDataSize <= 11776)) {
      *TxDataSize = 11776;
      *RxDataSize = 3840;
      *RxStatusSize = 256;
      TxFifoOption = 12;
    } else if ((*TxDataSize > 11776) && (*TxDataSize <= 12800)) {
      *TxDataSize = 12800;
      *RxDataSize = 2880;
      *RxStatusSize = 192;
      TxFifoOption = 13;
    } else if ((*TxDataSize > 12800) && (*TxDataSize <= 13824)) {
      *TxDataSize = 13824;
      *RxDataSize = 1920;
      *RxStatusSize = 128;
      TxFifoOption = 14;
    }
  } else {
    ASSERT(0); // Untested code path
    HwConf = 0;
    TxFifoOption = 0;
  }

  // Do we need DMA?
  if (Flags & ALLOC_USE_DMA) {
    return EFI_UNSUPPORTED; // Unsupported as of now
  }
  // Clear and assign the new size option
  HwConf &= ~(0xF0000);
  HwConf |= ((TxFifoOption & 0xF) << 16);
  MmioWrite32 (LAN9118_HW_CFG, HwConf);
  gBS->Stall (LAN9118_STALL);

  return EFI_SUCCESS;
}

