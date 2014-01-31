/** @file
  Serial I/O Port library functions with no library constructor/destructor


  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>
  
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Library/DebugLib.h>
#include <Library/SerialPortLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>

/*

  Programmed hardware of Serial port.

  @return    Always return EFI_UNSUPPORTED.

**/
RETURN_STATUS
EFIAPI
SerialPortInitialize (
  VOID
  )
{
  // assume assembly code at reset vector has setup UART
  return RETURN_SUCCESS;
}

/**
  Write data to serial device.

  @param  Buffer           Point of data buffer which need to be writed.
  @param  NumberOfBytes    Number of output bytes which are cached in Buffer.

  @retval 0                Write data failed.
  @retval !0               Actual number of bytes writed to serial device.

**/
// My headers are broken.. :(
extern UINT32 UartBase(INTN UartNumber);

#define ULCON      0x0000 /* Line Control             */
#define UCON       0x0004 /* Control                  */
#define UFCON      0x0008 /* FIFO Control             */
#define UMCON      0x000C /* Modem Control            */
#define UTRSTAT    0x0010 /* Tx/Rx Status             */
#define UERSTAT    0x0014 /* UART Error Status        */
#define UFSTAT     0x0018 /* FIFO Status              */
#define UMSTAT     0x001C /* Modem Status             */
#define UTXH       0x0020 /* Transmit Buffer          */
#define URXH       0x0024 /* Receive Buffer           */
#define UBRDIV     0x0028 /* Baud Rate Divisor        */
#define UFRACVAL   0x002C /* Divisor Fractional Value */
#define UINTP      0x0030 /* Interrupt Pending        */
#define UINTSP     0x0034 /* Interrupt Source Pending */
#define UINTM      0x0038 /* Interrupt Mask           */

#define UART_UFSTAT_TXFIFO_FULL     (0x1 << 9)
#define UART_UFSTAT_RXFIFO_FULL     (0x1 << 8)
#define UART_UTRSTAT_TRANSMITTEREMPTY   0x4
#define UART_UMSTAT_CTS         0x1

#define ULCON_OFFSET      0x00
#define UCON_OFFSET     0x04
#define UFCON_OFFSET      0x08
#define UMCON_OFFSET      0x0C
#define UTRSTAT_OFFSET      0x10
#define UERSTAT_OFFSET      0x14
#define UFSTAT_OFFSET     0x18
#define UMSTAT_OFFSET     0x1C
#define UTXH_OFFSET     0x20
#define URXH_OFFSET     0x24
#define UBRDIV_OFFSET     0x28
#define UDIVSLOT_OFFSET     0x2C
#define UINTP_OFFSET      0x30
#define UINTSP_OFFSET     0x34
#define UINTM_OFFSET      0x38

#define UART_TX_EMPTY_FLAG_MASK         (0x02)
#define UART_RX_EMPTY_FLAG_MASK         (0x01)

UINTN
EFIAPI
SerialPortWrite (
  IN UINT8     *Buffer,
  IN UINTN     NumberOfBytes
)
{
  UINT32  UARTConsoleBase = UartBase(0);
  UINTN   Count;
    
  for (Count = 0; Count < NumberOfBytes; Count++, Buffer++) {
    while ((MmioRead32 (UARTConsoleBase + UTRSTAT_OFFSET) & UART_TX_EMPTY_FLAG_MASK) == 0);
    MmioWrite8 (UARTConsoleBase + UTXH_OFFSET, *Buffer);
  }

  return NumberOfBytes;
}


/**
  Read data from serial device and save the datas in buffer.

  @param  Buffer           Point of data buffer which need to be writed.
  @param  NumberOfBytes    Number of output bytes which are cached in Buffer.

  @retval 0                Read data failed.
  @retval !0               Aactual number of bytes read from serial device.

**/
UINTN
EFIAPI
SerialPortRead (
  OUT UINT8     *Buffer,
  IN  UINTN     NumberOfBytes
)
{
  UINT32  UARTConsoleBase = UartBase(0);
  UINTN   Count;
    
  for (Count = 0; Count < NumberOfBytes; Count++, Buffer++) {
    while ((MmioRead32 (UARTConsoleBase + UTRSTAT_OFFSET) & UART_RX_EMPTY_FLAG_MASK) == 0);
    *Buffer = MmioRead8 (UARTConsoleBase + URXH_OFFSET);
  }

  return NumberOfBytes;
}


/**
  Check to see if any data is avaiable to be read from the debug device.

  @retval EFI_SUCCESS       At least one byte of data is avaiable to be read
  @retval EFI_NOT_READY     No data is avaiable to be read
  @retval EFI_DEVICE_ERROR  The serial device is not functioning properly

**/
BOOLEAN
EFIAPI
SerialPortPoll (
  VOID
  )
{
  UINT32 UARTConsoleBase;
  UARTConsoleBase = UartBase(0);

  return ((MmioRead32 (UARTConsoleBase + UTRSTAT_OFFSET) & UART_RX_EMPTY_FLAG_MASK) != 0);
}

