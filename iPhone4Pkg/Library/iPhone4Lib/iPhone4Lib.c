/** @file

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

#define GPIO_BASE   0xBFA00000
#define TIMER_BASE    0xBF101000
#define UART_BASE   0x82500000

UINT32 
GpioBase (
  IN  UINTN Port
  )
{
  return GPIO_BASE;
}

UINT32
TimerBase (
  IN  UINTN Timer
  )
{
  return TIMER_BASE;
}

UINTN
InterruptVectorForTimer (
  IN  UINTN Timer
  )
{
  return 5;
}

UINT32
UartBase (
  IN  UINTN Uart
  )
{
  return UART_BASE;
}

