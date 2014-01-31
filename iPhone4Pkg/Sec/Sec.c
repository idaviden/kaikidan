/** @file
  C Entry point for the SEC. First C code after the reset vector.

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>
  
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiPei.h>

#include <Library/DebugLib.h>
#include <Library/PrePiLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>
#include <Library/OmapLib.h>
#include <Library/ArmLib.h>
#include <Library/PeCoffGetEntryPointLib.h>
#include <Library/DebugAgentLib.h>

#include <Ppi/GuidedSectionExtraction.h>
#include <Guid/LzmaDecompress.h>
#include <Omap3530/Omap3530.h>

#include "LzmaDecompress.h"

VOID
PadConfiguration (
  VOID
  );

VOID
ClockInit (
  VOID
  );


VOID
TimerInit (
  VOID
  )
{

}

VOID
UartInit (
  VOID
  )
{

}

VOID
InitCache (
  IN  UINT32  MemoryBase,
  IN  UINT32  MemoryLength
  );

EFI_STATUS
EFIAPI
ExtractGuidedSectionLibConstructor (
  VOID
  );

EFI_STATUS
EFIAPI
LzmaDecompressLibConstructor (
  VOID
  );


VOID
CEntryPoint (
  IN  VOID  *MemoryBase,
  IN  UINTN MemorySize,
  IN  VOID  *StackBase,
  IN  UINTN StackSize
  )
{
  VOID *HobBase;

  // Build a basic HOB list
  HobBase      = (VOID *)(UINTN)(FixedPcdGet32(PcdEmbeddedFdBaseAddress) + FixedPcdGet32(PcdEmbeddedFdSize));
  CreateHobList (MemoryBase, MemorySize, HobBase, StackBase);

  // iBoot sets up pin muxing and clocks, we don't have to..

  // Enable program flow prediction, if supported.
  ArmEnableBranchPrediction ();

  // Initialize CPU cache
  InitCache ((UINT32)MemoryBase, (UINT32)MemorySize);

  ArmSetAuxCrBit(1);
  
  // Add memory allocation hob for relocated FD
  BuildMemoryAllocationHob (FixedPcdGet32(PcdEmbeddedFdBaseAddress), FixedPcdGet32(PcdEmbeddedFdSize), EfiBootServicesData);

  // Add the FVs to the hob list
  BuildFvHob (PcdGet32(PcdFlashFvMainBase), PcdGet32(PcdFlashFvMainSize));

  // Start talking
  UartInit ();
 
  InitializeDebugAgent (DEBUG_AGENT_INIT_PREMEM_SEC, NULL, NULL);
  SaveAndSetDebugTimerInterrupt (TRUE);

  DEBUG ((EFI_D_ERROR, "UART Enabled\n"));

  // Start up a free running timer so that the timer lib will work
  TimerInit ();

  // SEC phase needs to run library constructors by hand.
  ExtractGuidedSectionLibConstructor ();
  LzmaDecompressLibConstructor ();

  // Build HOBs to pass up our version of stuff the DXE Core needs to save space
  BuildPeCoffLoaderHob ();
  BuildExtractSectionHob (
    &gLzmaCustomDecompressGuid,
    LzmaGuidedSectionGetInfo,
    LzmaGuidedSectionExtraction
    );

  // Assume the FV that contains the SEC (our code) also contains a compressed FV.
  DecompressFirstFv ();

  // Load the DXE Core and transfer control to it
  LoadDxeCoreFromFv (NULL, 0);
  
  // DXE Core should always load and never return
  ASSERT (FALSE);
}

