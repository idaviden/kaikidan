/** @file

 Copyright (c) 2011, ARM Ltd. All rights reserved.<BR>
 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution.  The full text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __OMAP3_DSS_GRAPHICS__
#define __OMAP3_DSS_GRAPHICS__

#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/IoLib.h>

#include <Protocol/DevicePathToText.h>
#include <Protocol/EmbeddedExternalDevice.h>
#include <Protocol/Cpu.h>

#include <Guid/GlobalVariable.h>

typedef struct {
  VENDOR_DEVICE_PATH            Guid;
  EFI_DEVICE_PATH_PROTOCOL      End;
} LCD_GRAPHICS_DEVICE_PATH;

typedef struct {
  UINTN                                 Signature;
  EFI_HANDLE                            Handle;
  EFI_GRAPHICS_OUTPUT_MODE_INFORMATION  ModeInfo;
  EFI_GRAPHICS_OUTPUT_PROTOCOL_MODE     Mode;
  EFI_GRAPHICS_OUTPUT_PROTOCOL          Gop;
  LCD_GRAPHICS_DEVICE_PATH              DevicePath;
//  EFI_EVENT                             ExitBootServicesEvent;
} LCD_INSTANCE;

#define LCD_INSTANCE_SIGNATURE  SIGNATURE_32('l', 'c', 'd', '0')
#define LCD_INSTANCE_FROM_GOP_THIS(a)     CR (a, LCD_INSTANCE, Gop, LCD_INSTANCE_SIGNATURE)

typedef struct {
  UINTN             Mode;
  UINTN             HorizontalResolution;
  UINTN             VerticalResolution;

  UINT32            DssDivisor;
  UINT32            DispcDivisor;

  UINT32            HSync;
  UINT32            HFrontPorch;
  UINT32            HBackPorch;

  UINT32            VSync;
  UINT32            VFrontPorch;
  UINT32            VBackPorch;
} LCD_MODE;

EFI_STATUS
InitializeDisplay (
  IN LCD_INSTANCE* Instance
);

EFI_STATUS
EFIAPI
LcdGraphicsQueryMode (
  IN  EFI_GRAPHICS_OUTPUT_PROTOCOL          *This,
  IN  UINT32                                ModeNumber,
  OUT UINTN                                 *SizeOfInfo,
  OUT EFI_GRAPHICS_OUTPUT_MODE_INFORMATION  **Info
);

EFI_STATUS
EFIAPI
LcdGraphicsSetMode (
  IN EFI_GRAPHICS_OUTPUT_PROTOCOL  *This,
  IN UINT32                        ModeNumber
);

EFI_STATUS
EFIAPI
LcdGraphicsBlt (
  IN EFI_GRAPHICS_OUTPUT_PROTOCOL        *This,
	IN OUT EFI_GRAPHICS_OUTPUT_BLT_PIXEL   *BltBuffer,     OPTIONAL
	IN EFI_GRAPHICS_OUTPUT_BLT_OPERATION   BltOperation,
	IN UINTN                               SourceX,
	IN UINTN                               SourceY,
	IN UINTN                               DestinationX,
	IN UINTN                               DestinationY,
	IN UINTN                               Width,
	IN UINTN                               Height,
	IN UINTN                               Delta           OPTIONAL   // Number of BYTES in a row of the BltBuffer
);

#endif
