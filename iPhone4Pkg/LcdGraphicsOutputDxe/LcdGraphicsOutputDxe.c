/** @file
  Template for Timer Architecture Protocol driver of the ARM flavor

  Copyright (c) 2012, Samsung Electronics Co. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/


#include <PiDxe.h>

#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Protocol/GraphicsOutput.h>
#include <Protocol/DevicePath.h>
#include <Protocol/Cpu.h>

typedef struct {
    VENDOR_DEVICE_PATH DisplayDevicePath;
    EFI_DEVICE_PATH EndDevicePath;
} DISPLAY_DEVICE_PATH;

DISPLAY_DEVICE_PATH gDisplayDevicePath =
{
  {
    HARDWARE_DEVICE_PATH,
    HW_VENDOR_DP,
    (UINT8)(sizeof(VENDOR_DEVICE_PATH)),
    (UINT8)((sizeof(VENDOR_DEVICE_PATH)) >> 8),
    EFI_GRAPHICS_OUTPUT_PROTOCOL_GUID
  },
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    sizeof (EFI_DEVICE_PATH_PROTOCOL),
    0
  }
};

#define LCD_WIDTH  640
#define LCD_HEIGHT  960

EFI_STATUS
EFIAPI
DisplayQueryMode(
  IN  EFI_GRAPHICS_OUTPUT_PROTOCOL          *This,
  IN  UINT32                                ModeNumber,
  OUT UINTN                                 *SizeOfInfo,
  OUT EFI_GRAPHICS_OUTPUT_MODE_INFORMATION  **Info
  );

EFI_STATUS
EFIAPI
DisplaySetMode(
  IN  EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
  IN  UINT32                       ModeNumber
  );

EFI_STATUS
EFIAPI
DisplayBlt(
  IN  EFI_GRAPHICS_OUTPUT_PROTOCOL            *This,
  IN  EFI_GRAPHICS_OUTPUT_BLT_PIXEL           *BltBuffer,   OPTIONAL
  IN  EFI_GRAPHICS_OUTPUT_BLT_OPERATION       BltOperation,
  IN  UINTN                                   SourceX,
  IN  UINTN                                   SourceY,
  IN  UINTN                                   DestinationX,
  IN  UINTN                                   DestinationY,
  IN  UINTN                                   Width,
  IN  UINTN                                   Height,
  IN  UINTN                                   Delta         OPTIONAL
  );

EFI_GRAPHICS_OUTPUT_PROTOCOL gDisplay = {
  DisplayQueryMode,
  DisplaySetMode,
  DisplayBlt,
  NULL
};


EFI_STATUS
EFIAPI
DisplayQueryMode(
  IN  EFI_GRAPHICS_OUTPUT_PROTOCOL          *This,
  IN  UINT32                                ModeNumber,
  OUT UINTN                                 *SizeOfInfo,
  OUT EFI_GRAPHICS_OUTPUT_MODE_INFORMATION  **Info
  )
{
  EFI_STATUS Status;

  Status = gBS->AllocatePool(
      EfiBootServicesData,
      sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION),
      (VOID **)Info
      );
  ASSERT_EFI_ERROR(Status);

  *SizeOfInfo = sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION);

  (*Info)->Version = This->Mode->Info->Version;
  (*Info)->HorizontalResolution = This->Mode->Info->HorizontalResolution;
  (*Info)->VerticalResolution = This->Mode->Info->VerticalResolution;
  (*Info)->PixelFormat = This->Mode->Info->PixelFormat;
  (*Info)->PixelsPerScanLine = This->Mode->Info->PixelsPerScanLine;

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
DisplaySetMode(
  IN  EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
  IN  UINT32                       ModeNumber
  )
{
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
DisplayBlt(
  IN  EFI_GRAPHICS_OUTPUT_PROTOCOL            *This,
  IN  EFI_GRAPHICS_OUTPUT_BLT_PIXEL           *BltBuffer,   OPTIONAL
  IN  EFI_GRAPHICS_OUTPUT_BLT_OPERATION       BltOperation,
  IN  UINTN                                   SourceX,
  IN  UINTN                                   SourceY,
  IN  UINTN                                   DestinationX,
  IN  UINTN                                   DestinationY,
  IN  UINTN                                   Width,
  IN  UINTN                                   Height,
  IN  UINTN                                   Delta         OPTIONAL
  )
{
  UINT8 *VidBuf, *BltBuf, *VidBuf1;
  UINTN i, j;

  switch(BltOperation) {
    case EfiBltVideoFill:
      BltBuf = (UINT8 *)BltBuffer;

      for(i=0;i<Height;i++) {
  VidBuf = (UINT8 *)((UINT32)This->Mode->FrameBufferBase + \
       (DestinationY + i)*This->Mode->Info->PixelsPerScanLine*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL) + \
       DestinationX*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));

        for(j=0;j<Width;j++) {
    gBS->CopyMem((VOID *)VidBuf, (VOID *)BltBuf, sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));
    VidBuf += sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL);
  }
      }
      break;

    case EfiBltVideoToBltBuffer:
      if(Delta == 0)
        Delta = Width * sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL);

      for(i=0;i<Height;i++) {
  VidBuf = (UINT8 *)((UINT32)This->Mode->FrameBufferBase + \
       (SourceY + i)*This->Mode->Info->PixelsPerScanLine*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL) + \
       SourceX*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));
  BltBuf = (UINT8 *)((UINT32)BltBuffer + (DestinationY + i)*Delta + DestinationX*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));

        for(j=0;j<Width;j++) {
    gBS->CopyMem((VOID *)BltBuf, (VOID *)VidBuf, sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));
    VidBuf += sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL);
    BltBuf += sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL);
  }
      }
      break;

    case EfiBltBufferToVideo:
      if(Delta == 0)
        Delta = Width * sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL);

      for(i=0;i<Height;i++) {
  VidBuf = (UINT8 *)((UINT32)This->Mode->FrameBufferBase + \
       (DestinationY + i)*This->Mode->Info->PixelsPerScanLine*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL) + \
       DestinationX*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));
  BltBuf = (UINT8 *)((UINT32)BltBuffer + (SourceY + i)*Delta + SourceX*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));

        for(j=0;j<Width;j++) {
    gBS->CopyMem((VOID *)VidBuf, (VOID *)BltBuf, sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));
    VidBuf += sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL);
    BltBuf += sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL);
  }
      }
      break;

    case EfiBltVideoToVideo:
      for(i=0;i<Height;i++) {
  VidBuf = (UINT8 *)((UINT32)This->Mode->FrameBufferBase + \
       (SourceY + i)*This->Mode->Info->PixelsPerScanLine*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL) + \
       SourceX*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));

  VidBuf1 = (UINT8 *)((UINT32)This->Mode->FrameBufferBase + \
       (DestinationY + i)*This->Mode->Info->PixelsPerScanLine*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL) + \
       DestinationX*sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));

        for(j=0;j<Width;j++) {
    gBS->CopyMem((VOID *)VidBuf1, (VOID *)VidBuf, sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL));
    VidBuf += sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL);
    VidBuf1 += sizeof(EFI_GRAPHICS_OUTPUT_BLT_PIXEL);
  }
      }
      break;

    default:
      ASSERT_EFI_ERROR(EFI_SUCCESS);
  }

  return EFI_SUCCESS;
}



EFI_STATUS
GetVram (
  OUT EFI_PHYSICAL_ADDRESS*  VramBaseAddress,
  OUT UINTN*                 VramSize
  )
{
  EFI_STATUS             Status;
  EFI_CPU_ARCH_PROTOCOL  *Cpu;
  UINTN                  MaxSize;

  MaxSize = 0x500000;
  *VramSize = MaxSize;

  // Allocate VRAM from DRAM
  Status = gBS->AllocatePages (AllocateAnyPages, EfiBootServicesData, EFI_SIZE_TO_PAGES((MaxSize)), VramBaseAddress);
  if (EFI_ERROR(Status)) {
    return Status;
  }

  // Ensure the Cpu architectural protocol is already installed
  Status = gBS->LocateProtocol (&gEfiCpuArchProtocolGuid, NULL, (VOID **)&Cpu);
  ASSERT_EFI_ERROR(Status);

  // Mark the VRAM as un-cacheable. The VRAM is inside the DRAM, which is cacheable.
  Status = Cpu->SetMemoryAttributes (Cpu, *VramBaseAddress, *VramSize, EFI_MEMORY_UC);
  if (EFI_ERROR(Status)) {
    gBS->FreePool (VramBaseAddress);
    return Status;
  }

  return EFI_SUCCESS;
}

/**
  Initialize the state information for the Display Dxe

  @param  ImageHandle   of the loaded driver
  @param  SystemTable   Pointer to the System Table

  @retval EFI_SUCCESS           Protocol registered
  @retval EFI_OUT_OF_RESOURCES  Cannot allocate protocol data structure
  @retval EFI_DEVICE_ERROR      Hardware problems

**/
EFI_STATUS
EFIAPI
DisplayDxeInitialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  EFI_GUID GraphicsOutputProtocolGuid = EFI_GRAPHICS_OUTPUT_PROTOCOL_GUID;
  EFI_GUID DevicePathProtocolGuid = EFI_DEVICE_PATH_PROTOCOL_GUID;
  EFI_PHYSICAL_ADDRESS VramBaseAddress;
  UINT32 FBAddr;
  UINTN VramSize = 0x500000;

  Status = GetVram (&VramBaseAddress, &VramSize);
  ASSERT_EFI_ERROR(Status);

  FBAddr = VramBaseAddress;

  gBS->SetMem((VOID *)FBAddr, (LCD_WIDTH*LCD_HEIGHT*4), 00);
  MmioWrite32(0x89004044, FBAddr);

  /* Initialize Display */
  if(gDisplay.Mode == NULL){
    Status = gBS->AllocatePool(
        EfiBootServicesData,
        sizeof(EFI_GRAPHICS_OUTPUT_PROTOCOL_MODE),
        (VOID **)&gDisplay.Mode
        );
    ASSERT_EFI_ERROR(Status);
    ZeroMem(gDisplay.Mode,sizeof(EFI_GRAPHICS_OUTPUT_PROTOCOL_MODE));
  }
  if(gDisplay.Mode->Info==NULL){
    Status = gBS->AllocatePool(
        EfiBootServicesData,
        sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION),
        (VOID **)&gDisplay.Mode->Info
        );
    ASSERT_EFI_ERROR(Status);
    ZeroMem(gDisplay.Mode->Info,sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION));
  }
  /* Fill out mode information */
  gDisplay.Mode->MaxMode = 1;
  gDisplay.Mode->Mode = 0;
  gDisplay.Mode->Info->Version = 0;
  gDisplay.Mode->Info->HorizontalResolution = LCD_WIDTH;
  gDisplay.Mode->Info->VerticalResolution = LCD_HEIGHT;
  gDisplay.Mode->Info->PixelFormat = PixelBlueGreenRedReserved8BitPerColor;
  gDisplay.Mode->Info->PixelsPerScanLine = LCD_WIDTH;
  gDisplay.Mode->SizeOfInfo = sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION);
  gDisplay.Mode->FrameBufferBase = FBAddr;
  gDisplay.Mode->FrameBufferSize = (LCD_WIDTH * LCD_HEIGHT * 4);
 
#if 0
  Status = gBS->InstallMultipleProtocolInterfaces (
                   &ImageHandle,
                   &DevicePathProtocolGuid,
                   &gDisplayDevicePath,
                   &GraphicsOutputProtocolGuid,
                   &gDisplay,
                   NULL);
#else
  {
    EFI_HANDLE      gUEFIDisplayHandle = NULL;
     Status = gBS->InstallMultipleProtocolInterfaces (
                   &gUEFIDisplayHandle,
                   &DevicePathProtocolGuid,
                   &gDisplayDevicePath,
                   &GraphicsOutputProtocolGuid,
                   &gDisplay,
                   NULL);
    if (EFI_ERROR(Status)) {
      DEBUG((DEBUG_ERROR, "DisplayDxeInitialize: Can not install the protocol. Exit Status=%r\n", Status));
      return Status;
    }
  }
#endif

  return Status;
}
