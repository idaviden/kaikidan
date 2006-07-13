/*++

Copyright (c) 2006, Intel Corporation
All rights reserved. This program and the accompanying materials
are licensed and made available under the terms and conditions of the Software 
License Agreement which accompanies this distribution.


Module Name:

  Data.c

Abstract:

  Global data in the FAT Filesystem driver

Revision History

--*/

#include "Fat.h"

//
// Globals
//
//
// Unicode collation interface pointer
//
EFI_UNICODE_COLLATION_PROTOCOL  *gUnicodeCollationInterface;

//
// FatFsLock - Global lock for synchronizing all requests.
//
EFI_LOCK FatFsLock = EFI_INITIALIZE_LOCK_VARIABLE(EFI_TPL_CALLBACK);

//
// Filesystem interface functions
//
EFI_FILE                        FatFileInterface = {
  EFI_FILE_HANDLE_REVISION,
  FatOpen,
  FatClose,
  FatDelete,
  FatRead,
  FatWrite,
  FatGetPosition,
  FatSetPosition,
  FatGetInfo,
  FatSetInfo,
  FatFlush
};
