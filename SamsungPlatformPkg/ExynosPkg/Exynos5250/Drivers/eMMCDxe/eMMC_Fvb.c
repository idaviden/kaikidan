/** @file
  eMMC firmware volume block protocol driver
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
#include <Library/UncachedMemoryAllocationLib.h>

#include <Protocol/FirmwareVolumeBlock.h>
#include "eMMCDxe.h"


//#undef EFI_D_INFO
//#define EFI_D_INFO 1


EFI_FVB_ATTRIBUTES_2 gAttribute = (EFI_FVB2_READ_STATUS|EFI_FVB2_WRITE_STATUS|EFI_FVB2_ALIGNMENT_32);
UINT32 *FVBMemAddr;

/**
  The GetAttributes() function retrieves the attributes and
  current settings of the block.

  @param This       Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

  @param Attributes Pointer to EFI_FVB_ATTRIBUTES_2 in which the
                    attributes and current settings are
                    returned. Type EFI_FVB_ATTRIBUTES_2 is defined
                    in EFI_FIRMWARE_VOLUME_HEADER.

  @retval EFI_SUCCESS The firmware volume attributes were
                      returned.

**/

EFI_STATUS
EFIAPI
FvbGetAttributes (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  OUT       EFI_FVB_ATTRIBUTES_2                *Attributes
  )
{
    *Attributes = gAttribute;
    DEBUG ((EFI_D_INFO, "FvbGetAttributes 0x%x\n",  gAttribute));
    return EFI_SUCCESS;
}


/**
  The SetAttributes() function sets configurable firmware volume
  attributes and returns the new settings of the firmware volume.


  @param This        Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

  @param Attributes   On input, Attributes is a pointer to
                      EFI_FVB_ATTRIBUTES_2 that contains the
                      desired firmware volume settings. On
                      successful return, it contains the new
                      settings of the firmware volume. Type
                      EFI_FVB_ATTRIBUTES_2 is defined in
                      EFI_FIRMWARE_VOLUME_HEADER.

  @retval EFI_SUCCESS           The firmware volume attributes were returned.

  @retval EFI_INVALID_PARAMETER The attributes requested are in
                                conflict with the capabilities
                                as declared in the firmware
                                volume header.

**/
EFI_STATUS
EFIAPI
FvbSetAttributes (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  IN OUT    EFI_FVB_ATTRIBUTES_2                *Attributes
  )
{
    gAttribute |= *Attributes;
    *Attributes = gAttribute;
    DEBUG ((EFI_D_INFO, "FvbSetAttributes 0x%x\n",  gAttribute));
    return EFI_SUCCESS;
}


/**
  The GetPhysicalAddress() function retrieves the base address of
  a memory-mapped firmware volume. This function should be called
  only for memory-mapped firmware volumes.

  @param This     Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

  @param Address  Pointer to a caller-allocated
                  EFI_PHYSICAL_ADDRESS that, on successful
                  return from GetPhysicalAddress(), contains the
                  base address of the firmware volume.

  @retval EFI_SUCCESS       The firmware volume base address was returned.

  @retval EFI_NOT_SUPPORTED The firmware volume is not memory mapped.

**/
EFI_STATUS
EFIAPI
FvbGetPhysicalAddress (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  OUT       EFI_PHYSICAL_ADDRESS                *Address
  )
{
    UINT32 NVBase = PcdGet32(PcdFlashNvStorageVariableBase);
    UINT32 NVSize = PcdGet32(PcdFlashNvStorageVariableSize);

    DEBUG ((EFI_D_INFO, "FvbGetPhysicalAddress Base:0x%x, Size:0x%x\n", NVBase, NVSize));    

    if(FVBMemAddr==NULL)
    {
        FVBMemAddr = (UINT32 *)UncachedAllocatePool(NVSize);
        DEBUG ((EFI_D_INFO, "FvbGetPhysicalAddress MEM Alloc 0x%x\n", FVBMemAddr));    

        if(FVBMemAddr==NULL)
        {
            DEBUG ((EFI_D_ERROR, "FvbGetPhysicalAddress Alloc failed \n"));    
            return EFI_UNSUPPORTED;
        }
    }
    else
    {
        DEBUG ((EFI_D_INFO, "FvbGetPhysicalAddress already Allocated 0x%x \n", FVBMemAddr));    
    }
    
    CopyMem((VOID *)FVBMemAddr, (VOID *)NVBase, NVSize);
    Address = (EFI_PHYSICAL_ADDRESS *)FVBMemAddr;
    DEBUG ((EFI_D_INFO, "FvbGetPhysicalAddress Addr:0x%x\n", Address));    
    return EFI_SUCCESS;
}


/**
  The GetBlockSize() function retrieves the size of the requested
  block. It also returns the number of additional blocks with
  the identical size. The GetBlockSize() function is used to
  retrieve the block map (see EFI_FIRMWARE_VOLUME_HEADER).


  @param This        Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

  @param Lba         Indicates the block for which to return the size.

  @param BlockSize   Pointer to a caller-allocated UINTN in which
                     the size of the block is returned.

  @param NumberOfBlocks Pointer to a caller-allocated UINTN in
                        which the number of consecutive blocks,
                        starting with Lba, is returned. All
                        blocks in this range have a size of
                        BlockSize.


  @retval EFI_SUCCESS             The firmware volume base address was returned.

  @retval EFI_INVALID_PARAMETER   The requested LBA is out of range.

**/
EFI_STATUS
EFIAPI
FvbGetBlockSize (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  IN        EFI_LBA                             Lba,
  OUT       UINTN                               *BlockSize,
  OUT       UINTN                               *NumberOfBlocks
  )
{
    EFI_STATUS status = EFI_SUCCESS;
    *BlockSize = gSDMMCMedia.BlockSize;
    *NumberOfBlocks = 512;
    DEBUG ((EFI_D_INFO, "FvbGetBlockSize numblocks:%d\n",  *NumberOfBlocks));
    return status;
}



/**
  Reads the specified number of bytes into a buffer from the specified block.

  The Read() function reads the requested number of bytes from the
  requested block and stores them in the provided buffer.
  Implementations should be mindful that the firmware volume
  might be in the ReadDisabled state. If it is in this state,
  the Read() function must return the status code
  EFI_ACCESS_DENIED without modifying the contents of the
  buffer. The Read() function must also prevent spanning block
  boundaries. If a read is requested that would span a block
  boundary, the read must read up to the boundary but not
  beyond. The output parameter NumBytes must be set to correctly
  indicate the number of bytes actually read. The caller must be
  aware that a read may be partially completed.

  @param This     Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

  @param Lba      The starting logical block index
                  from which to read.

  @param Offset   Offset into the block at which to begin reading.

  @param NumBytes Pointer to a UINTN. At entry, *NumBytes
                  contains the total size of the buffer. At
                  exit, *NumBytes contains the total number of
                  bytes read.

  @param Buffer   Pointer to a caller-allocated buffer that will
                  be used to hold the data that is read.

  @retval EFI_SUCCESS         The firmware volume was read successfully,
                              and contents are in Buffer.

  @retval EFI_BAD_BUFFER_SIZE Read attempted across an LBA
                              boundary. On output, NumBytes
                              contains the total number of bytes
                              returned in Buffer.

  @retval EFI_ACCESS_DENIED   The firmware volume is in the
                              ReadDisabled state.

  @retval EFI_DEVICE_ERROR    The block device is not
                              functioning correctly and could
                              not be read.

**/
EFI_STATUS
EFIAPI
FvbRead (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  IN        EFI_LBA                             Lba,
  IN        UINTN                               Offset,
  IN OUT    UINTN                               *NumBytes,
  IN OUT    UINT8                               *Buffer
  )
{
    EFI_BLOCK_IO_PROTOCOL *EFIBlockIO = (EFI_BLOCK_IO_PROTOCOL *)This;
    EFI_STATUS status = EFI_SUCCESS;
    VOID *TempBuf = NULL;
    UINT32 NumBlock;
    UINT32 AllocSize=0;
    Lba += MSHC_BOOT_SECURE_OFFSET;

    DEBUG ((EFI_D_INFO, "FvbRead Offset : %d, Numbytes : %d\n", Offset, *NumBytes));
    
    if(gCardInit==TRUE)
    {
        if (0 == (*NumBytes%gSDMMCMedia.BlockSize)) 
        {
            NumBlock = (*NumBytes/gSDMMCMedia.BlockSize);
         } 
        else 
        {
        	NumBlock = (*NumBytes/gSDMMCMedia.BlockSize) + 1;
         } 
        //DEBUG ((EFI_D_INFO, "FvbRead numblock : %d, BlockSize : %d\n", NumBlock, gSDMMCMedia.BlockSize));

        AllocSize = NumBlock*gSDMMCMedia.BlockSize;
        TempBuf = AllocatePool(AllocSize);
        //ZeroMem (TempBuf, NumBlock*gSDMMCMedia.BlockSize);
        if(TempBuf==NULL)
        {
            DEBUG ((EFI_D_ERROR, "FvbRead AllocatePool Failed!!\n"));
            status = EFI_DEVICE_ERROR;
            goto Exit;
        }
            
        status = SdReadWrite(EFIBlockIO, Lba, TempBuf, AllocSize, READ);
        if(status!=EFI_SUCCESS)
        {
            DEBUG ((EFI_D_ERROR, "FvbRead Failed 0x%x\n",  status));
            status = EFI_ACCESS_DENIED;
            goto Exit;
        }
        
        CopyMem((VOID *)Buffer, (VOID *)(TempBuf+Offset), *NumBytes);
    }

    Exit:

    if (TempBuf != NULL) 
    {
        FreePool(TempBuf);
    }
     
    return status;  
}


/**
  Writes the specified number of bytes from the input buffer to the block.

  The Write() function writes the specified number of bytes from
  the provided buffer to the specified block and offset. If the
  firmware volume is sticky write, the caller must ensure that
  all the bits of the specified range to write are in the
  EFI_FVB_ERASE_POLARITY state before calling the Write()
  function, or else the result will be unpredictable. This
  unpredictability arises because, for a sticky-write firmware
  volume, a write may negate a bit in the EFI_FVB_ERASE_POLARITY
  state but cannot flip it back again.  Before calling the
  Write() function,  it is recommended for the caller to first call
  the EraseBlocks() function to erase the specified block to
  write. A block erase cycle will transition bits from the
  (NOT)EFI_FVB_ERASE_POLARITY state back to the
  EFI_FVB_ERASE_POLARITY state. Implementations should be
  mindful that the firmware volume might be in the WriteDisabled
  state. If it is in this state, the Write() function must
  return the status code EFI_ACCESS_DENIED without modifying the
  contents of the firmware volume. The Write() function must
  also prevent spanning block boundaries. If a write is
  requested that spans a block boundary, the write must store up
  to the boundary but not beyond. The output parameter NumBytes
  must be set to correctly indicate the number of bytes actually
  written. The caller must be aware that a write may be
  partially completed. All writes, partial or otherwise, must be
  fully flushed to the hardware before the Write() service
  returns.

  @param This     Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

  @param Lba      The starting logical block index to write to.

  @param Offset   Offset into the block at which to begin writing.

  @param NumBytes The pointer to a UINTN. At entry, *NumBytes
                  contains the total size of the buffer. At
                  exit, *NumBytes contains the total number of
                  bytes actually written.

  @param Buffer   The pointer to a caller-allocated buffer that
                  contains the source for the write.

  @retval EFI_SUCCESS         The firmware volume was written successfully.

  @retval EFI_BAD_BUFFER_SIZE The write was attempted across an
                              LBA boundary. On output, NumBytes
                              contains the total number of bytes
                              actually written.

  @retval EFI_ACCESS_DENIED   The firmware volume is in the
                              WriteDisabled state.

  @retval EFI_DEVICE_ERROR    The block device is malfunctioning
                              and could not be written.


**/
EFI_STATUS
EFIAPI
FvbWrite (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  IN        EFI_LBA                             Lba,
  IN        UINTN                               Offset,
  IN OUT    UINTN                               *NumBytes,
  IN        UINT8                               *Buffer
  )
{
    EFI_BLOCK_IO_PROTOCOL *EFIBlockIO = (EFI_BLOCK_IO_PROTOCOL *)This;    
    EFI_STATUS status = EFI_SUCCESS;
    VOID *TempBuf = NULL;
    UINT32 NumBlock;
    UINT32 AllocSize=0;
    Lba += MSHC_BOOT_SECURE_OFFSET;

    DEBUG ((EFI_D_INFO, "FvbWrite Offset : %d, Numbyte : %d\n", Offset, *NumBytes));
    
    if(gCardInit==TRUE)
    {
        if (0 == (*NumBytes%gSDMMCMedia.BlockSize)) 
        {
            NumBlock = (*NumBytes/gSDMMCMedia.BlockSize);
         } 
        else 
        {
        	NumBlock = (*NumBytes/gSDMMCMedia.BlockSize) + 1;
         }   
        //DEBUG ((EFI_D_INFO, "FvbWrite numblock : %d, BlockSize : %d\n", NumBlock, gSDMMCMedia.BlockSize));

        AllocSize = (NumBlock*gSDMMCMedia.BlockSize);
        TempBuf = AllocatePool(AllocSize);
        //ZeroMem (TempBuf, NumBlock*gSDMMCMedia.BlockSize);
        if(TempBuf==NULL)
        {
            DEBUG ((EFI_D_ERROR, "FvbWrite AllocatePool Failed!!\n"));
            status = EFI_DEVICE_ERROR;
            goto Exit;
        }
        
        status = SdReadWrite(EFIBlockIO, Lba, TempBuf, AllocSize, READ);
        if(status!=EFI_SUCCESS)
        {
            DEBUG ((EFI_D_ERROR, "FvbWrite Read Failed 0x%x\n",  status));
            status = EFI_DEVICE_ERROR;
            goto Exit;
        }

        CopyMem((VOID *)(TempBuf+Offset), (VOID *)Buffer, *NumBytes);

        status = SdReadWrite(EFIBlockIO, Lba, TempBuf, AllocSize, WRITE);
        if(status!=EFI_SUCCESS)
        {
            DEBUG ((EFI_D_ERROR, "FvbWrite Write Failed 0x%x\n",  status));
            status = EFI_ACCESS_DENIED;
            goto Exit;
        }
    }
    else
    {
        DEBUG ((EFI_D_ERROR, "FvbWrite Error eMMC is not ready\n"));
    }
    
    Exit:

    if (TempBuf != NULL) 
    {
        FreePool(TempBuf);
    }
     
    return status;
}


/**
  Erases and initializes a firmware volume block.

  The EraseBlocks() function erases one or more blocks as denoted
  by the variable argument list. The entire parameter list of
  blocks must be verified before erasing any blocks. If a block is
  requested that does not exist within the associated firmware
  volume (it has a larger index than the last block of the
  firmware volume), the EraseBlocks() function must return the
  status code EFI_INVALID_PARAMETER without modifying the contents
  of the firmware volume. Implementations should be mindful that
  the firmware volume might be in the WriteDisabled state. If it
  is in this state, the EraseBlocks() function must return the
  status code EFI_ACCESS_DENIED without modifying the contents of
  the firmware volume. All calls to EraseBlocks() must be fully
  flushed to the hardware before the EraseBlocks() service
  returns.

  @param This   Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL
                instance.

  @param ...    The variable argument list is a list of tuples.
                Each tuple describes a range of LBAs to erase
                and consists of the following:
                - An EFI_LBA that indicates the starting LBA
                - A UINTN that indicates the number of blocks to
                  erase.

                The list is terminated with an
                EFI_LBA_LIST_TERMINATOR. For example, the
                following indicates that two ranges of blocks
                (5-7 and 10-11) are to be erased: EraseBlocks
                (This, 5, 3, 10, 2, EFI_LBA_LIST_TERMINATOR);

  @retval EFI_SUCCESS The erase request successfully
                      completed.

  @retval EFI_ACCESS_DENIED   The firmware volume is in the
                              WriteDisabled state.
  @retval EFI_DEVICE_ERROR  The block device is not functioning
                            correctly and could not be written.
                            The firmware device may have been
                            partially erased.
  @retval EFI_INVALID_PARAMETER One or more of the LBAs listed
                                in the variable argument list do
                                not exist in the firmware volume.

**/
EFI_STATUS
EFIAPI
FvbEraseBlocks (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  ...
  )
{
    EFI_STATUS status = EFI_SUCCESS;
    UINTN StartBlock, NumBlock;
    UINTN Index;
    VA_LIST Marker;

    VA_START (Marker, This);
    
    for (Index = 0, status = EFI_SUCCESS; !EFI_ERROR (status); Index++)
    {
        StartBlock = VA_ARG (Marker, UINTN);
        NumBlock = VA_ARG (Marker, UINTN);
        DEBUG ((EFI_D_INFO, "FvbEraseBlocks  start:%d numblock:%d\n", StartBlock, NumBlock));
        
        if(StartBlock==0xFFFFFFFF)
        {
            break;
        }

       StartBlock += MSHC_BOOT_SECURE_OFFSET;   
        /* MMC High Capacity erase minimum size is 512KB */
        status = EraseBlockData(MSHC_BOOT_PARTITION, StartBlock, NumBlock);
    }
    
    VA_END (Marker);
    return status;
}


//
// Making this global saves a few bytes in image size
//
EFI_HANDLE  gFvbHandle = NULL;


///
/// The Firmware Volume Block Protocol is the low-level interface
/// to a firmware volume. File-level access to a firmware volume
/// should not be done using the Firmware Volume Block Protocol.
/// Normal access to a firmware volume must use the Firmware
/// Volume Protocol. Typically, only the file system driver that
/// produces the Firmware Volume Protocol will bind to the
/// Firmware Volume Block Protocol.
///
EFI_FIRMWARE_VOLUME_BLOCK_PROTOCOL gFvbProtocol = {
  FvbGetAttributes,
  FvbSetAttributes,
  FvbGetPhysicalAddress,
  FvbGetBlockSize,
  FvbRead,
  FvbWrite,
  FvbEraseBlocks,
  ///
  /// The handle of the parent firmware volume.
  ///
  NULL
};


#if 0
/**
  Initialize the state information for the CPU Architectural Protocol

  @param  ImageHandle   of the loaded driver
  @param  SystemTable   Pointer to the System Table

  @retval EFI_SUCCESS           Protocol registered
  @retval EFI_OUT_OF_RESOURCES  Cannot allocate protocol data structure
  @retval EFI_DEVICE_ERROR      Hardware problems

**/
EFI_STATUS
EFIAPI
FvbDxeInitialize ()
{
  EFI_STATUS  Status;


  Status = gBS->InstallMultipleProtocolInterfaces (
                  &gFvbHandle,
                  &gEfiFirmwareVolumeBlockProtocolGuid,   &gFvbProtocol,
                  NULL
                  );
  ASSERT_EFI_ERROR (Status);

  // SetVertAddressEvent ()

  // GCD Map NAND as RT

  return Status;
}
#endif
