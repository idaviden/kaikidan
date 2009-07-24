@REM
@REM Copyright (c) 2009, Intel Corporation
@REM  Redistribution and use in source and binary forms, with or without
@REM  modification, are permitted provided that the following conditions are
@REM  met:
@REM
@REM   Redistributions of source code must retain the above copyright
@REM   notice, this list of conditions and the following disclaimer.
@REM
@REM   Redistributions in binary form must reproduce the above copyright
@REM   notice, this list of conditions and the following disclaimer in
@REM   the documentation and/or other materials provided with the
@REM   distribution.
@REM
@REM   Neither the name of Intel nor the names of its contributors may
@REM   be used to endorse or promote products derived from this software
@REM   without specific prior written permission.
@REM
@REM  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
@REM  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
@REM  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
@REM  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
@REM  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
@REM  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
@REM  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
@REM  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
@REM  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
@REM  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@REM  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
@REM
@REM  Additional terms: In addition to the forgoing, redistribution and use
@REM  of the code is conditioned upon the FAT 32 File System Driver and all
@REM  derivative works thereof being used for and designed only to read
@REM  and/or write to a file system that is directly managed by an
@REM  Extensible Firmware Interface (EFI) implementation or by an emulator
@REM  of an EFI implementation.

@REM ##############################################################
@REM # You should not have to modify anything below this line
@REM #

@echo off

@REM
@REM Set the WORKSPACE to the current working directory
@REM
set WORKSPACE=%CD%


:check_new_build

@if /I "%1"=="NewBuild" goto NewBuild
@if /I "%1"=="NewReconfig" goto NewReconfig


:NewBuild
@set EDK_TOOLS_PATH=%WORKSPACE%\BaseTools
@call %WORKSPACE%\BaseTools\toolsetup.bat
@goto end

:NewReconfig
@set EDK_TOOLS_PATH=%WORKSPACE%\BaseTools
@call %WORKSPACE%\BaseTools\toolsetup.bat Reconfig
@goto end

:end
@echo on

