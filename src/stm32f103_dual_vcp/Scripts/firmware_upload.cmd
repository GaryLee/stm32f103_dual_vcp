@echo off
REM Upload firmware to MCU through Jlink tool.

set DEVICE=STM32F103C8
set JLINK="C:\Program Files (x86)\SEGGER\JLink_V632h\jlink.exe"
set INTF=SWD
set SPD=4000

%JLINK% -device %DEVICE% -if %INTF% -speed %SPD% -autoconnect 1 -commandfile jlinkcmd.txt

@echo on 