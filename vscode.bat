rem Location of Nordic SDK
set NRF_SDK=C:/Users/u0139004/Box/NOMADe/SOFTWARE/nRF5_SDK_17.0.2_d674dde

rem Location of Nordic Command Line tools (nrfjprog) 
set NRF_TOOLS=C:/Program Files (x86)/Nordic Semiconductor/nrf-command-line-tools/bin

rem location of GCC Cross-compiler https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
set GNU_GCC=C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2020-q4-major/bin

rem Location of Gnu Tools (make) https://github.com/gnu-mcu-eclipse/windows-build-tools/releases
set GNU_TOOLS=C:/Users/u0139004/Box/NOMADe/SOFTWARE/nRF5_SDK_17.0.2_d674dde/xpack-windows-build-tools-4.2.1-2/bin

rem Location of SEGGER JLink tools
set SEGGER_TOOLS=C:/Program Files (x86)/SEGGER/JLink

rem Location of java
set JAVA=C:/Program Files/Java/jre1.8.0_251/bin/java.exe

rem Serial numbers of nRF development boards
set PCA10056_SN=000000000
set PCA10040_SN=682864789

start "C:/Users/u0139004/AppData/Local/Programs/Microsoft VS Code/Code.exe" workspace.code-workspace
exit