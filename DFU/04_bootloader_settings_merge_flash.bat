

@echo off

:: # Check to make sure nrfutil and mergehex and nrfjprog are installed before moving on
WHERE >nul 2>nul nrfutil
IF %ERRORLEVEL% NEQ 0 (
ECHO "nrfutil was not found in PATH, please install using pip install"
goto :end
)
WHERE >nul 2>nul nrfjprog
IF %ERRORLEVEL% NEQ 0 (
ECHO "nrfjprog was not found in PATH, please install using windows installer from nordicsemi.com"
goto :end
)
WHERE >nul 2>nul mergehex
IF %ERRORLEVEL% NEQ 0 (
ECHO "mergehex was not found in PATH, please install using windows installer from nordicsemi.com"
goto :end
)

SET APPLICATION_HEX=nrf52832_xxaa.hex

echo "## Looking to make sure %APPLICATION_HEX% is present in folder"
if not exist %APPLICATION_HEX% (
echo "#### nrf52832_xxaa.hex file does not exist! Please copy a application .hex file into the folder, rename it, and try again!"
goto :end
)
echo.

echo "## Creating bootloader settings based on nrf52832_xxaa.hex"
nrfutil settings generate --family NRF52 --application nrf52832_xxaa.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 settings.hex
echo.

echo "## Merging bootloader settings and nrf52832_xxaa.hex into merged.hex"
mergehex -m nrf52832_xxaa.hex settings.hex -o merged.hex
echo.

nrfjprog --program merged.hex --sectorerase

:end
pause