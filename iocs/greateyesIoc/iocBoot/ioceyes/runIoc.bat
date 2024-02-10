@ECHO OFF
set _epics_home=C:\Users\Esther\EPICS
set _epics_support=%_epics_home%\support
set _epics_base=%_epics_home%\base-R7.0.4.1
set _epics_host_arch=windows-x64
SET "PATH=..\..\bin\windows-x64;%_epics_base%\bin\windows-x64;%_epics_support%\sscan\bin\windows-x64;%_epics_support%\calc\bin\windows-x64;%_epics_support%\asyn\bin\windows-x64;%_epics_support%\sscan\bin\windows-x64;%_epics_support%\calc\bin\windows-x64;%_epics_support%\greateyes\bin\windows-x64;%PATH%"
REM SET "PATH=C:\Users\Esther\git-repos\voxel-epics\iocs\greateyesIoc\bin\windows-x64;C:\Users\Esther\EPICS\base-R7.0.4.1/bin/windows-x64;C:\Users\Esther\EPICS\support/sscan/bin/windows-x64;C:\Users\Esther\EPICS\support/calc/bin/windows-x64;C:\Users\Esther\EPICS\support/asyn/bin/windows-x64;C:\Users\Esther\EPICS\support/stream/bin/windows-x64;C:\Users\Esther\EPICS\support/greateyes/bin/windows-x64;%PATH%"

..\..\bin\windows-x64\eyes.exe st.cmd

rem Don't leak variables into the environment
set _epics_home=
