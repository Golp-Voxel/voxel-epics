#!../../bin/windows-x64/eyes

#- You may have to change eyes to something else
#- everywhere it appears in this file

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/eyes.dbd"
eyes_registerRecordDeviceDriver pdbbase

## Load record instances
#dbLoadTemplate "db/user.substitutions"
#dbLoadRecords "db/eyesVersion.db", "user=Voxel"


#dbLoadRecords "db/dbExample1.db", "user=Voxel"
# Turn on asynTraceFlow and asynTraceError for global trace, i.e. no connected asynUser.
#asynSetTraceMask("", 0, 17)

#connectionType_USB = 0
#connectionType_Ethernet = 3
greateyesAPDriverConfigure("testDLL", 0, "192.168.1.234")

dbLoadRecords("db/greateyes.db","P=Voxel:,R=great:,PORT=testDLL,ADDR=0,TIMEOUT=1,NPOINTS=1000")
dbLoadRecords("${ASYN}/db/asynRecord.db","P=Voxel:,R=asyn1,PORT=testDLL,ADDR=0,OMAX=80,IMAX=80")
#asynSetTraceMask("testDLL",0,0xff)
asynSetTraceIOMask("testDLL",0,0x2)

#- Set this to see messages from mySub
#var mySubDebug 1

#- Run this to trace the stages of iocInit
#traceIocInit

cd "${TOP}/iocBoot/${IOC}"
iocInit

## Start any sequence programs
#seq sncExample, "user=Voxel"
