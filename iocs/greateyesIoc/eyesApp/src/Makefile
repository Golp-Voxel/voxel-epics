TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Use typed rset structure (see 3.16.1 release notes)
USR_CPPFLAGS += -DUSE_TYPED_RSET

# xxxRecord.h will be created from xxxRecord.dbd
DBDINC += xxxRecord

# Install xxxSupport.dbd into <top>/dbd
DBD += xxxSupport.dbd

# Build an IOC support library
LIBRARY_IOC += eyesSupport

# Compile and add code to the support library
eyesSupport_SRCS += xxxRecord.c
eyesSupport_SRCS += devXxxSoft.c

# Link locally-provided code into the support library,
# rather than directly into the IOC application, that
# causes problems on Windows DLL builds
eyesSupport_SRCS += dbSubExample.c
eyesSupport_SRCS += deveyesVersion.c
eyesSupport_SRCS += eyesHello.c
eyesSupport_SRCS += initTrace.c
#eyesSupport_SRCS += testAsynPortDriver.cpp
eyesSupport_SRCS += greateyesAPDriver.cpp

eyesSupport_LIBS += $(EPICS_BASE_IOC_LIBS)
eyesSupport_LIBS += asyn
eyesSupport_LIBS += greateyes

# Auto-generate a header file containing a version string.
# Version comes from the VCS if available, else date+time.
GENVERSION = eyesVersion.h
# Macro name
GENVERSIONMACRO = eyesVERSION

# Build the IOC application
PROD_IOC = eyes

# eyes.dbd will be created and installed
DBD += eyes.dbd

# eyes.dbd will include these files:
eyes_DBD += base.dbd
eyes_DBD += asyn.dbd
#eyes_DBD += xxxSupport.dbd
#eyes_DBD += dbSubExample.dbd
eyes_DBD += deveyesVersion.dbd
eyes_DBD += eyesHello.dbd
eyes_DBD += initTrace.dbd

# eyes_registerRecordDeviceDriver.cpp derives from eyes.dbd
eyes_SRCS += eyes_registerRecordDeviceDriver.cpp

# Build the main IOC entry point where needed
eyes_SRCS_DEFAULT += eyesMain.cpp
eyes_SRCS_vxWorks += -nil-

# Link in the code from our support library
eyes_LIBS += eyesSupport
eyes_LIBS += asyn
#eyes_LIBS += greateyes

# To build SNL programs, SNCSEQ must be defined
# in the <top>/configure/RELEASE file
ifneq ($(SNCSEQ),)
    # Build sncExample into eyesSupport
    sncExample_SNCFLAGS += +r
    eyes_DBD += sncExample.dbd
    # A .stt sequence program is *not* pre-processed:
    eyesSupport_SRCS += sncExample.stt
    eyesSupport_LIBS += seq pv
    eyes_LIBS += seq pv

    # Build sncProgram as a standalone program
    PROD_HOST += sncProgram
    sncProgram_SNCFLAGS += +m
    # A .st sequence program *is* pre-processed:
    sncProgram_SRCS += sncProgram.st
    sncProgram_LIBS += seq pv
    sncProgram_LIBS += $(EPICS_BASE_HOST_LIBS)
endif

# Link QSRV (pvAccess Server) if available
ifdef EPICS_QSRV_MAJOR_VERSION
    eyes_LIBS += qsrv
    eyes_LIBS += $(EPICS_BASE_PVA_CORE_LIBS)
    eyes_DBD += PVAServerRegister.dbd
    eyes_DBD += qsrv.dbd
endif

# Finally link IOC to the EPICS Base libraries
eyes_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE

# Explicit dependency needed for generated header file
deveyesVersion$(DEP): $(COMMON_DIR)/$(GENVERSION)
