TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS BELOW HERE

# Build the IOC application
PROD_IOC = txmOpticsApp

# txmOpticsApp.dbd will be created and installed
DBD += txmOpticsApp.dbd

# test.dbd will include these files:
txmOpticsApp_DBD += base.dbd
txmOpticsApp_DBD += busySupport.dbd
txmOpticsApp_DBD += asSupport.dbd

# txmOpticsApp_registerRecordDeviceDriver.cpp derives from test.dbd
txmOpticsApp_SRCS += txmOpticsApp_registerRecordDeviceDriver.cpp

# Build the main IOC entry point where needed
txmOpticsApp_SRCS_DEFAULT += txmOpticsMain.cpp
txmOpticsApp_SRCS_vxWorks += -nil-

txmOpticsApp_LIBS += autosave
txmOpticsApp_LIBS += busy
#txmOpticsApp_LIBS += asyn

# Finally link IOC to the EPICS Base libraries
txmOpticsApp_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD EXTRA GNUMAKE RULES BELOW HERE
