/*
 * greateyesAPDriver.h
 *
 * Asyn driver that inherits from the asynPortDriver class to demonstrate its use.
 * It simulates a digital scope looking at a 1kHz 1000-point noisy sine wave.  Controls are
 * provided for time/division, volts/division, volt offset, trigger delay, noise amplitude, update time,
 * and run/stop.
 * Readbacks are provides for the waveform data, min, max and mean values.
 *
 * Author: Mark Rivers
 *
 * Created Feb. 5, 2009
 */

#include "asynPortDriver.h"

#define NUM_VERT_SELECTIONS 4

/* These are the drvInfo strings that are used to identify the parameters.
 * They are used by asyn clients, including standard asyn device support */
#define P_RunString                "SCOPE_RUN"                  /* asynInt32,    r/w */
#define P_MaxPointsString          "SCOPE_MAX_POINTS"           /* asynInt32,    r/o */
#define P_TimePerDivString         "SCOPE_TIME_PER_DIV"         /* asynFloat64,  r/w */
#define P_TimePerDivSelectString   "SCOPE_TIME_PER_DIV_SELECT"  /* asynInt32,    r/w */
#define P_VertGainString           "SCOPE_VERT_GAIN"            /* asynFloat64,  r/w */
#define P_VertGainSelectString     "SCOPE_VERT_GAIN_SELECT"     /* asynInt32,    r/w */
#define P_VoltsPerDivString        "SCOPE_VOLTS_PER_DIV"        /* asynFloat64,  r/w */
#define P_VoltsPerDivSelectString  "SCOPE_VOLTS_PER_DIV_SELECT" /* asynInt32,    r/w */
#define P_UpdateTimeString         "SCOPE_UPDATE_TIME"          /* asynFloat64,  r/w */
#define P_WaveformString           "SCOPE_WAVEFORM"             /* asynFloat64Array,  r/o */
#define P_TimeBaseString           "SCOPE_TIME_BASE"            /* asynFloat64Array,  r/o */

#define P_GetCCDTemperatureString  "GET_CCD_TEMPERATURE"     /* asynFloat64,  r/o */
#define P_GetTECTemperatureString  "GET_TEC_TEMPERATURE"     /* asynFloat64,  r/o */
#define P_RunTempControlString     "RUN_TEMP_CONTROL"        /* asynInt32,    r/w */
#define P_SetExposureString        "EXPOSURE_TIME"           /* asynInt32,    r/w */


/** Class that demonstrates the use of the asynPortDriver base class to greatly simplify the task
  * of writing an asyn port driver.
  * This class does a simple simulation of a digital oscilloscope.  It computes a waveform, computes
  * statistics on the waveform, and does callbacks with the statistics and the waveform data itself.
  * I have made the methods of this class public in order to generate doxygen documentation for them,
  * but they should really all be private. */
class greateyesAPDriver : public asynPortDriver {
public:
    greateyesAPDriver(const char *portName, int maxArraySize, const char *ipAddress);	

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus readFloat64(asynUser  *pasynUser, epicsFloat64 *value);
    virtual asynStatus readFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                        size_t nElements, size_t *nIn);
    virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],
                                size_t nElements, size_t *nIn);
   // virtual asynStatus connect(asynUser *pasynUser);
   // virtual asynStatus disconnect(asynUser *pasynUser);
	
    /* These are the methods that are new to this class */
    void gsensorsTask(void);

protected:
    /** Values used for pasynUser->reason, and indexes into the parameter library. */
    int P_Run;
    int P_MaxPoints;
    int P_TimePerDiv;
    int P_TimePerDivSelect;
    int P_VertGain;
    int P_VertGainSelect;
    int P_VoltsPerDiv;
    int P_VoltsPerDivSelect;
    //int P_VoltOffset;
   
    int P_UpdateTime;
    int P_Waveform;
    int P_TimeBase;
    int P_MinValue;
    int P_MaxValue;
    int P_MeanValue;
	
	// GreatEyes
	int P_RunTempControl;
	int P_GetCCDTemperature;
	int P_GetTECTemperature;
	
	int P_SetExposure;


private:
    /* Our data */
    epicsEventId eventId_;
    epicsFloat64 *pData_;
    epicsFloat64 *pTimeBase_;
    // Actual volts per division are these values divided by vertical gain
    char *voltsPerDivStrings_[NUM_VERT_SELECTIONS];
    int voltsPerDivValues_[NUM_VERT_SELECTIONS];
    int voltsPerDivSeverities_[NUM_VERT_SELECTIONS];
    void setVertGain();
    void setVoltsPerDiv();
    void setTimePerDiv();
//static const	int connectionType_USB = 0;
//static const	int connectionType_Ethernet = 3;
	
	// GreatEyes
	//int connectionType;
	int lastStatus;
	int numberOfCamsConnected;
	//bool CCD_TEC_GetTemperature(epicsInt32 thermistor,
	//		epicsInt32 &temperature, epicsInt32 &statusMSG, epicsInt32 addr);
};
