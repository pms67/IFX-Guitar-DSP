#ifndef IFX_COMPRESSOR_H
#define IFX_COMPRESSOR_H

#include "math.h"
#include "IFX_MRMS.h"


#define IFX_COMPRESSOR_RMSWINDOW_MS			  50 // 50ms = 1 cycle of 20Hz

#define IFX_COMPRESSOR_THRESHOLD_MIN_DB		-100
#define IFX_COMPRESSOR_THRESHOLD_MAX_DB		   0

#define IFX_COMPRESSOR_KNEEWIDTH_MIN_DB		   0
#define IFX_COMPRESSOR_KNEEWIDTH_MAX_DB		  10

#define IFX_COMPRESSOR_RATIO_MIN			   1
#define IFX_COMPRESSOR_RATIO_MAX		      10

#define IFX_COMPRESSOR_ATTACKTIME_MIN_MS 	   1
#define IFX_COMPRESSOR_ATTACKTIME_MAX_MS 	  50

#define IFX_COMPRESSOR_RELEASETIME_MIN_MS 	   1
#define IFX_COMPRESSOR_RELEASETIME_MAX_MS 	1000

#define IFX_COMPRESOR_MAKEUPGAIN_MIN_DB		   0
#define IFX_COMPRESOR_MAKEUPGAIN_MAX_DB		  20


typedef enum {

	IFX_COMPRESSOR_DETECTORTYPE_INSTANT,
	IFX_COMPRESSOR_DETECTORTYPE_RMS

} IFX_Compressor_DetectorType;

typedef enum {

	IFX_COMPRESSOR_KNEETYPE_SOFT,
	IFX_COMPRESSOR_KNEETYPE_HARD,

} IFX_Compressor_KneeType;


typedef struct {

	IFX_Compressor_DetectorType detectorType;
	IFX_Compressor_KneeType		kneeType;

	IFX_MovingRMS mrms;

	float threshold_dB;
	float kneeWidth_dB;
	float ratio;

	float attackCoeff;
	float releaseCoeff;

	float makeupGain_dB;

	float gainSmoothed_dB;
	float gainFinal_dB;
	float gainFinal_lin;

	float out;

} IFX_Compressor;


void IFX_Compressor_Init(IFX_Compressor *comp,
						IFX_Compressor_DetectorType detectorType, IFX_Compressor_KneeType kneeType,
						float threshold_dB, float kneeWidth_dB, float ratio,
						float attackTime_ms, float releaseTime_ms,
						float makeupGain_dB,
						float sampleRate_Hz);

float IFX_Compressor_Update(IFX_Compressor *comp, float inp);

void IFX_Compressor_SetThreshold(IFX_Compressor *comp, float threshold_dB);
void IFX_Compressor_SetKneeWidth(IFX_Compressor *comp, float kneeWidth_dB);
void IFX_Compressor_SetRatio(IFX_Compressor *comp, float ratio);
void IFX_Compressor_SetAttackTime(IFX_Compressor *comp, float attackTime_ms, float sampleRate_Hz);
void IFX_Compressor_SetReleaseTime(IFX_Compressor *comp, float releaseTime_ms, float sampleRate_Hz);
void IFX_Compressor_SetMakeupGain(IFX_Compressor *comp, float makeupGain_dB);

#endif
