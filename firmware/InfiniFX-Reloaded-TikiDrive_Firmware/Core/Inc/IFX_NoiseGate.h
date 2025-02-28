#ifndef IFX_NOISE_GATE_H
#define IFX_NOISE_GATE_H

#include <math.h>

#include "IFX_MovingRMS.h"

#define IFX_NOISEGATE_RMS_HORIZON_MS 50.0f

typedef struct {
	float thresholdSq;
	float holdTimeS;

	float attackCoeff;
	float releaseCoeff;

	float attackCounter;
	float releaseCounter;

	float smoothedGain;

	float sampleTimeS;

	IFX_MovingRMS mrms;
} IFX_NoiseGate;

void  IFX_NoiseGate_Init(IFX_NoiseGate *ng, float threshold, float attackTimeMs, float releaseTimeMs, float holdTimeMs, float sampleRateHz);
float IFX_NoiseGate_Update(IFX_NoiseGate *ng, float inp);
void  IFX_NoiseGate_SetThreshold(IFX_NoiseGate *ng, float threshold);
void  IFX_NoiseGate_SetAttackReleaseTime(IFX_NoiseGate *ng, float attackTimeMs, float releaseTimeMs, float sampleRateHz);

#endif
