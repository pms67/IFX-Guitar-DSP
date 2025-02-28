#include "IFX_NoiseGate.h"

void IFX_NoiseGate_Init(IFX_NoiseGate *ng, float threshold, float attackTimeMs, float releaseTimeMs, float holdTimeMs, float sampleRateHz) {
	/* Store settings in struct */
	ng->thresholdSq = threshold * threshold; /* Threshold squared, since "RMS" returns (RMS)^2 value!!!*/
	ng->holdTimeS = 0.001f * holdTimeMs;

	/* Calculate attack and release coefficients */
	IFX_NoiseGate_SetAttackReleaseTime(ng, attackTimeMs, releaseTimeMs, sampleRateHz);

	/* Store sample time */
	ng->sampleTimeS = 1.0f / sampleRateHz;

	/* Reset counters */
	ng->attackCounter  = 0.0f;
	ng->releaseCounter = 0.0f;

	/* Reset smoothed gain value */
	ng->smoothedGain = 0.0f;

	/* Initialise moving RMS filter */
	IFX_MovingRMS_Init(&ng->mrms, (uint16_t) (sampleRateHz * IFX_NOISEGATE_RMS_HORIZON_MS * 0.001f));
}

float IFX_NoiseGate_Update(IFX_NoiseGate *ng, float inp) {

	/* [1] Estimate (RMS)^2 of input */
	float inRMSSq = IFX_MovingRMS_Update(&ng->mrms, inp);

	/* [2] Gain computer (static gain characteristic) */
	float gain = 1.0f;

	if (inRMSSq < ng->thresholdSq) { /* Input is below noise gate threshold, thus gain is zero to let no signal pass */

		gain = 0.0f;

	}

	/* [3] Gain smoothing */
	if (gain <= ng->smoothedGain) { /* ATTACK */

		if (ng->attackCounter > ng->holdTimeS) {

			/* Attack (decreasing gain -> reducing output as noise gate engages) */
			ng->smoothedGain = ng->attackCoeff * ng->smoothedGain + (1.0f - ng->attackCoeff) * gain;

		} else { /* Haven't reached hold time yet, increment counter */

			ng->attackCounter += ng->sampleTimeS;

		}

		/* Reset release counter since we're in attack stage */
		ng->releaseCounter = 0.0f;

	} else if (gain > ng->smoothedGain) { /* RELEASE */

		if (ng->releaseCounter > ng->holdTimeS) { /* Passed hold time, ready to apply release filtering */

			/* Release (increasing gain -> increasing output as noise gate disengages) */
			ng->smoothedGain = ng->releaseCoeff * ng->smoothedGain + (1.0f - ng->releaseCoeff) * gain;

		} else { /* Haven't reached hold time yet, increment counter */

			ng->releaseCounter += ng->sampleTimeS;
		}

		/* Reset attack counter since we're in release stage */
		ng->attackCounter = 0.0f;

	}

	return (inp * ng->smoothedGain);

}

void IFX_NoiseGate_SetThreshold(IFX_NoiseGate *ng, float threshold) {
	ng->thresholdSq = threshold * threshold; /* Threshold squared, since "RMS" returns (RMS)^2 value!!!*/
}

void IFX_NoiseGate_SetAttackReleaseTime(IFX_NoiseGate *ng, float attackTimeMs, float releaseTimeMs, float sampleRateHz) {
	ng->attackCoeff  = expf(-2197.22457734f / (sampleRateHz * attackTimeMs));
	ng->releaseCoeff = expf(-2197.22457734f / (sampleRateHz * releaseTimeMs));
}
