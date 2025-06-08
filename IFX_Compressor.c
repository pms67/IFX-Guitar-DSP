#include "IFX_Compressor.h"

void IFX_Compressor_Init(IFX_Compressor *comp, IFX_Compressor_DetectorType detectorType, IFX_Compressor_KneeType kneeType, float threshold_dB, float kneeWidth_dB, float ratio, float attackTime_ms, float releaseTime_ms, float makeupGain_dB, float sampleRate_Hz) {

	IFX_MovingRMS_Init(&(comp->mrms), (uint16_t) (0.001f * IFX_COMPRESSOR_RMSWINDOW_MS * sampleRate_Hz));

	if (detectorType > IFX_COMPRESSOR_DETECTORTYPE_RMS) {
		detectorType = IFX_COMPRESSOR_DETECTORTYPE_RMS;
	}
	comp->detectorType	= detectorType;

	if (kneeType > IFX_COMPRESSOR_KNEETYPE_HARD) {
		kneeType = IFX_COMPRESSOR_KNEETYPE_HARD;
	}
	comp->kneeType = kneeType;


	IFX_Compressor_SetThreshold(comp, threshold_dB);
	IFX_Compressor_SetKneeWidth(comp, kneeWidth_dB);
	IFX_Compressor_SetRatio(comp, ratio);

	IFX_Compressor_SetAttackTime(comp, attackTime_ms, sampleRate_Hz);
	IFX_Compressor_SetReleaseTime(comp, releaseTime_ms, sampleRate_Hz);

	IFX_Compressor_SetMakeupGain(comp, makeupGain_dB);

	comp->gainSmoothed_dB = 0.0f;

}

float IFX_Compressor_Update(IFX_Compressor *comp, float inp) {

	/*
	 * ENVELOPE DETECTOR
	 */
	float inp_dB;

	if (comp->detectorType == IFX_COMPRESSOR_DETECTORTYPE_INSTANT) {

		inp_dB = 20.0f * log10f(fabsf(inp) + 1e-12);

	} else {

		inp_dB = 20.0f * log10f(IFX_MovingRMS_Update(&(comp->mrms), inp) + 1e-12);

	}


	/*
	 * GAIN COMPUTER
	 */
	float outStatic_dB = 0.0f;

	// Soft knee
	if (comp->kneeType == IFX_COMPRESSOR_KNEETYPE_SOFT) {

		if (inp_dB < (comp->threshold_dB - 0.5f * comp->kneeWidth_dB)) {

			outStatic_dB = inp_dB;

		} else if ((inp_dB >= (comp->threshold_dB - 0.5f * comp->kneeWidth_dB)) && (inp_dB <= (comp->threshold_dB + 0.5f * comp->kneeWidth_dB))) {

			outStatic_dB = inp_dB + ((1.0f / comp->ratio - 1.0f) * (inp_dB - comp->threshold_dB + 0.5f * comp->kneeWidth_dB) * (inp_dB - comp->threshold_dB + 0.5f * comp->kneeWidth_dB)) / (2.0f * comp->kneeWidth_dB);

		} else {

			outStatic_dB = comp->threshold_dB + (inp_dB - comp->threshold_dB) / comp->ratio;

		}

	} else { // Hard knee

		if (inp_dB < comp->threshold_dB) {

			outStatic_dB = inp_dB;

		} else {

			outStatic_dB = comp->threshold_dB + (inp_dB - comp->threshold_dB) / comp->ratio;

		}

	}

	// Compute gain
	float gain_dB = outStatic_dB - inp_dB;


	/*
	 * GAIN SMOOTHING
	 */
	if (gain_dB <= comp->gainSmoothed_dB) {

		// Attack
		comp->gainSmoothed_dB = comp->attackCoeff  * comp->gainSmoothed_dB + (1.0f - comp->attackCoeff)  * gain_dB;

	} else {

		// Release
		comp->gainSmoothed_dB = comp->releaseCoeff * comp->gainSmoothed_dB + (1.0f - comp->releaseCoeff) * gain_dB;

	}


	/*
	 * MAKE-UP GAIN
	 */
	comp->gainFinal_dB = comp->gainSmoothed_dB + comp->makeupGain_dB;


	/*
	 * APPLY GAIN
	 */
	// Convert to linear
	comp->gainFinal_lin = powf(10.0f, 0.05f * comp->gainFinal_dB);

	// Clamp
	if (comp->gainFinal_lin > 1.0f) {
		comp->gainFinal_lin = 1.0f;
	}

	// Apply to input
	comp->out = comp->gainFinal_lin * inp;

	return comp->out;

}

void IFX_Compressor_SetThreshold(IFX_Compressor *comp, float threshold_dB) {

	if (threshold_dB > IFX_COMPRESSOR_THRESHOLD_MAX_DB) {
		threshold_dB = IFX_COMPRESSOR_THRESHOLD_MAX_DB;
	} else if (threshold_dB < IFX_COMPRESSOR_THRESHOLD_MIN_DB) {
		threshold_dB = IFX_COMPRESSOR_THRESHOLD_MIN_DB;
	}

	comp->threshold_dB = threshold_dB;

}

void IFX_Compressor_SetKneeWidth(IFX_Compressor *comp, float kneeWidth_dB) {

	if (kneeWidth_dB > IFX_COMPRESSOR_KNEEWIDTH_MAX_DB) {
		kneeWidth_dB = IFX_COMPRESSOR_KNEEWIDTH_MAX_DB;
	} else if (kneeWidth_dB < IFX_COMPRESSOR_KNEEWIDTH_MIN_DB) {
		kneeWidth_dB = IFX_COMPRESSOR_KNEEWIDTH_MIN_DB;
	}

	comp->kneeWidth_dB = kneeWidth_dB;

}

void IFX_Compressor_SetRatio(IFX_Compressor *comp, float ratio) {

	if (ratio > IFX_COMPRESSOR_RATIO_MAX) {
		ratio = IFX_COMPRESSOR_RATIO_MAX;
	} else if (ratio < IFX_COMPRESSOR_RATIO_MIN) {
		ratio = IFX_COMPRESSOR_RATIO_MIN;
	}

	comp->ratio	= ratio;

}

void IFX_Compressor_SetAttackTime(IFX_Compressor *comp, float attackTime_ms, float sampleRate_Hz) {

	if (attackTime_ms > IFX_COMPRESSOR_ATTACKTIME_MAX_MS) {
		attackTime_ms = IFX_COMPRESSOR_ATTACKTIME_MAX_MS;
	} else if (attackTime_ms < IFX_COMPRESSOR_ATTACKTIME_MIN_MS) {
		attackTime_ms = IFX_COMPRESSOR_ATTACKTIME_MIN_MS;
	}

	comp->attackCoeff = expf(-2.2f / (0.001f * attackTime_ms * sampleRate_Hz));

}

void IFX_Compressor_SetReleaseTime(IFX_Compressor *comp, float releaseTime_ms, float sampleRate_Hz) {

	if (releaseTime_ms > IFX_COMPRESSOR_RELEASETIME_MAX_MS) {
		releaseTime_ms = IFX_COMPRESSOR_RELEASETIME_MAX_MS;
	} else if (releaseTime_ms < IFX_COMPRESSOR_RELEASETIME_MIN_MS) {
		releaseTime_ms = IFX_COMPRESSOR_RELEASETIME_MIN_MS;
	}

	comp->releaseCoeff = expf(-2.2f / (0.001f * releaseTime_ms * sampleRate_Hz));

}

void IFX_Compressor_SetMakeupGain(IFX_Compressor *comp, float makeupGain_dB) {

	if (makeupGain_dB > IFX_COMPRESOR_MAKEUPGAIN_MAX_DB) {
		makeupGain_dB = IFX_COMPRESOR_MAKEUPGAIN_MAX_DB;
	} else if (makeupGain_dB < IFX_COMPRESOR_MAKEUPGAIN_MIN_DB) {
		makeupGain_dB = IFX_COMPRESOR_MAKEUPGAIN_MIN_DB;
	}

	comp->makeupGain_dB = makeupGain_dB;

}
