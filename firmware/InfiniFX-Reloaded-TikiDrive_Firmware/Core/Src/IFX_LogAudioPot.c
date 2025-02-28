#include "IFX_LogAudioPot.h"

float IFX_LogAudioPot_Get(float linVal) {

	/* Clamp pot value (0...1) */
	if (linVal > 1.0f) {
		linVal = 1.0f;
	} else if (linVal < 0.0f) {
		linVal = 0.0f;
	}

	/* Compute logarithmically adjusted value */
	float logVal = IFX_LOG_AUDIO_POT_A * (powf(IFX_LOG_AUDIO_POT_B, linVal) - 1.0f);

	/* Return log pot "output" */
	return logVal;

}
