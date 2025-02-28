#include "IFX_DelayLine.h"

void IFX_DelayLine_Init(IFX_DelayLine *dlyLn, float delayTime_ms, float sampleRate_Hz) {

    // Set delay line length
    IFX_DelayLine_SetLength(dlyLn, delayTime_ms, sampleRate_Hz);


    // Clear delay line circular buffer, reset index
    dlyLn->index = 0;

    for (uint32_t n = 0; n < IFX_DELAYLINE_MAXLENGTH; n++) {
        dlyLn->memory[n] = 0.0f;
    }

}

float IFX_DelayLine_Update(IFX_DelayLine *dlyLn, float inp) {

	// Get current delay line output
	float out = dlyLn->memory[dlyLn->index];

    // Store current input in delay line circular buffer
    dlyLn->memory[dlyLn->index] = inp;

    // Increment delay line index
    dlyLn->index++;
    if (dlyLn->index >= dlyLn->length) {

        dlyLn->index = 0;

    }

    // Return current output
    return out;

}

void IFX_DelayLine_SetLength(IFX_DelayLine *dlyLn, float delayTime_ms, float sampleRate_Hz) {

    dlyLn->length = (uint32_t) (0.001f * delayTime_ms * sampleRate_Hz);

    if (dlyLn->length > IFX_DELAYLINE_MAXLENGTH) {

        dlyLn->length = IFX_DELAYLINE_MAXLENGTH;

    }

}
