#include "IFX_Delay.h"

void IFX_Delay_Init(IFX_Delay *dly, float delayTime_ms, float mix, float feedback, float sampleRate_Hz) {

    // Set delay line length
    IFX_Delay_SetLength(dly, delayTime_ms, sampleRate_Hz);

    // Store delay setting
    dly->mix = mix;
    dly->feedback = feedback;

    // Clear delay line circular buffer, reset index
    dly->lineIndex = 0;

    for (uint32_t n = 0; n < IFX_DELAY_MAX_LINE_LENGTH; n++) {
        dly->line[n] = 0.0f;
    }

    // Clear output
    dly->out = 0.0f;

}

float IFX_Delay_Update(IFX_Delay *dly, float inp) {

	// Get current delay line output
	float delayLineOutput = dly->line[dly->lineIndex];

	// Compute current delay line input
	float delayLineInput  = inp + dly->feedback * delayLineOutput;

    // Store in delay line circular buffer
    dly->line[dly->lineIndex] = delayLineInput;

    // Increment delay line index
    dly->lineIndex++;
    if (dly->lineIndex >= dly->lineLength) {

        dly->lineIndex = 0;

    }

    // Mix dry and wet signals to compute output
    dly->out = (1.0f - dly->mix) * inp + dly->mix * delayLineOutput;

    // Limit output
    if (dly->out > 1.0f) {

        dly->out = 1.0f;

    } else if (dly->out < -1.0f) {

        dly->out = -1.0f;

    }

    // Return current output
    return dly->out;

}

void IFX_Delay_SetLength(IFX_Delay *dly, float delayTime_ms, float sampleRate_Hz) {

    dly->lineLength = (uint32_t) (0.001f * delayTime_ms * sampleRate_Hz);

    if (dly->lineLength > IFX_DELAY_MAX_LINE_LENGTH) {

        dly->lineLength = IFX_DELAY_MAX_LINE_LENGTH;

    }

}
