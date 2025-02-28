#include "IFX_Chorus.h"

void IFX_Chorus_Init(IFX_Chorus *cho,
                      float delayTimemsA, uint16_t delayTimemsB,
                      float depthA, float depthB,
					  float gainA, float gainB,
                      float rateA, float rateB,
                      float mix,
                      float sampleRateHz) {

    /* Store chorus parameters */
    cho->depthA = depthA;
    cho->depthB = depthB;

    cho->rateA = rateA;
    cho->rateB = rateB;

    cho->periodA = 1.0f / rateA;
    cho->periodB = 1.0f / rateB;

    cho->mix = mix;

    cho->sampleTime = 1.0f / sampleRateHz;

    /* Reset delay lines */
    for (uint16_t n = 0; n < IFX_CHORUS_MAX_DELAY_LENGTH; n++) {
        cho->delayLineA[n] = 0.0f;
        cho->delayLineB[n] = 0.0f;
    }

    cho->delayLineBaseLengthA = (uint16_t) (0.001f * delayTimemsA * sampleRateHz);
    cho->delayLineBaseLengthB = (uint16_t) (0.001f * delayTimemsB * sampleRateHz);

    cho->delayLineIndexA = 0;
    cho->delayLineIndexB = 0;

    /* Reset LFO timers */
    cho->timeA = 0.0f;
    cho->timeB = 0.0f;

    /* Clear output */
    cho->out = 0.0f;

}

float IFX_Chorus_Update(IFX_Chorus *cho, float inp) {

    /* Update delay lines */
    cho->delayLineA[cho->delayLineIndexA] = inp;
    cho->delayLineB[cho->delayLineIndexB] = inp;

    cho->delayLineIndexA++;
    cho->delayLineIndexB++;

    /* Get LFO output and modulate delay line lengths */
    int16_t lfoA = (int16_t) (cho->depthA * sinf(6.28318530718f * cho->rateA * cho->timeA));
    int16_t lfoB = (int16_t) (cho->depthB * sinf(6.28318530718f * cho->rateB * cho->timeB));

    cho->delayLineLengthA = (uint16_t) (cho->delayLineBaseLengthA + lfoA);
    cho->delayLineLengthB = (uint16_t) (cho->delayLineBaseLengthB + lfoB);

    if (cho->delayLineIndexA >= cho->delayLineLengthA) {
        cho->delayLineIndexA = 0;
    }

    if (cho->delayLineIndexB >= cho->delayLineLengthB) {
        cho->delayLineIndexB = 0;
    }

    /* Update LFO timers */
    cho->timeA += cho->sampleTime;
    cho->timeB += cho->sampleTime;

    if (cho->timeA >= cho->periodA) {
        cho->timeA -= cho->periodA;
    }

    if (cho->timeB >= cho->periodB) {
        cho->timeB -= cho->periodB;
    }

    /* Sum delay line outputs and mix with dry signal */
    cho->out = (1.0f - cho->mix) * inp + cho->mix * (cho->gainA * cho->delayLineA[cho->delayLineIndexA] + cho->gainB * cho->delayLineB[cho->delayLineIndexB]);

    /* Clamp output */
    if (cho-> out > 1.0f) {
        cho->out = 1.0f;
    } else if (cho->out < -1.0f) {
        cho->out = -1.0f;
    }

    /* Return output */
    return cho->out;

}
