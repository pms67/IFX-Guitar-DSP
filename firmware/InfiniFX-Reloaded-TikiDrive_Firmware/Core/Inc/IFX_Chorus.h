#ifndef IFX_CHORUS_H
#define IFX_CHORUS_H

#include <stdint.h>
#include <math.h>

#define IFX_CHORUS_MAX_DELAY_LENGTH 2048

typedef struct {
    /* Delay lines */
    float delayLineA[IFX_CHORUS_MAX_DELAY_LENGTH];
    float delayLineB[IFX_CHORUS_MAX_DELAY_LENGTH];
    uint16_t delayLineBaseLengthA;
    uint16_t delayLineBaseLengthB;
    uint16_t delayLineLengthA;
    uint16_t delayLineLengthB;
    uint16_t delayLineIndexA;
    uint16_t delayLineIndexB;

    /* Chorus parameters */
    float depthA;
    float depthB;

    float rateA;
    float rateB;

    float gainA;
    float gainB;
    float mix;

    /* Timers for LFOs */
    float sampleTime;
    float timeA;
    float timeB;
    float periodA;
    float periodB;

    /* Chorus output */
    float out;

} IFX_Chorus;

void IFX_Chorus_Init(IFX_Chorus *cho,
                      float delayTimemsA, uint16_t delayTimemsB,
                      float depthA, float depthB,
					  float gainA, float gainB,
                      float rateA, float rateB,
                      float mix,
                      float sampleRateHz);

float IFX_Chorus_Update(IFX_Chorus *cho, float inp);

#endif
