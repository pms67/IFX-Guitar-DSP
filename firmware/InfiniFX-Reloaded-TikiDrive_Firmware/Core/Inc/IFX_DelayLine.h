#ifndef IFX_DELAYLINE_H
#define IFX_DELAYLINE_H

#include <stdint.h>

// Pre-defined maximum delay line length
#define IFX_DELAYLINE_MAXLENGTH 32500

typedef struct {

	// Delay line length
	uint32_t length;

	// Delay line circular buffer
	uint32_t index;
	float memory[IFX_DELAYLINE_MAXLENGTH];

} IFX_DelayLine;

void   IFX_DelayLine_Init(IFX_DelayLine *dlyLn, float delayTime_ms, float sampleRate_Hz);
float  IFX_DelayLine_Update(IFX_DelayLine *dlyLn, float inp);
void   IFX_DelayLine_SetLength(IFX_DelayLine *dlyLn, float delayTime_ms, float sampleRate_Hz);

#endif
