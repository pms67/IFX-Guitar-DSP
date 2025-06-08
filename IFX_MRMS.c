#include "IFX_MRMS.h"

void IFX_MovingRMS_Init(IFX_MovingRMS *mrms, uint16_t L) {

	if (L > IFX_MOVING_RMS_BUF_LEN_MAX) {
		L = IFX_MOVING_RMS_BUF_LEN_MAX;
	}

	mrms->L = L;
	mrms->invL = 1.0f / ((float) L);

	mrms->count = 0;

	uint16_t index;
	for (index = 0; index < L; index++) {
		mrms->inpSqL[index] = 0.0f;
	}

	mrms->outSq = 0.0f;

}

float IFX_MovingRMS_Update(IFX_MovingRMS *mrms, float inp) {

	float inpSq = inp * inp;

	float inpSqNMinusL = mrms->inpSqL[mrms->count];
	mrms->inpSqL[mrms->count] = inpSq;

	if (mrms->count == (mrms->L - 1)) {
		mrms->count = 0;
	} else {
		mrms->count++;
	}

	mrms->outSq += mrms->invL * (inpSq - inpSqNMinusL);

	return mrms->outSq;

}
