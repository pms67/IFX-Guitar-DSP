#include "IFX_MovingRMS.h"

void IFX_MovingRMS_Init(IFX_MovingRMS *mrms, uint16_t M) {

	mrms->M = M;
	mrms->invM = 1.0f / ( (float) M );
	mrms->count = 0;

	// Clear circular buffer
	for (uint16_t n = 0; n < M; n++) {

		mrms->in_sq_M[n] = 0.0f;

	}

	// Clear output
	mrms->out_sq = 0.0f;

}

/* Returns (RMS)^2 !!! */
float IFX_MovingRMS_Update(IFX_MovingRMS *mrms, float in) {

	// Compute input-squared
	float in_sq = in * in;

	// Store value in circular buffer (for use in M samples)
	mrms->in_sq_M[mrms->count] = in_sq;
	
	// Update circular buffer pointer
	if ( mrms->count == (mrms->M - 1) ) {

		mrms->count = 0;

	} else {

		mrms->count++;

	}	

	// Compute latest output squared using recursive equation
	mrms->out_sq = mrms->out_sq + mrms->invM * (in_sq - mrms->in_sq_M[mrms->count]);

	// Return SQUARE!!! of RMS
	return mrms->out_sq;

}
