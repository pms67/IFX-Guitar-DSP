#include "IFX_PeakingFilter.h"

void IFX_PeakingFilter_Init(IFX_PeakingFilter *filt, float sampleRate_Hz) {

	/* Compute sample time */
	filt->sampleTime_s = 1.0f / sampleRate_Hz;
	
	/* Clear filter memory */
	for (uint8_t n = 0; n < 3; n++) {
		
		filt->x[n] = 0.0f;
		filt->y[n] = 0.0f;
		
	}
	
	/* Calculate 'default' filter coefficients (all-pass) */
	IFX_PeakingFilter_SetParameters(filt, 1.0f, 0.0f, 1.0f);
	
}

/* Compute filter coefficients. If boostCut_linear > 1.0f -> boost, if boostCut_linear < 1.0f -> cut */
void IFX_PeakingFilter_SetParameters(IFX_PeakingFilter *filt, float centerFrequency_Hz, float bandwidth_Hz, float boostCut_linear) {
	
	/* Convert Hz to rad/s (rps), pre-warp cut-off frequency (bilinear transform), multiply by sampling time (wc*T = 2/T * tan(2*pi*fc*T/2) * T = 2*tan(pi*fc*T) */
	float wcT = 2.0f * tanf( M_PI * centerFrequency_Hz * filt->sampleTime_s);
	
	/* Compute quality factor (Q = f(Bandwidth) / f(Center)) */
	float Q = bandwidth_Hz / centerFrequency_Hz;
	
	/* Compute filter coefficients */
	filt->a[0] = 4.0f + 2.0f * boostCut_linear * Q * wcT + wcT * wcT;
	filt->a[1] = 2.0f * wcT * wcT - 8.0f;
	filt->a[2] = 4.0f - 2.0f * boostCut_linear * Q * wcT + wcT * wcT;
	
	filt->b[0] = 1.0f / (4.0f + 2.0f * Q * wcT + wcT * wcT); 	/* Note: this is 1/coefficient! */
	filt->b[1] = -(2.0f * wcT * wcT - 8.0f);					/* Note: this is -coefficient! */	
	filt->b[2] = -(4.0f - 2.0f * Q * wcT + wcT * wcT);			/* Note: this is -coefficient! */
	
}

float IFX_PeakingFilter_Update(IFX_PeakingFilter *filt, float in) {
	
	/* Shift samples */
	filt->x[2] = filt->x[1];
	filt->x[1] = filt->x[0];
	filt->x[0] = in;
	
	filt->y[2] = filt->y[1];
	filt->y[1] = filt->y[0];
	
	/* Compute new filter output */
	filt->y[0] = (filt->a[0] * filt->x[0] + filt->a[1] * filt->x[1] + filt->a[2] * filt->x[2]
			   + 						   (filt->b[1] * filt->y[1] + filt->b[2] * filt->y[2])) * filt->b[0];
			   
	/* Return current output sample */
	return (filt->y[0]);
	
}