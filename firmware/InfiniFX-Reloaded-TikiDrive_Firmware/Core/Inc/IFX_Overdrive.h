/*
*
*   InfiniFX - Overdrive
*
*   Author: Philip Salmony @ phils-lab.net
*
*   Recommended settings:
*   hpfCutoffFrequencyHz = 150.0f;
*	odPreGain = 175.0f;
*	odThreshold = 0.333f;
*	lpfCutoffFrequencyHz = 5000.0f;
*	lpfDamping = 1.0f;
*
*/

#ifndef IFX_OVERDRIVE_H
#define IFX_OVERDRIVE_H

#define _USE_MATH_DEFINES

#include <stdint.h>
#include <math.h>

/* Input low-pass filter (fc = fs / 4), 0.25dB ripple in pass-band, 60dB attenuation at stop-band, 69 taps */
#define IFX_OVERDRIVE_LPF_INP_LENGTH 69
extern float IFX_OD_LPF_INP_COEF[IFX_OVERDRIVE_LPF_INP_LENGTH];


typedef struct {
	/* Sampling time */
	float T;

	/* Input low-pass filter */
	float   lpfInpBuf[IFX_OVERDRIVE_LPF_INP_LENGTH];
	uint8_t lpfInpBufIndex;
	float   lpfInpOut;

	/* Input high-pass filter */
	float hpfInpBufIn[2];
	float hpfInpBufOut[2];
	float hpfInpWcT;
	float hpfInpOut;

	/* Overdrive settings */
	float preGain;
	float boostGain;
	float threshold;

	/* Output low-pass filter */
	float lpfOutBufIn[3];
	float lpfOutBufOut[3];
	float lpfOutWcT;
	float lpfOutDamp;
	float lpfOutOut;

	float out;

	float Q;

} IFX_Overdrive;

void IFX_Overdrive_Init(IFX_Overdrive *od, float samplingFrequencyHz, float hpfCutoffFrequencyHz, float odPreGain, float lpfCutoffFrequencyHz, float lpfDamping);
void IFX_Overdrive_SetGain(IFX_Overdrive *od, float gain);
void IFX_Overdrive_SetHPF(IFX_Overdrive *od, float hpfCutoffFrequencyHz);
void IFX_Overdrive_SetLPF(IFX_Overdrive *od, float lpfCutoffFrequencyHz, float lpfDamping);
float IFX_Overdrive_Update(IFX_Overdrive *od, float inp);

#endif
