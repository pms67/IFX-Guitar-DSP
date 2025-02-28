#include "IFX_Overdrive.h"

float IFX_OD_LPF_INP_COEF[IFX_OVERDRIVE_LPF_INP_LENGTH] = {
		  -0.00020692388031130378,
		  -0.0005449777163912186,
		  -0.0010648637855421347,
		  -0.0016364252077762365,
		  -0.002012399024754339,
		  -0.001863732890917493,
		  -0.000893772118525287,
		  0.0010039250791041001,
		  0.003603402015637201,
		  0.006295503824772816,
		  0.008189064906492259,
		  0.00837592349490335,
		  0.006298513670065863,
		  0.00209212864464293,
		  -0.003255494658311472,
		  -0.008051092706911124,
		  -0.010376200403688028,
		  -0.008792118107324508,
		  -0.003037554751423106,
		  0.005568054109378603,
		  0.014251053145796779,
		  0.019515960562734174,
		  0.01833642159152087,
		  0.009447624551409354,
		  -0.005725703771431606,
		  -0.022919941654740428,
		  -0.03585932375996474,
		  -0.03793377289247158,
		  -0.024280160492278255,
		  0.006380286560875572,
		  0.050788580809754395,
		  0.10148403327365461,
		  0.1484439587786635,
		  0.18160934822751035,
		  0.19357173063571662,
		  0.18160934822751035,
		  0.1484439587786635,
		  0.10148403327365461,
		  0.050788580809754395,
		  0.006380286560875572,
		  -0.024280160492278255,
		  -0.03793377289247158,
		  -0.03585932375996474,
		  -0.022919941654740428,
		  -0.005725703771431606,
		  0.009447624551409354,
		  0.01833642159152087,
		  0.019515960562734174,
		  0.014251053145796779,
		  0.005568054109378603,
		  -0.003037554751423106,
		  -0.008792118107324508,
		  -0.010376200403688028,
		  -0.008051092706911124,
		  -0.003255494658311472,
		  0.00209212864464293,
		  0.006298513670065863,
		  0.00837592349490335,
		  0.008189064906492259,
		  0.006295503824772816,
		  0.003603402015637201,
		  0.0010039250791041001,
		  -0.000893772118525287,
		  -0.001863732890917493,
		  -0.002012399024754339,
		  -0.0016364252077762365,
		  -0.0010648637855421347,
		  -0.0005449777163912186,
		  -0.00020692388031130378
					 };

void IFX_Overdrive_Init(IFX_Overdrive *od, float samplingFrequencyHz, float hpfCutoffFrequencyHz, float odPreGain, float lpfCutoffFrequencyHz, float lpfDamping) {
	/* Sampling time */
	od->T = 1.0f / samplingFrequencyHz;

	/* Input high-pass filter */
	od->hpfInpBufIn[0]  = 0.0f; od->hpfInpBufIn[1]  = 0.0f;
	od->hpfInpBufOut[0] = 0.0f; od->hpfInpBufOut[1] = 0.0f;
	od->hpfInpWcT = 2.0f * M_PI * hpfCutoffFrequencyHz * od->T;
	od->hpfInpOut = 0.0f;

	/* Input low-pass filter */
	for (uint8_t n = 0; n < IFX_OVERDRIVE_LPF_INP_LENGTH; n++) {
		od->lpfInpBuf[n] = 0.0f;
	}
	od->lpfInpBufIndex = 0;
	od->lpfInpOut   = 0.0f;

	/* Overdrive settings */
	od->preGain   = odPreGain;
	od->boostGain = 0.0f;
	od->threshold = 1.0f / 3.0f;

	/* Output low-pass filter */
	od->lpfOutWcT  = 2.0f * M_PI * lpfCutoffFrequencyHz * od->T;
	od->lpfOutDamp = lpfDamping;

	od->Q = -0.2f;
}

void IFX_Overdrive_SetGain(IFX_Overdrive *od, float gain) {

	od->preGain = gain;

}

void IFX_Overdrive_SetHPF(IFX_Overdrive *od, float hpfCutoffFrequencyHz) {

    od->hpfInpWcT = 2.0f * M_PI * hpfCutoffFrequencyHz * od->T;

}

void IFX_Overdrive_SetLPF(IFX_Overdrive *od, float lpfCutoffFrequencyHz, float lpfDamping) {

    od->lpfOutWcT  = 2.0f * M_PI * lpfCutoffFrequencyHz * od->T;
    od->lpfOutDamp = lpfDamping;

}

float IFX_Overdrive_Update(IFX_Overdrive *od, float inp) {
	/* FIR Low-pass filter at fs / 4 as squaring operation will double bandwidth and would otherwise cause aliasing distortion */
	od->lpfInpBuf[od->lpfInpBufIndex] = inp;
	od->lpfInpBufIndex++;

	if (od->lpfInpBufIndex == IFX_OVERDRIVE_LPF_INP_LENGTH) {
		od->lpfInpBufIndex = 0;
	}

	od->lpfInpOut = 0.0f;
	uint8_t index = od->lpfInpBufIndex;
	for (uint8_t n = 0; n < IFX_OVERDRIVE_LPF_INP_LENGTH; n++) {
		if (index == 0) {
			index = IFX_OVERDRIVE_LPF_INP_LENGTH - 1;
		} else {
			index--;
		}

		od->lpfInpOut += IFX_OD_LPF_INP_COEF[n] * od->lpfInpBuf[index];
	}

	/* Variable first-order IIR High-pass filter to remove some of the low frequency components, as these sound muddy when distorted */
	od->hpfInpBufIn[1] = od->hpfInpBufIn[0];
	od->hpfInpBufIn[0] = od->lpfInpOut;

	od->hpfInpBufOut[1] = od->hpfInpBufOut[0];
	od->hpfInpBufOut[0] = (2.0f * (od->hpfInpBufIn[0] - od->hpfInpBufIn[1]) + (2.0f - od->hpfInpWcT) * od->hpfInpBufOut[1]) / (2.0f + od->hpfInpWcT);
	od->hpfInpOut = od->hpfInpBufOut[0];

	/* Overdrive */
/*	float clipIn = (od->preGain + od->boostGain) * od->hpfInpOut;
	float absClipIn  = fabs(clipIn);
	float signClipIn = (clipIn >= 0.0f) ? 1.0f : -1.0f;

	float clipOut = 0.0f;

	if (absClipIn < od->threshold) {
		clipOut = 2.0f * clipIn;
	} else if (absClipIn >= od->threshold && absClipIn < (2.0f * od->threshold)) {
		clipOut = signClipIn * (3.0f - (2.0f - 3.0f * absClipIn) * (2.0f - 3.0f * absClipIn)) / 3.0f;
	} else {
		clipOut = signClipIn;
	}*/

	/* Asymmetrical clipping */
	float xGain = od->preGain * od->hpfInpOut;
	static const float d =  8.0f;

	float clipOut = od->Q / (1.0f - expf(d * od->Q));

	if ((xGain - od->Q) >= 0.00001f) {

		clipOut += (xGain - od->Q) / (1.0f - expf(-d * (xGain - od->Q)));

	}

	/* Variable IIR low-pass filter to remove high frequency components after clipping */
	od->lpfOutBufIn[2] = od->lpfOutBufIn[1];
	od->lpfOutBufIn[1] = od->lpfOutBufIn[0];
	od->lpfOutBufIn[0] = clipOut;

	od->lpfOutBufOut[2] = od->lpfOutBufOut[1];
	od->lpfOutBufOut[1] = od->lpfOutBufOut[0];
	od->lpfOutBufOut[0] = od->lpfOutWcT * od->lpfOutWcT * (od->lpfOutBufIn[0] + 2.0f * od->lpfOutBufIn[1] + od->lpfOutBufIn[2])
						- 2.0f * (od->lpfOutWcT * od->lpfOutWcT - 4.0f) * od->lpfOutBufOut[1]
						- (4.0f - 4.0f * od->lpfOutDamp * od->lpfOutWcT + od->lpfOutWcT * od->lpfOutWcT) * od->lpfOutBufOut[2];
	od->lpfOutBufOut[0] /= (4.0f + 4.0f * od->lpfOutDamp * od->lpfOutWcT + od->lpfOutWcT * od->lpfOutWcT);
	od->lpfOutOut = od->lpfOutBufOut[0];

	/* Limit output */
	od->out = od->lpfOutOut;

	if (od->out > 1.0f) {
		od->out = 1.0f;
	} else if (od->out < -1.0f) {
		od->out = -1.0f;
	}

	return od->out;
}
