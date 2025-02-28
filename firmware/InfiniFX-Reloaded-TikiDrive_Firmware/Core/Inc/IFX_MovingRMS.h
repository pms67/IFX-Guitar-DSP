#ifndef IFX_MOVING_RMS_H
#define IFX_MOVING_RMS_H

#include <stdint.h>

#define IFX_MOVING_RMS_MAX_BUF 2500

typedef struct {

	uint16_t M;	
	uint16_t count;

	float invM;

	float in_sq_M[IFX_MOVING_RMS_MAX_BUF];
	float out_sq;

} IFX_MovingRMS;

void IFX_MovingRMS_Init(IFX_MovingRMS *mrms, uint16_t M);
float IFX_MovingRMS_Update(IFX_MovingRMS *mrms, float in);

#endif
