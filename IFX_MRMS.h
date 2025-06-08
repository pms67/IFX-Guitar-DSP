#ifndef MOVING_RMS_H
#define MOVING_RMS_H

#include <stdint.h>

#define IFX_MOVING_RMS_BUF_LEN_MAX 2400

typedef struct {

	uint16_t L;
	float invL;

	uint16_t count;
	float inpSqL[IFX_MOVING_RMS_BUF_LEN_MAX];

	float outSq;

} IFX_MovingRMS;

void  IFX_MovingRMS_Init(IFX_MovingRMS *mrms, uint16_t L);
float IFX_MovingRMS_Update(IFX_MovingRMS *mrms, float inp);


#endif
