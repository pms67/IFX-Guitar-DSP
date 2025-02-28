/* Mapping linear potentiometer (0...1) to audio taper logarithmic potentiometer (0...1)
 * When lin=0.5, log=0.1
 */

#ifndef IFX_LOG_AUDIO_POT_H
#define IFX_LOG_AUDIO_POT_H

#include <math.h>

#define IFX_LOG_AUDIO_POT_A 0.0125f
#define IFX_LOG_AUDIO_POT_B 81.0f

float IFX_LogAudioPot_Get(float linVal);

#endif
