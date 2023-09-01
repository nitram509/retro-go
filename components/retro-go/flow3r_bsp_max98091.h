#pragma once

// MAX98091 audio codec support.

#include "stdbool.h"
#include "stdint.h"

void flow3r_bsp_max98091_init(void);
float flow3r_bsp_max98091_headphones_set_volume(bool mute, float dB);
float flow3r_bsp_max98091_speaker_set_volume(bool mute, float dB);
