/*
* Copyright (c) 2012 Scott Driessens
* Licensed under the MIT License
*/

#ifndef __OPTIMA_H
#define __OPTIMA_H

#include <stdint.h>
#include "types.h"

/* Define the optima receiver */
#define OPTIMA_PINS    7
#define PPM_OUT_PIN    OPTIMA_PINS-1

#if defined(SBUS_PPM_SEL_MIXED_FS_ENABLED)
#define MODE_SEL_PIN   OPTIMA_PINS-2
#define PWM_PINS       MODE_SEL_PIN
#endif

#define PWM_PINS       PPM_OUT_PIN


/* number of PPM channels */
#define PPM_CHANNELS	      8

/* Always 9 channels unless Hitec decides to change their protocol */
#define OPTIMA_NUM_CHANNELS	9

extern pin_map outputs[OPTIMA_NUM_CHANNELS];

/* OPTIMA serial protocol IDs */
#define OPTIMA_PROTOCOL_START  0xFF
#define OPTIMA_PROTOCOL_DATA   0xFF
#define OPTIMA_PROTOCOL_FS_EN  0xDD
#define OPTIMA_PROTOCOL_FS_DIS 0xD5
#define OPTIMA_PROTOCOL_END    0xEE



#endif
