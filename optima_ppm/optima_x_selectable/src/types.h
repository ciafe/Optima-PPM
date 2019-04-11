/*
* Copyright (c) 2012 Scott Driessens
* Licensed under the MIT License
*/

#ifndef __TYPES_H
#define __TYPES_H

typedef struct pin_map {
	uint8_t mask;
	volatile uint8_t *pin;
	volatile uint8_t *port;
	volatile uint8_t *ddr;
} pin_map;

typedef enum
{
    FS_DISABLED = 0x00,
    FS_ENABLED  = 0x01
}Type_FailSafeStates;

#define PPM_ENABLED     1
#define SBUS_ENABLED    2

#define RC_CHANNEL_MIN  990
#define RC_CHANNEL_MAX 2010

typedef void (*output_callback)(void);


	
#endif
