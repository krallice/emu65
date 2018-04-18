#ifndef CORE_H
#define CORE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct core_t {

	// 16bit Program Counter:
	uint16_t pc;

	// Accumulator:
	uint8_t a;

	// X & Y Registers:
	uint8_t x;
	uint8_t y;

	// Stack Pointer:
	uint8_t s;

	// Used Flags:
	uint8_t f_carry :1;
	uint8_t f_zero :1;
	uint8_t f_overflow :1;
	uint8_t f_sign :1;

} core_t;

#endif
