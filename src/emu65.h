#ifndef EMU65_H
#define EMU65_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <unistd.h>
#include <time.h>

#include <termios.h>

#include "core.h"

// #define EMU65_CPU_FREQ 2e6 // 2 Mhz
// #define EMU65_CPU_FREQ 2500 // 2.5 KHZ
#define EMU65_CPU_FREQ 1500 // 1.5 KHZ
#define EMU65_STEP_DURATION 10e6 // 10 ms
#define ONE_SECOND 1e9

#define EMU65_WRITE_ADDR 0xF001
#define EMU65_READ_ADDR 0xF004

typedef struct machine_t {
	core_t *core;
} machine_t;

#endif
