#include "nesblue.h"

#define NESBLUE_CPU_FREQ 2e6 // 2 Mhz
//#define NESBLUE_CPU_FREQ 100 // 100 Hz 
#define STEP_DURATION 10e6 // 10 ms
#define ONE_SECOND 1e9

typedef struct nes_t {
	core_t *core;
} nes_t;

nes_t *init_nes() {

	nes_t *nes = (nes_t*)malloc(sizeof(nes_t));
	memset(nes, 0, sizeof(nes_t));

	nes->core = init_core();

	return nes;
}

// Sleep for STEP_DURATION nanoseconds:
void nes_sleep(void) {
        struct timespec req, rem;
        req.tv_sec = 0;
        req.tv_nsec = STEP_DURATION;
        nanosleep(&req, &rem);
}

void execute_nes(nes_t *nes) {

	// Evenly spread cycles over the period of a second to achieve NESBLUE_CPU_FREQ:
        uint16_t cycles_per_step = (NESBLUE_CPU_FREQ / (ONE_SECOND / STEP_DURATION));

        while (1) {

		// Execute CPU Instructions:
		for ( nes->core->cyclecount %= cycles_per_step; nes->core->cyclecount < cycles_per_step; ) {
			step_core(nes->core);
                }

		// Sleep the rest of our interval:
                nes_sleep();
	}
}

int main(void) {

	nes_t *nes = init_nes();
	core_t *core = nes->core;

	core->ram[0x0000] = LDX_I;
	core->ram[0x0001] = 0x03;

	core->ram[0x0003] = BRK;
	core->ram[0x0004] = JMP_A;
	core->ram[0x0005] = 0x00;
	core->ram[0x0006] = 0x00;

	core->ram[0x3042] = RTI; // Return From Interrupt

	// .data
	core->ram[0xFFFE] = 0x40;
	core->ram[0xFFFF] = 0x30;

	execute_nes(nes);

	return 0;
}
