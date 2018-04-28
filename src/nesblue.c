#include "nesblue.h"

#define NESBLUE_CPU_FREQ 2e6 // 2 Mhz
#define STEP_DURATION 10e6 // 10 ms
#define ONE_SECOND 1e9

// Sleep for STEP_DURATION nanoseconds:
void nesblue_sleep(void) {
        struct timespec req, rem;
        req.tv_sec = 0;
        req.tv_nsec = STEP_DURATION;
        nanosleep(&req, &rem);
}

// Neslite execution cycle:
void run_nesblue(core_t *core) {

	// Evenly spread cycles over the period of a second to achieve NESBLUE_CPU_FREQ:
        unsigned int cycles_per_step = (NESBLUE_CPU_FREQ / (ONE_SECOND / STEP_DURATION));
        while (1) {

		// Execute CPU Instructions:
		for ( core->cyclecount %= cycles_per_step; core->cyclecount < cycles_per_step; ) {
			step_core(core);
                }

		// Sleep the rest of our interval:
                nesblue_sleep();
	}
}

int main(void) {

	core_t *core = init_core();

	core->ram[0x0000] = LDX_I;
	core->ram[0x0001] = 0x03;
	//core->ram[0x0002] = 0x22;

	core->ram[0x0003] = BRK;
	core->ram[0x0004] = JMP_A;
	core->ram[0x0005] = 0x00;
	core->ram[0x0006] = 0x00;

	//core->ram[0x3040] = 0x22; 
	//core->ram[0x3041] = 0x22; 
	core->ram[0x3042] = RTI; // Return From Interrupt

	// .data
	core->ram[0xFFFE] = 0x40;
	core->ram[0xFFFF] = 0x30;

	run_nesblue(core);

	dump_core_state(core);
	return 0;
}
