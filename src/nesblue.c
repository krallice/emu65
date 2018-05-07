#include "nesblue.h"

#define DEBUG_NESBLUE 2

#define NESBLUE_CPU_FREQ 2e6 // 2 Mhz
//#define NESBLUE_CPU_FREQ 100 // 100 Hz 
#define STEP_DURATION 10e6 // 10 ms
#define ONE_SECOND 1e9

// NES Structure:
typedef struct nes_t {
	core_t *core;
} nes_t;

nes_t *init_nes() {

	// Allocate and Init our NES containter struct:
	nes_t *nes = (nes_t*)malloc(sizeof(nes_t));
	memset(nes, 0, sizeof(nes_t));

	// Init our individual components:
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
	uint16_t total_cycles = 0;

        while (1) {

		// Execute CPU Instructions:
		for ( nes->core->cyclecount %= cycles_per_step; nes->core->cyclecount < cycles_per_step; ) {
			step_core(nes->core);
			total_cycles += nes->core->cyclecount;
			//printf("Total Cycles: %d\n", total_cycles);
                }

		// Sleep the rest of our interval:
                nes_sleep();
	}
}

void run_functional_test(nes_t *nes) {

	// Evenly spread cycles over the period of a second to achieve NESBLUE_CPU_FREQ:
        uint16_t cycles_per_step = (NESBLUE_CPU_FREQ / (ONE_SECOND / STEP_DURATION));
	uint16_t total_cycles = 0;

        while (1) {

		// Execute CPU Instructions:
		for ( nes->core->cyclecount %= cycles_per_step; nes->core->cyclecount < cycles_per_step; ) {
			step_core(nes->core);
			printf("PC: %.4x\n", nes->core->pc);
			total_cycles += nes->core->cyclecount;
			//printf("Total Cycles: %d\n", total_cycles);
                }

		// Sleep the rest of our interval:
                nes_sleep();
	}
}

void load_functional_test(nes_t *nes) {

	core_t *core = nes->core;

	FILE *fp;
	fp = fopen("./6502_functional_test.bin", "rb");

	// Read one byte at a time:
	uint16_t i = 0;
	while ( fread( &(core->ram[i]), 1, 1, fp) ) {
		#if DEBUG_NESBLUE == 1
		printf("Read Byte: %.2x\n", core->ram[i]);
		#endif
		++i;
	}
	#if DEBUG_NESBLUE == 1
		printf("Done Reading File, Read %d Bytes\n", i);
		printf("Filled up to Memory Address %.4x\n", i);
	#endif

	// Set our PC to the start of the functional test:
	core->pc = 0xC000;

	fclose(fp);
}

void run_nestest(nes_t *nes) {

	// Evenly spread cycles over the period of a second to achieve NESBLUE_CPU_FREQ:
        uint16_t cycles_per_step = (NESBLUE_CPU_FREQ / (ONE_SECOND / STEP_DURATION));
	uint16_t total_cycles = 0;

        while (1) {

		// Execute CPU Instructions:
		for ( nes->core->cyclecount %= cycles_per_step; nes->core->cyclecount < cycles_per_step; ) {
			step_core(nes->core);
			total_cycles += nes->core->cyclecount;
			//printf("Total Cycles: %d\n", total_cycles);
                }

		// Sleep the rest of our interval:
                nes_sleep();
	}
}


void load_nestest(nes_t *nes) {

	core_t *core = nes->core;

	FILE *fp;
	fp = fopen("./nestest.nes", "rb");
	if ( !fp ) {
		return;	
	}

	// Skip first 16Bytes of ROM:
	fseek(fp, 0x10, SEEK_SET);

	uint8_t val = 0x00;
	for (uint16_t i = 0; i < 0x4000; i++) {

		// Read byte and manually mirror (Assumption there is no mirror):
		fread(&val, 1, 1, fp);	
		core->ram[0x8000 + i] = val;
		core->ram[0xC000 + i] = val;
	}

	// Set our PC to the start of the nes test:
	core->pc = 0xC000;

	fclose(fp);
}

int main(void) {

	nes_t *nes = init_nes();

	load_nestest(nes);
	run_nestest(nes);
	
	//execute_nes(nes);
	return 0;
}
