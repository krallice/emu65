#include "emu65.h"

// Init our machine structure:
static machine_t *init_machine() {

	// Allocate and Init our machine containter struct:
	machine_t *machine = (machine_t*)malloc(sizeof(machine_t));
	memset(machine, 0, sizeof(machine_t));

	// Init our individual components:
	machine->core = init_core();

	return machine;
}

// Sleep for EMU65_STEP_DURATION nanoseconds:
static void machine_sleep(void) {

        struct timespec req, rem;

        req.tv_sec = 0;
        req.tv_nsec = EMU65_STEP_DURATION;

        nanosleep(&req, &rem);
}

static void run_machine(machine_t *machine) {

	// Evenly spread cycles over the period of a second to achieve EMU65_CPU_FREQ:
	long total_cycles = 0;
        int cycles_per_step = (EMU65_CPU_FREQ / (ONE_SECOND / EMU65_STEP_DURATION));

	// machine->core->pc = ( (machine->core->ram[0xFFFC] << 8) + machine->core->ram[0xFFFD]);
	machine->core->pc = ( (machine->core->ram[0xFFFD] << 8) + machine->core->ram[0xFFFC]);
	printf("PC is 0x%04X.\n", machine->core->pc);

	while (1) {

		//printf("Cycles per step: %d.\n", cycles_per_step);
		for ( total_cycles %= cycles_per_step; total_cycles < cycles_per_step; ) {
			step_core(machine->core);
			total_cycles += machine->core->cyclecount;
			//printf("Total Cycles: %ld\n", total_cycles);

		}
			// If our write byte has changed:
			//if ( machine->core->ram[write_addr] != write_val ) {
				//printf("%0x2X ", machine->core->ram[write_addr]);
				//write_val = machine->core->ram[write_addr];
			//}
		machine_sleep();
	}
}

// Load our EH BASIC ROM into our machine's RAM:
static int load_basic_rom(machine_t *machine) {

	core_t *core = machine->core;
	const uint16_t start_load_address = 0xC000;

	FILE *fp = fopen("./ehbasic.bin", "rb");
	if (!fp) {
		fprintf(stderr, "Unable to load EH BASIC ROM. Exiting.\n");
		return 1;
	}

	// Calculate remaining address size:
	const uint16_t max_loadable = 0x10000 - start_load_address;
	printf("Loading EH BASIC into 0x%04X. Maximum Bytes Loadable: 0x%04X.\n", start_load_address, max_loadable);
	uint16_t loaded = fread(&(core->ram[start_load_address]), 1, (size_t)max_loadable, fp);
	if ( loaded < 1 ) {
		return 1;
		fclose(fp);
	}

	printf("Loaded 0x%04X (%d) Bytes into RAM.\n", loaded, loaded);
	core->pc = start_load_address;
	fclose(fp);
	return 0;
}

int main(void) {

	// Create our machine object:
	machine_t *machine = init_machine();
	if ( machine == NULL ) {
		return 1;
	}

	if ( load_basic_rom(machine) != 0 ) {
		return 1;
	}

	run_machine(machine);
	return 0;
}
