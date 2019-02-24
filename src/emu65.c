#include "emu65.h"

struct termios term_original_attributes;
struct termios term_read_attributes;

// Callback triggered through every write to memory
// Header defined in callbacks.h:
void emu65_write_callback(const core_t *core, const uint16_t addy) {
	if ( addy == EMU65_WRITE_ADDR ) {
		printf("%c", core->ram[addy]);
		fflush(stdout);
	}
	return;
}

// Read our input from terminal:
static int read_kb_input(void) {

    int character;

    // Set the terminal to RAW mode:
    // Copy orignal attributes across:
    memcpy(&term_read_attributes, &term_original_attributes, sizeof(struct termios));

    term_read_attributes.c_lflag &= ~(ICANON);
    term_read_attributes.c_lflag &= ~(ECHO);
    term_read_attributes.c_cc[VTIME] = 0;
    term_read_attributes.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &term_read_attributes);

    // Do not block, attempt to read from the input STDIN stream.
    // Return -1 if there is no character available:
    character = fgetc(stdin);

    // Restore:
    tcsetattr(fileno(stdin), TCSANOW, &term_original_attributes);

    // Return our read character from the input stream:
    return character;
}

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

        static struct timespec req, rem;

        req.tv_sec = 0;
        req.tv_nsec = EMU65_STEP_DURATION;

        nanosleep(&req, &rem);
}

static void run_machine(machine_t *machine) {

	// Evenly spread cycles over the period of a second to achieve EMU65_CPU_FREQ:
	long total_cycles = 0;
        int cycles_per_step = (EMU65_CPU_FREQ / (ONE_SECOND / EMU65_STEP_DURATION));

	int input_char = -1;

	// Load our PC:
	machine->core->pc = ( (machine->core->ram[0xFFFD] << 8) + machine->core->ram[0xFFFC]);
	printf("PC is 0x%04X. Staring Program Execution.\n", machine->core->pc);

	// Execute:
	while (1) {
		
		// Attempt to capture input if possible:
		if ((input_char = read_kb_input()) != -1) {
			machine->core->ram[EMU65_READ_ADDR] = input_char;
		}

		for ( total_cycles %= cycles_per_step; total_cycles < cycles_per_step; ) {
			// Execute:
			step_core(machine->core);
			total_cycles += machine->core->cyclecount;
			// Reset memory value if we previously entered something:
			//if (input_char != -1) {
				//machine->core->ram[EMU65_READ_ADDR] = 0;
			//}
		}
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

// Save our original terminal attributes (with echo disabled):
static inline void save_original_terminal_attributes(void) {
	tcgetattr(fileno(stdin), &term_original_attributes);
	term_original_attributes.c_lflag &= ~ECHO;
	tcsetattr(fileno(stdin), TCSANOW, &term_original_attributes);
}

int main(void) {

	save_original_terminal_attributes();

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
