#include "nesblue.h"

#define NESBLUE_CPU_FREQ 2e6 // 2 Mhz
//#define NESBLUE_CPU_FREQ 100 // 100 Hz 
#define STEP_DURATION 10e6 // 10 ms
#define ONE_SECOND 1e9

char nestestnames[256][4] = {
    //FUT represents unimplemented op codes
    "BRK", "ORA", "NOP", "NOP", "NOP", "ORA", "ASL", "NOP",
    "PHP", "ORA", "ASL", "NOP", "NOP", "ORA", "ASL", "NOP",
    "BPL", "ORA", "NOP", "NOP", "NOP", "ORA", "ASL", "NOP",
    "CLC", "ORA", "NOP", "NOP", "NOP", "ORA", "ASL", "NOP",
    "JSR", "AND", "NOP", "NOP", "BIT", "AND", "ROL", "NOP",
    "PLP", "AND", "ROL", "NOP", "BIT", "AND", "ROL", "NOP",
    "BMI", "AND", "NOP", "NOP", "NOP", "AND", "ROL", "NOP",
    "SEC", "AND", "NOP", "NOP", "NOP", "AND", "ROL", "NOP",
    "RTI", "EOR", "NOP", "NOP", "NOP", "EOR", "LSR", "NOP",
    "PHA", "EOR", "LSR", "NOP", "JMP", "EOR", "LSR", "NOP",
    "BVC", "EOR", "NOP", "NOP", "NOP", "EOR", "LSR", "NOP",
    "CLI", "EOR", "NOP", "NOP", "NOP", "EOR", "LSR", "NOP",
    "RTS", "ADC", "NOP", "NOP", "NOP", "ADC", "ROR", "NOP",
    "PLA", "ADC", "ROR", "NOP", "JMP", "ADC", "ROR", "NOP",
    "BVS", "ADC", "NOP", "NOP", "NOP", "ADC", "ROR", "NOP",
    "SEI", "ADC", "NOP", "NOP", "NOP", "ADC", "ROR", "NOP",
    "NOP", "STA", "NOP", "NOP", "STY", "STA", "STX", "NOP",
    "DEY", "NOP", "TXA", "NOP", "STY", "STA", "STX", "NOP",
    "BCC", "STA", "NOP", "NOP", "STY", "STA", "STX", "NOP",
    "TYA", "STA", "TXS", "NOP", "NOP", "STA", "NOP", "NOP",
    "LDY", "LDA", "LDX", "NOP", "LDY", "LDA", "LDX", "NOP",
    "TAY", "LDA", "TAX", "NOP", "LDY", "LDA", "LDX", "NOP",
    "BCS", "LDA", "NOP", "NOP", "LDY", "LDA", "LDX", "NOP",
    "CLV", "LDA", "TSX", "NOP", "LDY", "LDA", "LDX", "NOP",
    "CPY", "CMP", "NOP", "NOP", "CPY", "CMP", "DEC", "NOP",
    "INY", "CMP", "DEX", "NOP", "CPY", "CMP", "DEC", "NOP",
    "BNE", "CMP", "NOP", "NOP", "NOP", "CMP", "DEC", "NOP",
    "CLD", "CMP", "NOP", "NOP", "NOP", "CMP", "DEC", "NOP",
    "CPX", "SBC", "NOP", "NOP", "CPX", "SBC", "INC", "NOP",
    "INX", "SBC", "NOP", "NOP", "CPX", "SBC", "INC", "NOP",
    "BEQ", "SBC", "NOP", "NOP", "NOP", "SBC", "INC", "NOP",
    "SED", "SBC", "NOP", "NOP", "NOP", "SBC", "INC", "NOP"
};


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
		++i;
	}

	// Set our PC to the start of the functional test:
	core->pc = 0x0400;

	fclose(fp);
}

void print_nestest_diag(nes_t *nes) {

	core_t *core = nes->core;

	// Debug output:
	printf("%.4X  ", core->d_pc);
	printf("%.2X ", core->d_op);

	// First Operand:
	if ( core->d_op1_en != 0 ) {
		printf("%.2X ", core->d_op1);
	} else {
		printf("   ");
	}

	// Second Operand:
	if ( core->d_op2_en != 0 ) {
		printf("%.2X  ", core->d_op2);
	} else {
		printf("    ");
	}
	
	printf("%s ", nestestnames[core->d_op]);
	printf("%s", core->d_str);

	printf("\t\t\t");

	printf("A:%.2X X:%.2X Y:%.2X P:%.2X SP:%.2X", core->d_a, core->d_x, core->d_y,
							core->d_p, core->d_sp);

	printf("\n");

}

void reset_nestest_diag(nes_t *nes) {

	core_t *core = nes->core;

	// Reset Debugs:
	strcpy(core->d_str, "     ");

	core->d_op = 0;
	core->d_op1_en = 0;
	core->d_op2_en = 0;
	core->d_op1 = 0;
	core->d_op2 = 0;

	core->d_a = 0;
	core->d_x = 0;
	core->d_y = 0;

	core->d_sp = 0;
	core->d_p = 0;
	core->d_pc = 0;

}

void run_nestest(nes_t *nes) {

	// Evenly spread cycles over the period of a second to achieve NESBLUE_CPU_FREQ:
        uint16_t cycles_per_step = (NESBLUE_CPU_FREQ / (ONE_SECOND / STEP_DURATION));
	uint16_t total_cycles = 0;
	
	core_t *core = nes->core;

        while (1) {

		// Execute CPU Instructions:
		for ( nes->core->cyclecount %= cycles_per_step; nes->core->cyclecount < cycles_per_step; ) {

			// Step through CPU:
			reset_nestest_diag(nes);
			step_core(nes->core);
			print_nestest_diag(nes);

			total_cycles += core->cyclecount;
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
