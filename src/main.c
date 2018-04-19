#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Initialisation Macros:
#define CORE_RAM_SIZE 0xFFFF
#define CORE_OPCODE_SIZE 0xFF

// OPCODE Macros:

// Load/Store:
#define LDA_I		0xA9
#define LDA_ZPG 	0xA5
#define LDA_ZPG_X	0xB5
#define LDA_A 		0xAD
#define LDA_A_X		0xBD
#define LDA_A_Y		0xB9
#define LDA_IND_X	0xA1
#define LDA_IND_Y	0xB1

#define LDX_I		0xA2
#define LDX_ZPG 	0xA6
#define LDX_ZPG_Y	0xB6
#define LDX_A 		0xAE
#define LDX_A_Y		0xBE

#define LDY_I		0xA0
#define LDY_ZPG 	0xA4
#define LDY_ZPG_X	0xB4
#define LDY_A 		0xAC
#define LDY_A_X		0xBC

// Struct for our 6502 CPU Core:
typedef struct core_t {

        // 16bit Program Counter:
        uint16_t pc;

        // Accumulator:
        uint8_t a;

        // X & Y Registers:
        uint8_t x;
        uint8_t y;

        // Stack Pointer:
        uint16_t sp;

        // Used Flags:
        uint8_t fcarry :1;
        uint8_t fzero :1;
        uint8_t foverflow :1;
        uint8_t fsign :1;

	// RAM:
	uint8_t *ram;

} core_t;

// Memory Addressing Modes:
// Each have a difference method of returning an 8bit value:
uint8_t addr_accumulator(core_t *core) {
	return core->a;
}

uint8_t addr_immediate(core_t *core) {
	return core->ram[++(core->pc)];
}

uint8_t addr_zeropage(core_t *core) {
	return core->ram[(0x00 << 8) + core->ram[++(core->pc)]];
}

uint8_t addr_zeropage_x(core_t *core) {
	return core->ram[(0x00 << 8) + core->ram[++(core->pc)] + core->x];
}

uint8_t addr_zeropage_y(core_t *core) {
	return core->ram[(0x00 << 8) + core->ram[++(core->pc)] + core->y];
}

uint8_t addr_absolute(core_t *core) {
	uint8_t lsb = core->ram[++(core->pc)];
	uint8_t msb = core->ram[++(core->pc)];
	return core->ram[(msb << 8) + lsb];
}

uint8_t addr_absolute_x(core_t *core) {
	uint8_t lsb = core->ram[++(core->pc)];
	uint8_t msb = core->ram[++(core->pc)];
	return core->ram[((msb << 8) + lsb) + core->x];
}

uint8_t addr_absolute_y(core_t *core) {
	uint8_t lsb = core->ram[++(core->pc)];
	uint8_t msb = core->ram[++(core->pc)];
	return core->ram[((msb << 8) + lsb) + core->y];
}

uint8_t addr_indirect_x(core_t *core) {
	// Our pointer to a memory address in the zero page (Modified by X):
	uint16_t zpg = (0x00 << 8) + (core->ram[++(core->pc)] + core->x);

	// Our address that the zero page pointer points to:
	// Todo: Fix any potential roll over issues:
	uint16_t ind_add = ((core->ram[zpg+1] << 8) + core->ram[zpg]);
	return core->ram[ind_add];
}

uint8_t addr_indirect_y(core_t *core) {

	// Our pointer to a memory address in the zero page:
	uint16_t zpg = (0x00 << 8) + (core->ram[++(core->pc)]);

	// Address located in Zero page which points to our final target (Modified by Y):
	uint16_t ind_add = ((core->ram[zpg+1] << 8) + core->ram[zpg]);
	ind_add += core->y;
	return core->ram[ind_add];
}

// Todo: addr_relative(core_t *core)

// Initialise the 6502 core:
core_t *init_core() {

	core_t *core = (core_t*)malloc(sizeof(core_t));
	memset(core, 0, sizeof(core_t));

	core->ram = (uint8_t*)malloc(sizeof(uint8_t)*CORE_RAM_SIZE);
	memset(core->ram, 0, sizeof(uint8_t)*CORE_RAM_SIZE);

	return core;
}

// Debugging:
void dump_core_state(core_t *core) {
	printf("Register A (Accumulator): 0x%.2x\n", core->a);

	printf("Register X:\t\t0x%.2x\n", core->x);
	printf("Register Y:\t\t0x%.2x\n", core->y);

	printf("Zero Flag:\t\t%d\n",  core->fzero);
	printf("Sign Flag:\t\t%d\n",  core->fsign);
	printf("Carry Flag:\t\t%d\n", core->fcarry);

	printf("Program Counter:\t0x%.4x\n", core->pc);
	printf("Stack Pointer:\t\t0x%.4x\n", core->sp);
}


// Flag Operations:
static inline void set_fzero(core_t *core, uint8_t *val) {
	core->fzero = (*val == 0) ? 1 : 0;
}

static inline void set_fsign(core_t *core, uint8_t *val) {
	core->fsign = (*val & (0x01 << 7)) ? 1 : 0;
}

// Instructions:
// Each instruction supports multiple memory addressing modes, passed as a function pointer,
// dispatched by the relevant opcode case in the master switch table.
static inline void instr_lda(core_t *core, uint8_t (*addr_mode)(core_t *core)) {
	core->a = addr_mode(core);
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

static inline void instr_ldx(core_t *core, uint8_t (*addr_mode)(core_t *core)) {
	core->x = addr_mode(core);
	set_fzero(core, &(core->x));
	set_fsign(core, &(core->x));
	++(core->pc);
}

static inline void instr_ldy(core_t *core, uint8_t (*addr_mode)(core_t *core)) {
	core->y = addr_mode(core);
	set_fzero(core, &(core->y));
	set_fsign(core, &(core->y));
	++(core->pc);
}

// Main excecution cycle and instruction dispatch table:
void exec_core(core_t *core) {

	// 0xFF is our Temp Quit Value
	// Scaffolding for building the core:
	while ( core->ram[core->pc] != 0xFF ) {
		switch ( core->ram[core->pc] ) {

		// Load/Store:

			// LDX:
			case LDX_I:
				instr_ldx(core, addr_immediate);
				break;
			case LDX_ZPG:
				instr_ldx(core, addr_zeropage);
				break;
			case LDX_ZPG_Y:
				instr_ldx(core, addr_zeropage_y);
				break;
			case LDX_A:
				instr_ldx(core, addr_absolute);
				break;
			case LDX_A_Y:
				instr_ldx(core, addr_absolute_y);
				break;

			// LDY:
			case LDY_I:
				instr_ldy(core, addr_immediate);
				break;
			case LDY_ZPG:
				instr_ldy(core, addr_zeropage);
				break;
			case LDY_ZPG_X:
				instr_ldy(core, addr_zeropage_x);
				break;
			case LDY_A:
				instr_ldy(core, addr_absolute);
				break;
			case LDY_A_X:
				instr_ldy(core, addr_absolute_x);
				break;

			// LDA:
			case LDA_I:
				instr_lda(core, addr_immediate);
				break;
			case LDA_ZPG:
				instr_lda(core, addr_zeropage);
				break;
			case LDA_ZPG_X:
				instr_lda(core, addr_zeropage_x);
				break;
			case LDA_A:
				instr_lda(core, addr_absolute);
				break;
			case LDA_A_X:
				instr_lda(core, addr_absolute_x);
				break;
			case LDA_A_Y:
				instr_lda(core, addr_absolute_y);
				break;
			case LDA_IND_X:
				instr_lda(core, addr_indirect_x);
				break;
			case LDA_IND_Y:
				instr_lda(core, addr_indirect_y);
				break;

			default:
				++(core->pc);
				break;
		}
	}
}

int main(void) {

	core_t *core = init_core();

	core->ram[0x0000] = LDX_I; //LDX_I
	core->ram[0x0001] = 0xCC;

	core->ram[0x0002] = LDY_ZPG;
	core->ram[0x0003] = 0xCC;

	core->ram[0x0004] = 0xFF; // Exit

	core->ram[0x00CC] = 0x0E;

	exec_core(core);
	
	dump_core_state(core);
	return 0;
}
