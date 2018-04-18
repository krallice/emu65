#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define CORE_RAM_SIZE 0xFFFF
#define CORE_OPCODE_SIZE 0xFF

#define TOGGLE_CARRY(x) (x->fcarry = ~(x->fcarry))
#define TOGGLE_ZERO(x) (x->fzero = ~(x->fzero))
#define TOGGLE_OVERFLOW(x) (x->foverflow = ~(x->foverflow))
#define TOGGLE_SIGN(x) (x->fsign = ~(x->fsign))

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

struct opcode_t;

// Struct for the 6502 CPU:
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

// Addressing Modes:
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

// Todo: addr_relative(core_t *core)
// Todo: addr_indirect_x(core_t *core)
// Todo: addr_indirect_y(core_t *core)

// Initialise the 6502 core:
core_t *init_core() {

	core_t *core = (core_t*)malloc(sizeof(core_t));
	memset(core, 0, sizeof(core_t));

	core->ram = (uint8_t*)malloc(sizeof(uint8_t)*CORE_RAM_SIZE);
	memset(core->ram, 0, sizeof(uint8_t)*CORE_RAM_SIZE);

	return core;
}

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

static inline void set_fzero(core_t *core, uint8_t *val) {
	core->fzero = (*val == 0) ? 1 : 0;
}

static inline void set_fsign(core_t *core, uint8_t *val) {
	core->fsign = (*val & (0x01 << 7)) ? 1 : 0;
}

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

void exec_core(core_t *core) {

	while ( core->ram[core->pc] != 0xFF ) {
		switch ( core->ram[core->pc] ) {

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

			case LDX_ZPG_A:
				instr_ldx(core, addr_absolute);
				break;

			case LDX_ZPG_A_Y:
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

			case LDY_ZPG_A:
				instr_ldy(core, addr_absolute);
				break;

			case LDX_ZPG_A_X:
				instr_ldy(core, addr_absolute_x);
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
