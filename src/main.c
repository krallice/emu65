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

	// Opcode Table:
	struct opcode_t *opcode;

} core_t;

// Opcode Struct:
typedef struct opcode_t {
	uint8_t (*addrmode)(core_t *core); // Addressing Mode
} opcode_t;

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

// Initialise the opcode table:
void init_opcode_table(core_t *core) {
	core->opcode[0].addrmode = addr_accumulator;
	core->opcode[1].addrmode = addr_immediate;
	core->opcode[2].addrmode = addr_zeropage;
}

// Initialise the 6502 core:
core_t *init_core() {

	core_t *core = (core_t*)malloc(sizeof(core_t));
	memset(core, 0, sizeof(core_t));

	core->ram = (uint8_t*)malloc(sizeof(uint8_t)*CORE_RAM_SIZE);
	memset(core->ram, 0, sizeof(uint8_t)*CORE_RAM_SIZE);

	core->opcode = (opcode_t*)malloc(sizeof(opcode_t*)*CORE_OPCODE_SIZE);
	memset(core->opcode, 0, sizeof(opcode_t*)*CORE_OPCODE_SIZE);

	init_opcode_table(core);
	
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

void exec_core(core_t *core) {

	while ( core->ram[core->pc] != 0xFF ) {
		switch ( core->ram[core->pc] ) {

			// LDX:

			case LDX_I:
				core->x = addr_immediate(core);
				++(core->pc);
				break;

			case LDX_ZPG:
				core->x = addr_zeropage(core);
				++(core->pc);
				break;

			case LDX_ZPG_Y:
				core->x = addr_zeropage_y(core);
				++(core->pc);
				break;

			//////////////////////////////////////

			// LDY:

			case LDY_I:
				core->y = addr_immediate(core);
				++(core->pc);
				break;

			case LDY_ZPG:
				core->y = addr_zeropage(core);
				++(core->pc);
				break;

			case LDY_ZPG_X:
				core->y = addr_zeropage_x(core);
				++(core->pc);
				break;

			//////////////////////////////////////

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
