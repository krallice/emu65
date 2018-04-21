#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Initialisation Macros:
#define CORE_RAM_SIZE 0xFFFF
#define CORE_OPCODE_SIZE 0xFF

// Stack Macros:
#define CORE_STACK_PAGE 0x01 // 1st Page for the Stack
#define CORE_STACK_SIZE 0xFF // Size of the stack (0x0100 -> 0x01FF)
#define CORE_STACK_POINTER_INIT 0xFF // Initial Address to init the SP to
#define CORE_STACK_ADDRESS(X) ((CORE_STACK_PAGE << 8) + X->sp) // Return Full 2 Byte Stack Address (0x01XX)

// Flag Macros:
#define FLAG_CARRY	0x01
#define FLAG_ZERO	0x02
#define FLAG_INTDIS	0x04 // Interrupt Disable
#define FLAG_DEC	0x08 // (Unused)
#define FLAG_VECT	0x10 // Clear if Interrupt Vectoring, set if BRK or PHP
#define FLAG_ALWAYS	0x20 // Always Set (Unused)
#define FLAG_OVERFLOW	0x40	
#define FLAG_SIGN	0x80

// Opcode Macros:

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

#define STA_ZPG		0x85
#define STA_ZPG_X	0x95
#define STA_A		0x8D
#define STA_A_X		0x9D
#define STA_A_Y		0x99
#define STA_IND_X	0x81
#define STA_IND_Y	0x91

#define STX_ZPG		0x86
#define STX_ZPG_Y	0x96
#define STX_A		0x8E

#define STY_ZPG		0x84
#define STY_ZPG_X	0x94
#define STY_A		0x8C

// Register Transfers:
#define TAX		0xAA
#define TAY		0xA8
#define TXA		0x8A
#define TYA		0x98

// todo: implement:
// Stack Operations:
#define TXS		0x9A // Transfer value of X to Stack
#define TSX		0xBA // Transfer value on Stack to X
#define PHA		0x48 // Push the value of A onto Stack
#define PHP		0x08 // Push the value of the Processor Status onto the Stack
#define PLA		0x68 // Pull the value of the Stack onto A
#define PLP		0x28 // Pull the value of the Stack onto the Processor Status

// Arithmetic
#define ADC_I		0x69
#define ADC_ZPG		0x65
#define ADC_ZPG_X	0x75
#define ADC_A		0x6D
#define ADC_A_X		0x7D
#define ADC_A_Y		0x79
#define ADC_IND_X	0x61
#define ADC_IND_Y	0x71

// Increment/Decrements:
#define INC_ZPG		0xE6
#define INC_ZPG_X	0xF6
#define INC_A		0xEE
#define INC_A_X		0xFE

#define INX		0xE8
#define INY		0xC8

#define DEC		0xDE
#define DEX		0xCA
#define DEY		0x88

// Flag Sets/Clears
#define CLC		0x18
#define CLI		0x58
#define CLV		0xB8
#define SEC		0x38
#define SEI		0x78

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
        uint8_t sp;

        // Used Flags:
        uint8_t fcarry :1;
        uint8_t fzero :1;
        uint8_t fintdisable :1;
	uint8_t fdec :1; // Unused
	uint8_t fvect :1; // Clear if Interrupt Vectoring, set if BPK or PHP
	uint8_t falways :1; // Unused
        uint8_t fsign :1;
        uint8_t foverflow :1;

	uint8_t flags;
	uint8_t f;

	// RAM:
	uint8_t *ram;

} core_t;

// Memory Addressing Modes:
// Each have a difference method of returning an 8bit value:
//uint16_t addr_accumulator(core_t *core) {
	//return core->a;
//}

uint16_t addr_immediate(core_t *core) {
	return ++(core->pc);
}

uint16_t addr_zeropage(core_t *core) {
	return (0x00 << 8) + core->ram[++(core->pc)];
}

uint16_t addr_zeropage_x(core_t *core) {
	return (0x00 << 8) + core->ram[++(core->pc)] + core->x;
}

uint16_t addr_zeropage_y(core_t *core) {
	return (0x00 << 8) + core->ram[++(core->pc)] + core->y;
}

uint16_t addr_absolute(core_t *core) {
	uint8_t lsb = core->ram[++(core->pc)];
	uint8_t msb = core->ram[++(core->pc)];
	return (msb << 8) + lsb;
}

uint16_t addr_absolute_x(core_t *core) {
	uint8_t lsb = core->ram[++(core->pc)];
	uint8_t msb = core->ram[++(core->pc)];
	return ((msb << 8) + lsb) + core->x;
}

uint16_t addr_absolute_y(core_t *core) {
	uint8_t lsb = core->ram[++(core->pc)];
	uint8_t msb = core->ram[++(core->pc)];
	return ((msb << 8) + lsb) + core->y;
}

uint16_t addr_indirect_x(core_t *core) {
	// Our pointer to a memory address in the zero page (Modified by X):
	uint16_t zpg = (0x00 << 8) + (core->ram[++(core->pc)] + core->x);

	// Our address that the zero page pointer points to:
	// Todo: Fix any potential roll over issues:
	return (core->ram[zpg+1] << 8) + core->ram[zpg];
}

uint16_t addr_indirect_y(core_t *core) {

	// Our pointer to a memory address in the zero page:
	uint16_t zpg = (0x00 << 8) + (core->ram[++(core->pc)]);

	// Address located in Zero page which points to our final target (Modified by Y):
	return ((core->ram[zpg+1] << 8) + core->ram[zpg]) + core->y;
}

// Todo: addr_relative(core_t *core)

// Initialise the 6502 core:
core_t *init_core() {

	core_t *core = (core_t*)malloc(sizeof(core_t));
	memset(core, 0, sizeof(core_t));

	core->ram = (uint8_t*)malloc(sizeof(uint8_t)*CORE_RAM_SIZE);
	memset(core->ram, 0, sizeof(uint8_t)*CORE_RAM_SIZE);

	core->sp = CORE_STACK_POINTER_INIT;

	core->falways = 1;

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
	printf("Overflow Flag:\t\t%d\n", core->foverflow);

	printf("Program Counter:\t0x%.4x\n", core->pc);
	printf("Stack Pointer:\t\t\t0x%.4x\n", core->sp);
	printf("Stack Pointer Value:\t\t0x%.2x\n", core->ram[CORE_STACK_ADDRESS(core)]);
	printf("Prev Stack Pointer Value:\t0x%.2x\n", core->ram[CORE_STACK_ADDRESS(core) + 1]);
}

// Flag Operations:
static inline void set_fzero(core_t *core, uint8_t *val) {
	core->fzero = (*val == 0) ? 1 : 0;
}

static inline void set_fsign(core_t *core, uint8_t *val) {
	core->fsign = (*val & (0x01 << 7)) ? 1 : 0;
}

static inline void set_foverflow(core_t *core, uint8_t *a, uint8_t *b, uint8_t *sum) {
	// ~(a + b) == If the sign bits were the same (inverse of XOR)
	// & (a ^ sum) == If the sign bits of a+b were different
	// & 0x80 mask off the highest bit:
	core->foverflow = ((~(*a ^ *b)) & (*a ^ *sum) & 0x80) ? 1 : 0;
}

// Instructions:
// Each instruction supports multiple memory addressing modes, passed as a function pointer,
// dispatched by the relevant opcode case in the master switch table.

// Generic Instruction Implementations:

// Generic Load Function:
static inline void instr_ld(core_t *core, uint8_t *reg, uint16_t (*addr_mode)(core_t *core)) {
	*reg = core->ram[addr_mode(core)];
	set_fzero(core, reg);
	set_fsign(core, reg);
	++(core->pc);
}

// Generic Store Function:
static inline void instr_st(core_t *core, uint8_t *reg, uint16_t (*addr_mode)(core_t *core)) {
	core->ram[addr_mode(core)] = *reg;
	++(core->pc);
}

// Generic Register Transfer Function (TAX, TAY, TYA, TXA etc..):
static inline void instr_t__(core_t *core, const uint8_t *src, uint8_t *dst) {
	*dst = *src;
	set_fzero(core, dst);
	set_fsign(core, dst);
	++(core->pc);
}

// Generic Register Address Increment Function:
static inline void instr_in_(core_t *core, uint8_t *reg, uint16_t (*addr_mode)(core_t *core)) {
	++(*reg);
	set_fzero(core, reg);
	set_fsign(core, reg);
	++(core->pc);
}

// Generic Register Address Decrement Function:
static inline void instr_de_(core_t *core, uint8_t *reg, uint16_t (*addr_mode)(core_t *core)) {
	--(*reg);
	set_fzero(core, reg);
	set_fsign(core, reg);
	++(core->pc);
}

// Specific Instruction Implementations:

// Specific Memory Address Increment Function:
static inline void instr_inc(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	uint8_t newval = ++(core->ram[addr_mode(core)]);
	set_fzero(core, &(newval));
	set_fsign(core, &(newval));
	++(core->pc);
}

// Specific Memory Address Decrement Function:
static inline void instr_dec(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	uint8_t newval = --(core->ram[addr_mode(core)]);
	set_fzero(core, &(newval));
	set_fsign(core, &(newval));
	++(core->pc);
}

// Specific Push onto Stack Operation:
static inline void instr_pha(core_t *core) {
	core->ram[CORE_STACK_ADDRESS(core)] = core->a;
	--(core->sp); // Decrement Stack Pointer
	++(core->pc);
}

// Specific Pull off the Stack
static inline void instr_pla(core_t *core) {
	core->a = core->ram[CORE_STACK_ADDRESS(core)];
	++(core->sp);
	++(core->pc);
}

// Specific ADC Funtion:
static inline void instr_adc(core_t *core, uint16_t (*addr_mode)(core_t *core)) {

	// Set Overflow:
	uint8_t addition = core->a + core->ram[addr_mode(core)] + core->fcarry;
	uint8_t sum = core->a + addition;
	set_foverflow(core, &(core->a), &addition, &sum);

	// Set Accumulator
	core->a = sum;

	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific Load Functions for Registers:
// todo: Probably no longer required
/*
static inline void instr_lda(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	core->a = core->ram[addr_mode(core)];
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

static inline void instr_ldx(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	core->x = core->ram[addr_mode(core)];
	set_fzero(core, &(core->x));
	set_fsign(core, &(core->x));
	++(core->pc);
}

static inline void instr_ldy(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	core->y = core->ram[addr_mode(core)];
	set_fzero(core, &(core->y));
	set_fsign(core, &(core->y));
	++(core->pc);
}
*/

// Main excecution cycle and instruction dispatch table:
void exec_core(core_t *core) {

	// 0xFF is our Temp Quit Value
	// Scaffolding for building the core:
	while ( core->ram[core->pc] != 0xFF ) {
		switch ( core->ram[core->pc] ) {

		// Load/Store:

			// LDX:
			case LDX_I:
				instr_ld(core, &(core->x), addr_immediate);
				break;
			case LDX_ZPG:
				instr_ld(core, &(core->x), addr_zeropage);
				break;
			case LDX_ZPG_Y:
				instr_ld(core, &(core->x), addr_zeropage_y);
				break;
			case LDX_A:
				instr_ld(core, &(core->x), addr_absolute);
				break;
			case LDX_A_Y:
				instr_ld(core, &(core->x), addr_absolute_y);
				break;

			// LDY:
			case LDY_I:
				instr_ld(core, &(core->y), addr_immediate);
				break;
			case LDY_ZPG:
				instr_ld(core, &(core->y), addr_zeropage);
				break;
			case LDY_ZPG_X:
				instr_ld(core, &(core->y), addr_zeropage_x);
				break;
			case LDY_A:
				instr_ld(core, &(core->y), addr_absolute);
				break;
			case LDY_A_X:
				instr_ld(core, &(core->y), addr_absolute_x);
				break;

			// LDA:
			case LDA_I:
				instr_ld(core, &(core->a), addr_immediate);
				break;
			case LDA_ZPG:
				instr_ld(core, &(core->a), addr_zeropage);
				break;
			case LDA_ZPG_X:
				instr_ld(core, &(core->a), addr_zeropage_x);
				break;
			case LDA_A:
				instr_ld(core, &(core->a), addr_absolute);
				break;
			case LDA_A_X:
				instr_ld(core, &(core->a), addr_absolute_x);
				break;
			case LDA_A_Y:
				instr_ld(core, &(core->a), addr_absolute_y);
				break;
			case LDA_IND_X:
				instr_ld(core, &(core->a), addr_indirect_x);
				break;
			case LDA_IND_Y:
				instr_ld(core, &(core->a), addr_indirect_y);
				break;

			// STA:
			case STA_ZPG:
				instr_st(core, &(core->a), addr_zeropage);
				break;
			case STA_ZPG_X:
				instr_st(core, &(core->a), addr_zeropage_x);
				break;
			case STA_A:
				instr_st(core, &(core->a), addr_absolute);
				break;
			case STA_A_X:
				instr_st(core, &(core->a), addr_absolute_x);
				break;
			case STA_A_Y:
				instr_st(core, &(core->a), addr_absolute_y);
				break;
			case STA_IND_X:
				instr_st(core, &(core->a), addr_indirect_x);
				break;
			case STA_IND_Y:
				instr_st(core, &(core->a), addr_indirect_y);
				break;

			// STX:
			case STX_ZPG:
				instr_st(core, &(core->x), addr_zeropage);
				break;
			case STX_ZPG_Y:
				instr_st(core, &(core->x), addr_zeropage_y);
				break;
			case STX_A:
				instr_st(core, &(core->x), addr_absolute);
				break;

			// STY:
			case STY_ZPG:
				instr_st(core, &(core->y), addr_zeropage);
				break;
			case STY_ZPG_X:
				instr_st(core, &(core->y), addr_zeropage_x);
				break;
			case STY_A:
				instr_st(core, &(core->y), addr_absolute);
				break;

		// Register Transfers:
			case TAX:
				instr_t__(core, &(core->a), &(core->x));
				break;
			case TAY:
				instr_t__(core, &(core->a), &(core->y));
				break;
			case TXA:
				instr_t__(core, &(core->x), &(core->a));
				break;
			case TYA:
				instr_t__(core, &(core->y), &(core->a));
				break;

		// Stack Operations:
			case TXS:
				instr_t__(core, &(core->x), &(core->ram[CORE_STACK_ADDRESS(core)]));
				break;
			case TSX:
				instr_t__(core, &(core->ram[CORE_STACK_ADDRESS(core)]), &(core->x));
				break;
			case PHA:
				instr_pha(core);
				break;

		// Arithmetic Operations:
			case ADC_I:
				instr_adc(core, addr_immediate);
				break;
			case ADC_ZPG:
				instr_adc(core, addr_zeropage);
				break;
			case ADC_ZPG_X:
				instr_adc(core, addr_zeropage_x);
				break;
			case ADC_A:
				instr_adc(core, addr_absolute);
				break;
			case ADC_A_X:
				instr_adc(core, addr_absolute_x);
				break;
			case ADC_A_Y:
				instr_adc(core, addr_absolute_y);
				break;
			case ADC_IND_X:
				instr_adc(core, addr_indirect_x);
				break;
			case ADC_IND_Y:
				instr_adc(core, addr_indirect_y);
				break;

		// Increments/Decrements:
			case INC_ZPG:
				instr_inc(core, addr_zeropage);
				break;
			case INC_ZPG_X:
				instr_inc(core, addr_zeropage_x);
				break;
			case INC_A:
				instr_inc(core, addr_absolute);
				break;
			case INC_A_X:
				instr_inc(core, addr_absolute_x);
				break;
			case INX:
				instr_in_(core, &(core->x), addr_zeropage);
				break;
			case INY:
				instr_in_(core, &(core->y), addr_zeropage);
				break;
			case DEC:
				instr_dec(core, addr_zeropage);
				break;
			case DEX:
				instr_de_(core, &(core->x), addr_zeropage);
				break;
			case DEY:
				instr_de_(core, &(core->y), addr_zeropage);
				break;

		// Flag Sets and Clears:
			case CLC:
				core->fcarry = 0;
				++(core->pc);
				break;
			case CLI:
				core->fintdisable = 0;
				++(core->pc);
				break;
			case CLV:
				core->foverflow = 0;
				++(core->pc);
				break;
			case SEC:
				core->fcarry = 1;
				++(core->pc);
				break;
			case SEI:
				core->fintdisable = 1;
				++(core->pc);
				break;

			default:
				++(core->pc);
				break;
		}
	}
}

int main(void) {

	core_t *core = init_core();

	core->ram[0x0000] = LDX_ZPG;
	core->ram[0x0001] = 0xCC;

	core->ram[0x0002] = TXS;

	core->ram[0x0003] = INX;
	core->ram[0x0004] = TXA;
	core->ram[0x0005] = PHA;
	
	core->ram[0x0006] = 0xFF;

	core->ram[0x000F] = 0xFF; // Exit

	exec_core(core);
	
	dump_core_state(core);
	return 0;
}
