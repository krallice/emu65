#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define CORE_DEBUG 1

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

// Initialisation Macros:
#define CORE_RAM_SIZE 0xFFFF
#define CORE_OPCODE_SIZE 0xFF

// Stack Macros:
#define CORE_STACK_PAGE 0x01 // 1st Page for the Stack
#define CORE_STACK_SIZE 0xFF // Size of the stack (0x0100 -> 0x01FF)
#define CORE_STACK_POINTER_INIT 0xFF // Initial Address to init the SP to
#define CORE_STACK_ADDRESS(X, Y) ((CORE_STACK_PAGE << 8) + (uint8_t)(X->sp + Y)) // Return Full 2 Byte Stack Address (0x01XX)

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

// Stack Operations:
#define TXS		0x9A // Transfer value of X to Stack
#define TSX		0xBA // Transfer value on Stack to X
#define PHA		0x48 // Push the value of A onto Stack
#define PHP		0x08 // Push the value of the Processor Status onto the Stack
#define PLA		0x68 // Pull the value of the Stack onto A
#define PLP		0x28 // Pull the value of the Stack onto the Processor Status

// Logical:
#define AND_I		0x29
#define AND_ZPG		0x25
#define AND_ZPG_X	0x35
#define AND_A		0x2D
#define AND_A_X		0x3D
#define AND_A_Y		0x39
#define AND_IND_X	0x21
#define AND_IND_Y	0x31

#define EOR_I		0x49
#define EOR_ZPG		0x45
#define EOR_ZPG_X	0x55
#define EOR_A		0x4D
#define EOR_A_X		0x5D
#define EOR_A_Y		0x59
#define EOR_IND_X	0x41
#define EOR_IND_Y	0x51

#define ORA_I		0x09
#define ORA_ZPG		0x05
#define ORA_ZPG_X	0x15
#define ORA_A		0x0D
#define ORA_A_X		0x1D
#define ORA_A_Y		0x19
#define ORA_IND_X	0x01
#define ORA_IND_Y	0x11

#define BIT_ZPG		0x24 // todo
#define BIT_A		0x2C // todo

// todo: complete
// Arithmetic
#define ADC_I		0x69
#define ADC_ZPG		0x65
#define ADC_ZPG_X	0x75
#define ADC_A		0x6D
#define ADC_A_X		0x7D
#define ADC_A_Y		0x79
#define ADC_IND_X	0x61
#define ADC_IND_Y	0x71

#define CMP_I		0xC9
#define CMP_ZPG		0xC5
#define CMP_ZPG_X	0xD5
#define CMP_A		0xCD
#define CMP_A_X		0xDD
#define CMP_A_Y		0xD9
#define CMP_IND_X	0xC1
#define CMP_IND_Y	0xD1

#define CPX_I		0xE0
#define CPX_ZPG		0xE4
#define CPX_A		0xEC

#define CPY_I		0xC0
#define CPY_ZPG		0xC4
#define CPY_A		0xCC

// Increment/Decrements:
#define INC_ZPG		0xE6
#define INC_ZPG_X	0xF6
#define INC_A		0xEE
#define INC_A_X		0xFE

#define DEC_ZPG		0xC6
#define DEC_ZPG_X	0xD6
#define DEC_A		0xCE
#define DEC_A_X		0xDE

#define INX		0xE8
#define INY		0xC8
#define DEX		0xCA
#define DEY		0x88

// Shifts:
#define ASL_ACC		0x0A
#define ASL_ZPG		0x06
#define ASL_ZPG_X	0x16
#define ASL_A		0x0E
#define ASL_A_X		0x1E

#define LSR_ACC		0x4A
#define LSR_ZPG		0x46
#define LSR_ZPG_X	0x56
#define LSR_A		0x4E
#define LSR_A_X		0x5E

#define ROL_ACC		0x2A
#define ROL_ZPG		0x26
#define ROL_ZPG_X	0x36
#define ROL_A		0x2E
#define ROL_A_X		0x3E

#define ROR_ACC		0x6A
#define ROR_ZPG		0x66
#define ROR_ZPG_X	0x76
#define ROR_A		0x6E
#define ROR_A_X		0x7E

// Jumps:
#define JMP_A		0x4C
#define JMP_IND		0x6C
#define JSR_A		0x20
#define RTS		0x60

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
// Each have a difference method of returning a 16bit Memory Address:
uint16_t addr_immediate(core_t *core) {
	return (uint16_t)(++(core->pc));
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

// Used by JMP:
uint16_t addr_indirect(core_t *core) {
	uint8_t lsb = core->ram[++(core->pc)];
	uint8_t msb = core->ram[++(core->pc)];
	uint16_t indirect = ((msb << 8) + lsb);

	uint8_t final_lsb = core->ram[indirect];
	uint8_t final_msb = core->ram[indirect+1];

	return ((final_msb << 8) + final_lsb);
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
	printf("Register A: "BYTE_TO_BINARY_PATTERN"\n", 
			BYTE_TO_BINARY(core->a));

	printf("Register X:\t\t0x%.2x\n", core->x);
	printf("Register Y:\t\t0x%.2x\n", core->y);

	printf("Carry Flag:\t\t%d\n", core->fcarry);
	printf("Zero Flag:\t\t%d\n",  core->fzero);

	printf("Overflow Flag:\t\t%d\n", core->foverflow);
	printf("Sign Flag:\t\t%d\n",  core->fsign);

	printf("Program Counter:\t0x%.4x\n", core->pc);

	printf("Stack Pointer %.4x - 8 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, -8), core->ram[CORE_STACK_ADDRESS(core, -8)]);
	printf("Stack Pointer %.4x - 7 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, -7), core->ram[CORE_STACK_ADDRESS(core, -7)]);
	printf("Stack Pointer %.4x - 6 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, -6), core->ram[CORE_STACK_ADDRESS(core, -6)]);
	printf("Stack Pointer %.4x - 5 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, -5), core->ram[CORE_STACK_ADDRESS(core, -5)]);
	printf("Stack Pointer %.4x - 4 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, -4), core->ram[CORE_STACK_ADDRESS(core, -4)]);
	printf("Stack Pointer %.4x - 3 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, -3), core->ram[CORE_STACK_ADDRESS(core, -3)]);
	printf("Stack Pointer %.4x - 2 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, -2), core->ram[CORE_STACK_ADDRESS(core, -2)]);
	printf("Stack Pointer %.4x - 1 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, -1), core->ram[CORE_STACK_ADDRESS(core, -1)]);
	printf("Stack Pointer Address:\t\t\t0x%.4x (Value: %.2x)\n", core->sp, core->ram[CORE_STACK_ADDRESS(core, 0)]);
	printf("Stack Pointer %.4x + 1 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, 1), core->ram[CORE_STACK_ADDRESS(core, 1)]);
	printf("Stack Pointer %.4x + 2 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, 2), core->ram[CORE_STACK_ADDRESS(core, 2)]);
	printf("Stack Pointer %.4x + 3 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, 3), core->ram[CORE_STACK_ADDRESS(core, 3)]);
	printf("Stack Pointer %.4x + 4 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, 4), core->ram[CORE_STACK_ADDRESS(core, 4)]);
	printf("Stack Pointer %.4x + 5 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, 5), core->ram[CORE_STACK_ADDRESS(core, 5)]);
	printf("Stack Pointer %.4x + 6 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, 6), core->ram[CORE_STACK_ADDRESS(core, 6)]);
	printf("Stack Pointer %.4x + 7 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, 7), core->ram[CORE_STACK_ADDRESS(core, 7)]);
	printf("Stack Pointer %.4x + 8 Value:\t0x%.2x\n", CORE_STACK_ADDRESS(core, 8), core->ram[CORE_STACK_ADDRESS(core, 8)]);
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
	uint16_t addy = addr_mode(core);
	#if CORE_DEBUG == 1
	printf("addy: %.4x\n", addy);
	printf("val: %.2x\n", core->ram[addy]);
	#endif
	*reg = core->ram[addy];
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
static inline void instr_in_(core_t *core, uint8_t *reg) {
	++(*reg);
	set_fzero(core, reg);
	set_fsign(core, reg);
	++(core->pc);
}

// Generic Register Address Decrement Function:
static inline void instr_de_(core_t *core, uint8_t *reg) {
	--(*reg);
	set_fzero(core, reg);
	set_fsign(core, reg);
	++(core->pc);
}

// Generic A/X/Y CMP Implementation:
static inline void instr_cmp(core_t *core, uint8_t *reg, uint16_t (*addr_mode)(core_t *core)) {

	uint8_t result = *reg - core->ram[addr_mode(core)];
	core->fcarry = (result >= 0) ? 1 : 0;
	set_fzero(core, &result);
	set_fsign(core, &result);
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
	core->ram[CORE_STACK_ADDRESS(core, 0)] = core->a;
	--(core->sp); // Decrement Stack Pointer
	++(core->pc);
}

// Specific Pull off the Stack
static inline void instr_pla(core_t *core) {
	++(core->sp);
	core->a = core->ram[CORE_STACK_ADDRESS(core, 0)];
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific Push Status onto Stack:
static inline void instr_php(core_t *core) {

	uint8_t status = 0x00;
	status |= (core->fcarry 	<< 0);
	status |= (core->fzero 		<< 1);
	status |= (core->fintdisable 	<< 2);
	status |= (core->fdec 		<< 3);
	status |= (core->fvect 		<< 4);
	status |= (core->falways	<< 5);
	status |= (core->foverflow 	<< 6);
	status |= (core->fsign		<< 7);

	#if CORE_DEBUG == 1
	printf("Zero Flag: %.2X\n", core->fzero);
	printf("Status Word: %.2X\n", status);
	#endif

	core->ram[CORE_STACK_ADDRESS(core, 0)] = status;
	--(core->sp); // Decrement Stack Pointer

	++(core->pc);
}

// Specific Pull Status onto Stack:
static inline void instr_plp(core_t *core) {

	++(core->sp);
	uint8_t status = core->ram[CORE_STACK_ADDRESS(core, 0)];

	core->fcarry = 		((status >> 0) & 0x01) ? 1 : 0;
	core->fzero = 		((status >> 1) & 0x01) ? 1 : 0;
	core->fintdisable = 	((status >> 2) & 0x01) ? 1 : 0;
	core->fdec = 		((status >> 3) & 0x01) ? 1 : 0;
	core->fvect =		((status >> 4) & 0x01) ? 1 : 0;
	core->falways =		((status >> 5) & 0x01) ? 1 : 0;
	core->foverflow = 	((status >> 6) & 0x01) ? 1 : 0;
	core->fsign =		((status >> 7) & 0x01) ? 1 : 0;

	++(core->pc);
}

// Specific Logical AND Operation:
static inline void instr_and(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	core->a &= core->ram[addr_mode(core)];
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific Logical eXclusive OR Operation:
static inline void instr_eor(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	core->a ^= core->ram[addr_mode(core)];
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific Logical OR Operation:
static inline void instr_ora(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	core->a |= core->ram[addr_mode(core)];
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific Logical BIT Operation:
static inline void instr_bit(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	// Zero Flag - Non Destructive AND between A and M. Set Zero Flag if required.
	// Don't overwrite A
	// Overflow Flag - Copied from Memory
	// Sign Flag - Copied from Memory

	uint8_t mem = core->ram[addr_mode(core)]; 

	((core->a & mem) == 0) ? (core->fzero = 1) : (core->fzero = 0);
	core->foverflow = ((mem >> 6) & 0x01);
	core->fsign 	= ((mem >> 7) & 0x01);
	
	#if CORE_DEBUG == 1
	printf("BIT: "BYTE_TO_BINARY_PATTERN"\n", 
			BYTE_TO_BINARY(mem)); 
	#endif

	++(core->pc);
}

// Specific ASL for Accumulator
static inline void instr_asl_acc(core_t *core) {
	core->fcarry = core->a >> 7;
	core->a = core->a << 1;
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific ASL for Memory Address:
static inline void instr_asl(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	uint16_t address = addr_mode(core);
	core->fcarry = core->ram[address] >> 7;
	core->ram[address] = core->ram[address] << 1;
	set_fzero(core, &(core->ram[address]));
	set_fsign(core, &(core->ram[address]));
	++(core->pc);
}

// Specific LSR for Accumulator;
static inline void instr_lsr_acc(core_t *core) {
	core->fcarry = (core->a & 0x01); // Carry = Bit Zero
	core->a = core->a >> 1;
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific LSR for Memory Address:
static inline void instr_lsr(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	uint16_t address = addr_mode(core);
	core->fcarry = (core->ram[address] & 0x01); // Carry = Bit Zero
	core->ram[address] = core->ram[address] >> 1;
	set_fzero(core, &(core->ram[address]));
	set_fsign(core, &(core->ram[address]));
	++(core->pc);
}

// Specific ROL for Accumulator
static inline void instr_rol_acc(core_t *core) {
	uint8_t oldcarry = core->fcarry;
	core->fcarry = core->a >> 7; // Carry = Bit 7
	core->a = core->a << 1; // Shift Left
	core->a |= oldcarry; // Bit 0 == Old Carry Value
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific ROL for Memory Address:
static inline void instr_rol(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	uint16_t address = addr_mode(core);
	uint8_t oldcarry = core->fcarry;
	core->fcarry = core->ram[address] >> 7; // Carry = Bit 7
	core->ram[address] = core->ram[address] << 1;
	core->ram[address] |= oldcarry;
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific ROR for Accumulator:
static inline void instr_ror_acc(core_t *core) {
	uint8_t oldcarry = core->fcarry;
	core->fcarry = (core->a & 0x01); // Carry = Bit 1
	core->a = core->a >> 1; // Shift Left
	core->a |= (oldcarry << 7); // Bit 7 == Old Carry Value
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific ROR for Memory Address:
static inline void instr_ror(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	uint16_t address = addr_mode(core);
	uint8_t oldcarry = core->fcarry;
	core->fcarry = (core->ram[address] & 0x01); // Carry = Bit 7
	core->ram[address] = core->ram[address] >> 1;
	core->ram[address] |= (oldcarry << 7);
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

static inline void instr_jmp(core_t *core, uint16_t (*addr_mode)(core_t *core)) {
	core->pc = addr_mode(core);
}

static inline void instr_jsr(core_t *core, uint16_t (*addr_mode)(core_t *core)) {

	uint16_t jsr_address = addr_mode(core);
	uint16_t rts_address = core->pc - 1;

	uint8_t msb = rts_address >> 8;
	uint8_t lsb = (rts_address & 0x00FF);

	core->ram[CORE_STACK_ADDRESS(core, 0)] = msb;
	--(core->sp);

	core->ram[CORE_STACK_ADDRESS(core, 0)] = lsb;
	--(core->sp);

	core->pc = jsr_address;
}

static inline void instr_rts(core_t *core) {

	++(core->sp);
	uint8_t lsb = core->ram[CORE_STACK_ADDRESS(core, 0)];

	++(core->sp);
	uint8_t msb = core->ram[CORE_STACK_ADDRESS(core, 0)];

	core->pc = ((msb << 8) + lsb) + 1;
	#if CORE_DEBUG == 1
		printf("RTS LSB: %.2x\n", lsb);
		printf("RTS MSB: %.2x\n", msb);
		printf("RTS: PC Set to %.4x\n", core->pc);
	#endif
}

// todo: actually implement
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
				instr_t__(core, &(core->x), &(core->ram[CORE_STACK_ADDRESS(core, 0)]));
				break;
			case TSX:
				instr_t__(core, &(core->ram[CORE_STACK_ADDRESS(core, 0)]), &(core->x));
				break;
			case PHA:
				instr_pha(core);
				break;
			case PLA:
				instr_pla(core);
				break;
			case PHP:
				instr_php(core);
				break;
			case PLP:
				instr_plp(core);
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

			case CMP_I:
				instr_cmp(core, &(core->a), addr_immediate);
				break;
			case CMP_ZPG:
				instr_cmp(core, &(core->a), addr_zeropage);
				break;
			case CMP_ZPG_X:
				instr_cmp(core, &(core->a), addr_zeropage_x);
				break;
			case CMP_A:
				instr_cmp(core, &(core->a), addr_absolute);
				break;
			case CMP_A_X:
				instr_cmp(core, &(core->a), addr_absolute_x);
				break;
			case CMP_A_Y:
				instr_cmp(core, &(core->a), addr_absolute_y);
				break;
			case CMP_IND_X:
				instr_cmp(core, &(core->a), addr_indirect_x);
				break;
			case CMP_IND_Y:
				instr_cmp(core, &(core->a), addr_indirect_y);
				break;

			case CPX_I:
				instr_cmp(core, &(core->x), addr_immediate);
				break;
			case CPX_ZPG:
				instr_cmp(core, &(core->x), addr_zeropage);
				break;
			case CPX_A:
				instr_cmp(core, &(core->x), addr_absolute);
				break;

			case CPY_I:
			      	instr_cmp(core, &(core->y), addr_immediate);
			      	break;
			case CPY_ZPG:
				instr_cmp(core, &(core->y), addr_zeropage);
			      	break;
			case CPY_A:
				instr_cmp(core, &(core->y), addr_absolute);
				break;

		// Logic Operations:
			case AND_I:
				instr_and(core, addr_immediate);
				break;
			case AND_ZPG:
				instr_and(core, addr_zeropage);
				break;
			case AND_ZPG_X:
				instr_and(core, addr_zeropage_x);
				break;
			case AND_A:
				instr_and(core, addr_absolute);
				break;
			case AND_A_X:
				instr_and(core, addr_absolute_x);
				break;
			case AND_A_Y:
				instr_and(core, addr_absolute_y);
				break;
			case AND_IND_X:
				instr_and(core, addr_indirect_x);
				break;
			case AND_IND_Y:
				instr_and(core, addr_indirect_y);
				break;

			case EOR_I:
				instr_eor(core, addr_immediate);
				break;
			case EOR_ZPG:
				instr_eor(core, addr_zeropage);
				break;
			case EOR_ZPG_X:
				instr_eor(core, addr_zeropage_x);
				break;
			case EOR_A:
				instr_eor(core, addr_absolute);
				break;
			case EOR_A_X:
				instr_eor(core, addr_absolute_x);
				break;
			case EOR_A_Y:
				instr_eor(core, addr_absolute_y);
				break;
			case EOR_IND_X:
				instr_eor(core, addr_indirect_x);
				break;
			case EOR_IND_Y:
				instr_eor(core, addr_indirect_y);
				break;

			case ORA_I:
				instr_ora(core, addr_immediate);
				break;
			case ORA_ZPG:
				instr_ora(core, addr_zeropage);
				break;
			case ORA_ZPG_X:
				instr_ora(core, addr_zeropage_x);
				break;
			case ORA_A:
				instr_ora(core, addr_absolute);
				break;
			case ORA_A_X:
				instr_ora(core, addr_absolute_x);
				break;
			case ORA_A_Y:
				instr_ora(core, addr_absolute_y);
				break;
			case ORA_IND_X:
				instr_ora(core, addr_indirect_x);
				break;
			case ORA_IND_Y:
				instr_ora(core, addr_indirect_y);
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
				instr_in_(core, &(core->x));
				break;
			case INY:
				instr_in_(core, &(core->y));
				break;
			case DEC_ZPG:
				instr_dec(core, addr_zeropage);
				break;
			case DEC_ZPG_X:
				instr_dec(core, addr_zeropage_x);
				break;
			case DEC_A:
				instr_dec(core, addr_absolute);
				break;
			case DEC_A_X:
				instr_dec(core, addr_absolute_x);
				break;
			case DEX:
				instr_de_(core, &(core->x));
				break;
			case DEY:
				instr_de_(core, &(core->y));
				break;
			case BIT_ZPG:
				instr_bit(core, addr_zeropage);
				break;
			case BIT_A:
				instr_bit(core, addr_absolute);
				break;

		// Shifts:
			case ASL_ACC:
				instr_asl_acc(core);
				break;
			case ASL_ZPG:
				instr_asl(core, addr_zeropage);
				break;
			case ASL_ZPG_X:
				instr_asl(core, addr_zeropage_x);
				break;
			case ASL_A:
				instr_asl(core, addr_absolute);
				break;
			case ASL_A_X:
				instr_asl(core, addr_absolute_x);
				break;

			case LSR_ACC:
				instr_lsr_acc(core);
				break;
			case LSR_ZPG:
				instr_lsr(core, addr_zeropage);
				break;
			case LSR_ZPG_X:
				instr_lsr(core, addr_zeropage_x);
				break;
			case LSR_A:
				instr_lsr(core, addr_absolute);
				break;
			case LSR_A_X:
				instr_lsr(core, addr_absolute_x);
				break;

			case ROL_ACC:
				instr_rol_acc(core);
				break;
			case ROL_ZPG:
				instr_rol(core, addr_zeropage);
				break;
			case ROL_ZPG_X:
				instr_rol(core, addr_zeropage_x);
				break;
			case ROL_A:
				instr_rol(core, addr_absolute);
				break;
			case ROL_A_X:
				instr_rol(core, addr_absolute_x);
				break;

			case ROR_ACC:
				instr_ror_acc(core);
				break;
			case ROR_ZPG:
				instr_ror(core, addr_zeropage);
				break;
			case ROR_ZPG_X:
				instr_ror(core, addr_zeropage_x);
				break;
			case ROR_A:
				instr_ror(core, addr_absolute);
				break;
			case ROR_A_X:
				instr_ror(core, addr_absolute_x);
				break;

		// Jumps:
			case JMP_A:
				instr_jmp(core, addr_absolute);
				break;
			case JMP_IND:
				instr_jmp(core, addr_indirect);
				break;
			case JSR_A:
				instr_jsr(core, addr_absolute);
				break;
			case RTS:
				instr_rts(core);
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

			#if CORE_DEBUG == 1
			case 0x22:
				dump_core_state(core);
				printf("\n");
				++(core->pc);
				break;
			#endif

			default:
				++(core->pc);
				break;
		}
	}
}

void pg_jmptest(core_t *core) {

	core->ram[0x0000] = JMP_A;
	core->ram[0x0001] = 0x00;
	core->ram[0x0002] = 0xC0;

	core->ram[0xC000] = LDA_ZPG;
	core->ram[0xC001] = 0x31;
	core->ram[0xC002] = PHA;
	core->ram[0xC003] = 0x22;

	core->ram[0xC004] = JSR_A;
	core->ram[0xC005] = 0xD0;
	core->ram[0xC006] = 0xDC;
	core->ram[0xC007] = 0x22; //DEBUG

	core->ram[0xC008] = 0xFF;

	core->ram[0xDCD0] = LDA_I;
	core->ram[0xDCD1] = 0x66;
	core->ram[0xDCD2] = LDA_I;
	core->ram[0xDCD3] = 0x67;
	core->ram[0xDCD4] = 0x22; //DEBUG
	core->ram[0xDCD5] = RTS;

	//core->ram[0x000A] = PLA;
	//core->ram[0x000B] = 0x22;

	// .data
	core->ram[0x0030] = 0x07; 
	core->ram[0x0031] = 0x09; 
	core->ram[0x0032] = 0x0A; 

}

int main(void) {

	core_t *core = init_core();

	//pg_jmptest(core);

	core->ram[0x0000] = LDA_ZPG;
	core->ram[0x0001] = 0x33;
	core->ram[0x0002] = 0x22;

	core->ram[0x0003] = CMP_I;
	core->ram[0x0004] = 0x24;

	core->ram[0x0005] = 0xFF;

	// .data
	core->ram[0x0033] = 0x28;

	exec_core(core);
	
	dump_core_state(core);
	return 0;
}
