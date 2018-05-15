#include "core.h"

// Memory Addressing Modes:
// Each have a difference method of returning a 16bit Memory Address:
uint16_t addr_immediate(core_t *core) {
	#if CORE_NESTEST == 1
		core->d_op1_en = 1;
		core->d_op1 = core->ram[core->pc + 1];
		sprintf(core->d_str, "#$%.2X", core->ram[(core->pc + 1)]);
	#endif
	return (uint16_t)(++(core->pc));
}

uint16_t addr_zeropage(core_t *core) {
	#if CORE_NESTEST == 1
		core->d_op1_en = 1;
		core->d_op1 = core->ram[core->pc + 1];
		sprintf(core->d_str, "$%.2X = %.2X", core->d_op1, core->ram[(0x00 << 8) + core->ram[(core->pc) + 1]]);
	#endif
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
	#if CORE_NESTEST == 1
		core->d_op1_en = 1;
		core->d_op2_en = 1;
		core->d_op1 = lsb;
		core->d_op2 = msb;
		sprintf(core->d_str, "$%.2X%.2X", msb, lsb);
	#endif
	return (msb << 8) + lsb;
}

uint16_t addr_absolute_x(core_t *core) {
	uint8_t lsb = core->ram[++(core->pc)];
	uint8_t msb = core->ram[++(core->pc)];
	uint16_t ret = ((msb << 8) + lsb) + core->x;
	if ( core->checkpageboundary == 1 )
		if ((ret & 0xFF00) != (msb << 8)) {
			#if CORE_DEBUG_TIMING == 1
			printf("Page Boundary Cross Penalty\n");
			#endif
			++(core->cyclecount);
		}
	return ret;
}

uint16_t addr_absolute_y(core_t *core) {
	uint8_t lsb = core->ram[++(core->pc)];
	uint8_t msb = core->ram[++(core->pc)];
	uint16_t ret = ((msb << 8) + lsb) + core->y;
	if ( core->checkpageboundary == 1 )
		if ((ret & 0xFF00) != (msb << 8)) {
			#if CORE_DEBUG_TIMING == 1
			printf("Page Boundary Cross Penalty\n");
			#endif
			++(core->cyclecount);
		}
	return ret;
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
	uint16_t ret = ((core->ram[zpg+1] << 8) + core->ram[zpg]) + core->y;

	// Check Page Boundary Penalty:
	if ( core->checkpageboundary == 1 )
		if ((ret & 0xFF00) != (0x0000)) {
			#if CORE_DEBUG_TIMING == 1
			printf("Page Boundary Cross Penalty\n");
			#endif
			++(core->cyclecount);
		}

	return ret;
}

// Todo: addr_relative(core_t *core)

const uint8_t *init_cycle_table(core_t *core) {

	static const uint8_t ticktable[256] = {
/*        |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  A  |  B  |  C  |  D  |  E  |  F  |     */
/* 0 */      7,    6,    2,    8,    3,    3,    5,    5,    3,    2,    2,    2,    4,    4,    6,    6,  /* 0 */
/* 1 */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7,  /* 1 */
/* 2 */      6,    6,    2,    8,    3,    3,    5,    5,    4,    2,    2,    2,    4,    4,    6,    6,  /* 2 */
/* 3 */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7,  /* 3 */
/* 4 */      6,    6,    2,    8,    3,    3,    5,    5,    3,    2,    2,    2,    3,    4,    6,    6,  /* 4 */
/* 5 */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7,  /* 5 */
/* 6 */      6,    6,    2,    8,    3,    3,    5,    5,    4,    2,    2,    2,    5,    4,    6,    6,  /* 6 */
/* 7 */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7,  /* 7 */
/* 8 */      2,    6,    2,    6,    3,    3,    3,    3,    2,    2,    2,    2,    4,    4,    4,    4,  /* 8 */
/* 9 */      2,    6,    2,    6,    4,    4,    4,    4,    2,    5,    2,    5,    5,    5,    5,    5,  /* 9 */
/* A */      2,    6,    2,    6,    3,    3,    3,    3,    2,    2,    2,    2,    4,    4,    4,    4,  /* A */
/* B */      2,    5,    2,    5,    4,    4,    4,    4,    2,    4,    2,    4,    4,    4,    4,    4,  /* B */
/* C */      2,    6,    2,    8,    3,    3,    5,    5,    2,    2,    2,    2,    4,    4,    6,    6,  /* C */
/* D */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7,  /* D */
/* E */      2,    6,    2,    8,    3,    3,    5,    5,    2,    2,    2,    2,    4,    4,    6,    6,  /* E */
/* F */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7   /* F */
	};
	return ticktable;
}

// Initialise the 6502 core:
core_t *init_core() {

	core_t *core = (core_t*)malloc(sizeof(core_t));
	memset(core, 0, sizeof(core_t));

	// Fill memory with NOPs:
	core->ram = (uint8_t*)malloc(sizeof(uint8_t)*CORE_RAM_SIZE);
	memset(core->ram, NOP, sizeof(uint8_t)*CORE_RAM_SIZE);
	#if CORE_NESTEST == 1
	memset(core->ram, 0x00, sizeof(uint8_t)*CORE_RAM_SIZE);
	#endif

	// Init Stack Pointer:
	core->sp = CORE_STACK_POINTER_INIT;
	--(core->sp);
	--(core->sp);

	// Init flag that's always set:
	core->falways = 1;

	core->fintdisable = 1;

	// Point to our const ticktable:
	core->ticktable = init_cycle_table(core);

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

	#if CORE_NESTEST == 1
	if (addr_mode == addr_absolute) {
		uint8_t oldval = core->ram[addy];
		char absolute_val[32];
		sprintf(absolute_val, " = %.2X", oldval);
		strcat(core->d_str, absolute_val);
	}
	#endif


	*reg = core->ram[addy];
	set_fzero(core, reg);
	set_fsign(core, reg);
	++(core->pc);
}

// Generic Store Function:
static inline void instr_st(core_t *core, uint8_t *reg, uint16_t (*addr_mode)(core_t *core)) {

	uint16_t addy = addr_mode(core);

	#if CORE_NESTEST == 1
	if (addr_mode == addr_absolute) {
		uint8_t oldval = core->ram[addy];
		char absolute_val[32];
		sprintf(absolute_val, " = %.2X", oldval);
		strcat(core->d_str, absolute_val);
	}
	#endif

	core->ram[addy] = *reg;
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
	uint8_t val = core->ram[addr_mode(core)];
	uint8_t result = *reg - val; 
	core->fcarry = (*reg >= val) ? 1 : 0;
	#if CORE_NESTEST == 1
	//printf("CMP: VAL: %.2X RESULT: %.2X CARRY: %.2X\n", val, result, core->fcarry);
	#endif
	//core->fcarry = *reg > (val - (1 - core->fcarry));
	//core->fcarry = *reg > (val);
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

// Specific X <-> Stack Pointer Transfers:
static inline void instr_tsx(core_t *core) {
	core->x = core->sp;
	set_fzero(core, &(core->x));
	set_fsign(core, &(core->x));
	++(core->pc);
}

static inline void instr_txs(core_t *core) {
	core->sp= core->x;
	++(core->pc);
}

// Specific Push Status onto Stack:
static inline void instr_php(core_t *core) {

	core->falways = 1; // Reset our always flag
	//core->fvect = 1; // Set if BRK or PHP

	uint8_t status = 0x00;
	status |= (core->fcarry 	<< 0);
	status |= (core->fzero 		<< 1);
	status |= (core->fintdisable 	<< 2);
	status |= (core->fdec 		<< 3);
	status |= ((1)	 		<< 4);
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

// Specific Pull Status from Stack:
static inline void instr_plp(core_t *core) {

	++(core->sp);
	uint8_t status = core->ram[CORE_STACK_ADDRESS(core, 0)];

	core->fcarry = 		((status >> 0) & 0x01) ? 1 : 0;
	core->fzero = 		((status >> 1) & 0x01) ? 1 : 0;
	core->fintdisable = 	((status >> 2) & 0x01) ? 1 : 0;
	core->fdec = 		((status >> 3) & 0x01) ? 1 : 0;
	//core->fvect =		((status >> 4) & 0x01) ? 1 : 0; // Technically not accurate: http://nesdev.com/the%20'B'%20flag%20&%20BRK%20instruction.txt
	core->fvect =		0; // Technically not accurate: http://nesdev.com/the%20'B'%20flag%20&%20BRK%20instruction.txt
	core->falways =		1;
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
	#if CORE_NESTEST == 1
	strcat(core->d_str, "A");
	#endif 
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
	#if CORE_NESTEST == 1
	strcat(core->d_str, "A");
	#endif 
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
	#if CORE_NESTEST == 1
	strcat(core->d_str, "A");
	#endif 
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
	#if CORE_NESTEST == 1
	strcat(core->d_str, "A");
	#endif 
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
	uint16_t rts_address = core->pc;

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

// Specific ADC Funtion:
static inline void instr_adc(core_t *core, uint16_t (*addr_mode)(core_t *core)) {

	// Our value from our memory addressing mode:
	uint8_t mem = core->ram[addr_mode(core)];

	// Perform our addition:
	uint8_t sum = core->a + mem + core->fcarry;

	// Set our overflow
	// ~(a + b) == If the sign bits were the same (inverse of XOR)
	// & (a ^ sum) == If the sign bits of a+b were different
	// & 0x80 mask off the highest bit:
	core->foverflow = (~(core->a ^ mem) & (core->a ^ sum) & 0x80) ? 1 : 0;
		//#if CORE_NESTEST == 1
		//if ( core->foverflow == 1 ) {
			//printf("ADC OVERFLOW\n");
		//}
		//#endif

	// Detect a uint8_t wraparound and set carry flag if appropriate
	core->fcarry = core->a > UCHAR_MAX - (mem + core->fcarry);

	core->a = sum;

	// Zero and Sign:
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Specific SBC Funtion:
static inline void instr_sbc(core_t *core, uint16_t (*addr_mode)(core_t *core)) {

	// Our value from our memory addressing mode:
	uint8_t mem = core->ram[addr_mode(core)];

	// Credit to https://stackoverflow.com/questions/29193303/6502-emulation-proper-way-to-implement-adc-and-sbc
	mem = ~mem;

	// Perform our addition:
	uint8_t sum = core->a  + mem + core->fcarry;

	// Set our overflow
	// ~(a + b) == If the sign bits were the same (inverse of XOR)
	// & (a ^ sum) == If the sign bits of a+b were different
	// & 0x80 mask off the highest bit:
	core->foverflow = (~(core->a ^ mem) & (core->a ^ sum) & 0x80) ? 1 : 0;
		//#if CORE_NESTEST == 1
		//if ( core->foverflow == 1 ) {
			//printf("ADC OVERFLOW\n");
		//}
		//#endif

	// Detect a uint8_t wraparound and set carry flag if appropriate
	core->fcarry = core->a > UCHAR_MAX - (mem + core->fcarry);

	core->a = sum;

	// Zero and Sign:
	set_fzero(core, &(core->a));
	set_fsign(core, &(core->a));
	++(core->pc);
}

// Branches:
static inline void instr_bcc(core_t *core) {
	int8_t rel = (int8_t)core->ram[addr_immediate(core)];
	#if CORE_NESTEST == 1
	sprintf(core->d_str, "$%.4X", core->pc + rel + 1);
	#endif
	if (core->fcarry == 0 ) {
		if ( ((core->pc + rel + 1) & 0xFF00) != (core->pc & 0xFF00) )
			++(core->cyclecount);
		++(core->cyclecount);
		core->pc += rel + 1;
	} else {
		++(core->pc);
	}
}
static inline void instr_bcs(core_t *core) {
	int8_t rel = (int8_t)core->ram[addr_immediate(core)];
	#if CORE_NESTEST == 1
	sprintf(core->d_str, "$%.4X", core->pc + rel + 1);
	#endif
	if (core->fcarry == 1 ) {
		if ( ((core->pc + rel + 1) & 0xFF00) != (core->pc & 0xFF00) )
			++(core->cyclecount);
		++(core->cyclecount);
		core->pc += rel + 1;
	} else {
		++(core->pc);
	}
}
static inline void instr_bne(core_t *core) {
	int8_t rel = (int8_t)(core->ram[addr_immediate(core)]);
	#if CORE_NESTEST == 1
	sprintf(core->d_str, "$%.4X", core->pc + rel + 1);
	#endif
	if (core->fzero == 0 ) {
		if ( ((core->pc + rel + 1) & 0xFF00) != (core->pc & 0xFF00) )
			++(core->cyclecount);
		++(core->cyclecount);
		core->pc = core->pc + rel + 1;
	} else {
		++(core->pc);
	}
}
static inline void instr_beq(core_t *core) {
	int8_t rel = (int8_t)core->ram[addr_immediate(core)];
	#if CORE_NESTEST == 1
	sprintf(core->d_str, "$%.4X", core->pc + rel + 1);
	#endif
	if (core->fzero == 1 ) {
		if ( ((core->pc + rel + 1) & 0xFF00) != (core->pc & 0xFF00) )
			++(core->cyclecount);
		++(core->cyclecount);
		core->pc += rel + 1;
	} else {
		++(core->pc);
	}
}
static inline void instr_bpl(core_t *core) {
	int8_t rel = (int8_t)core->ram[addr_immediate(core)];
	#if CORE_NESTEST == 1
	sprintf(core->d_str, "$%.4X", core->pc + rel + 1);
	#endif
	if (core->fsign == 0 ) {
		if ( ((core->pc + rel + 1) & 0xFF00) != (core->pc & 0xFF00) )
			++(core->cyclecount);
		++(core->cyclecount);
		core->pc += rel + 1;
	} else {
		++(core->pc);
	}
}
static inline void instr_bmi(core_t *core) {
	int8_t rel = (int8_t)core->ram[addr_immediate(core)];
	#if CORE_NESTEST == 1
	sprintf(core->d_str, "$%.4X", core->pc + rel + 1);
	#endif
	if (core->fsign == 1 ) {
		if ( ((core->pc + rel + 1) & 0xFF00) != (core->pc & 0xFF00) )
			++(core->cyclecount);
		++(core->cyclecount);
		core->pc += rel + 1;
	} else {
		++(core->pc);
	}
}
static inline void instr_bvc(core_t *core) {
	int8_t rel = (int8_t)core->ram[addr_immediate(core)];
	#if CORE_NESTEST == 1
	sprintf(core->d_str, "$%.4X", core->pc + rel + 1);
	#endif
	if (core->foverflow == 0 ) {
		if ( ((core->pc + rel + 1) & 0xFF00) != (core->pc & 0xFF00) )
			++(core->cyclecount);
		++(core->cyclecount);
		core->pc += rel + 1;
	} else {
		++(core->pc);
	}
}
static inline void instr_bvs(core_t *core) {
	int8_t rel = (int8_t)core->ram[addr_immediate(core)];
	#if CORE_NESTEST == 1
	sprintf(core->d_str, "$%.4X", core->pc + rel + 1);
	#endif
	if (core->foverflow == 1 ) {
		if ( ((core->pc + rel + 1) & 0xFF00) != (core->pc & 0xFF00) )
			++(core->cyclecount);
		++(core->cyclecount);
		core->pc += rel + 1;
	} else {
		++(core->pc);
	}
}

void instr_brk(core_t *core) {
	
	++(core->pc);

	// Push MSB of PC onto Stack:
	core->ram[CORE_STACK_ADDRESS(core, 0)] = (core->pc >> 8);
	--(core->sp);

	// Push LSB of PC onto Stack:
	core->ram[CORE_STACK_ADDRESS(core, 0)] = (core->pc & 0x00FF);
	--(core->sp);
	
	// Push Status onto Stack
	uint8_t status = 0x00;
	core->fvect = 1;
	core->falways = 1;
	status |= (core->fcarry 	<< 0);
	status |= (core->fzero 		<< 1);
	status |= (core->fintdisable 	<< 2);
	status |= (core->fdec 		<< 3);
	status |= (core->fvect	 	<< 4); // Set Bit 4
	status |= (core->falways	<< 5);
	status |= (core->foverflow 	<< 6);
	status |= (core->fsign		<< 7);
	core->ram[CORE_STACK_ADDRESS(core, 0)] = status;
	--(core->sp); // Decrement Stack Pointer

	// Load our Interrupt handler and set our PC to that value
	core->pc = (uint16_t)(core->ram[CORE_IRQ_HI] << 8) + core->ram[CORE_IRQ_LO];

	#if CORE_DEBUG == 1
	printf("BRK, PC is now %.4x\n", core->pc);
	#endif
}

void instr_rti(core_t *core) {

	uint8_t status, pc_lo, pc_hi;

	// Pop Status:
	++(core->sp);
	status = core->ram[CORE_STACK_ADDRESS(core, 0)];
	core->fcarry = 		((status >> 0) & 0x01) ? 1 : 0;
	core->fzero = 		((status >> 1) & 0x01) ? 1 : 0;
	core->fintdisable = 	((status >> 2) & 0x01) ? 1 : 0;
	core->fdec = 		((status >> 3) & 0x01) ? 1 : 0;
	core->fvect =		((status >> 4) & 0x01) ? 1 : 0;
	core->falways =		(1);
	core->foverflow = 	((status >> 6) & 0x01) ? 1 : 0;
	core->fsign =		((status >> 7) & 0x01) ? 1 : 0;

	// Pop PC Low:
	++(core->sp);
	pc_lo = core->ram[CORE_STACK_ADDRESS(core, 0)];

	// Pop PC High:
	++(core->sp);
	pc_hi = core->ram[CORE_STACK_ADDRESS(core, 0)];

	// Reset our program counter:
	core->pc = (pc_hi << 8) + pc_lo;

}

void do_irq(core_t *core) {
	
	// No increment of PC

	#if CORE_DEBUG == 1
		printf("HW INTERRUPT!\n");
	#endif

	// Push MSB of PC onto Stack:
	core->ram[CORE_STACK_ADDRESS(core, 0)] = (core->pc >> 8);
	--(core->sp);

	// Push LSB of PC onto Stack:
	core->ram[CORE_STACK_ADDRESS(core, 0)] = (core->pc & 0x00FF);
	--(core->sp);
	
	// Push Status onto Stack
	uint8_t status = 0x00;
	core->fvect = 0; // Native IRQ, fvect is unset
	core->falways = 1;
	status |= (core->fcarry 	<< 0);
	status |= (core->fzero 		<< 1);
	status |= (core->fintdisable 	<< 2);
	status |= (core->fdec 		<< 3);
	status |= (core->fvect	 	<< 4); // Set Bit 4
	status |= (core->falways	<< 5);
	status |= (core->foverflow 	<< 6);
	status |= (core->fsign		<< 7);
	core->ram[CORE_STACK_ADDRESS(core, 0)] = status;
	--(core->sp); // Decrement Stack Pointer

	core->cyclecount += 7;

	// Load our Interrupt handler and set our PC to that value
	core->pc = (uint16_t)(core->ram[CORE_IRQ_HI] << 8) + core->ram[CORE_IRQ_LO];
}

void do_nmi(core_t *core) {
	
	// No increment of PC
	#if CORE_DEBUG == 1
		printf("NMI INTERRUPT!\n");
	#endif

	// Push MSB of PC onto Stack:
	core->ram[CORE_STACK_ADDRESS(core, 0)] = (core->pc >> 8);
	--(core->sp);

	// Push LSB of PC onto Stack:
	core->ram[CORE_STACK_ADDRESS(core, 0)] = (core->pc & 0x00FF);
	--(core->sp);
	
	// Push Status onto Stack
	uint8_t status = 0x00;
	core->fvect = 0; // Native IRQ, fvect is unset
	core->falways = 1;
	status |= (core->fcarry 	<< 0);
	status |= (core->fzero 		<< 1);
	status |= (core->fintdisable 	<< 2);
	status |= (core->fdec 		<< 3);
	status |= (core->fvect	 	<< 4); // Set Bit 4
	status |= (core->falways	<< 5);
	status |= (core->foverflow 	<< 6);
	status |= (core->fsign		<< 7);
	core->ram[CORE_STACK_ADDRESS(core, 0)] = status;
	--(core->sp); // Decrement Stack Pointer

	core->cyclecount += 7;

	// Load our Interrupt handler and set our PC to that value
	core->pc = (uint16_t)(core->ram[CORE_NMI_HI] << 8) + core->ram[CORE_NMI_LO];
}

// Main excecution cycle and instruction dispatch table:
void step_core(core_t *core) {

	uint8_t opcode = 0x00;

	// Reset Page Boundary Penalty Counters:
	core->checkpageboundary = 0;

	// Check for pending irq/nmis:
	switch ( core->interruptstate ) {
		case interruptnmi:
			do_nmi(core);
			core->interruptstate = interruptnone;
			#if CORE_DEBUG == 1
			printf("NMI FINISHED!\n");
			#endif
			return;

		case interruptirq:
			do_irq(core);
			core->interruptstate = interruptnone;
			#if CORE_DEBUG == 1
			printf("HW INTERRUPT FINISHED!\n");
			#endif
			return;

		default:
			break;
	}


	// Fetch:
	opcode = core->ram[core->pc];

	#if CORE_NESTEST == 1
		core->d_op = opcode;
		core->d_pc = core->pc;

		core->d_a = core->a;
		core->d_x = core->x;
		core->d_y = core->y;

		core->d_sp = core->sp;

		core->d_p |= (core->fcarry 		<< 0);
		core->d_p |= (core->fzero 		<< 1);
		core->d_p |= (core->fintdisable 	<< 2);
		core->d_p |= (core->fdec 		<< 3);
		core->d_p |= (core->fvect	 	<< 4); // Set Bit 4
		core->d_p |= (core->falways		<< 5);
		core->d_p |= (core->foverflow 		<< 6);
		core->d_p |= (core->fsign		<< 7);

		core->d_pc = core->pc;
	#endif	

	core->cyclecount += core->ticktable[core->ram[core->pc]];

	// Scaffolding Exit:
	if (opcode == 0xFF) {
		return;
	}

	// todo: Replace with a jump table?
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
			core->checkpageboundary = 1;
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
			core->checkpageboundary = 1;
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
			core->checkpageboundary = 1;
			instr_ld(core, &(core->a), addr_absolute_x);
			break;
		case LDA_A_Y:
			core->checkpageboundary = 1;
			instr_ld(core, &(core->a), addr_absolute_y);
			break;
		case LDA_IND_X:
			instr_ld(core, &(core->a), addr_indirect_x);
			break;
		case LDA_IND_Y:
			core->checkpageboundary = 1;
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
			instr_txs(core);
			break;
		case TSX:
			instr_tsx(core);
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
			core->checkpageboundary = 1;
			instr_adc(core, addr_absolute_x);
			break;
		case ADC_A_Y:
			core->checkpageboundary = 1;
			instr_adc(core, addr_absolute_y);
			break;
		case ADC_IND_X:
			instr_adc(core, addr_indirect_x);
			break;
		case ADC_IND_Y:
			core->checkpageboundary = 1;
			instr_adc(core, addr_indirect_y);
			break;

		case SBC_I:
			instr_sbc(core, addr_immediate);
			break;
		case SBC_ZPG:
			instr_sbc(core, addr_zeropage);
			break;
		case SBC_ZPG_X:
			instr_sbc(core, addr_zeropage_x);
			break;
		case SBC_A:
			instr_sbc(core, addr_absolute);
			break;
		case SBC_A_X:
			core->checkpageboundary = 1;
			instr_sbc(core, addr_absolute_x);
			break;
		case SBC_A_Y:
			core->checkpageboundary = 1;
			instr_sbc(core, addr_absolute_y);
			break;
		case SBC_IND_X:
			instr_sbc(core, addr_indirect_x);
			break;
		case SBC_IND_Y:
			core->checkpageboundary = 1;
			instr_sbc(core, addr_indirect_y);
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
			core->checkpageboundary = 1;
			instr_cmp(core, &(core->a), addr_absolute_x);
			break;
		case CMP_A_Y:
			core->checkpageboundary = 1;
			instr_cmp(core, &(core->a), addr_absolute_y);
			break;
		case CMP_IND_X:
			instr_cmp(core, &(core->a), addr_indirect_x);
			break;
		case CMP_IND_Y:
			core->checkpageboundary = 1;
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
			core->checkpageboundary = 1;
			instr_eor(core, addr_absolute_x);
			break;
		case EOR_A_Y:
			core->checkpageboundary = 1;
			instr_eor(core, addr_absolute_y);
			break;
		case EOR_IND_X:
			instr_eor(core, addr_indirect_x);
			break;
		case EOR_IND_Y:
			core->checkpageboundary = 1;
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
			core->checkpageboundary = 1;
			instr_ora(core, addr_absolute_x);
			break;
		case ORA_A_Y:
			core->checkpageboundary = 1;
			instr_ora(core, addr_absolute_y);
			break;
		case ORA_IND_X:
			instr_ora(core, addr_indirect_x);
			break;
		case ORA_IND_Y:
			core->checkpageboundary = 1;
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

	// Branches:
		case BCC:
			instr_bcc(core);
			break;
		case BCS:
			instr_bcs(core);
			break;
		case BEQ:
			instr_beq(core);
			break;
		case BMI:
			instr_bmi(core);
			break;
		case BNE:
			instr_bne(core);
			break;
		case BPL:
			instr_bpl(core);
			break;
		case BVS:
			instr_bvs(core);
			break;
		case BVC:
			instr_bvc(core);
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
		case CLD:
			core->fdec = 0;
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
		case SED:
			core->fdec = 1;
			++(core->pc);
			break;

	// Misc:
		case NOP:
			++(core->pc);
			break;
		case BRK:
			instr_brk(core);
			break;
		case RTI:
			instr_rti(core);
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
