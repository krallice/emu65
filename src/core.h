#ifndef CORE_H
#define CORE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>
#include <unistd.h>

// Debug Macros:
#define CORE_DEBUG 0
#define CORE_DEBUG_TIMING 0

#define CORE_NESTEST 1

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

// Timing Macros:
#define CORE_HZ 2

// Initialisation Macros:
#define CORE_RAM_SIZE 0xFFFF
#define CORE_OPCODE_SIZE 0xFF

// Stack Macros:
#define CORE_STACK_PAGE 0x01 // 1st Page for the Stack
#define CORE_STACK_SIZE 0xFF // Size of the stack (0x0100 -> 0x01FF)
#define CORE_STACK_POINTER_INIT 0xFF // Initial Address to init the SP to
#define CORE_STACK_ADDRESS(X, Y) ((CORE_STACK_PAGE << 8) + (uint8_t)(X->sp + Y)) // Return Full 2 Byte Stack Address (0x01XX)

// Interrupt Macros:
#define CORE_IRQ_LO	0xFFFE
#define CORE_IRQ_HI	0xFFFF
#define CORE_NMI_LO	0xFFFA
#define CORE_NMI_HI	0xFFFB

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

// Arithmetic
#define ADC_I		0x69
#define ADC_ZPG		0x65
#define ADC_ZPG_X	0x75
#define ADC_A		0x6D
#define ADC_A_X		0x7D
#define ADC_A_Y		0x79
#define ADC_IND_X	0x61
#define ADC_IND_Y	0x71

#define SBC_I		0xE9
#define SBC_ZPG		0xE5
#define SBC_ZPG_X	0xF5
#define SBC_A		0xED
#define SBC_A_X		0xFD
#define SBC_A_Y		0xF9
#define SBC_IND_X	0xE1
#define SBC_IND_Y	0xF1

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

// Branches:
#define BCC		0x90
#define BCS		0xB0
#define BEQ		0xF0
#define BMI		0x30
#define BNE		0xD0
#define BPL		0x10
#define BVC		0x50
#define BVS		0x70

// Jumps:
#define JMP_A		0x4C
#define JMP_IND		0x6C
#define JSR_A		0x20
#define RTS		0x60

// Flag Sets/Clears
#define CLC		0x18
#define CLI		0x58
#define CLV		0xB8
#define CLD		0xD8
#define SEC		0x38
#define SEI		0x78
#define SED		0xF8

// Misc
#define NOP		0xEA
#define BRK		0x00 //To Implement
#define RTI		0x40

// Unofficials
#define NOP_3_1		0x1C
#define NOP_3_2		0x3C
#define NOP_3_3		0x5C
#define NOP_3_4		0x7C
#define NOP_3_5		0xDC
#define NOP_3_6		0xFC
#define NOP_3_7		0x0C

#define NOP_2_1		0x14
#define NOP_2_2		0x34
#define NOP_2_3		0x54
#define NOP_2_4		0x74
#define NOP_2_5		0xD4
#define NOP_2_6		0xF4
#define NOP_2_7		0x04
#define NOP_2_8		0x44
#define NOP_2_9		0x64

#define NOP_1_1		0x1A
#define NOP_1_2		0x3A
#define NOP_1_3		0x5A
#define NOP_1_4		0x7A
#define NOP_1_5		0xDA
#define NOP_1_6		0xFA
#define NOP_1_7		0x0A

typedef enum {
	interruptnone,
	interruptirq,
	interruptnmi
} interruptstate_t;

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
        uint8_t foverflow :1;
        uint8_t fsign :1;

	// RAM:
	uint8_t *ram;

	// Ticks:
	const uint8_t *ticktable; // Opcode Tick Table
	uint8_t checkpageboundary; // Flag to indicate that a page boundary crossing could cause a penalty:
	uint16_t cyclecount; // Current Cycle Count

	interruptstate_t interruptstate;

		// Debug:
		uint16_t d_pc;
		uint8_t d_op;

		uint8_t d_op1_en;
		uint8_t d_op2_en;
		uint8_t d_op1;
		uint8_t d_op2;
		char d_str[256];

		uint8_t d_a;
		uint8_t d_x;
		uint8_t d_y;

		uint8_t d_sp;
		uint8_t d_p;

} core_t;

// Prototypes:
core_t *init_core(void);
const uint8_t *init_cycle_table(core_t *core);
void dump_core_state(core_t *core);
void step_core(core_t *core);
#endif
