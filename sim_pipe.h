#ifndef SIM_PIPE_H_
#define SIM_PIPE_H_

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <cassert>
#include <map>
#include <cstdlib>
#include <malloc.h>

#include <vector>
#include <algorithm>

#define UNDEFINED 0xFFFFFFFF //constant used to initialize registers
#define NUM_SP_REGISTERS 9
#define NUM_GP_REGISTERS 32
#define NUM_OPCODES 16
#define NUM_STAGES 5

#define ASSERT( condition, statement, ... ) \
   if( !(condition) ) { \
      printf( "[ASSERT] In File: %s, Line: %d => " #statement "\n", __FILE__, __LINE__, ##__VA_ARGS__ ); \
      abort(); \
   }

using namespace std;
typedef enum {PC, NPC, IR, A, B, IMM, COND, ALU_OUTPUT, LMD} sp_register_t;

typedef enum {LW, SW, ADD, SUB, XOR, OR, AND, MULT, DIV, ADDI, SUBI, XORI, ORI, ANDI, BEQZ, BNEZ, BLTZ, BGTZ, BLEZ, BGEZ, JUMP, EOP, NOP} opcode_t;
const string opcode_str[] = {"LW", "SW", "ADD", "SUB", "XOR", "OR", "AND", "MULT", "DIV", "ADDI", "SUBI", "XORI", "ORI", "ANDI", "BEQZ", "BNEZ", "BLTZ", "BGTZ", "BLEZ", "BGEZ", "JUMP", "EOP", "NOP"};

typedef enum {IF, ID, EX, MEM, WB} stage_t;

typedef struct instruction_t* instruction_pointer;

struct instruction_t{
   opcode_t           opcode;
   unsigned           dest;
   unsigned           src1;
   unsigned           src2;
   unsigned           immediate;
   bool               dest_op;
   bool               src1_op;
   bool               src2_op;
   bool               branch_op;
   bool               is_stall;

   instruction_t(){
      no_operation();
   }

   void no_operation(){
      opcode     = NOP;
      dest       = UNDEFINED;
      src1       = UNDEFINED;
      src2       = UNDEFINED;
      immediate  = UNDEFINED;
      dest_op    = false;
      src1_op    = false;
      src2_op    = false;
      is_stall   = false;
      branch_op  = false;
   }

   void set_stall(){
      no_operation();
      is_stall   = true;
   }
};


class sim_pipe{

public:
   struct int_file_t{
      int            value;
      int            busy;
   };

   int                  cc_count;
   int                  instCount;
   int                  stall_count;
   int                  latCount;
   bool                 latency;

   instruction_t        instruction_register[NUM_STAGES];
   int_file_t           int_file[NUM_GP_REGISTERS];
   unsigned             sp_registers[NUM_STAGES][NUM_SP_REGISTERS];

   unsigned char*       data_memory;

   unsigned             baseAddress;
   instruction_pointer  *instruct_memory;

   unsigned             dataMemSize;
   unsigned             Data_Memory_Latency;
   unsigned             data_memory_latency_count;

   sim_pipe(unsigned data_mem_size, unsigned data_mem_latency);

   ~sim_pipe();
   instruction_t index_instruction ( unsigned pc );

   void     MIPS_IF(bool stall);
   bool     MIPS_ID(); 
   void     MIPS_EXE();
   bool     MIPS_MEM();
   bool     MIPS_WB();

   unsigned address_gen(instruction_t instruct);
   unsigned alu (unsigned value1, unsigned value2, opcode_t opcode);

   int parse(const char *filename);
   int labelToPC( const char* filename, const char* label, unsigned pc_index );

   //loads the assembly program in file "filename" in instruction memory at the specified address
   void load_program(const char *filename, unsigned base_address=0x0);

   //runs the simulator for "cycles" clock cycles (run the program to completion if cycles=0) 
   void run(unsigned cycles=0);
	
	//resets the state of the simulator
        /* Note: 
	   - registers should be reset to UNDEFINED value 
	   - data memory should be reset to all 0xFF values
	*/
	void reset();

	// returns value of the specified special purpose register for a given stage (at the "entrance" of that stage)
        // if that special purpose register is not used in that stage, returns UNDEFINED
        //
        // Examples (refer to page C-37 in the 5th edition textbook, A-32 in 4th edition of textbook)::
        // - get_sp_register(PC, IF) returns the value of PC
        // - get_sp_register(NPC, ID) returns the value of IF/ID.NPC
        // - get_sp_register(NPC, EX) returns the value of ID/EX.NPC
        // - get_sp_register(ALU_OUTPUT, MEM) returns the value of EX/MEM.ALU_OUTPUT
        // - get_sp_register(ALU_OUTPUT, WB) returns the value of MEM/WB.ALU_OUTPUT
	// - get_sp_register(LMD, ID) returns UNDEFINED
	/* Note: you are allowed to use a custom format for the IR register.
           Therefore, the test cases won't check the value of IR using this method. 
	   You can add an extra method to retrieve the content of IR */
	unsigned get_sp_register(sp_register_t reg, stage_t stage);
   void set_sp_register(sp_register_t reg, stage_t s, unsigned value);
  void clear_sp_register(stage_t s);

	//returns value of the specified general purpose register
	int get_gp_register(unsigned reg);

	// set the value of the given general purpose register to "value"
	void set_gp_register(unsigned reg, int value);

	//returns the IPC
	float get_IPC();

	//returns the number of instructions fully executed
	unsigned get_instructions_executed();

	//returns the number of stalls added by processor
	unsigned get_stalls();

	//returns the number of clock cycles
	unsigned get_clock_cycles();

	//prints the content of the data memory within the specified address range
	void print_memory(unsigned start_address, unsigned end_address);

	// writes an integer value to data memory at the specified address (use little-endian format: https://en.wikipedia.org/wiki/Endianness)
	void write_memory(unsigned address, unsigned value);

   unsigned read_memory(unsigned address);

	//prints the values of the registers 
	void print_registers();

};

#endif /*SIM_PIPE_H_*/
