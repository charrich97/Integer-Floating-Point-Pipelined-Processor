#ifndef SIM_PIPE_FP_H_
#define SIM_PIPE_FP_H_

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

#define UNDEFINED 0xFFFFFFFF 
#define DATA_UNDEF 0
#define NUM_SP_REGISTERS 9
#define NUM_GP_REGISTERS 32
#define NUM_OPCODES 29
#define NUM_STAGES 5

#define ASSERT( condition, statement, ... ) \
   if( !(condition) ) { \
      printf( "[ASSERT] In File: %s, Line: %d => " #statement "\n", __FILE__, __LINE__, ##__VA_ARGS__ ); \
      abort(); \
   }

using namespace std;

typedef enum {PC, NPC, IR, A, B, IMM, COND, ALU_OUTPUT, LMD} sp_register_t;

typedef enum {ADD, SUB, XOR, OR, AND, MULT, DIV, BEQZ, BNEZ, BLTZ, BGTZ, BLEZ, BGEZ, ADDI, SUBI, XORI, ORI, ANDI, JUMP, EOP, NOP, LW, SW, LWS, SWS, ADDS, SUBS, MULTS, DIVS} opcode_t;

const string opcode_str[] = {"ADD", "SUB", "XOR", "OR", "AND", "MULT", "DIV", "BEQZ", "BNEZ", "BLTZ", "BGTZ", "BLEZ", "BGEZ", "ADDI", "SUBI", "XORI", "ORI", "ANDI", "JUMP", "EOP", "NOP", "LW", "SW", "LWS", "SWS", "ADDS", "SUBS", "MULTS", "DIVS"};

typedef enum {IF, ID, EX, MEM, WB} stage_t;

typedef enum {INTEGER, ADDER, MULTIPLIER, DIVIDER, EXE_UNIT_SIZE} exe_unit_t;

typedef struct instruction_t* instructPT;

struct instruction_t{
   opcode_t           opcode;
   uint32_t           dest;
   uint32_t           src1;
   uint32_t           src2;
   uint32_t           imm;
   bool               dest_op;
   bool               src1_op;
   bool               src2_op;
   bool               dest_float_op;
   bool               src1_float_op;
   bool               src2_float_op;
   bool               is_stall;
   bool               branch_op;

   instruction_t(){
      nop();
   }

   void print(){
      cout << "Opcode: " << opcode_str[opcode] << ", dest: " << dest << ", src1: " << src1 << ", src2: " << src2 << ", imm: " << imm << ", dest_op: " << dest_op << ", src1_op: " << src1_op << ", src2_op: " << src2_op << ", dest_float_op: " << dest_float_op << ", src1_float_op: " << src1_float_op << ", src2_float_op: " << src2_float_op << ", is_stall: " << is_stall << ", branch_op: " << branch_op << endl;
   }

   void nop(){
      opcode        = NOP;
      dest           = UNDEFINED;
      src1          = UNDEFINED;
      src2          = UNDEFINED;
      imm           = UNDEFINED;
      dest_op       = false;
      src1_op       = false;
      src2_op       = false;
      is_stall      = false;
      branch_op     = false;
      dest_float_op = false;
      src1_float_op = false;
      src2_float_op = false;
   }

   void set_stall(){
      nop();
      is_stall   = true;
   }
};

class sim_pipe_fp{

   public:
 

      struct execLaneT{
         instruction_t      instruction;
         int            latency_exe;
         unsigned       b;
         unsigned       exNpc;

         execLaneT(){
            latency_exe            = 0;
         }

      };

      struct execUnitT{
         execLaneT      *exe_pipe_units;
         int            num_exe_pipe_units;
         int            latency;

         execUnitT(){
            exe_pipe_units          = NULL;
            num_exe_pipe_units       = 0;
            latency        = 0;
         }

         void init(int num_exe_pipe_units, int latency){
            ASSERT( latency > 0, "Impractical latency found (=%d)", latency );
            ASSERT( num_exe_pipe_units > 0, "Unsupported number of exe_pipe_units (=%d)", num_exe_pipe_units );
            this->num_exe_pipe_units += num_exe_pipe_units;
            this->latency   = latency;
            exe_pipe_units           = (execLaneT*)realloc(exe_pipe_units, this->num_exe_pipe_units * sizeof(execLaneT));
         }
      };
      

      int               cycleCount;
      int               instruction_count;
      int               latCount;
      int               stall_count;
      bool              latency;
      int               instMemSize;

      struct int_file_t{
         int            value;
         int            busy;
      };

      int_file_t        int_file[NUM_GP_REGISTERS];

      struct fp_file_t{
         float          value;
         int            busy;
      };

      fp_file_t          fp_file[NUM_GP_REGISTERS];

      execUnitT         float_point_exe_reg[EXE_UNIT_SIZE];

      unsigned          sp_registers[NUM_STAGES][NUM_SP_REGISTERS];

      instruction_t         instruction_register[NUM_STAGES];

      unsigned char     *data_memory;
      unsigned          data_memory_size;

      instructPT        *instMemory;
      unsigned          dataMemSize;

      unsigned          memLatency;
      unsigned          data_memory_latency_count;
      unsigned          baseAddress;

   public:

      sim_pipe_fp(unsigned data_mem_size, unsigned data_mem_latency);

      ~sim_pipe_fp();

      void init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances=1);
      instruction_t fetchInstruction ( uint32_t pc );
      bool check_busy_status(unsigned regNo, bool isF);
      bool check_branch();
      exe_unit_t convert_op_to_exe_unit(opcode_t opcode);
      int exLatency(opcode_t opcode);
      void clear_sp_register(stage_t s); 

      void     MIPS_IF(bool stall);
      bool     MIPS_ID(); 
      uint32_t address_gen(instruction_t instruct);
      unsigned alu (unsigned _value1, unsigned _value2, bool value1F, bool value2F, opcode_t opcode);
      unsigned aluF (unsigned _value1, unsigned _value2, bool value1F, bool value2F, opcode_t opcode);
      void     MIPS_EXE();
      bool     MIPS_MEM();
      bool     MIPS_WB();

      int parse(const char *filename);
      int  labelToPC( const char* filename, const char* label, uint32_t pc_index );

      void load_program(const char *filename, unsigned base_address=0x0);

      void run(unsigned cycles=0);

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
      void set_sp_register(sp_register_t reg, stage_t s, uint32_t value);
      unsigned regRead(unsigned reg, bool isF);

      //returns value of the specified integer general purpose register
      int get_int_register(unsigned reg);

      //set the value of the given integer general purpose register to "value"
      void set_int_register(unsigned reg, int value);

      //returns value of the specified floating point general purpose register
      float get_fp_register(unsigned reg);

      //set the value of the given floating point general purpose register to "value"
      void set_fp_register(unsigned reg, float value);

      //returns the IPC
      float get_IPC();

      //returns the number of instructions fully executed
      unsigned get_instructions_executed();

      //returns the number of stalls inserted
      unsigned get_stalls();

      //returns the number of clock cycles
      unsigned get_clock_cycles();

      //prints the content of the data memory within the specified address range
      void print_memory(unsigned start_address, unsigned end_address);

      // writes an integer value to data memory at the specified address (use little-endian format: https://en.wikipedia.org/wiki/Endianness)
      void write_memory(unsigned address, unsigned value);

      unsigned read_memory(unsigned address);
      instruction_t execInst(int& count, uint32_t& b, uint32_t& npc);

      //prints the values of the registers 
      void print_registers();
      
      int getMaxTtl();
      uint32_t parseReg( FILE* trace, bool& is_float );
      inline unsigned float2unsigned(float value);
      inline float unsigned2float(unsigned value);
};

#endif /*SIM_PIPE_FP_H_*/
