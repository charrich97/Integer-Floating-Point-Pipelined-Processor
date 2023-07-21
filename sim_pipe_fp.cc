#include "sim_pipe_fp.h"
#include <stdlib.h>
#include <iostream>
#include <iomanip>

using namespace std;

static const char *reg_names[NUM_SP_REGISTERS] = {"PC", "NPC", "IR", "A", "B", "IMM", "COND", "ALU_OUTPUT", "LMD"};
static const char *stage_names[NUM_STAGES] = {"IF", "ID", "EX", "MEM", "WB"};

map <string, opcode_t> opcode_2str = { {"LW", LW}, {"SW", SW}, {"ADD", ADD}, {"ADDI", ADDI}, {"SUB", SUB}, {"SUBI", SUBI}, {"XOR", XOR}, {"XORI", XORI}, {"OR", OR}, {"ORI", ORI}, {"AND", AND}, {"ANDI", ANDI}, {"MULT", MULT}, {"DIV", DIV}, {"BEQZ", BEQZ}, {"BNEZ", BNEZ}, {"BLTZ", BLTZ}, {"BGTZ", BGTZ}, {"BLEZ", BLEZ}, {"BGEZ", BGEZ}, {"JUMP", JUMP}, {"EOP", EOP}, {"NOP", NOP}, {"LWS", LWS}, {"SWS", SWS}, {"ADDS", ADDS}, {"SUBS", SUBS}, {"MULTS", MULTS}, {"DIVS", DIVS}};

sim_pipe_fp::sim_pipe_fp(unsigned mem_size, unsigned mem_latency){
   dataMemSize  = mem_size;
   memLatency   = mem_latency;
   instMemory   = NULL;
   stall_count          = 0;
   reset();
}

sim_pipe_fp::~sim_pipe_fp(){
}

void sim_pipe_fp::init_exec_unit(exe_unit_t exec_unit, unsigned latency, unsigned instances){
   float_point_exe_reg[exec_unit].init(instances, latency+1);
}

void sim_pipe_fp::load_program(const char *filename, unsigned base_address){
   instMemSize            = parse(filename);
   sp_registers[IF][PC]  = base_address;
   baseAddress      = base_address;
}

instruction_t sim_pipe_fp::fetchInstruction ( unsigned pc ) {
   int      index     = (pc - baseAddress)/4;
   ASSERT((index >= 0) && (index < instMemSize), "out of bound access of instruction memory %d", index);
   instruction_t instruction = *(instMemory[index]);
   if(instruction.opcode != EOP)
      instruction_count++;
   return instruction;
}

void sim_pipe_fp::MIPS_IF(bool stall) {
   bool cond                       = get_sp_register(COND, MEM);
   uint32_t alu_output             = get_sp_register(ALU_OUTPUT, MEM);
   sp_registers[IF][PC]            = cond ? alu_output : sp_registers[IF][PC];
   uint32_t currentFetchPC         = sp_registers[IF][PC];


   if( !stall ){
      instruction_t instruction           = fetchInstruction(currentFetchPC);
      if(instruction.opcode != EOP )
         set_sp_register(PC, IF, currentFetchPC + 4);

      sp_registers[ID][NPC]       = sp_registers[IF][PC];

      instruction_register[ID]         = instruction;
   }
}

bool sim_pipe_fp::check_busy_status(unsigned regNo, bool isF) {
   return isF ? fp_file[regNo].busy : int_file[regNo].busy;
}

bool sim_pipe_fp::check_branch(){
   for(int i = 0; i < float_point_exe_reg[INTEGER].num_exe_pipe_units; i++) {
      if( float_point_exe_reg[INTEGER].exe_pipe_units[i].instruction.branch_op ) {
         return true;
      }
   }
   return false;
}

exe_unit_t sim_pipe_fp::convert_op_to_exe_unit(opcode_t opcode){
   exe_unit_t unit;
   switch( opcode ){
      case ADD ... AND:
      case BEQZ ... SWS:
         unit = INTEGER;
         break;

      case ADDS:
      case SUBS:
         unit = ADDER;
         break;

      case MULTS:
      case MULT:
         unit = MULTIPLIER;
         break;

      case DIVS:
      case DIV:
         unit = DIVIDER;
         break;

      default: 
         ASSERT (false, "Opcode not supported");
         unit = INTEGER;
         break;
   }
   ASSERT( float_point_exe_reg[unit].num_exe_pipe_units > 0, "No exe_pipe_units found for opcode: %s", opcode_str[opcode].c_str());
   return unit;
}


int sim_pipe_fp::exLatency(opcode_t opcode) {
   return float_point_exe_reg[convert_op_to_exe_unit(opcode)].latency;
}

//----------------------------------------------------------------------
// MIPS: Instruction Decode
//----------------------------------------------------------------------
bool sim_pipe_fp::MIPS_ID() {
   bool stall_execute                   = false;
   instruction_t instruction            = instruction_register[ID];
   int latency                          = instruction.is_stall ? 0 : exLatency(instruction.opcode);
   sp_registers[EX][NPC]                = sp_registers[ID][NPC];

   
  //-------------------------------------------------------------------
  // Check for RAW Hazards
  //-------------------------------------------------------------------
   if( (instruction.src1_op && check_busy_status(instruction.src1, instruction.src1_float_op)) || 
    (instruction.src2_op && check_busy_status(instruction.src2, instruction.src2_float_op)) ) {
      stall_execute                 = true;
   }

   for(int i = 0; i < EXE_UNIT_SIZE && !stall_execute; i++){
      for(int j = 0; j < float_point_exe_reg[i].num_exe_pipe_units; j++){
         execLaneT exe_pipe_unit = float_point_exe_reg[i].exe_pipe_units[j];
         if(latency == exe_pipe_unit.latency_exe && latency != 0) {
            stall_execute           = true;
            break;
         }
      }
   }

  //-------------------------------------------------------------------
  // Check for WAW Hazards
  //-------------------------------------------------------------------
   if(!stall_execute) { 
      for(int i = 0; i < EXE_UNIT_SIZE && !stall_execute; i++){
         for(int j = 0; j < float_point_exe_reg[i].num_exe_pipe_units; j++) {
            execLaneT exe_pipe_unit       = float_point_exe_reg[i].exe_pipe_units[j];
            if( (instruction.dest_op && exe_pipe_unit.instruction.dest_op)                && 
                  (instruction.dest == exe_pipe_unit.instruction.dest)                    && 
                  (instruction.dest_float_op == exe_pipe_unit.instruction.dest_float_op)  && 
                  (latency <= exe_pipe_unit.latency_exe && latency != 0) ){       
               stall_execute     = true;
               break;
            }
         }
      }
   }

  //-------------------------------------------------------------------
  // Check for Free Executional Units- Structural Hazard
  //-------------------------------------------------------------------
   if(!stall_execute) {
      bool is_exe_unit_avail    = false;
      for(int j = 0; j < float_point_exe_reg[convert_op_to_exe_unit(instruction.opcode)].num_exe_pipe_units; j++){
         if(float_point_exe_reg[convert_op_to_exe_unit(instruction.opcode)].exe_pipe_units[j].latency_exe == 0) {
            is_exe_unit_avail   = true;
            break;
         }
      }
      stall_execute            = !is_exe_unit_avail;
   }
   

   bool branch_op                     = instruction.branch_op || instruction_register[EX].branch_op || check_branch();

  //-------------------------------------------------------------------
  // Check for Control Hazards
  //-------------------------------------------------------------------

   if( branch_op && !stall_execute ) { 
      instruction_register[ID].set_stall();
      if(!(instruction.opcode == EOP)) stall_count++;
      clear_sp_register(ID);
   }

   if( stall_execute ){
      instruction_register[EX].set_stall();
      clear_sp_register(EX);
      if(!(instruction.opcode == EOP))  stall_count++;
      else sp_registers[EX][NPC]       = sp_registers[ID][NPC];
      return true;
   } 
   else{
      if(instruction.dest_op){
         int_file[instruction.dest].busy += !(instruction.dest_float_op); 
         fp_file[instruction.dest].busy  +=  (instruction.dest_float_op); 
      }

      sp_registers[EX][A]                 = (instruction.src1_op) ? regRead(instruction.src1, instruction.src1_float_op) : UNDEFINED;
      sp_registers[EX][B]                 = (instruction.src2_op) ? regRead(instruction.src2, instruction.src2_float_op) : UNDEFINED;
      sp_registers[EX][IMM]               = instruction.imm;

      instruction_register[EX]            = instruction;
      return (instruction.opcode == EOP || branch_op);
   }
}
   
//----------------------------------------------------------------------
// Floating-Point ALU
//----------------------------------------------------------------------
unsigned sim_pipe_fp::aluF (unsigned _value1, unsigned _value2, bool value1F, bool value2F, opcode_t opcode){
   float output;
   float value1 = value1F ? unsigned2float(_value1) : _value1;
   float value2 = value2F ? unsigned2float(_value2) : _value2; 

   switch( opcode ){
      case ADD:
      case BEQZ ... ADDI:
      case JUMP ... ADDS:
         output      = value1 + value2;
         break;

      case SUB:
      case SUBI:
      case SUBS:
         output      = value1 - value2;
         break;

      case XOR:
      case XORI:
         output      = (unsigned)value1 ^ (unsigned)value2;
         break;

      case AND:
      case ANDI:
         output      = (unsigned)value1 & (unsigned)value2;
         break;

      case OR:
      case ORI:
         output      = (unsigned)value1 | (unsigned)value2;
         break;

      case MULT:
      case MULTS:   
         output      = value1 * value2;
         break;

      case DIV:
      case DIVS:
         output      = value1 / value2;
         break;

      default: 
         output      = UNDEFINED;
         break;
   }
   return float2unsigned(output);
}

//----------------------------------------------------------------------
// Address Generator
//----------------------------------------------------------------------
uint32_t sim_pipe_fp::address_gen ( instruction_t instruct) {
   return (instruct.imm + regRead(instruct.src1, instruct.src1_float_op));
}

//----------------------------------------------------------------------
// Integer ALU
//----------------------------------------------------------------------
unsigned sim_pipe_fp::alu (unsigned _value1, unsigned _value2, bool value1F, bool value2F, opcode_t opcode){

   if( value1F || value2F ) return aluF(_value1, _value2, value1F, value2F, opcode);

   unsigned output;
   unsigned value1   = value1F ? float2unsigned(_value1) : _value1;
   unsigned value2   = value2F ? float2unsigned(_value2) : _value2; 

   switch( opcode ){
      case ADD:
      case BEQZ ... ADDI:
      case JUMP ... ADDS:
         output      = value1 + value2;
         break;

      case SUB:
      case SUBI:
      case SUBS:
         output      = value1 - value2;
         break;

      case XOR:
      case XORI:
         output      = (unsigned)value1 ^ (unsigned)value2;
         break;

      case AND:
      case ANDI:
         output      = (unsigned)value1 & (unsigned)value2;
         break;

      case OR:
      case ORI:
         output      = (unsigned)value1 | (unsigned)value2;
         break;

      case MULT:
      case MULTS:   
         output      = value1 * value2;
         break;

      case DIV:
      case DIVS:
         output      = value1 / value2;
         break;

      default: 
         output      = UNDEFINED;
         break;
   }
   return output;
}

instruction_t sim_pipe_fp::execInst(int& count, uint32_t& b, uint32_t& npc){
   instruction_t instruction;
   count = 0;
   for(int i = 0; i < EXE_UNIT_SIZE; i++){
      for(int j = 0; j < float_point_exe_reg[i].num_exe_pipe_units; j++){
         if( float_point_exe_reg[i].exe_pipe_units[j].latency_exe != 0 ) {
            float_point_exe_reg[i].exe_pipe_units[j].latency_exe--;
            if( float_point_exe_reg[i].exe_pipe_units[j].latency_exe == 0 ) {
               count++;
               instruction  = float_point_exe_reg[i].exe_pipe_units[j].instruction;
               b            = float_point_exe_reg[i].exe_pipe_units[j].b;
               npc          = float_point_exe_reg[i].exe_pipe_units[j].exNpc;
            }
         } else{
            float_point_exe_reg[i].exe_pipe_units[j].instruction.set_stall();
         }
      }
   }
   ASSERT ( count <= 1, "STRUCTURAL HAZARD AT MEM DETECTED" );
   return instruction;
}

int sim_pipe_fp::getMaxTtl() {
   int latency_exe = 0;
   for(int i = 0; i < EXE_UNIT_SIZE; i++){
      for(int j = 0; j < float_point_exe_reg[i].num_exe_pipe_units; j++){
         latency_exe = max(latency_exe, float_point_exe_reg[i].exe_pipe_units[j].latency_exe);
      }
   }
   return latency_exe;
}

void sim_pipe_fp::MIPS_EXE() {

   instruction_t instruction                   = instruction_register[EX]; 
   clear_sp_register(MEM);
   sp_registers[MEM][COND]     = 0;

   for(int j = 0; j < float_point_exe_reg[convert_op_to_exe_unit(instruction.opcode)].num_exe_pipe_units; j++){
      if(float_point_exe_reg[convert_op_to_exe_unit(instruction.opcode)].exe_pipe_units[j].latency_exe == 0) {
         float_point_exe_reg[convert_op_to_exe_unit(instruction.opcode)].exe_pipe_units[j].instruction = instruction;
         if(instruction.opcode == EOP) {
            float_point_exe_reg[convert_op_to_exe_unit(instruction.opcode)].exe_pipe_units[j].latency_exe   = getMaxTtl() + 1;
         }
         else 
            float_point_exe_reg[convert_op_to_exe_unit(instruction.opcode)].exe_pipe_units[j].latency_exe   = instruction.is_stall ? 0 : exLatency(instruction.opcode);
         float_point_exe_reg[convert_op_to_exe_unit(instruction.opcode)].exe_pipe_units[j].b        = sp_registers[EX][B];
         float_point_exe_reg[convert_op_to_exe_unit(instruction.opcode)].exe_pipe_units[j].exNpc    = sp_registers[EX][NPC];
         break;
      }
   }

   int count;
   uint32_t b;
   uint32_t npc;
   instruction = execInst(count, b, npc);

   if(count != 0) {
      uint32_t src1 = instruction.src1;
      uint32_t src2 = instruction.src2;
      bool src1_float_op    = instruction.src1_float_op;
      bool src2_float_op    = instruction.src2_float_op;

      sp_registers[MEM][B] = b;
      switch(instruction.opcode) {
         case LW ... SWS:
            sp_registers[MEM][ALU_OUTPUT] = address_gen (instruction);
            break;

         case ADD ... DIV:
         case ADDS ... DIVS:
            sp_registers[MEM][ALU_OUTPUT] = alu(regRead(src1, src1_float_op), regRead(src2, src2_float_op), src1_float_op, src2_float_op, instruction.opcode);
            break;

         case ADDI ... ANDI:
            sp_registers[MEM][ALU_OUTPUT] = alu(regRead(src1, src1_float_op), instruction.imm, src1_float_op, false, instruction.opcode);
            break;

         case BLTZ:
            sp_registers[MEM][COND]       = regRead(src1, src1_float_op) < 0;
            sp_registers[MEM][ALU_OUTPUT] = alu(npc, instruction.imm, false, false, instruction.opcode);
            break;

         case BNEZ:
            sp_registers[MEM][ALU_OUTPUT] = alu(npc, instruction.imm, false, false, instruction.opcode);
            sp_registers[MEM][COND]       = regRead(src1, src1_float_op) != 0;
            break;

         case BEQZ:
            sp_registers[MEM][ALU_OUTPUT] = alu(npc, instruction.imm, false, false, instruction.opcode);
            sp_registers[MEM][COND]       = regRead(src1, src1_float_op) == 0;
            break;

         case BGTZ:
            sp_registers[MEM][ALU_OUTPUT] = alu(npc, instruction.imm, false, false, instruction.opcode);
            sp_registers[MEM][COND]       = regRead(src1, src1_float_op) > 0;
            break;

         case BGEZ:
            sp_registers[MEM][ALU_OUTPUT] = alu(npc, instruction.imm, false, false, instruction.opcode);
            sp_registers[MEM][COND]       = regRead(src1, src1_float_op) >= 0;
            break;

         case BLEZ:
            sp_registers[MEM][ALU_OUTPUT] = alu(npc, instruction.imm, false, false, instruction.opcode);
            sp_registers[MEM][COND]       = regRead(src1, src1_float_op) <= 0;
            break;

         case JUMP:
            sp_registers[MEM][ALU_OUTPUT] = alu(npc, instruction.imm, false, false, instruction.opcode);
            sp_registers[MEM][COND]       = 1;
            break;

         case NOP:
         case EOP:
            break;

         default:
            ASSERT(false, "Unknown operation encountered");
            break;
      }
   }
   instruction_register[MEM]  = instruction;
   data_memory_latency_count                    = memLatency;
}

bool sim_pipe_fp::MIPS_MEM() {

   instruction_t instruction                      = instruction_register[MEM]; 
   sp_registers[WB][LMD]                          = UNDEFINED;

   switch(instruction.opcode) {
      case LW:
      case LWS:
         while(data_memory_latency_count--){
            stall_count++;
            instruction_register[WB].set_stall();
            clear_sp_register(WB);
            return true;
         }
         sp_registers[WB][LMD]                 = read_memory( sp_registers[MEM][ALU_OUTPUT] );
         break;

      case SW:
      case SWS:
         while(data_memory_latency_count--){
            stall_count++;
            instruction_register[WB].set_stall();
            clear_sp_register(WB);
            return true;
         }
         write_memory(sp_registers[MEM][ALU_OUTPUT], regRead(instruction.src2, instruction.src2_float_op));
         break;

      default: break;
   }
   instruction_register[WB]                    = instruction;
   sp_registers[WB][ALU_OUTPUT]                 = sp_registers[MEM][ALU_OUTPUT];
   return false;
}

bool sim_pipe_fp::MIPS_WB() {
   instruction_t instruction                = instruction_register[WB]; 
   if (instruction.opcode == EOP){
      return true;
   }
   if(instruction.dest_op) {
      unsigned result = (instruction.opcode == LW || instruction.opcode == LWS) ? sp_registers[WB][LMD] : sp_registers[WB][ALU_OUTPUT];
      if(instruction.dest_float_op) {
         set_fp_register(instruction.dest, unsigned2float(result));
      }
      else
         set_int_register(instruction.dest, result);
   }
   return false;
}

void sim_pipe_fp::run(unsigned cycles){
   bool run_to_completion = (cycles == 0);
   while(cycles-- || run_to_completion) {
      if(MIPS_WB()) return;
      if( !MIPS_MEM() ) {
         MIPS_EXE();
         bool stall          = MIPS_ID();
         MIPS_IF(stall);
      }
      cycleCount++;
   }
}
void sim_pipe_fp::reset(){
   data_memory       = new unsigned char[dataMemSize];
   for(unsigned i = 0; i < dataMemSize; i++) {
      data_memory[i] = UNDEFINED; 
   }

   for(int i = 0; i < NUM_STAGES; i++) {
      instruction_register[i].set_stall();
   }

   for(int i = 0; i < NUM_GP_REGISTERS; i++) {
      int_file[i].value = UNDEFINED;
   }

   for(int i = 0; i < NUM_GP_REGISTERS; i++) {
      fp_file[i].value = UNDEFINED;
   }

   for(int i = 0; i < NUM_STAGES; i++) {
      for(int j = 0; j < NUM_SP_REGISTERS; j++) {
         sp_registers[i][j]  = UNDEFINED;
      }
      sp_registers[i][COND]  = 0;
   }
}


void sim_pipe_fp::print_memory(unsigned start_address, unsigned end_address){
	cout << "data_memory[0x" << hex << setw(8) << setfill('0') << start_address << ":0x" << hex << setw(8) << setfill('0') <<  end_address << "]" << endl;
	unsigned i;
	for (i=start_address; i<end_address; i++){
		if (i%4 == 0) cout << "0x" << hex << setw(8) << setfill('0') << i << ": "; 
		cout << hex << setw(2) << setfill('0') << int(data_memory[i]) << " ";
		if (i%4 == 3) cout << endl;
	} 
}

void sim_pipe_fp::print_registers(){
	cout << "Special purpose registers:" << endl;
        unsigned i, s;
	for (s=0; s<NUM_STAGES; s++){
		cout << "Stage: " << stage_names[s] << endl;  
		for (i=0; i< NUM_SP_REGISTERS; i++)
			if ((sp_register_t)i != IR && (sp_register_t)i != COND && get_sp_register((sp_register_t)i, (stage_t)s)!=UNDEFINED) cout << reg_names[i] << " = " << dec <<  get_sp_register((sp_register_t)i, (stage_t)s) << hex << " / 0x" << get_sp_register((sp_register_t)i, (stage_t)s) << endl;
	}
	cout << "General purpose registers:" << endl;
	for (i=0; i< NUM_GP_REGISTERS; i++)
		if (get_int_register(i)!=UNDEFINED) cout << "R" << dec << i << " = " << get_int_register(i) << hex << " / 0x" << get_int_register(i) << endl;
	for (i=0; i< NUM_GP_REGISTERS; i++)
		if (get_fp_register(i)!=UNDEFINED) cout << "F" << dec << i << " = " << get_fp_register(i) << hex << " / 0x" << float2unsigned(get_fp_register(i)) << endl;
}

int sim_pipe_fp::labelToPC( const char* filename, const char* label, uint32_t pc_index ){
   FILE* temp  = fopen(filename, "r");
   int line    = 0;
   do{
      char str[25];
      fscanf(temp, "%s", str);
      if(str[strlen(str)-1] == ':'){
         str[strlen(str)-1] = '\0';
         if(!strcmp(str, label)){
            break;
         }
      }
      if( opcode_2str.count( string( str ) ) > 0 )
         line++;
   }while(!feof(temp));
   fclose(temp);
   return ((line - pc_index - 1) * 4);
}

int sim_pipe_fp::parse( const char* filename ){
   FILE* trace;
   char buff[1024], label[495];
   int a, b, c, lineNo = 0;
   char imm[32];

   trace  = fopen(filename, "r");
   ASSERT(trace, "Unable to open file %s", filename);

   do {
      instMemory               = (instructPT*) realloc(instMemory, (lineNo + 1)*sizeof(instructPT));
      instructPT instructP     = new instruction_t;

      instMemory[lineNo]       = instructP;
      fscanf(trace, "%s ", buff);

      if( opcode_2str.count( string(buff) ) <= 0 ){
         ASSERT( buff[strlen(buff)-1] == ':', "Unkown buff(%s) encountered", buff );
         fscanf(trace, "%s ", buff);
      }

      instructP->opcode        = opcode_2str[ string(buff) ];

      switch( instructP->opcode ){
         case ADD ... DIV:
         case ADDS ... DIVS:
            a                     = parseReg(trace, instructP->dest_float_op);
            b                     = parseReg(trace, instructP->src1_float_op);
            c                     = parseReg(trace, instructP->src2_float_op);
            instructP->dest        = a;
            instructP->src1       = b;
            instructP->src2       = c;
            instructP->dest_op   = true;
            instructP->src1_op  = true;
            instructP->src2_op  = true;
            break;

         case BEQZ ... BGEZ:
            a                     = parseReg(trace, instructP->src1_float_op);
            fscanf(trace, "%s", label);
            
            instructP->src1       = a;
            instructP->imm        = labelToPC( filename, label, lineNo );
            instructP->src1_op  = true;
            instructP->branch_op  = true;
            break;

         case ADDI ... ANDI:
            a                     = parseReg(trace, instructP->dest_float_op);
            b                     = parseReg(trace, instructP->src1_float_op);
            fscanf(trace, "%s", imm);
            if( imm[1] == 'x' || imm[1] == 'X' ){
               c                  = /*HEX*/     strtol( imm + 2, NULL, 16 );
            } else{
               c                  = /*DECIMAL*/ strtol( imm, NULL, 10 );
            }
            instructP->dest        = a;
            instructP->src1       = b;
            instructP->imm        = c;
            instructP->dest_op   = true;
            instructP->src1_op  = true;
            break;

         case JUMP:
            fscanf(trace, "%s", label);
            instructP->imm        = labelToPC( filename, label, lineNo );
            instructP->branch_op  = true;
            break;

         case LW:
         case LWS:
            a                     = parseReg(trace, instructP->dest_float_op);
            fscanf(trace, "%d(", &b);
            c                     = parseReg(trace, instructP->src1_float_op);
            instructP->dest        = a;
            instructP->imm        = b;
            instructP->src1       = c;
            instructP->dest_op   = true;
            instructP->src1_op  = true;
            break;

         case SW:
         case SWS:
            a                     = parseReg(trace, instructP->src2_float_op);
            fscanf(trace, "%d(", &b);
            c                     = parseReg(trace, instructP->src1_float_op);
            instructP->src2       = a;
            instructP->imm        = b;
            instructP->src1       = c;
            instructP->src2_op  = true;
            instructP->src1_op  = true;
            break;

         case EOP:
         case NOP:
            break;

         default:
            ASSERT(false, "Unknown operation encountered");
            break;
      }
      lineNo++;
   }while(!feof(trace));

   return lineNo;
}

uint32_t sim_pipe_fp::parseReg( FILE* trace, bool& is_float ){
   uint32_t reg;
   char     regIdentifier, dummy;
   fscanf(trace, "%c%d%c", &regIdentifier, &reg, &dummy);
   if( regIdentifier == 'R' || regIdentifier == 'r' ){
      is_float  = false;
   }
   else if( regIdentifier == 'F' || regIdentifier == 'f' ){
      is_float  = true;
   }
   else {
      ASSERT(false, "Unknown register identifier found (=%c)", regIdentifier);
   }
   return reg;
}

inline unsigned sim_pipe_fp::float2unsigned(float value){
        unsigned result;
        memcpy(&result, &value, sizeof value);
        return result;
}

inline float sim_pipe_fp::unsigned2float(unsigned value){
        float result;
        memcpy(&result, &value, sizeof value);
        return result;
}


unsigned sim_pipe_fp::regRead(unsigned reg, bool isF){
   return isF ? float2unsigned(fp_file[reg].value) : int_file[reg].value;
}

//----------------------------------------------------------------------
// Clear Special Purpose Registers for a Specific Stage
//----------------------------------------------------------------------
void sim_pipe_fp::clear_sp_register(stage_t s) {
  for(int i = 0; i < NUM_SP_REGISTERS; i++) {
    sp_registers[s][i] = UNDEFINED;
  }
}

//----------------------------------------------------------------------
// Sets value of Special-Purpose register
//----------------------------------------------------------------------
void sim_pipe_fp::set_sp_register(sp_register_t reg, stage_t s, uint32_t value){
   sp_registers[s][reg] = value; 
}

//----------------------------------------------------------------------
// Returns value of Special-Purpose register
//----------------------------------------------------------------------
unsigned sim_pipe_fp::get_sp_register(sp_register_t reg, stage_t s){
	return sp_registers[s][reg]; 
}

//----------------------------------------------------------------------
// Gets value of Integer General-Purpose register
//----------------------------------------------------------------------
int sim_pipe_fp::get_int_register(unsigned reg){
	return int_file[reg].value; 
}

//----------------------------------------------------------------------
// Sets value of Integer General-Purpose register
//----------------------------------------------------------------------
void sim_pipe_fp::set_int_register(unsigned reg, int value){
   int_file[reg].value = value;
   if(int_file[reg].busy != 0)
      int_file[reg].busy--;
}

//----------------------------------------------------------------------
// Returns value of Floating-Point General-Purpose register
//----------------------------------------------------------------------
float sim_pipe_fp::get_fp_register(unsigned reg){
	return fp_file[reg].value;
}

//----------------------------------------------------------------------
// Sets value of Floating-Point General-Purpose register
//----------------------------------------------------------------------
void sim_pipe_fp::set_fp_register(unsigned reg, float value){
   fp_file[reg].value = value;
   if(fp_file[reg].busy != 0)
      fp_file[reg].busy--;
}

//----------------------------------------------------------------------
// Calculates and returns Instruction per Clock Cycle
//----------------------------------------------------------------------
float sim_pipe_fp::get_IPC(){
   return (double) get_instructions_executed() / (double) cycleCount;
}

//----------------------------------------------------------------------
// Returns Total Instructions Executed
//----------------------------------------------------------------------
unsigned sim_pipe_fp::get_instructions_executed(){
	return instruction_count; 
}

//----------------------------------------------------------------------
// Returns Stalls
//----------------------------------------------------------------------
unsigned sim_pipe_fp::get_stalls(){
	return stall_count; 
}

//----------------------------------------------------------------------
// Returns Clock Cycles
//----------------------------------------------------------------------
unsigned sim_pipe_fp::get_clock_cycles(){
   return cycleCount; 
}

unsigned sim_pipe_fp::read_memory(unsigned address){
   unsigned value = 0;
   ASSERT( address % 4 == 0, "Unaligned memory access found at address %x", address ); 
   ASSERT ( (address >= 0) && (address < dataMemSize), "Out of bounds memory accessed: Seg Fault!!!!" );
   value |= data_memory[address + 0];
   value |= data_memory[address + 1] << 8;  
   value |= data_memory[address + 2] << 16;
   value |= data_memory[address + 3] << 24;
   return value;
}

void sim_pipe_fp::write_memory(unsigned address, unsigned value){
   ASSERT( address % 4 == 0, "Unaligned memory access found at address %x", address ); 
   ASSERT ( (address >= 0) && (address < dataMemSize), "Out of bounds memory accessed: Seg Fault!!!!" );
   data_memory[address + 0] = value;
   data_memory[address + 1] = value >> 8;
   data_memory[address + 2] = value >> 16;
   data_memory[address + 3] = value >> 24;
}
