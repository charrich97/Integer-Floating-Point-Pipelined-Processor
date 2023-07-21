#include "sim_pipe.h"

//used for debugging purposes
static const char *reg_names[NUM_SP_REGISTERS] = {"PC", "NPC", "IR", "A", "B", "IMM", "COND", "ALU_OUTPUT", "LMD"};
static const char *stage_names[NUM_STAGES] = {"IF", "ID", "EX", "MEM", "WB"};

//Mapping strings into its opcode
map <string, opcode_t> opcode_2str = { {"LW", LW}, {"SW", SW}, {"ADD", ADD}, {"SUB", SUB}, {"XOR", XOR}, {"OR", OR}, {"AND", AND}, {"MULT", MULT}, {"DIV", DIV}, {"ADDI", ADDI}, {"SUBI", SUBI}, {"XORI", XORI}, {"ORI", ORI}, {"ANDI", ANDI}, {"BEQZ", BEQZ}, {"BNEZ", BNEZ}, {"BLTZ", BLTZ}, {"BGTZ", BGTZ}, {"BLEZ", BLEZ}, {"BGEZ", BGEZ}, {"JUMP", JUMP}, {"EOP", EOP}, {"NOP", NOP} };

sim_pipe::sim_pipe(unsigned mem_size, unsigned mem_latency){
   this->dataMemSize            = mem_size;
   this->Data_Memory_Latency    = mem_latency;
   this->instruct_memory        = NULL;
   stall_count                  = 0;
   reset();
}

//----------------------------------------------------------------------
// Reset the Pipeline
//----------------------------------------------------------------------
void sim_pipe::reset(){
   data_memory       = new unsigned char[dataMemSize];
   // Reset Data Memory
   for(unsigned i = 0; i < dataMemSize; i++) {
      data_memory[i] = UNDEFINED; 
   }
   //Reset the Instruction Memory 
   for(int i = 0; i < NUM_STAGES; i++) {
      instruction_register[i].no_operation();
   }
   //Reset the General Purpose Registers 
   for(int i = 0; i < NUM_GP_REGISTERS; i++) {
      int_file[i].value = UNDEFINED;
   }
   //Reset the Special Purpose Registers
   for(int i = 0; i < NUM_STAGES; i++) {
      for(int j = 0; j < NUM_SP_REGISTERS; j++) {
         sp_registers[i][j]  = UNDEFINED;
      }
      sp_registers[i][COND]  = 0;
   }
}

//----------------------------------------------------------------------
// Run Function
//----------------------------------------------------------------------
void sim_pipe::run(unsigned cycles){
  bool run_2_completion = (cycles == 0);
  bool stall;  
   while(cycles-- || run_2_completion) {
      if(MIPS_WB()) return;
      if( !MIPS_MEM() ) {
         MIPS_EXE();
         stall = MIPS_ID();
         MIPS_IF(stall);
      }
      cc_count++;
   }
}

instruction_t sim_pipe::index_instruction ( unsigned pc ) {
   unsigned index = (pc - this->baseAddress)/4;
   instCount++;
   return *(instruct_memory[index]);
}

//----------------------------------------------------------------------
// MIPS Instruction Fetch
//----------------------------------------------------------------------
void sim_pipe::MIPS_IF(bool stall) {
   unsigned program_counter         ;
   instruction_t instruction        ; 

   if(sp_registers[MEM][COND] == 1) { 
    sp_registers[IF][PC]      = sp_registers[MEM][ALU_OUTPUT]; 
   }
   program_counter            = sp_registers[IF][PC];

   //Check if were not in stall
   if( !stall ){
    instruction      = index_instruction(program_counter);
      if(instruction.opcode != EOP )
          set_sp_register(PC, IF, program_counter + 4);
          sp_registers[ID][NPC]     = sp_registers[IF][PC];
          instruction_register[ID]  = instruction;
   }
}

//----------------------------------------------------------------------
// MIPS: Instruction Decode
//----------------------------------------------------------------------
bool sim_pipe::MIPS_ID() {
   instruction_t instruction;                   
   instruction = instruction_register[ID];
   sp_registers[EX][NPC]               = sp_registers[ID][NPC];
   
   // Handling of RAW Data Hazards
   if(( instruction.src1_op && int_file[instruction.src1].busy ) || (instruction.src2_op && int_file[instruction.src2].busy)) {
      stall_count++;
      instruction_register[EX].set_stall();
      clear_sp_register(EX);
      return true;
   }
    
   //Increment busy to say that the destination register is busy being written to 
   if(instruction.dest_op)
      int_file[instruction.dest].busy++;

    // Set Special Purpose Registers in the Execute Stage
    sp_registers[EX][A]                 = (instruction.src1_op) ? get_gp_register(instruction.src1) : UNDEFINED;
    sp_registers[EX][B]                 = (instruction.src2_op) ? get_gp_register(instruction.src2) : UNDEFINED;
    sp_registers[EX][IMM]               = instruction.immediate;

   //Checking if we have a Control Hazard
   if(instruction.branch_op) { 
      stall_count++;
      instruction_register[ID].set_stall();
      clear_sp_register(ID);
      instruction_register[EX]              = instruction;
      return true;
   }
   else if (instruction_register[EX].branch_op) {
      stall_count++;
      instruction_register[ID].set_stall();
      clear_sp_register(ID);
      instruction_register[EX]              = instruction;
      return true;
   }
   else {
      instruction_register[EX]              = instruction;
      return (instruction.opcode == EOP);
   }
}

// Function to Generate Address for LW/SW Instructions
unsigned sim_pipe::address_gen ( instruction_t instruct) {
   return (instruct.immediate + get_gp_register(instruct.src1));
}

// ALU Operator
unsigned sim_pipe::alu (unsigned value1, unsigned value2, opcode_t opcode){
   unsigned output;

   switch( opcode ){
      case LW ... ADD:
      case ADDI:
      case BEQZ ... NOP:
         output      = value1 + value2;
         break;

      case SUB:
      case SUBI:
         output      = value1 - value2;
         break;

      case XOR:
      case XORI:
         output      = value1 ^ value2;
         break;

      case AND:
      case ANDI:
         output      = value1 & value2;
         break;

      case OR:
      case ORI:
         output      = value1 | value2;
         break;

      case MULT:
         output      = value1 * value2;
         break;

      case DIV:
         output      = value1 / value2;
         break;

      default: 
         output      = UNDEFINED;
         break;
   }
   return output;
}

//----------------------------------------------------------------------
// MIPS Execute Stage
//----------------------------------------------------------------------
void sim_pipe::MIPS_EXE() {
   instruction_t instruction;

   instruction  = instruction_register[EX]; 

   for(int i = 0; i < NUM_SP_REGISTERS; i++) {
      sp_registers[MEM][i]    = UNDEFINED;
   }
   sp_registers[MEM][COND]    = 0;
   sp_registers[MEM][B]       = sp_registers[EX][B];
   switch(instruction.opcode) {
      case ADD ... DIV:
         sp_registers[MEM][ALU_OUTPUT] = alu(get_gp_register(instruction.src1), 
         get_gp_register(instruction.src2), instruction.opcode);
         break;
      case ADDI ... ANDI:
         sp_registers[MEM][ALU_OUTPUT] = alu(get_gp_register(instruction.src1),
         sp_registers[EX][IMM], instruction.opcode);
         break;
      case LW:
         sp_registers[MEM][ALU_OUTPUT] = address_gen (instruction);
         break;
      case SW:
         sp_registers[MEM][ALU_OUTPUT] = address_gen (instruction);
         break;
      case BEQZ:
         sp_registers[MEM][ALU_OUTPUT] = alu(sp_registers[EX][NPC], sp_registers[EX][IMM], instruction.opcode);
         sp_registers[MEM][COND]       = get_gp_register(instruction.src1) == 0;
         break;
      case BLEZ:
         sp_registers[MEM][ALU_OUTPUT] = alu(sp_registers[EX][NPC], sp_registers[EX][IMM], instruction.opcode);
         sp_registers[MEM][COND]       = get_gp_register(instruction.src1) <= 0;
         break;
      case BGEZ:
         sp_registers[MEM][ALU_OUTPUT] = alu(sp_registers[EX][NPC], sp_registers[EX][IMM], instruction.opcode);
         sp_registers[MEM][COND]       = get_gp_register(instruction.src1) >= 0;
         break;
      case BNEZ:
         sp_registers[MEM][ALU_OUTPUT] = alu(sp_registers[EX][NPC], sp_registers[EX][IMM], instruction.opcode);
         sp_registers[MEM][COND]       = (get_gp_register(instruction.src1) != 0);
         break;
      case BLTZ:
         sp_registers[MEM][ALU_OUTPUT] = alu(sp_registers[EX][NPC], sp_registers[EX][IMM], instruction.opcode);
         sp_registers[MEM][COND]       = get_gp_register(instruction.src1) < 0;
         break;
      case BGTZ:
         sp_registers[MEM][ALU_OUTPUT] = alu(sp_registers[EX][NPC], sp_registers[EX][IMM], instruction.opcode);
         sp_registers[MEM][COND]       = get_gp_register(instruction.src1) > 0;
         break;
      case JUMP:
         sp_registers[MEM][ALU_OUTPUT] = alu(sp_registers[EX][NPC], sp_registers[EX][IMM], instruction.opcode);
         sp_registers[MEM][COND]        = 1;
         break;
      case NOP:
      case EOP:
         break;
   }
   instruction_register[MEM]  = instruction;
   data_memory_latency_count  = Data_Memory_Latency;
}

//----------------------------------------------------------------------
// MIPS Memory Stage
//----------------------------------------------------------------------
bool sim_pipe::MIPS_MEM() {
   instruction_t instruction                   = instruction_register[MEM]; 
   sp_registers[WB][LMD]                       = UNDEFINED;

  //--------------------------------------------------------------------
  // Case of Load or Store
  //--------------------------------------------------------------------
   switch(instruction.opcode) {
      case LW:
         while(data_memory_latency_count--){ // Introduce Data Memory Latency
            stall_count++;
            instruction_register[WB].set_stall();
            clear_sp_register(WB);
            return true;
         }
         sp_registers[WB][LMD]               = read_memory( sp_registers[MEM][ALU_OUTPUT] );
         break;

      case SW:
         while(data_memory_latency_count--){ // Introduce Data Memory Latency
            stall_count++;
            instruction_register[WB].set_stall();
            clear_sp_register(WB);
            return true;
         }
         write_memory(sp_registers[MEM][ALU_OUTPUT], get_gp_register(instruction.src2));
         break;

      default: break;
   }
   instruction_register[WB]                  = instruction;
   sp_registers[WB][ALU_OUTPUT]              = sp_registers[MEM][ALU_OUTPUT];
   return false;
}

//----------------------------------------------------------------------
// MIPS Writeback Stage
//----------------------------------------------------------------------
bool sim_pipe::MIPS_WB() {
  instruction_t instruction; 
  instruction = instruction_register[WB];
   
  //If End of Operation Return
  if (instruction.opcode == EOP){
    return true;
  }
  //else Writeback to the General Purpose Register
  if(instruction.dest_op) {
     set_gp_register(instruction.dest, (instruction.opcode == LW) ? sp_registers[WB][LMD] : sp_registers[WB][ALU_OUTPUT]);
  }
  return false;
}

// de-allocate the simulator
sim_pipe::~sim_pipe(){
}

void sim_pipe::load_program(const char *filename, unsigned base_address){
   parse(filename);
   sp_registers[IF][PC]   = base_address;
   baseAddress            = base_address;
}

//----------------------------------------------------------------------
// Clear Special Purpose Registers for a Specific Stage
//----------------------------------------------------------------------
void sim_pipe::clear_sp_register(stage_t s) {
  for(int i = 0; i < NUM_SP_REGISTERS; i++) {
    sp_registers[s][i] = UNDEFINED;
  }
}

//----------------------------------------------------------------------
// Sets value of special purpose register
//----------------------------------------------------------------------
void sim_pipe::set_sp_register(sp_register_t reg, stage_t s, unsigned value){
   sp_registers[s][reg] = value; 
}

//----------------------------------------------------------------------
// Returns value of special purpose register
//----------------------------------------------------------------------
unsigned sim_pipe::get_sp_register(sp_register_t reg, stage_t s){
   return sp_registers[s][reg];
}

//----------------------------------------------------------------------
// Returns value of general purpose register
//----------------------------------------------------------------------
int sim_pipe::get_gp_register(unsigned reg){
   return this->int_file[reg].value;
}

//----------------------------------------------------------------------
// Sets value of general purpose register
//----------------------------------------------------------------------
void sim_pipe::set_gp_register(unsigned reg, int value){
   this->int_file[reg].value         = value;
   if( this->int_file[reg].busy != 0 )
      this->int_file[reg].busy--;
}

//----------------------------------------------------------------------
// Calculates and returns Instruction per Clock Cycle
//----------------------------------------------------------------------
float sim_pipe::get_IPC(){
   return (float) get_instructions_executed() / (float) cc_count;
}

//----------------------------------------------------------------------
// Returns Total Instructions Executed
//----------------------------------------------------------------------
unsigned sim_pipe::get_instructions_executed(){
   return instCount - 1; 
}

//----------------------------------------------------------------------
// Returns Stalls
//----------------------------------------------------------------------
unsigned sim_pipe::get_stalls(){
   return stall_count; 
}

//----------------------------------------------------------------------
// Returns Clock Cycles
//----------------------------------------------------------------------
unsigned sim_pipe::get_clock_cycles(){
   return cc_count; 
}

int sim_pipe::labelToPC( const char* filename, const char* label, unsigned pc_index ){
   FILE* temp  = fopen(filename, "r");
   int line    = 0;
   do{
      char str[4096];
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

int sim_pipe::parse( const char* filename ){
   FILE* trace;
   char buff[4096], label[1024];
   int a, b, c, lineNo = 0;
   char imm[100];

   trace  = fopen(filename, "r");

   do {
      instruct_memory               = (instruction_pointer*) realloc(instruct_memory, (lineNo + 1)*sizeof(instruction_pointer));
      instruction_pointer instructP     = new instruction_t;

      instruct_memory[lineNo]       = instructP;
      fscanf(trace, "%s ", buff);

      if( opcode_2str.count( string(buff) ) <= 0 ){
         ASSERT( buff[strlen(buff)-1] == ':', "Unknown buff(%s) encountered", buff );
         fscanf(trace, "%s ", buff);
      }

      instructP->opcode        = opcode_2str[ string(buff) ];

      switch( instructP->opcode ){
         case ADD ... DIV:
            fscanf(trace, "R%d R%d R%d", &a, &b, &c);
            instructP->dest        = a;
            instructP->src1       = b;
            instructP->src2       = c;
            instructP->dest_op  = true;
            instructP->src1_op  = true;
            instructP->src2_op  = true;
            break;

         case BEQZ ... BGEZ:
            fscanf(trace, "R%d %s", &a, label);
            
            instructP->src1       = a;
            instructP->immediate  = labelToPC( filename, label, lineNo );
            instructP->src1_op  = true;
            instructP->branch_op  = true;
            break;

         case ADDI ... ANDI:
            fscanf(trace, "R%d R%d ", &a, &b);
            fscanf(trace, "%s", imm);
            if( imm[1] == 'x' || imm[1] == 'X' ){
               c                  = /*HEX*/     strtol( imm + 2, NULL, 16 );
            } else{
               c                  = /*DECIMAL*/ strtol( imm, NULL, 10 );
            }
            instructP->dest        = a;
            instructP->src1       = b;
            instructP->immediate  = c;
            instructP->dest_op  = true;
            instructP->src1_op  = true;
            break;

         case JUMP:
            fscanf(trace, "%s", label);
            instructP->immediate  = labelToPC( filename, label, lineNo );
            instructP->branch_op  = true;
            break;

         case LW:
            fscanf(trace, "R%d %d(R%d)", &a, &b, &c);
            instructP->dest        = a;
            instructP->immediate  = b;
            instructP->src1       = c;
            instructP->dest_op  = true;
            instructP->src1_op  = true;
            break;

         case SW:
            fscanf(trace, "R%d %d(R%d)", &a, &b, &c);
            instructP->src2       = a;
            instructP->immediate  = b;
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

//----------------------------------------------------------------------
// Read Memory
//----------------------------------------------------------------------
unsigned sim_pipe::read_memory(unsigned address){
   unsigned value = 0;
   value |= data_memory[address + 0];
   value |= data_memory[address + 1] << 8;  
   value |= data_memory[address + 2] << 16;
   value |= data_memory[address + 3] << 24;
   return value;
}

//----------------------------------------------------------------------
// Write Memory
//----------------------------------------------------------------------
void sim_pipe::write_memory(unsigned address, unsigned value){
   ASSERT( address % 4 == 0, "Unaligned memory access found at address %x", address ); 
   data_memory[address + 0] = value;
   data_memory[address + 1] = value >> 8;
   data_memory[address + 2] = value >> 16;
   data_memory[address + 3] = value >> 24;
}

//----------------------------------------------------------------------
// Print Memory
//----------------------------------------------------------------------
void sim_pipe::print_memory(unsigned start_address, unsigned end_address){
   cout << "data_memory[0x" << hex << setw(8) << setfill('0') << start_address << ":0x" << hex << setw(8) << setfill('0') <<  end_address << "]" << endl;
   unsigned i;
   for (i=start_address; i<end_address; i++){
      if (i%4 == 0) cout << "0x" << hex << setw(8) << setfill('0') << i << ": "; 
      cout << hex << setw(2) << setfill('0') << int(data_memory[i]) << " ";
      if (i%4 == 3) cout << endl;
   } 
}

//----------------------------------------------------------------------
// Print Registers
//----------------------------------------------------------------------
void sim_pipe::print_registers(){
   cout << "Special purpose registers:" << endl;
   unsigned i, s;
   for (s=0; s<NUM_STAGES; s++){
      cout << "Stage: " << stage_names[s] << endl;  
      for (i=0; i< NUM_SP_REGISTERS; i++)
         if ((sp_register_t)i != IR && (sp_register_t)i != COND && get_sp_register((sp_register_t)i, (stage_t)s)!=UNDEFINED) cout << reg_names[i] << " = " << dec <<  get_sp_register((sp_register_t)i, (stage_t)s) << hex << " / 0x" << get_sp_register((sp_register_t)i, (stage_t)s) << endl;
   }
   cout << "General purpose registers:" << endl;
   for (i=0; i< NUM_GP_REGISTERS; i++)
      if (get_gp_register(i)!=UNDEFINED) cout << "R" << dec << i << " = " << get_gp_register(i) << hex << " / 0x" << get_gp_register(i) << endl;
}
