# Pipelined MIPS Architecture (Phase 2)

## Description

Phase 2 of this project extends the pipelined MIPS architecture, building upon the Phase 1 design. In this phase, we introduce a delay of 3 clock cycles in the reading path from memory, which is an important enhancement for handling memory-related instructions efficiently.

### Implemented Instructions

Phase 2 of the project retains the support for the same MIPS instructions implemented in Phase 1:

**R Format Instructions:**
- `add`
- `addu`
- `sub`
- `subu`
- `and`
- `or`
- `xor`
- `nor`
- `slt`
- `sltu`

**I Format Instructions:**
- `addi`
- `addiu`
- `slti`
- `sltiu`
- `andi`
- `ori`
- `xori`
- `lui`
- `beq`
- `bne`
- `lw`
- `sw`

## Pipeline Stages

In Phase 2, the pipeline architecture remains the same, consisting of the following stages:

1. **Instruction Fetch (IF)**: Fetches instructions from memory.
2. **Instruction Decode (ID)**: Decodes instructions and extracts relevant operands.
3. **Execute (EX)**: Executes ALU operations.
4. **Memory Access (MEM)**: Accesses data memory for load and store instructions. In this phase, reading from memory introduces a 3-clock-cycle delay.
5. **Write-Back (WB)**: Writes data back to registers.

### Hazard Unit

The project includes a hazard unit that handles data hazards and stalls the pipeline when necessary to ensure correct execution. The hazard unit accounts for the memory read delay introduced in Phase 2.

### Test Benches

To validate the functionality and correctness of this Phase 2 pipelined architecture, you can use the existing test benches provided in your project. These test benches are designed to evaluate both basic functionality and more complex scenarios, taking into account the memory read delay.

### Simulation

You can simulate and evaluate the Phase 2 pipelined MIPS architecture using ModelSim or any other Verilog simulator of your choice. Keep in mind that the memory read delay will affect the timing of memory-related instructions, and your simulations should reflect this behavior.

## Usage

To use this enhanced Phase 2 architecture, you can instantiate the `pipelined_mips` module in your higher-level test bench. Ensure that you provide the appropriate clock (`clk`) and reset (`reset`) signals to drive the simulation, taking into account the updated pipeline behavior with the memory read delay.

Feel free to reach out if you have any questions or need further assistance with testing or using this Phase 2 pipelined architecture.

---
