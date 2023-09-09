# Pipelined MIPS Architecture (Phase 3)

## Description

Phase 3 of this project further extends the pipelined MIPS architecture, building upon the enhancements made in Phase 2. In this phase, we introduce a 16-word direct-mapped data cache to improve memory access performance. This cache design allows for faster data retrieval and reduces the memory access delay.

### Implemented Instructions

Phase 3 of the project continues to support the same MIPS instructions implemented in previous phases:

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

The pipeline architecture in Phase 3 remains consistent with the previous phases, featuring the following stages:

1. **Instruction Fetch (IF)**: Fetches instructions from memory.
2. **Instruction Decode (ID)**: Decodes instructions and extracts relevant operands.
3. **Execute (EX)**: Executes ALU operations.
4. **Memory Access (MEM)**: Accesses data memory for load and store instructions. In this phase, memory access involves the 16-word direct-mapped data cache.
5. **Write-Back (WB)**: Writes data back to registers.

### Data Cache

In Phase 3, we introduce a 16-word direct-mapped data cache to improve memory access speed. The cache design reduces memory access latency by storing frequently accessed data in a smaller, faster memory unit. This enhancement enhances overall system performance.

### Hazard Unit

The project continues to utilize the hazard unit introduced in previous phases, which handles data hazards and pipeline stalls as needed to ensure correct execution. The hazard unit accounts for the memory access delay and cache behavior introduced in Phase 3.

### Test Benches

To validate the functionality and correctness of this Phase 3 pipelined architecture, you can use the existing test benches provided in your project. These test benches are designed to evaluate both basic functionality and more complex scenarios, considering the cache and memory access delay.

### Simulation

Simulate and evaluate the Phase 3 pipelined MIPS architecture using ModelSim or your preferred Verilog simulator. Ensure that your simulations reflect the improved memory access performance achieved through the data cache.

## Usage

To use this enhanced Phase 3 architecture, instantiate the `pipelined_mips` module in your higher-level test bench. Provide the appropriate clock (`clk`) and reset (`reset`) signals to drive the simulation, taking into account the cache behavior and memory access delay.

Feel free to reach out if you have any questions or need further assistance with testing or using this Phase 3 pipelined architecture.

---
