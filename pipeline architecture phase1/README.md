# Pipelined MIPS Architecture (Phase 1)

## Description

This project represents Phase 1 of a pipelined MIPS architecture implementation. The pipeline design is aimed at executing a subset of MIPS commands efficiently. In this phase, none of the blocks in the pipeline introduce any latency, making it a straightforward pipeline setup.

### Implemented Instructions

This phase of the project implements the following MIPS instructions:

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

In this phase, the pipeline architecture consists of the following stages, each processing instructions without introducing any latency:

1. **Instruction Fetch (IF)**: Fetches instructions from memory.
2. **Instruction Decode (ID)**: Decodes instructions and extracts relevant operands.
3. **Execute (EX)**: Executes ALU operations.
4. **Memory Access (MEM)**: Accesses data memory for load and store instructions.
5. **Write-Back (WB)**: Writes data back to registers.

### Hazard Unit

The project includes a hazard unit that handles data hazards and stalls the pipeline when necessary to ensure correct execution.

### Test Benches

To validate the functionality and correctness of this Phase 1 pipeline architecture, you can use test benches provided in your project. These test benches evaluate basic functionality as well as more complex scenarios.

### Simulation

You can simulate and evaluate the Phase 1 pipelined MIPS architecture using ModelSim or any other Verilog simulator of your choice. Refer to your simulator's documentation for guidance on setting up and running simulations effectively.

## Usage

To use this project, you can instantiate the `pipelined_mips` module in your higher-level test bench. Be sure to provide the appropriate clock (`clk`) and reset (`reset`) signals to drive the simulation.

Feel free to reach out if you have any questions or need further assistance with testing or using this pipeline architecture.

---
