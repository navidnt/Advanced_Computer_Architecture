# MIPS Pipeline Architecture (Phase 1)

## Description

This project represents Phase 1 of a MIPS pipeline architecture implementation. The pipeline design is aimed at executing MIPS commands efficiently. In this phase, none of the blocks in the pipeline have any latency, making it a straightforward pipeline setup.

The project supports a range of MIPS instructions, including both R-format and I-format instructions. Below is a list of the supported instructions:

**R Format:**
- `add`
- `sub`
- `addu`
- `subu`
- `and`
- `or`
- `xor`
- `nor`
- `slt`
- `sltu`

**I Format:**
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

## Pipeline Design

In this phase, the pipeline architecture consists of stages for instruction fetch (IF), instruction decode (ID), execute (EX), memory access (MEM), and write-back (WB). Each stage processes instructions without introducing any latency.

### Test

To validate the functionality and correctness of the Phase 1 pipeline architecture, we provide two test benches:

1. `pipeline_mips__tb_basic.v`: Use this test bench for basic functionality testing.
2. `pipeline_mips__tb_complex.v`: Utilize this test bench to assess the architecture's performance with more complex programs.

### Test Files

To execute the provided test benches, make sure to consider the files located in the "test files" folder. These files contain sample MIPS assembly programs suitable for testing and validation.

### Simulation

You can simulate and evaluate the Phase 1 pipeline architecture using ModelSim or any other Verilog simulator of your choice. Refer to your simulator's documentation for guidance on setting up and running simulations effectively.

Feel free to reach out if you have any questions or require further assistance with testing or utilizing this pipeline architecture.

---
