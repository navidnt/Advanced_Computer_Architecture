# Single-Cycle MIPS Architecture

## Description

This project implements a single-cycle architecture capable of executing MIPS commands. It supports both R-format and I-format instructions, making it versatile for various MIPS assembly programs. The following instructions are supported:

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

## Testing

To ensure the correctness and functionality of this architecture, we provide two test benches:

1. `single_cycle_mips__tb_basic.v`: This test bench is designed for basic functionality testing.
2. `single_cycle_mips__tb_isort.v`: Use this test bench to evaluate the architecture's performance with more complex programs, like sorting algorithms.

### Test Files

To run the provided test benches, you'll need to consider the files located in the "test files" folder. These files contain sample MIPS assembly programs that can be used for testing and validation.

### Simulation

You can simulate and test the single-cycle MIPS architecture using ModelSim or any other Verilog simulator of your choice. Follow your simulator's documentation to set up and run the simulations effectively.

Feel free to reach out if you have any questions or need further assistance with testing or using this architecture.

---
