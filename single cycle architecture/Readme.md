### Description
A single-cycle architecture that can execute MIPS commands.
List of supported commands:
R format: add, sub, addu, subu, and, or, xor, nor, slt, sltu
I format: addi, addiu, slti, sltiu, andi, ori, xori, lui, beq, bne, lw, sw

### Test
For testing this architecture 2 test benches are provided:
* single_cycle_mips__tb_basic.v
* single_cycle_mips__tb_isort.v
In order to use these test benches, files in the test files folder should be considered.
