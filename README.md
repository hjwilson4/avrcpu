# avrcpu
An implementation of the AVR CPU in VHDL with pipelining. 
Atmel AVR CPU with Pipelining Entity Declaration

  This is the top level entity declaration for the complete AVR CPU with Pipelining. The components making up the 
  complete design are the Arithmetic Logic Unit (ALU), Control Unit (CU), Data Memory Address Unit (DAU), Program 
  Memory Address Unit (PAU), Status Register (SREG), and the Register Array Unit. The CPU contains 32 general purpose 
  registers 0-31. Registers 26-27 make up the X-pointer. Registers 28-29 make up the Y-pointer. Registers 30-31 make 
  up the Z-pointer. The 8-bit status register contains the resulting flags from an ALU operation as well the interrupt 
  (I) and transfer (T) flags. Lastly, there are the Program Counter (PC) and Stack Pointer (SP) which are top level 
  registers; these are initialized to 0 on reset. The Instruction Register (IR) contains the current instruction
  being executed and is updated internally within the CU component. 

  ** 
     Compared with the previous iteration of this CPU, the now pipelined version is constructed such that 4 instructions
     are being executed at any given time within the CPU. A number of control hazards are present for e.g. conditional jumps 
     and ALU instructions. In order to deal with these control hazards, pipeline bubbles have been inserted for these 
     instructions which increases the delay before the output is executed. Additionally, various tweaks have been made 
     for certain instructions SREG/flag handling due to the pipelining nature of the CPU. Aside from these changes, 
     the pipelined CPU is the same as the unpipelined version.
  ** 

  The CU component handles all instruction decoding and outputs various control signals to the rest of the CPU. The 
  CU also contains the 16-bit instruction register (IR) which contains the current instruction being executed by the
  CPU. Additionally, the CU computes the Data Read and Write signals are additionally sent out for use in the top 
  level file for data addressing. 
 
  The ALU component performs all of the standard logic and arithmetic operations. These include Boolean operations, 
  shifts and rotates, bit functions, addition, subtraction, and comparison. The operands may be registers, or immediate 
  values (from the IR). The ALU also computes 6 status flags based on the result of the operation being executed. 
 
  The status register is an 8-bit register which holds the resulting status from an ALU operation. Not all status 
  bits are affected by every ALU operation. For details on which bits are affected by the ALU operations see the 
  instruction descriptions below. The status bits (from high bit to low bit) are: 
    | Bit Name | Bit Number |                                       Description
    |    I     |     7      |  Global Interrupt Enable/Disable                                                          |
    |    T     |     6      |  Transfer bit used by BLD and BST instructions                                            |
    |    H     |     5      |  Half Carry (carry out of bit 3, into bit 4)                                              |
    |    S     |     4      |  Signed bit (N xor V)                                                                     |
    |    V     |     3      |  Signed overflow (carry 6 doesn't match carry 7), 0 for logic ops, N xor C for shift ops  |
    |    N     |     2      |  Negative (sign-bit, high of result)                                                      |
    |    Z     |     1      |  Zero (result is zero)                                                                    |
    |    C     |     0      |  Carry (carry out of bit 7), set for COM instruction                                      |
    
  The DAU component generates the address and reads or writes the data for the data memory. The data memory is 
  addressed as bytes with 16-bits of address (64 Kbytes total data memory space). The address bus is called DataAB 
  and the data bus is called DataDB. Both are std_logic_vectors of the appropriate size. There are also two active-low 
  control lines for accessing the memory: DataRd indicates the data memory is being read and DataWr indicates it is being 
  written. These signals are only active during the second half of the clock (while clock is low) during the actual read 
  or write cycle (to give the address time to stabilize). The addressing modes are described in the instruction description. 
  In brief, an address may come from the second word of the instruction, the X, Y, Z, or SP registers (unchanged or with 
  pre/post-increment/decrement), or the Y or Z registers with a 6-bit unsigned offset (range 0 to 63).

  The PAU component generates the addresses for reads of the program memory data. The program memory is addressed as 
  16-bit words with 16-bits of address (64 Kwords total program memory space). Note that the AVR architecture actually 
  supports up to 4 Mwords of program memory space, but this implementation will only use 64 Kwords. The address bus is 
  called ProgAB and the data bus is called ProgDB. Both are declared as std_logic_vector (15 downto 0). Note that this 
  interface is read only.


  The complete list of instructions included in the CPU are listed below: 
      AVR ALU Instructions
     | Opcode | Op1 | Op2 | Result | Flags Affected |       Description         |
     |  ADC   | Rd  | Rr  |   Rd   |  C Z N V S H   | Rd = Rd + Rr + C          |
     |  ADD   | Rd  | Rr  |   Rd   |  C Z N V S H   | Rd = Rd + Rr              |
     |  ADIW  | Rd  | K   |   Rd   |  C Z N V S     | Rd = Rd + K               |
     |  AND   | Rd  | Rr  |   Rd   |    Z N V S     | Rd = Rd and Rr            |
     |  ANDI  | Rd  | K   |   Rd   |    Z N V S     | Rd = Rd and K             |
     |  ASR   | Rd  |     |   Rd   |  C Z N V S     | Arithmetic shift right Rd |
     |  BCLR  | s   |     | SREG(s)|     SREG(s)    | SREG(s) = 0               |
     |  BLD   | Rd  | b   |   Rd   |                | Rd(b) = T                 |
     |  BSET  | s   |     | SREG(s)|     SREG(s)    | SREG(s) = 1               |
     |  BST   | Rd  | b   |   T    |       T        | T = Rr(b)                 |
     |  COM   | Rd  |     |   Rd   |  C Z N V S     | Rd = not Rd               |
     |  CP    | Rd  | Rr  |        |  C Z N V S H   | Rd - Rr                   |
     |  CPC   | Rd  | Rr  |        |  C Z N V S H   | Rd - Rr - C               |
     |  CPI   | Rd  | K   |        |  C Z N V S H   | Rd - K                    |
     |  DEC   | Rd  |     |   Rd   |    Z N V S     | Rd = Rd - 1               |
     |  EOR   | Rd  | Rr  |   Rd   |    Z N V S     | Rd = Rd xor Rr            |
     |  INC   | Rd  |     |   Rd   |    Z N V S     | Rd = Rd + 1               |
     |  LSR   | Rd  |     |   Rd   |  C Z N V S     | Logical shift right Rd    |
     |  NEG   | Rd  |     |   Rd   |  C Z N V S H   | Rd = -Rd                  |
     |  OR    | Rd  | Rr  |   Rd   |    Z N V S     | Rd = Rd or Rr             |
     |  ORI   | Rd  | K   |   Rd   |    Z N V S     | Rd = Rd or K              |
     |  ROR   | Rd  |     |   Rd   |  C Z N V S     | Rotate right Rd           |
     |  SBC   | Rd  | Rr  |   Rd   |  C Z N V S H   | Rd = Rd - Rr - C          |
     |  SBCI  | Rd  | K   |   Rd   |  C Z N V S H   | Rd = Rd - K - C           |
     |  SBIW  | Rd  | K   |   Rd   |  C Z N V S     | Rd = Rd - K               |
     |  SUB   | Rd  | Rr  |   Rd   |  C Z N V S H   | Rd = Rd - Rr              |
     |  SUBI  | Rd  | K   |   Rd   |  C Z N V S H   | Rd = Rd - K               |
     |  SWAP  | Rd  |     |   Rd   |                | Swap nibbles of Rd        |
     *ADIW and SBIW operate on pairs of registers 24|25, 26|27, 28|29, 30|31. The immediate instructions (ANDI, CPI, ORI, 
      SBCI, and SUBI) only operate on the second half of the register set (registers 16 to 31). All instructions execute 
      in one (1) clock cycle except the ADIW, MUL, and SBIW instructions which execute in two (2) clock cycles.

      AVR Load/Store Instructions
     | Opcode | Op1 | Op2 | Cycles |  Result   |                 Description                    |
     |   LD   | Rd  | X   |    2   | Rd = (X)  | Load indirect with X                           |
     |   LD   | Rd  | X+  |    2   | Rd = (X)  | Load indirect with X and post-inc              |
     |        |     |     |        |  X = X+1  |                                                |
     |   LD   | Rd  | -X  |    2   |  X = X-1  | Load indirect with X and pre-dec               |
     |        |     |     |        | Rd = (X)  |                                                |
     |   LD   | Rd  | Y+  |    2   | Rd = (Y)  | Load indirect with Y and post-inc              |
     |        |     |     |        |  Y = Y+1  |                                                |
     |   LD   | Rd  | -Y  |    2   |  Y = Y-1  | Load indirect with Y and pre-dec               |
     |        |     |     |        | Rd = (Y)  |                                                |
     |   LD   | Rd  | Z+  |    2   | Rd = (Z)  | Load indirect with Z and post-inc              |
     |        |     |     |        |  Z = Z+1  |                                                |
     |   LD   | Rd  | -Z  |    2   |  Z = Z-1  | Load indirect with Z and pre-dec               |
     |        |     |     |        | Rd = (Z)  |                                                |
     |   LDD  | Rd  | Y+g |    2   | Rd = (Y+g)| Load indirect with Y and unsigned displacement |
     |   LDD  | Rd  | Z+g |    2   | Rd = (Z+g)| Load indirect with Z and unsigned displacement |
     |   LDI  | Rd  | k   |    1   | Rd = k    | Load immediate value                           |
     |   LDS  | Rd  | m   |    3   | Rd = (m)  | Load from memory                               | 
     |   MOV  | Rd  | Rr  |    1   | Rd = Rr   | Move register to register                      | 
     |   ST   | X   | Rr  |    2   | (X) = Rr  | Store indirect with X                          |
     |   ST   | X+  | Rr  |    2   | (X) = Rr  | Store indirect with X and post-inc             |
     |        |     |     |        |  X = X+1  |                                                |
     |   ST   | -X  | Rr  |    2   |  X = X-1  | Store indirect with X and pre-dec              |
     |        |     |     |        | (X) = Rr  |                                                |
     |   ST   | Y+  | Rr  |    2   | (Y) = Rr  | Store indirect with Y and post-inc             |
     |        |     |     |        |  Y = Y+1  |                                                |
     |   ST   | -Y  | Rr  |    2   |  Y = Y-1  | Store indirect with Y and pre-dec              |
     |        |     |     |        | (Y) = Rr  |                                                |
     |   ST   | Z+  | Rr  |    2   | (Z) = Rr  | Store indirect with Z and post-inc             | 
     |        |     |     |        |  Z = Z+1  |                                                |
     |   ST   | -Z  | Rr  |    2   |  Z = Z-1  | Store indirect with Z and pre-dec              |
     |        |     |     |        | (Z) = Rr  |                                                |
     |   STD  | Y+g | Rr  |    2   | (Y+g) = Rr| Store indirect with Y and unsigned displacement|
     |   STD  | Z+g | Rr  |    2   | (Z+g) = Rr| Store indirect with Z and unsigned displacement|
     |   STS  | m   | Rr  |    3   | (m) = Rr  | Store to memory                                |
     |   POP  | Rd  |     |    2   | SP = SP+1 | Pop register off stack                         |
     |        |     |     |        | Rd = (SP) |                                                |
     |   PUSH | Rr  |     |    2   | (SP) = Rr | Push register onto stack                       | 
     |        |     |     |        | SP = SP-1 |                                                |
     k = 8-bit immediate value
     q = 6-bit unsigned value 
     m = 16-bit address

     Unconditional Branch Instructions
     | Opcode | Op | Cycles |               Action             |
     |  JMP   | a  |   3    | PC = a                           |
     |  RJMP  | j  |   2    | PC = PC + 1 + j                  | 
     |  IJMP  |    |   2    | PC = Z                           | 
     |  CALL  | a  |   4    | Direct Subroutine Call           |
     |        |    |        | (SP) = (PC+2)[15:8]; SP = SP - 1 |  
     |        |    |        | (SP) = (PC+2)[7:0]; SP = SP - 1  |
     |        |    |        | PC = a                           |
     |  RCALL | j  |   3    | Relative Subroutine Call         |
     |        |    |        | (SP) = (PC+1)[15:8]; SP = SP - 1 |  
     |        |    |        | (SP) = (PC+1)[7:0]; SP = SP - 1  |
     |        |    |        | PC = PC = PC + 1 + j             | 
     |  ICALL |    |   3    | Indirect Subroutine Call         |
     |        |    |        | (SP) = (PC+1)[15:8]; SP = SP - 1 |  
     |        |    |        | (SP) = (PC+1)[7:0]; SP = SP - 1  |
     |        |    |        | PC = Z                           | 
     |  RET   |    |   4    | Subroutine Return (PC popped)    | 
     |        |    |        | SP = SP + 1; PC[7:0] = (SP)      |
     |        |    |        | SP = SP + 1; PC[15:8] = (SP)     |
     |  RETI  |    |   4    | Interrupt Return (PC popped)     | 
     |        |    |        | I = 1                            |
     |        |    |        | SP = SP + 1; PC[7:0] = (SP)      |
     |        |    |        | SP = SP + 1; PC[15:8] = (SP)     |
     a = 22-bit program address 
     j = 12-bit relative address
 
     Conditional Branch Instructions
     | Opcode | Op1 | Op2 | Cycles |              Action               | 
     |  BRBC  |  b  |  r  |  1/2   |  Branch if status bit clear       |   
     |        |     |     |        | if SREG(b)=0 then PC = PC + 1 + r |
     |  BRBS  |  b  |  r  |  1/2   |  Branch if status bit set         |
     |        |     |     |        | if SREG(b)=1 then PC = PC + 1 + r |
     b = 3-bit unsigned value
     r = 7-bit signed value
 
     Skip Instructions
     | Opcode | Op1 | Op2 | Cycles |              Action               |
     |  CPSE  | Rd  | Rr  | 1/2/3  |  Compare and skip if equal        | 
     |        |     |     |        | if Rd=Rr then PC = PC+2 or 3      |
     |  SBRC  | Rr  | b   | 1/2/3  |  Skip if bit clear                | 
     |        |     |     |        | if Rr(b)=0 then PC = PC+2 or 3    |
     |  SBRS  | Rr  | b   | 1/2/3  |  Skip if bit set                  | 
     |        |     |     |        | if Rr(b)=1 then PC = PC+2 or 3    |
     b = 3-bit unsigned value
