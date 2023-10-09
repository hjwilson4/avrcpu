--------------------------------------------------------------------------------------------------------------------
--
--  Atmel AVR CPU with Pipelining Entity Declaration
--
--  This is the top level entity declaration for the complete AVR CPU with Pipelining. The components making up the 
--  complete design are the Arithmetic Logic Unit (ALU), Control Unit (CU), Data Memory Address Unit (DAU), Program 
--  Memory Address Unit (PAU), Status Register (SREG), and the Register Array Unit. The CPU contains 32 general purpose 
--  registers 0-31. Registers 26-27 make up the X-pointer. Registers 28-29 make up the Y-pointer. Registers 30-31 make 
--  up the Z-pointer. The 8-bit status register contains the resulting flags from an ALU operation as well the interrupt 
--  (I) and transfer (T) flags. Lastly, there are the Program Counter (PC) and Stack Pointer (SP) which are top level 
--  registers; these are initialized to 0 on reset. The Instruction Register (IR) contains the current instruction
--  being executed and is updated internally within the CU component. 
--
--  ** 
--     Compared with the previous iteration of this CPU, the now pipelined version is constructed such that 4 instructions
--     are being executed at any given time within the CPU. A number of control hazards are present for e.g. conditional jumps 
--     and ALU instructions. In order to deal with these control hazards, pipeline bubbles have been inserted for these 
--     instructions which increases the delay before the output is executed. Additionally, various tweaks have been made 
--     for certain instructions SREG/flag handling due to the pipelining nature of the CPU. Aside from these changes, 
--     the pipelined CPU is the same as the unpipelined version.
--  ** 
--
--  The CU component handles all instruction decoding and outputs various control signals to the rest of the CPU. The 
--  CU also contains the 16-bit instruction register (IR) which contains the current instruction being executed by the
--  CPU. Additionally, the CU computes the Data Read and Write signals are additionally sent out for use in the top 
--  level file for data addressing. 
-- 
--  The ALU component performs all of the standard logic and arithmetic operations. These include Boolean operations, 
--  shifts and rotates, bit functions, addition, subtraction, and comparison. The operands may be registers, or immediate 
--  values (from the IR). The ALU also computes 6 status flags based on the result of the operation being executed. 
-- 
--  The status register is an 8-bit register which holds the resulting status from an ALU operation. Not all status 
--  bits are affected by every ALU operation. For details on which bits are affected by the ALU operations see the 
--  instruction descriptions below. The status bits (from high bit to low bit) are: 
--    | Bit Name | Bit Number |                                       Description
--    |    I     |     7      |  Global Interrupt Enable/Disable                                                          |
--    |    T     |     6      |  Transfer bit used by BLD and BST instructions                                            |
--    |    H     |     5      |  Half Carry (carry out of bit 3, into bit 4)                                              |
--    |    S     |     4      |  Signed bit (N xor V)                                                                     |
--    |    V     |     3      |  Signed overflow (carry 6 doesn't match carry 7), 0 for logic ops, N xor C for shift ops  |
--    |    N     |     2      |  Negative (sign-bit, high of result)                                                      |
--    |    Z     |     1      |  Zero (result is zero)                                                                    |
--    |    C     |     0      |  Carry (carry out of bit 7), set for COM instruction                                      |
--    
--  The DAU component generates the address and reads or writes the data for the data memory. The data memory is 
--  addressed as bytes with 16-bits of address (64 Kbytes total data memory space). The address bus is called DataAB 
--  and the data bus is called DataDB. Both are std_logic_vectors of the appropriate size. There are also two active-low 
--  control lines for accessing the memory: DataRd indicates the data memory is being read and DataWr indicates it is being 
--  written. These signals are only active during the second half of the clock (while clock is low) during the actual read 
--  or write cycle (to give the address time to stabilize). The addressing modes are described in the instruction description. 
--  In brief, an address may come from the second word of the instruction, the X, Y, Z, or SP registers (unchanged or with 
--  pre/post-increment/decrement), or the Y or Z registers with a 6-bit unsigned offset (range 0 to 63).
--
--  The PAU component generates the addresses for reads of the program memory data. The program memory is addressed as 
--  16-bit words with 16-bits of address (64 Kwords total program memory space). Note that the AVR architecture actually 
--  supports up to 4 Mwords of program memory space, but this implementation will only use 64 Kwords. The address bus is 
--  called ProgAB and the data bus is called ProgDB. Both are declared as std_logic_vector (15 downto 0). Note that this 
--  interface is read only.
--
--
--  The complete list of instructions included in the CPU are listed below: 
--      AVR ALU Instructions
--     | Opcode | Op1 | Op2 | Result | Flags Affected |       Description         |
--     |  ADC   | Rd  | Rr  |   Rd   |  C Z N V S H   | Rd = Rd + Rr + C          |
--     |  ADD   | Rd  | Rr  |   Rd   |  C Z N V S H   | Rd = Rd + Rr              |
--     |  ADIW  | Rd  | K   |   Rd   |  C Z N V S     | Rd = Rd + K               |
--     |  AND   | Rd  | Rr  |   Rd   |    Z N V S     | Rd = Rd and Rr            |
--     |  ANDI  | Rd  | K   |   Rd   |    Z N V S     | Rd = Rd and K             |
--     |  ASR   | Rd  |     |   Rd   |  C Z N V S     | Arithmetic shift right Rd |
--     |  BCLR  | s   |     | SREG(s)|     SREG(s)    | SREG(s) = 0               |
--     |  BLD   | Rd  | b   |   Rd   |                | Rd(b) = T                 |
--     |  BSET  | s   |     | SREG(s)|     SREG(s)    | SREG(s) = 1               |
--     |  BST   | Rd  | b   |   T    |       T        | T = Rr(b)                 |
--     |  COM   | Rd  |     |   Rd   |  C Z N V S     | Rd = not Rd               |
--     |  CP    | Rd  | Rr  |        |  C Z N V S H   | Rd - Rr                   |
--     |  CPC   | Rd  | Rr  |        |  C Z N V S H   | Rd - Rr - C               |
--     |  CPI   | Rd  | K   |        |  C Z N V S H   | Rd - K                    |
--     |  DEC   | Rd  |     |   Rd   |    Z N V S     | Rd = Rd - 1               |
--     |  EOR   | Rd  | Rr  |   Rd   |    Z N V S     | Rd = Rd xor Rr            |
--     |  INC   | Rd  |     |   Rd   |    Z N V S     | Rd = Rd + 1               |
--     |  LSR   | Rd  |     |   Rd   |  C Z N V S     | Logical shift right Rd    |
--     |  NEG   | Rd  |     |   Rd   |  C Z N V S H   | Rd = -Rd                  |
--     |  OR    | Rd  | Rr  |   Rd   |    Z N V S     | Rd = Rd or Rr             |
--     |  ORI   | Rd  | K   |   Rd   |    Z N V S     | Rd = Rd or K              |
--     |  ROR   | Rd  |     |   Rd   |  C Z N V S     | Rotate right Rd           |
--     |  SBC   | Rd  | Rr  |   Rd   |  C Z N V S H   | Rd = Rd - Rr - C          |
--     |  SBCI  | Rd  | K   |   Rd   |  C Z N V S H   | Rd = Rd - K - C           |
--     |  SBIW  | Rd  | K   |   Rd   |  C Z N V S     | Rd = Rd - K               |
--     |  SUB   | Rd  | Rr  |   Rd   |  C Z N V S H   | Rd = Rd - Rr              |
--     |  SUBI  | Rd  | K   |   Rd   |  C Z N V S H   | Rd = Rd - K               |
--     |  SWAP  | Rd  |     |   Rd   |                | Swap nibbles of Rd        |
--     *ADIW and SBIW operate on pairs of registers 24|25, 26|27, 28|29, 30|31. The immediate instructions (ANDI, CPI, ORI, 
--      SBCI, and SUBI) only operate on the second half of the register set (registers 16 to 31). All instructions execute 
--      in one (1) clock cycle except the ADIW, MUL, and SBIW instructions which execute in two (2) clock cycles.
--
--      AVR Load/Store Instructions
--     | Opcode | Op1 | Op2 | Cycles |  Result   |                 Description                    |
--     |   LD   | Rd  | X   |    2   | Rd = (X)  | Load indirect with X                           |
--     |   LD   | Rd  | X+  |    2   | Rd = (X)  | Load indirect with X and post-inc              |
--     |        |     |     |        |  X = X+1  |                                                |
--     |   LD   | Rd  | -X  |    2   |  X = X-1  | Load indirect with X and pre-dec               |
--     |        |     |     |        | Rd = (X)  |                                                |
--     |   LD   | Rd  | Y+  |    2   | Rd = (Y)  | Load indirect with Y and post-inc              |
--     |        |     |     |        |  Y = Y+1  |                                                |
--     |   LD   | Rd  | -Y  |    2   |  Y = Y-1  | Load indirect with Y and pre-dec               |
--     |        |     |     |        | Rd = (Y)  |                                                |
--     |   LD   | Rd  | Z+  |    2   | Rd = (Z)  | Load indirect with Z and post-inc              |
--     |        |     |     |        |  Z = Z+1  |                                                |
--     |   LD   | Rd  | -Z  |    2   |  Z = Z-1  | Load indirect with Z and pre-dec               |
--     |        |     |     |        | Rd = (Z)  |                                                |
--     |   LDD  | Rd  | Y+g |    2   | Rd = (Y+g)| Load indirect with Y and unsigned displacement |
--     |   LDD  | Rd  | Z+g |    2   | Rd = (Z+g)| Load indirect with Z and unsigned displacement |
--     |   LDI  | Rd  | k   |    1   | Rd = k    | Load immediate value                           |
--     |   LDS  | Rd  | m   |    3   | Rd = (m)  | Load from memory                               | 
--     |   MOV  | Rd  | Rr  |    1   | Rd = Rr   | Move register to register                      | 
--     |   ST   | X   | Rr  |    2   | (X) = Rr  | Store indirect with X                          |
--     |   ST   | X+  | Rr  |    2   | (X) = Rr  | Store indirect with X and post-inc             |
--     |        |     |     |        |  X = X+1  |                                                |
--     |   ST   | -X  | Rr  |    2   |  X = X-1  | Store indirect with X and pre-dec              |
--     |        |     |     |        | (X) = Rr  |                                                |
--     |   ST   | Y+  | Rr  |    2   | (Y) = Rr  | Store indirect with Y and post-inc             |
--     |        |     |     |        |  Y = Y+1  |                                                |
--     |   ST   | -Y  | Rr  |    2   |  Y = Y-1  | Store indirect with Y and pre-dec              |
--     |        |     |     |        | (Y) = Rr  |                                                |
--     |   ST   | Z+  | Rr  |    2   | (Z) = Rr  | Store indirect with Z and post-inc             | 
--     |        |     |     |        |  Z = Z+1  |                                                |
--     |   ST   | -Z  | Rr  |    2   |  Z = Z-1  | Store indirect with Z and pre-dec              |
--     |        |     |     |        | (Z) = Rr  |                                                |
--     |   STD  | Y+g | Rr  |    2   | (Y+g) = Rr| Store indirect with Y and unsigned displacement|
--     |   STD  | Z+g | Rr  |    2   | (Z+g) = Rr| Store indirect with Z and unsigned displacement|
--     |   STS  | m   | Rr  |    3   | (m) = Rr  | Store to memory                                |
--     |   POP  | Rd  |     |    2   | SP = SP+1 | Pop register off stack                         |
--     |        |     |     |        | Rd = (SP) |                                                |
--     |   PUSH | Rr  |     |    2   | (SP) = Rr | Push register onto stack                       | 
--     |        |     |     |        | SP = SP-1 |                                                |
--     k = 8-bit immediate value
--     q = 6-bit unsigned value 
--     m = 16-bit address
--
--     Unconditional Branch Instructions
--     | Opcode | Op | Cycles |               Action             |
--     |  JMP   | a  |   3    | PC = a                           |
--     |  RJMP  | j  |   2    | PC = PC + 1 + j                  | 
--     |  IJMP  |    |   2    | PC = Z                           | 
--     |  CALL  | a  |   4    | Direct Subroutine Call           |
--     |        |    |        | (SP) = (PC+2)[15:8]; SP = SP - 1 |  
--     |        |    |        | (SP) = (PC+2)[7:0]; SP = SP - 1  |
--     |        |    |        | PC = a                           |
--     |  RCALL | j  |   3    | Relative Subroutine Call         |
--     |        |    |        | (SP) = (PC+1)[15:8]; SP = SP - 1 |  
--     |        |    |        | (SP) = (PC+1)[7:0]; SP = SP - 1  |
--     |        |    |        | PC = PC = PC + 1 + j             | 
--     |  ICALL |    |   3    | Indirect Subroutine Call         |
--     |        |    |        | (SP) = (PC+1)[15:8]; SP = SP - 1 |  
--     |        |    |        | (SP) = (PC+1)[7:0]; SP = SP - 1  |
--     |        |    |        | PC = Z                           | 
--     |  RET   |    |   4    | Subroutine Return (PC popped)    | 
--     |        |    |        | SP = SP + 1; PC[7:0] = (SP)      |
--     |        |    |        | SP = SP + 1; PC[15:8] = (SP)     |
--     |  RETI  |    |   4    | Interrupt Return (PC popped)     | 
--     |        |    |        | I = 1                            |
--     |        |    |        | SP = SP + 1; PC[7:0] = (SP)      |
--     |        |    |        | SP = SP + 1; PC[15:8] = (SP)     |
--     a = 22-bit program address 
--     j = 12-bit relative address
-- 
--     Conditional Branch Instructions
--     | Opcode | Op1 | Op2 | Cycles |              Action               | 
--     |  BRBC  |  b  |  r  |  1/2   |  Branch if status bit clear       |   
--     |        |     |     |        | if SREG(b)=0 then PC = PC + 1 + r |
--     |  BRBS  |  b  |  r  |  1/2   |  Branch if status bit set         |
--     |        |     |     |        | if SREG(b)=1 then PC = PC + 1 + r |
--     b = 3-bit unsigned value
--     r = 7-bit signed value
-- 
--     Skip Instructions
--     | Opcode | Op1 | Op2 | Cycles |              Action               |
--     |  CPSE  | Rd  | Rr  | 1/2/3  |  Compare and skip if equal        | 
--     |        |     |     |        | if Rd=Rr then PC = PC+2 or 3      |
--     |  SBRC  | Rr  | b   | 1/2/3  |  Skip if bit clear                | 
--     |        |     |     |        | if Rr(b)=0 then PC = PC+2 or 3    |
--     |  SBRS  | Rr  | b   | 1/2/3  |  Skip if bit set                  | 
--     |        |     |     |        | if Rr(b)=1 then PC = PC+2 or 3    |
--     b = 3-bit unsigned value
-- 
--
--  Revision History:
--     11 May 98  Glen George       Initial revision.
--      9 May 00  Glen George       Updated comments.
--      7 May 02  Glen George       Updated comments.
--     21 Jan 08  Glen George       Updated comments.
--     02 Feb 23  Hector Wilson     Initial revision. 
--     18 Mar 23  Hector Wilson     Added top level controls for flags.
--     19 Mar 23  Hector Wilson     Finalized design. Added comments. 
--     20 Mar 23  Hector Wilson     Added multiply functionality. 
--     22 Mar 23  Hector Wilson     Started working on pipelining. 
--     23 Mar 23  Hector Wilson     Added top level DFF arrays for pipelining control signals.
--     24 Mar 23  Hector Wilson     Added more DFF arrays. 
--     26 Mar 23  Hector Wilson     Added more DFF arrays. 
--     30 Mar 23  Hector Wilson     Tweaked SREG control signals for pipelining. 
--     31 Mar 23  Hector Wilson     Completed pipelining, passes test vectors. 
--     01 Apr 23  Hector Wilson     Updated comments. 
--
--------------------------------------------------------------------------------------------------------------------
--
--  AVR_CPU_PIPELINE
--
--  This is the complete entity declaration for the AVR CPU.  It is used to
--  test the complete design.
--
--  Inputs:
--    ProgDB - program memory data bus (16 bits)
--    Reset  - active low reset signal
--    INT0   - active low interrupt
--    INT1   - active low interrupt
--    clock  - the system clock
--
--  Outputs:
--    ProgAB - program memory address bus (16 bits)
--    DataAB - data memory address bus (16 bits)
--    DataWr - data write signal
--    DataRd - data read signal
--
--  Inputs/Outputs:
--    DataDB - data memory data bus (8 bits)
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.numeric_std.all;

library opcodes;
use opcodes.opcodes.all;


entity  AVR_CPU  is
    generic (
        addrsize : integer := 16; -- default address size is 16 bits
        regsize : integer := 8 -- default register size is 8 bits
    );

    port (
        ProgDB  :  in     std_logic_vector(15 downto 0);   -- program memory data bus
        Reset   :  in     std_logic;                       -- reset signal (active low)
        INT0    :  in     std_logic;                       -- interrupt signal (active low)
        INT1    :  in     std_logic;                       -- interrupt signal (active low)
        clock   :  in     std_logic;                       -- system clock
        ProgAB  :  out    std_logic_vector(15 downto 0);   -- program memory address bus
        DataAB  :  out    std_logic_vector(15 downto 0);   -- data memory address bus
        DataWr  :  out    std_logic;                       -- data memory write enable (active low)
        DataRd  :  out    std_logic;                       -- data memory read enable (active low)
        DataDB  :  inout  std_logic_vector(7 downto 0)     -- data memory data bus
    );

end  AVR_CPU;

architecture Behavioral of AVR_CPU is
--------------------------------------------------------------------------------------------------------------------
-- COMPONENT DECLERATIONS ------------------------------------------------------------------------------------------
    component ALU 
        generic (
            wordsize : integer := 8      -- default width is 8-bits
        );

        port(
            ALUOpA   : in      std_logic_vector(wordsize - 1 downto 0);   -- first operand
            ALUOpB   : in      std_logic_vector(wordsize - 1 downto 0);   -- second operand
            Cin      : in      std_logic;                                 -- carry in
            FCmd     : in      std_logic_vector(3 downto 0);              -- F-Block operation
            CinCmd   : in      std_logic_vector(1 downto 0);              -- carry in operation
            SCmd     : in      std_logic_vector(2 downto 0);              -- shift operation
            ALUCmd   : in      std_logic_vector(1 downto 0);              -- ALU result select
            NOp      : in      std_logic;                                 -- indicates if N flag is zero'd
            NegOp    : in      std_logic;                                 -- indicates if Neg Operation is performed
            Mulcycle : in      std_logic;                                 -- cycle (0/1) of multiplication operation
            Result   : buffer  std_logic_vector(wordsize - 1 downto 0);   -- ALU result
            Cout     : out     std_logic;                                 -- carry out
            HalfCout : out     std_logic;                                 -- half carry out
            Overflow : out     std_logic;                                 -- signed overflow
            Zero     : out     std_logic;                                 -- result is zero
            Signed   : out     std_logic;                                 -- adjusted sign of result               
            Negative : out     std_logic                                  -- sign of result
        );
    end component; 

    component StatusReg
        generic (
            wordsize : integer := 8      -- default width is 8-bits
        );

        port(
            RegIn    : in      std_logic_vector(wordsize - 1 downto 0);   -- data to write to register
            RegMask  : in      std_logic_vector(wordsize - 1 downto 0);   -- write mask
            HCHoldOp : in      std_logic;                                 -- signal high if holding status register flag H
            SHoldOp  : in      std_logic;                                 -- signal high if holding status register flag S
            VHoldOp  : in      std_logic;                                 -- signal high if holding status register flag V
            NHoldOp  : in      std_logic;                                 -- signal high if holding status register flag N
            ZHoldOp  : in      std_logic;                                 -- signal high if holding status register flag Z
            CHoldOp  : in      std_logic;                                 -- signal high if holding status register flag C
            clock    : in      std_logic;                                 -- system clock
            reset    : in      std_logic;                                 -- system reset
            RegOut   : buffer  std_logic_vector(wordsize - 1 downto 0)    -- current register value
        );
    end component;

    component MemUnit 
        generic (
            srcCnt       : integer;
            offsetCnt    : integer;
            maxIncDecBit : integer := 0; -- default is only inc/dec bit 0
            wordsize     : integer := 16 -- default address width is 16 bits
        );

        port(
            AddrSrc    : in   std_logic_vector(srccnt * wordsize - 1 downto 0);
            SrcSel     : in   integer  range srccnt - 1 downto 0;
            AddrOff    : in   std_logic_vector(offsetcnt * wordsize - 1 downto 0);
            OffsetSel  : in   integer  range offsetcnt - 1 downto 0;
            IncDecSel  : in   std_logic;
            IncDecBit  : in   integer  range maxIncDecBit downto 0;
            PrePostSel : in   std_logic;
            Address    : out  std_logic_vector(wordsize - 1 downto 0);
            AddrSrcOut : out  std_logic_vector(wordsize - 1 downto 0)
        );
    end component;

    component RegArray 
        generic (
            regcnt   : integer := 32;    -- default number of registers is 32
            wordsize : integer := 8      -- default width is 8-bits
        );

        port(
            RegIn     : in   std_logic_vector(wordsize - 1 downto 0);
            RegInSel  : in   integer  range regcnt - 1 downto 0;
            RegStore  : in   std_logic;
            RegASel   : in   integer  range regcnt - 1 downto 0;
            RegBSel   : in   integer  range regcnt - 1 downto 0;
            RegDIn    : in   std_logic_vector(2 * wordsize - 1 downto 0);
            RegDInSel : in   integer  range regcnt/2 - 1 downto 0;
            RegDStore : in   std_logic;
            RegDSel   : in   integer  range regcnt/2 - 1 downto 0;
            clock     : in   std_logic;
            RegA      : out  std_logic_vector(wordsize - 1 downto 0);
            RegB      : out  std_logic_vector(wordsize - 1 downto 0);
            RegD      : out  std_logic_vector(2 * wordsize - 1 downto 0)
        );
    end component;

    component ControlUnit 
        generic (
            regcnt   : integer := 32;    -- default number of registers is 32
            addrsize : integer := 16;    -- address width is 16-bits
            regsize  : integer := 8;     -- register width is 8-bits

            dausrccnt: integer := 4;     -- DAU has 5 sources RegD, SP, ProgDB, and 0s
            dauoffcnt: integer := 2;     -- DAU has 2 offset (6-bit) and 0s
            pausrccnt: integer := 1;     -- PAU has 1 source (PC)
            pauoffcnt: integer := 3      -- PAU has 5 offsets (+- 7-bit, +- 12-bit) and 0s
        );

        port(
            -- generic outputs
            DataBus   : in   std_logic_vector(addrsize-1 downto 0);      -- Incoming Program Data Bus 
            Flags     : in   std_logic_vector(7 downto 0);               -- ALU Flags
            SREG      : in   std_logic_vector(7 downto 0);               -- SREG Flags
            clock     : in   std_logic;                                  -- Clock
            Reset     : in   std_logic;                                  -- Reset signal
            Wr        : out  std_logic;                                  -- Write signal
            Rd        : out  std_logic;                                  -- Read signal
            NoWrite   : out  std_logic;
            NoWrite2  : out  std_logic;
            IR        : out  std_logic_vector(addrsize-1 downto 0);      -- Current Instruction Register

            -- ALU control outputs
            FCmd     : out     std_logic_vector(3 downto 0);             -- F-Block operation
            CinCmd   : out     std_logic_vector(1 downto 0);             -- carry in operation
            SCmd     : out     std_logic_vector(2 downto 0);             -- shift operation
            ALUCmd   : out     std_logic_vector(1 downto 0);             -- ALU result select
            Mulcycle : out     std_logic;                                -- cycle (0/1) of multiplication operation

            -- DAU control outputs
            SrcSelDAU     : out integer range dausrccnt-1 downto 0;
            OffsetSelDAU  : out integer range dauoffcnt-1 downto 0;
            IncDecSelDAU  : out std_logic;
            IncDecBitDAU  : out integer range 7 downto 0;
            PrePostSelDAU : out std_logic;

            -- PAU control outputs
            SrcSelPAU     : out integer range pausrccnt-1 downto 0;
            OffsetSelPAU  : out integer range pauoffcnt-1 downto 0;
            IncDecSelPAU  : out std_logic;
            IncDecBitPAU  : out integer range 1 downto 0;
            PrePostSelPAU : out std_logic;

            -- Reg Array Control outputs
            RegInSel  : out   integer  range regcnt - 1 downto 0;
            RegStore  : out   std_logic;
            RegASel   : out   integer  range regcnt - 1 downto 0;
            RegBSel   : out   integer  range regcnt - 1 downto 0;
            RegDInSel : out   integer  range regcnt/2 - 1 downto 0;
            RegDStore : out   std_logic;
            RegDSel   : out   integer  range regcnt/2 - 1 downto 0;

            -- Top level control outputs 
            DOFS      : out   std_logic;                     -- top level data-address offset-select mux control
                                                             -- selects what to feed into the DataAB.
            POFS      : out   std_logic;                     -- top level program-address offset-select mux control
                                                             -- selects what to feed into the PrgAB. 
            DDBS      : out   std_logic_vector(1 downto 0);  -- top level data-data-bus mux control selects what is being
                                                             -- output to the data data bus. 
            HoldPC    : out   std_logic;                     -- indicates if holding value of PC for multiple cycles
            ALUOpASel : out   std_logic_vector(1 downto 0);
            ALUOpBSel : out   std_logic_vector(1 downto 0);
            SPU       : out   std_logic;                     -- indicates if loading to stack pointer
            PCU       : out   std_logic;                     -- indicates if loading to program counter
            SUBR      : out   std_logic;                     -- indicates if doing subroutine call (DataAB determined by PAU)
            SOp       : out   std_logic_vector(1 downto 0);  -- indicates if writing to status register
            ZOp       : out   std_logic;                     -- indicates first byte Z flag for double width ALU cmds
            NOp       : out   std_logic;
            SubOp     : out   std_logic;                     -- indicates if doing a subtraction and should invert carries
            HCHoldOp  : out   std_logic;                     -- signal high if holding status register flag H
            SHoldOp   : out   std_logic;                     -- signal high if holding status register flag S
            VHoldOp   : out   std_logic;                     -- signal high if holding status register flag V
            NHoldOp   : out   std_logic;                     -- signal high if holding status register flag N
            ZHoldOp   : out   std_logic;                     -- signal high if holding status register flag Z
            CHoldOp   : out   std_logic;                     -- signal high if holding status register flag C
            NegOp     : out   std_logic;                     -- indicates if flags should update from NEG operation
            TflOp     : out   integer range 0 to 7;
            RegLoadSel : out  std_logic_vector(1 downto 0)
        );
    end component;

--------------------------------------------------------------------------------------------------------------------
-- SIGNAL DECLERATIONS ---------------------------------------------------------------------------------------------
    -- REG ARRAY SIGNALS 
    signal RegIn                    : std_logic_vector(7 downto 0); 
    signal RegDIn                   : std_logic_vector(15 downto 0); 
  
    -- ALU SIGNALS
    signal ALUOpA, ALUOpB : std_logic_vector(7 downto 0);
    signal Cin,Cout,HalfCout,Signed,Overflow,Zero,Negative : std_logic;

    -- SREG SIGNALS
    signal RegInS,RegMask,RegOut : std_logic_vector(7 downto 0);
    signal HCHoldOp,SHoldOp,VHoldOp,NHoldOp,ZHoldOp,CHoldOp : std_logic;

    -- MAU SIGNALS
    -- (DAU):
    signal AddrSrcDAU    : std_logic_vector(63 downto 0);
    signal AddrOffDAU    : std_logic_vector(31 downto 0);
    signal AddressDAU    : std_logic_vector(15 downto 0);
    signal AddrSrcOutDAU : std_logic_vector(15 downto 0);
    -- (PAU):
    signal AddrSrcPAU    : std_logic_vector(15 downto 0);
    signal AddrOffPAU    : std_logic_vector(47 downto 0);
    signal AddressPAU    : std_logic_vector(15 downto 0);
    signal AddrSrcOutPAU : std_logic_vector(15 downto 0);

    -- TOP LEVEL Program Counter and Stack Pointer
    signal SP : std_logic_vector(addrsize-1 downto 0);
    signal PC : std_logic_vector(addrsize-1 downto 0);

    signal PCS   : std_logic_vector(2 downto 0);
    signal ALUFlags : std_logic_vector(7 downto 0);

    signal NoWrite : std_logic;
    signal NoWrite2 : std_logic;

     signal DataWr_i : std_logic;

    -- zero vectors
    constant zerobyte : std_logic_vector(15 downto 0) := (others => '0'); -- 16 bits of zero
    constant zero10 : std_logic_vector(9 downto 0) := (others => '0');  -- 10 bits of zero
--------------------------------------------------------------------------------------------------------------------
-- PIPELINING DFF ARRAYS -------------------------------------------------------------------------------------------
--
-- To implement pipelining for this CPU we have DFF arrays for various control signals throughout the CPU. Each of these
-- arrays contains 4 stages/registers (or 3 DFFs) although not all DFFs are used for each control signal. The synthesizer
-- eliminates these surplus DFFs. 
-- 
    type flops3_16 is array (0 to 3) of std_logic_vector(15 downto 0); 
    type flops3_8 is array (0 to 3) of std_logic_vector(7 downto 0);
    type flops3_4 is array (0 to 3) of std_logic_vector(3 downto 0);
    type flops3_3 is array (0 to 3) of std_logic_vector(2 downto 0);
    type flops3_2 is array (0 to 3) of std_logic_vector(1 downto 0);
    type flops3_1 is array (0 to 3) of std_logic;
    type flops3_i_8 is array (0 to 3) of integer range 0 to 7;
    type flops3_i_32 is array (0 to 3) of integer range 0 to 31;

    -- IR DFF array
    signal IR_p : flops3_16; 

    -- DAU control signals DFF arrays
    signal PrePostSelDAU_p : flops3_1;
    signal IncDecSelDAU_p : flops3_1;
    signal IncDecBitDAU_p : flops3_i_8;
    signal OffsetSelDAU_p : flops3_i_8;
    signal SrcSelDAU_p : flops3_i_8;

    -- PAU control signals DFF arrays
    signal PrePostSelPAU_p : flops3_1;
    signal IncDecSelPAU_p : flops3_1;
    signal IncDecBitPAU_p : flops3_i_8;
    signal OffsetSelPAU_p : flops3_i_8;
    signal SrcSelPAU_p : flops3_i_8;

    -- PC control signals DFF arrays
    signal SUBR_p : flops3_1; 
    signal HoldPC_p : flops3_1;
    signal PCU_p : flops3_1;
    signal SPU_p : flops3_1;


    -- PAU/DAU offset-select DFF arrays
    signal DOFS_p : flops3_1;
    signal POFS_p : flops3_1;

    -- Program Data Bus DFF array
    signal ProgDB_p : flops3_16;

    -- Read/Write for Data Data Bus DFF arrays
    signal Write_p : flops3_1;
    signal DataRd_p : flops3_1;
    signal DDBS_p : flops3_2;

    -- Reg Array control signal DFF arrays
    signal RegASel_p : flops3_i_32;
    signal RegBSel_p : flops3_i_32;
    signal RegDSel_p : flops3_i_32;
    signal RegA_p : flops3_8;
    signal RegB_p : flops3_8;
    signal RegD_p : flops3_16;
    signal RegInSel_p : flops3_i_32;
    signal RegDInSel_p : flops3_i_32;
    signal RegStore_p : flops3_1;
    signal RegDStore_p : flops3_1;

    -- Top level ALU control signal DFF arrays
    signal RegLoadSel_p : flops3_2;
    signal ALUOpASel_p : flops3_2;
    signal ALUOpBSel_p : flops3_2;

    -- ALU control signal DFF arrays
    signal FCmd_p : flops3_4;
    signal ALUCmd_p : flops3_2;
    signal CinCmd_p : flops3_2;
    signal SCmd_p : flops3_3;
    signal HCHoldOp_p : flops3_1;
    signal SHoldOp_p : flops3_1;
    signal VHoldOp_p : flops3_1;
    signal NHoldOp_p : flops3_1;
    signal ZHoldOp_p : flops3_1;
    signal CHoldOp_p : flops3_1;
    signal Mulcycle_p : flops3_1;
    signal Result_p : flops3_8;

    -- Top level SREG DFF arrays
    signal SOp_p : flops3_2;
    signal NOp_p : flops3_1;
    signal ZOp_p : flops3_1;
    signal SubOp_p : flops3_1;
    signal NegOp_p : flops3_1;
    signal TflOp_p : flops3_i_8;

--------------------------------------------------------------------------------------------------------------------

begin
    ALUFlags <= RegOut(7 downto 6) & HalfCout & Signed & Overflow & Negative & Zero & Cout;
    -- INSTANTIATIONS -------------------------------------------------------------------------------------------------- 
    -- Reg Array instantiation
    REG : RegArray 
    generic map (
        regcnt => 32,
        wordsize => 8
    )
    port map (
        RegIn => RegIn,
        RegInSel => RegInSel_p(3),
        RegStore => RegStore_p(3),
        RegASel => RegASel_p(3),
        RegBSel => RegBSel_p(3),
        RegDIn => RegDIn,
        RegDInSel => RegDInSel_p(3),
        RegDStore => RegDStore_p(3),
        RegDSel => RegDSel_p(3),
        clock => clock,
        RegA => RegA_p(0),
        RegB => RegB_p(0),
        RegD => RegD_p(0)
    );

    -- ALU instantiation 
    ALU1 : ALU 
    generic map(
        wordsize => 8
    )
    port map ( 
        ALUOpA => ALUOpA,
        ALUOpB => ALUOpB,
        Cin => RegOut(0),    -- carry in to the ALU is the first bit of SREG
        FCmd => FCmd_p(3), 
        CinCmd => CinCmd_p(3),
        SCmd => SCmd_p(3), 
        ALUCmd => ALUCmd_p(3), 
        NOp => NOp_p(3),
        NegOp => NegOp_p(3),
        Mulcycle => Mulcycle_p(3),
        Result => Result_p(0),
        Cout => Cout,
        HalfCout => HalfCout,
        Overflow => Overflow,
        Zero => Zero,
        Signed => Signed,       
        Negative => Negative
    );

    -- Status Register instantiation
    SREG1 : StatusReg 
    generic map(
        wordsize => 8      -- 8-bit status register (8 flags)
    )
    port map (
        RegIn => RegInS,
        RegMask => RegMask,
        HCHoldOp => HCHoldOp_p(3),
        SHoldOp => SHoldOp_p(3),
        VHoldOp => VHoldOp_p(3),
        NHoldOp => NHoldOp_p(3),
        ZHoldOp => ZHoldOp_p(0),
        CHoldOp => CHoldOp_p(3),
        clock => clock,
        reset => Reset,
        RegOut => RegOut
    );

    -- DAU instantiation
    DAU: MemUnit 
    generic map (
        srcCnt => 4,       -- (X,Y,Z)/RegD, SP, ProgDB, and 0s
        offsetCnt => 2,    -- two offsets for DAU (6-bit) and 0s
        maxIncDecBit => 7, -- max inc/dec bit 7
        wordsize => 16     -- address width if 16 bits
    )
    port map (
        AddrSrc => AddrSrcDAU,
        SrcSel => SrcSelDAU_p(3),
        AddrOff => AddrOffDAU,
        OffsetSel => OffsetSelDAU_p(3),
        IncDecSel => IncDecSelDAU_p(3),
        IncDecBit => IncDecBitDAU_p(3),
        PrePostSel => PrePostSelDAU_p(3),
        Address => AddressDAU,
        AddrSrcOut => AddrSrcOutDAU
    );

    -- PAU instantiation
    PAU: MemUnit 
    generic map (
        srcCnt => 1,       -- PC
        offsetCnt => 3,    -- three offsets for PAU (+- 7-bit or +- 12-bit) and 0s
        maxIncDecBit => 1, -- inc/dec bit 0
        wordsize => 16     -- address width if 16 bits
    )
    port map (
        AddrSrc => AddrSrcPAU,
        SrcSel => SrcSelPAU_p(3),
        AddrOff => AddrOffPAU,
        OffsetSel => OffsetSelPAU_p(3),
        IncDecSel => IncDecSelPAU_p(3),
        IncDecBit => IncDecBitPAU_p(3),
        PrePostSel => PrePostSelPAU_p(3),
        Address => AddressPAU,
        AddrSrcOut => AddrSrcOutPAU
    );

    -- CU instantiation
    CU: ControlUnit
    generic map (
        regcnt => 32,    -- default number of registers is 32
        addrsize => 16,  -- address width is 16-bits
        regsize => 8,    -- register width is 8-bits

        dausrccnt => 4,  -- DAU has 5 sources RegD, SP, ProgDB, and 0s
        dauoffcnt => 2,  -- DAU has 2 offset (0, 6-bit)
        pausrccnt => 1,  -- PAU has 1 source PC
        pauoffcnt => 3   -- PAU has 3 offsets (0, sign extended 7-bit, sign extended 12-bit)
    )

    port map (
        -- generic outputs
        DataBus => ProgDB, -- pass program data bus into Control Unit Data Bus to decode
        Flags => ALUFlags,   -- pass ALU Flag outputs into CU for flags 
        SREG => RegOut,
        clock => clock,
        Reset => Reset,
        Wr => Write_p(0),
        Rd => DataRd_p(0),
        NoWrite => NoWrite,
        NoWrite2 => NoWrite2,
        IR => IR_p(0),

        -- ALU control outputs
        FCmd => FCmd_p(0),            -- F-Block operation
        CinCmd => CinCmd_p(0),        -- carry in operation
        SCmd => SCmd_p(0),            -- shift operation
        ALUCmd => ALUCmd_p(0),        -- ALU result select
        Mulcycle => Mulcycle_p(0),    -- cycle (0/1) of multiplication operation

        -- DAU control outputs
        SrcSelDAU => SrcSelDAU_p(0),
        OffsetSelDAU => OffsetSelDAU_p(0),
        IncDecSelDAU => IncDecSelDAU_p(0),
        IncDecBitDAU => IncDecBitDAU_p(0),
        PrePostSelDAU => PrePostSelDAU_p(0),

        -- PAU control outputs
        SrcSelPAU => SrcSelPAU_p(0),
        OffsetSelPAU => OffsetSelPAU_p(0),
        IncDecSelPAU => IncDecSelPAU_p(0),
        IncDecBitPAU => IncDecBitPAU_p(0),
        PrePostSelPAU => PrePostSelPAU_p(0),

        -- Reg Array Control outputs
        RegInSel => RegInSel_p(0),
        RegStore => RegStore_p(0),
        RegASel => RegASel_p(0),
        RegBSel => RegBSel_p(0),
        RegDInSel => RegDInSel_p(0),
        RegDStore => RegDStore_p(0),
        RegDSel => RegDSel_p(0),

        -- Top level control outputs
        DOFS => DOFS_p(0),                               -- top level data-address offset-select mux control
                                                         -- selects whether or not to output offset-adjusted address
                                                         -- from the DAU.
        POFS => POFS_p(0),                               -- top level program-address offset-select mux control
                                                         -- selects whether or not to output offset-adjusted address
                                                         -- from the PAU.
        DDBS => DDBS_p(0),                               -- top level data-data-bus mux control selects what is being
                                                         -- output to the data data bus. 
        HoldPC => HoldPC_p(0),
        ALUOpASel => ALUOpASel_p(0),                          -- these select what to feed to the ALU inputs A/B
        ALUOpBSel => ALUOpBSel_p(0),
        PCU => PCU_p(0),
        SPU => SPU_p(0), 
        SUBR => SUBR_p(0),
        SOp => SOp_p(0),
        ZOp => ZOp_p(0),
        NOp => NOp_p(0),
        TflOp => TflOp_p(0),
        SubOp => SubOp_p(0),                                  -- indicates if doing a subtraction (i.e. invert carry outs)
        HCHoldOp => HCHoldOp_p(0),
        SHoldOp => SHoldOp_p(0),
        VHoldOp => VHoldOp_p(0),
        NHoldOp => NHoldOp_p(0),
        ZHoldOp => ZHoldOp_p(0),
        CHoldOp => CHoldOp_p(0),
        NegOp => NegOp_p(0),                                  -- indicates if flags should update from NEG operation
        RegLoadSel => RegLoadSel_p(0)
    );

    -----------------------------------------------------------------------------------------
    -- Program Counter (PC) and Stack Pointer (SP) MUX
    -- This process implements the Program Counter and Stack Pointer.
    -- On the rising edge of the clock, the process checks updates the value of the 16-bit PC and SP. 
    -- 
    -- There are three control signals (SUBR,HoldPC,PCU) from the CU which make up the PCS signal that
    -- determines how to update the PC. 
    --    PCS = 000
    --          PC is updated with AddrSrcOutPAU if POFS = 0 (inc/dec data addr source no offset)
    --          PC is updated with AddressPAU if POFS = 1 (inc/dec data addr source with offset)
    --    PCS = 010 
    --          PC is held 
    --    PCS = 001 
    --          PC is updated with data from Program Data Bus (ProgDB)
    --    PCS = 011 
    --          PC is updated with data from RegD (used for IJMP-loads Z)
    --    PCS = 100
    --          low byte of PC is loaded with the DataDB (RET)
    --    Others
    --          high byte of PC is loaded with DataDB (RET)
    --
    -- There is one control signal (SPU) from the CU that determines how to update the SP. 
    --    SPU = 0 
    --          SP is held
    --    SPU = 1 
    --          SP is updated with AddrSrcOutDAU (inc/dec data addr source no offset)
    --
    --
    -- Both PC and SP are reset synchronously by the system reset. 
    --

    PCS <= SUBR_p(3) & HoldPC_p(3) & PCU_p(3);
    process(clock) 
    begin  
        if rising_edge(clock) then
            if Reset = '1' then 
                    PC <= (others => '0');
                    SP <= (others => '0');
            else 
                if PCS = "000" then 
                     if POFS_p(3) = '0' then 
                        PC <= AddrSrcOutPAU;
                     else 
                        PC <= AddressPAU;
                     end if;
                elsif PCS = "010" then 
                    PC <= PC;      -- holding program counter
                elsif PCS = "001" then 
                    PC <= ProgDB_p(2);
                elsif PCS = "011" then -- for IJMP 
                    PC <= RegD_p(0);
                elsif PCS = "100" then 
                    PC(7 downto 0) <= DataDB; 
                else
                    PC(15 downto 8) <= DataDB; 
                end if;

                if SPU_p(3) = '0' then 
                    SP <= SP; 
                else 
                    SP <= AddrSrcOutDAU;
                end if;
            end if;
        end if;
    end process;

    -----------------------------------------------------------------------------------------
    -- PIPELINING
    --
    -- This process implements all top level pipelining for the CPU. DFF arrays are used to properly delay 
    -- control signals before reaching their target destination. 
    -- 
    -- We include a synchronous reset for each DFF included. 
    --
    process(clock) begin
        if rising_edge(clock) then
            ProgDB_p(0) <= ProgDB;
            if Reset = '1' then  -- Reset CPU system registers
                IR_p(1) <= (others => '0');
                IR_p(2) <= (others => '0');
                IR_p(3) <= (others => '0');

                PrePostSelDAU_p(1) <= '0';
                PrePostSelDAU_p(2) <= '0';
                PrePostSelDAU_p(3) <= '0';
                
                IncDecSelDAU_p(1) <= '0';
                IncDecSelDAU_p(2) <= '0';
                IncDecSelDAU_p(3) <= '0';

                IncDecBitDAU_p(1) <= 0;
                IncDecBitDAU_p(2) <= 0;
                IncDecBitDAU_p(3) <= 0;

                OffsetSelDAU_p(1) <= 0;
                OffsetSelDAU_p(2) <= 0;
                OffsetSelDAU_p(3) <= 0;

                SrcSelDAU_p(1) <= 0;
                SrcSelDAU_p(2) <= 0;
                SrcSelDAU_p(3) <= 0;

                PrePostSelPAU_p(1) <= '0';
                PrePostSelPAU_p(2) <= '0';
                PrePostSelPAU_p(3) <= '0';
                
                IncDecSelPAU_p(1) <= '0';
                IncDecSelPAU_p(2) <= '0';
                IncDecSelPAU_p(3) <= '0';

                IncDecBitPAU_p(1) <= 0;
                IncDecBitPAU_p(2) <= 0;
                IncDecBitPAU_p(3) <= 0;

                OffsetSelPAU_p(1) <= 0;
                OffsetSelPAU_p(2) <= 0;
                OffsetSelPAU_p(3) <= 0;

                SrcSelPAU_p(1) <= 0;
                SrcSelPAU_p(2) <= 0;
                SrcSelPAU_p(3) <= 0;

                SUBR_p(1) <= '0';
                SUBR_p(2) <= '0';
                SUBR_p(3) <= '0';

                HoldPC_p(1) <= '1';
                HoldPC_p(2) <= '1';
                HoldPC_p(3) <= '1'; 

                PCU_p(1) <= '0';
                PCU_p(2) <= '0';
                PCU_p(3) <= '0';

                SPU_p(1) <= '0';
                SPU_p(2) <= '0';
                SPU_p(3) <= '0';

                DOFS_p(1) <= '0';
                DOFS_p(2) <= '0';
                DOFS_p(3) <= '0';

                POFS_p(1) <= '0';
                POFS_p(2) <= '0';
                POFS_p(3) <= '0';

                ProgDB_p(1) <= (others => '0');
                ProgDB_p(2) <= (others => '0');
                ProgDB_p(3) <= (others => '0');

                Write_p(1) <= '1';
                Write_p(2) <= '1';
                Write_p(3) <= '1'; 

                DataRd_p(1) <= '1';
                DataRd_p(2) <= '1';
                DataRd_P(3) <= '1';

                DDBS_p(1) <= "00";
                DDBS_p(2) <= "00";
                DDBS_p(3) <= "00";

                RegASel_p(1) <= 0;
                RegASel_p(2) <= 0;
                RegASel_p(3) <= 0;

                RegBSel_p(1) <= 0;
                RegBSel_p(2) <= 0;
                RegBSel_p(3) <= 0;

                RegDSel_p(1) <= 0;
                RegDSel_p(2) <= 0;
                RegDSel_p(3) <= 0;

                RegA_p(1) <= (others => '0');
                RegA_p(2) <= (others => '0');
                RegA_p(3) <= (others => '0');

                RegB_p(1) <= (others => '0');
                RegB_p(2) <= (others => '0');
                RegB_p(3) <= (others => '0');

                RegD_p(1) <= (others => '0');
                RegD_p(2) <= (others => '0');
                RegD_p(3) <= (others => '0');

                RegInSel_p(1) <= 0;
                RegInSel_p(2) <= 0;
                RegInSel_p(3) <= 0;

                RegDInSel_p(1) <= 0;
                RegDInSel_p(2) <= 0;
                RegDInSel_p(3) <= 0;

                RegStore_p(1) <= '0';
                RegStore_p(2) <= '0';
                RegStore_p(3) <= '0';

                RegDStore_p(1) <= '0';
                RegDStore_p(2) <= '0';
                RegDStore_p(3) <= '0';

                RegLoadSel_p(1) <= "00";
                RegLoadSel_p(2) <= "00";
                RegLoadSel_p(3) <= "00";

                ALUOpASel_p(1) <= "00";
                ALUOpASel_p(2) <= "00";
                ALUOpASel_p(3) <= "00";

                ALUOpBSel_p(1) <= "00";
                ALUOpBSel_p(2) <= "00";
                ALUOpBSel_p(3) <= "00";

                FCmd_p(1) <= "0000";
                FCmd_p(2) <= "0000";
                FCmd_p(3) <= "0000";

                ALUCmd_p(1) <= "00";
                ALUCmd_p(2) <= "00";
                ALUCmd_p(3) <= "00";

                CinCmd_p(1) <= "00";
                CinCmd_p(2) <= "00";
                CinCmd_p(3) <= "00";

                SCmd_p(1) <= "000";
                SCmd_p(2) <= "000";
                SCmd_p(3) <= "000";

                HCHoldOp_p(1) <= '0';
                HCHoldOp_p(2) <= '0';
                HCHoldOp_p(3) <= '0';

                SHoldOp_p(1) <= '0';
                SHoldOp_p(2) <= '0';
                SHoldOp_p(3) <= '0';

                VHoldOp_p(1) <= '0';
                VHoldOp_p(2) <= '0';
                VHoldOp_p(3) <= '0';

                NHoldOp_p(1) <= '0';
                NHoldOp_p(2) <= '0';
                NHoldOp_p(3) <= '0';

                ZHoldOp_p(1) <= '0';
                ZHoldOp_p(2) <= '0';
                ZHoldOp_p(3) <= '0';

                CHoldOp_p(1) <= '0';
                CHoldOp_p(2) <= '0';
                CHoldOp_p(3) <= '0';

                Mulcycle_p(1) <= '0';
                Mulcycle_p(2) <= '0';
                Mulcycle_p(3) <= '0';

                Result_p(1) <= (others => '0');
                Result_p(2) <= (others => '0');
                Result_p(3) <= (others => '0');

                SOp_p(1) <= "00";
                SOp_p(2) <= "00";
                SOp_p(3) <= "00";

                NOp_p(1) <= '0';
                NOp_p(2) <= '0';
                NOp_p(3) <= '0';

                ZOp_p(1) <= '0';
                ZOp_p(2) <= '0';
                ZOp_p(3) <= '0';

                SubOp_p(1) <= '0';
                SubOp_p(2) <= '0';
                SubOp_p(3) <= '0';

                NegOp_p(1) <= '0';
                NegOp_p(2) <= '0';
                NegOp_p(3) <= '0';

                TflOp_p(1) <= 0;
                TflOp_p(2) <= 0;
                TflOp_p(3) <= 0;


            else
                ProgDB_p(1) <= ProgDB_p(0);
                ProgDB_p(2) <= ProgDB_p(1);
                ProgDB_p(3) <= ProgDB_p(2);

                IR_p(1) <= IR_p(0); 
                IR_p(2) <= IR_p(1); 
                IR_p(3) <= IR_p(2);

                SUBR_p(1) <= SUBR_p(0);
                SUBR_p(2) <= SUBR_p(1);
                SUBR_p(3) <= SUBR_p(2);

                HoldPC_p(1) <= HoldPC_p(0);
                if NoWrite2 = '0' then 
                    HoldPC_p(2) <= HoldPC_p(1);
                else
                    HoldPC_p(2) <= '0';  -- normal PC increment if cancelling write
                end if;
                if NoWrite = '0' then 
                    HoldPC_p(3) <= HoldPC_p(2);
                else   
                    HoldPC_p(3) <= '0'; -- normal PC increment if cancelling write
                end if;

                PCU_p(1) <= PCU_p(0);
                PCU_p(2) <= PCU_p(1);
                PCU_p(3) <= PCU_p(2);

                SPU_p(1) <= SPU_p(0);
                SPU_p(2) <= SPU_p(1);
                SPU_p(3) <= SPU_p(2);

                OffsetSelDAU_p(1) <= OffsetSelDAU_p(0);
                OffsetSelDAU_p(2) <= OffsetSelDAU_p(1);
                OffsetSelDAU_p(3) <= OffsetSelDAU_p(2);

                IncDecBitDAU_p(1) <= IncDecBitDAU_p(0);
                IncDecBitDAU_p(2) <= IncDecBitDAU_p(1);
                IncDecBitDAU_p(3) <= IncDecBitDAU_p(2);

                SrcSelDAU_p(1) <= SrcSelDAU_p(0);
                SrcSelDAU_p(2) <= SrcSelDAU_p(1);
                SrcSelDAU_p(3) <= SrcSelDAU_p(2);

                IncDecSelDAU_p(1) <= IncDecSelDAU_p(0);
                IncDecSelDAU_p(2) <= IncDecSelDAU_p(1);
                IncDecSelDAU_p(3) <= IncDecSelDAU_p(2);

                PrePostSelDAU_p(1) <= PrePostSelDAU_p(0);
                PrePostSelDAU_p(2) <= PrePostSelDAU_p(1); 
                PrePostSelDAU_p(3) <= PrePostSelDAU_p(2);

                OffsetSelPAU_p(1) <= OffsetSelPAU_p(0);
                OffsetSelPAU_p(2) <= OffsetSelPAU_p(1);
                OffsetSelPAU_p(3) <= OffsetSelPAU_p(2);

                IncDecBitPAU_p(1) <= IncDecBitPAU_p(0);
                IncDecBitPAU_p(2) <= IncDecBitPAU_p(1);
                IncDecBitPAU_p(3) <= IncDecBitPAU_p(2);

                SrcSelPAU_p(1) <= SrcSelPAU_p(0);
                SrcSelPAU_p(2) <= SrcSelPAU_p(1);
                SrcSelPAU_p(3) <= SrcSelPAU_p(2);

                IncDecSelPAU_p(1) <= IncDecSelPAU_p(0);
                IncDecSelPAU_p(2) <= IncDecSelPAU_p(1);
                IncDecSelPAU_p(3) <= IncDecSelPAU_p(2);

                PrePostSelPAU_p(1) <= PrePostSelPAU_p(0);
                PrePostSelPAU_p(2) <= PrePostSelPAU_p(1); 
                PrePostSelPAU_p(3) <= PrePostSelPAU_p(2);

                DOFS_p(1) <= DOFS_p(0);
                DOFS_p(2) <= DOFS_p(1);
                DOFS_p(3) <= DOFS_p(2);

                POFS_p(1) <= POFS_p(0);
                POFS_p(2) <= POFS_p(1);
                POFS_p(3) <= POFS_p(2);

                if NoWrite2 = '0' then 
                    Write_p(1) <= Write_p(0);
                else
                    Write_p(1) <= '1'; -- cancel write
                end if;
                if NoWrite = '0' then 
                    Write_p(2) <= Write_p(1);
                else
                    Write_p(2) <= '1'; -- cancel write
                end if;
                Write_p(3) <= Write_p(2);

                if NoWrite2 = '0' then 
                    DataRd_p(1) <= DataRd_p(0);
                else
                    DataRd_p(1) <= '1'; -- cancel read
                end if;
                if NoWrite = '0' then 
                    DataRd_p(2) <= DataRd_p(1);
                else 
                    DataRd_p(2) <= '1'; -- cancel read
                end if;
                DataRd_p(3) <= DataRd_p(2);

                if NoWrite2 = '0' then 
                    DDBS_p(1) <= DDBS_p(0);
                else
                    DDBS_p(1) <= "00";
                end if;
                if NoWrite = '0' then 
                    DDBS_p(2) <= DDBS_p(1);
                else 
                    DDBS_p(2) <= "00"; 
                end if;
                DDBS_p(3) <= DDBS_p(2);

                RegASel_p(1) <= RegASel_p(0);
                RegASel_p(2) <= RegASel_p(1);
                RegASel_p(3) <= RegASel_p(2);

                RegBSel_p(1) <= RegBSel_p(0);
                RegBSel_p(2) <= RegBSel_p(1);
                RegBSel_p(3) <= RegBSel_p(2);

                RegDSel_p(1) <= RegDSel_p(0);
                RegDSel_p(2) <= RegDSel_p(1);
                RegDSel_p(3) <= RegDSel_p(2);

                RegA_p(1) <= RegA_p(0);

                RegInSel_p(1) <= RegInSel_p(0);
                RegInSel_p(2) <= RegInSel_p(1);
                RegInSel_p(3) <= RegInSel_p(2);

                RegDInSel_p(1) <= RegDInSel_p(0);
                RegDInSel_p(2) <= RegDInSel_p(1);
                RegDInSel_p(3) <= RegDInSel_p(2);

                RegStore_p(1) <= RegStore_p(0);
                if NoWrite2 = '0' then 
                    RegStore_p(2) <= RegStore_p(1);
                else 
                    RegStore_p(2) <= '0';  -- cancel register write
                end if;
                if NoWrite = '0' then 
                    RegStore_p(3) <= RegStore_p(2);
                else
                    RegStore_p(3) <= '0'; -- cancel register write
                end if;

                RegDStore_p(1) <= RegDStore_p(0);
                if NoWrite2 = '0' then 
                    RegDStore_p(2) <= RegDStore_p(1);
                else 
                    RegStore_p(2) <= '0';  -- cancel double register write
                end if; 
                if NoWrite = '0' then 
                    RegDStore_p(3) <= RegDStore_p(2);
                else
                    RegDStore_p(3) <= '0'; -- cancel double register write
                end if;

                RegLoadSel_p(1) <= RegLoadSel_p(0);
                RegLoadSel_p(2) <= RegLoadSel_p(1);
                RegLoadSel_p(3) <= RegLoadSel_p(2);

                ALUOpASel_p(1) <= ALUOpASel_p(0);
                ALUOpASel_p(2) <= ALUOpASel_p(1);
                ALUOpASel_p(3) <= ALUOpASel_p(2);

                ALUOpBSel_p(1) <= ALUOpBSel_p(0);
                ALUOpBSel_p(2) <= ALUOpBSel_p(1);
                ALUOpBSel_p(3) <= ALUOpBSel_p(2);

                ALUCmd_p(1) <= ALUCmd_p(0);
                ALUCmd_p(2) <= ALUCmd_p(1);
                ALUCmd_p(3) <= ALUCmd_p(2);

                FCmd_p(1) <= FCmd_p(0);
                FCmd_p(2) <= FCmd_p(1);
                FCmd_p(3) <= FCmd_p(2);

                SCmd_p(1) <= SCmd_p(0);
                SCmd_p(2) <= SCmd_p(1);
                SCmd_p(3) <= SCmd_p(2);

                CinCmd_p(1) <= CinCmd_p(0);
                CinCmd_p(2) <= CinCmd_p(1);
                CinCmd_p(3) <= CinCmd_p(2);

                HCHoldOp_p(1) <= HCHoldOp_p(0);
                HCHoldOp_p(2) <= HCHoldOp_p(1);
                HCHoldOp_p(3) <= HCHoldOp_p(2);

                VHoldOp_p(1) <= VHoldOp_p(0);
                VHoldOp_p(2) <= VHoldOp_p(1);
                VHoldOp_p(3) <= VHoldOp_p(2);

                SHoldOp_p(1) <= SHoldOp_p(0);
                SHoldOp_p(2) <= SHoldOp_p(1);
                SHoldOp_p(3) <= SHoldOp_p(2);

                NHoldOp_p(1) <= NHoldOp_p(0);
                NHoldOp_p(2) <= NHoldOp_p(1);
                NHoldOp_p(3) <= NHoldOp_p(2);

                ZHoldOp_p(1) <= ZHoldOp_p(0);
                ZHoldOp_p(2) <= ZHoldOp_p(1);
                ZHoldOp_p(3) <= ZHoldOp_p(2);

                CHoldOp_p(1) <= CHoldOp_p(0);
                CHoldOp_p(2) <= CHoldOp_p(1);
                CHoldOp_p(3) <= CHoldOp_p(2);

                SOp_p(1) <= SOp_p(0);
                SOp_p(2) <= SOp_p(1);
                SOp_p(3) <= SOp_p(2);

                TflOp_p(1) <= TflOp_p(0);
                TflOp_p(2) <= TflOp_p(1);
                TflOp_p(3) <= TflOp_p(2);

                NOp_p(1) <= NOp_p(0);
                NOp_p(2) <= NOp_p(1);
                NOp_p(3) <= NOp_p(2);

                ZOp_p(1) <= ZOp_p(0);
                ZOp_p(2) <= ZOp_p(1);
                ZOp_p(3) <= ZOp_p(2);

                SubOp_p(1) <= SubOp_p(0);
                SubOp_p(2) <= SubOp_p(1);
                SubOp_p(3) <= SubOp_p(2);

                NegOp_p(1) <= NegOp_p(0);
                NegOp_p(2) <= NegOp_p(1);
                NegOp_p(3) <= NegOp_p(2);

                Mulcycle_p(1) <= Mulcycle_p(0);
                Mulcycle_p(2) <= Mulcycle_p(1);
                Mulcycle_p(3) <= Mulcycle_p(2);

                Result_p(1) <= Result_p(0);
            end if; 
        end if;
    end process;
    -----------------------------------------------------------------------------------------
    -- Program Address Bus
    -- ProgAB always outputs PC. 
    ProgAB <= PC;


    -- Data Data Bus Mux
    -- This process controls what is being output to the internal data data bus for an instruction. 
    -- If writing (active-low), Write = 0 then:
    --    DDBS = 00 
    --           we hold 
    --    DDBS = 01 
    --           we load in the RegA bus from Reg Array
    --    DDBS = 10 
    --           we load in the high byte of PC
    --    DDBS = 11 
    --           we load in the low byte of PC
    --
    -- If not writing (active-low), Write = 1 then:
    --    DataDB is driven to 'Z' so that it be written to by memory during read ops.  
    -- 
    DataRd <= DataRd_p(3) or clock;
    DataWr <= Write_p(3) or clock; 
     DataWr_i <= Write_p(3) or clock;
    process(clock,DataDB,DDBS_p,PC,RegA_p,DataWr_i)
    begin 
    if DataWr_i = '0' then         -- if writing data output through DataDB
        case(DDBS_p(3)) is 
        when "00" => 
            DataDB <= DataDB;
        when "01" =>
            DataDB <= RegA_p(0);
        when "10" =>
            DataDB <= PC(15 downto 8);
        when others =>
            DataDB <= PC(7 downto 0);
        end case;
    else
        DataDB <= (others => 'Z'); -- if reading data drive DataDB to Z here so can write data in externally
    end if;
    end process;


    -- Data Address Bus Mux
    -- controls what is being output to the data address bus for an instruction 
    --
    -- when DOFS = 0 (no offset) we load in the inc/dec source from the DAU
    -- when DOFS = 1 (with offset) we load in the offset adjusted inc/dec source from the DAU
    DataAB <= AddrSrcOutDAU when DOFS_p(3) = '0' else
              AddressDAU;

    ----------------------------------------------------------------------------------------
    -- ALU Inputs
    -- ALU OpA Mux
    -- controls what is being loaded in for OpA at the ALU
    -- when ALUOpASel = 00 we load in the RegA line from the RegArray
    -- when ALUOpASel = 01 we load a bit mask from DAU
    -- when ALUOpASel = 10 we load in 0
    -- when ALUOpASel = 11 we load in 0
    --
    ALUOpA <= RegA_p(0) when ALUOpASel_p(3) = "00" else 
              AddressDAU(7 downto 0) when ALUOpASel_p(3) = "01" else
              (others => '0');

    -- ALU OpB Mux
    -- controls what is being loaded in for OpB at the ALU
    -- when ALUOpBSel = 00 we load in the RegB line from the REG ARAAY
    -- when ALUOpBSel = 01 we load in a 8-bit immediate value from the IR 
    -- when ALUOpBSel = 10 we load in a 6-bit immediate value from the IR
    -- when ALUOpBSel = 11 we load in 0 
    --
    ALUOpB <= RegB_p(0) when ALUOpBSel_p(3) = "00" else
              (IR_p(3)(11 downto 8) & IR_p(3)(3 downto 0)) when ALUOpBSel_p(3) = "01" else
              ("00" & IR_p(3)(7 downto 6) & IR_p(3)(3 downto 0)) when ALUOpBSel_p(3) = "10" else
              (others => '0');

    ----------------------------------------------------------------------------------------
    -- SREG Inputs
    -- SOp = 11 : after an ALU operation, we send the 6 flags from the ALU along with the old T and I flags back 
    --            into the Status Register along with a 1's as the bit mask to write in. 
    -- SOp = 01 : used by BSET and BCLR to set and clear certain bits inside the status register. A bit mask 
    --            is created by the DAU and sent into SREG. 
    -- SOp = 10 : used by BLD and BST to set the transfer bit (T) of SREG. 
    -- SOp = 00 : normal operation, not writing anything to SREG, values hold. 
    -- 

    RegInS <= RegOut(7 downto 6) & (HalfCout xor SubOp_p(3)) & Signed & Overflow & Negative & (Zero and ZOp_p(0)) & (Cout xor SubOp_p(3)) when SOp_p(3) = "11" else
              AddressDAU(7 downto 0) when SOp_p(3) = "01" else
              '0'&RegA_p(0)(TflOp_p(3))&"000000" when SOp_p(3) = "10" else
              (others => '0');

    RegMask <= (others => '1') when SOp_p(3) = "11" else
               AddrSrcOutDAU(7 downto 0) when SOp_p(3) = "01" else
               AddrSrcOutDAU(7 downto 0) when SOp_p(3) = "10" else
               (others => '0');

    ----------------------------------------------------------------------------------------
    -- REG ARRAY Inputs
    -- controls what is being loaded into a register for an instruction
    -- when RegLoadSel = 00 we load in the ALU result
    -- when RegLoadSel = 01 we load in an immediate value from the IR 
    -- when RegLoadSel = 10 we load in DataDB
    -- when RegLoadSel = 11 we load in the RegA bus 
    --
    -- RegDIn is always set to AddrSrcOutDAU : inc/dec data address source

    RegIn <= Result_p(0) when RegLoadSel_p(3) = "00" else
             (IR_p(3)(11 downto 8) & IR_p(3)(3 downto 0)) when RegLoadSel_p(3) = "01" else
             DataDB when RegLoadSel_p(3) = "10" else
             RegA_p(0); 

    RegDIn <= AddrSrcOutDAU;

    ----------------------------------------------------------------------------------------
    -- PAU Inputs
    -- controls what is input as address into the PAU
    AddrSrcPAU <= PC; -- source is always PC for PAU

    -- controls what is input as offset into the PAU
    -- PAU offset is either 0s or signed extended 7 bits/12 bits from IR 
    AddrOffPAU <= zerobyte &                                                                   -- zeros
                  IR_p(2)(9)&IR_p(2)(9)&IR_p(2)(9)&IR_p(2)(9)&IR_p(2)(9)&IR_p(2)(9)&IR_p(2)(9)&IR_p(2)(9)&IR_P(2)(9) & IR_p(2)(9 downto 3) &     -- sign extended 7 bits IR
                  IR_p(2)(11)&IR_p(2)(11)&IR_p(2)(11)&IR_p(2)(11)&IR_p(2)(11 downto 0);                                                          -- sign extended 12 bits IR

    ----------------------------------------------------------------------------------------
    -- DAU Inputs
    -- controls what is input as address into the DAU
    -- DAU sources are the (X,Y,Z) pointers from RegD Bus, SP, ProgDB and 0s
    AddrSrcDAU <= zerobyte & ProgDB_p(2) & SP & RegD_p(0);

    -- controls what is input as offset into the DAU
    -- DAU offset is either 6 bits from IR or 0s
    AddrOffDAU <= zerobyte & zero10 & IR_p(3)(13)&IR_p(3)(11)&IR_p(3)(10)&IR_p(3)(2 downto 0); 
end Behavioral;