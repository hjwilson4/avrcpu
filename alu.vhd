----------------------------------------------------------------------------
--
--  Generic ALU and Status Register
--
--  This is an implementation of the ALU for the AVR CPU. The ALU defined below
--  contains a generic shift, adder, and F block unit which are used to perform 
--  standard logic and arithmetic operations. The top level ALU connects these
--  generic sub components and generates the status flags for the ALU and CPU
--  based on the result of the operation. 
--
--  Packages included are:
--     ALUConstants - constants for all entities making up the ALU
--
--  Entities included are:
--     FBlockBit - one bit of an F-Block
--     AdderBit  - one bit of an adder (a full adder)
--     FBlock    - F-Block (logical operations)
--     Adder     - adder
--     Shifter   - shift and rotate and swap operations
--     ALU       - the actual ALU
--     StatusReg - the status register
--
--  Revision History:
--     25 Jan 21  Glen George       Initial revision.
--     27 Jan 21  Glen George       Changed left/right shift selection to a
--                                  constant.
--     27 Jan 21  Glen George       Changed F-Block to be on B input of adder.
--     27 Jan 21  Glen George       Updated comments.
--     29 Jan 21  Glen George       Fixed a number of wordsize bugs.
--     29 Jan 21  Glen George       Fixed overflow signal in adder.
--     02 Feb 23  Hector Wilson     Modified calculation of overflow.
--     12 Mar 23  Hector Wilson     Added synchronous SREG reset.
--     17 Mar 23  Hector Wilson     Modified ALU flags. 
--     19 Mar 23  Hector Wilson     Edited calculation of ALU flags.
--                                  Updated comments. 
--     20 Mar 23  Hector Wilson     Added multiply functionality. 
--
----------------------------------------------------------------------------


--
--  Package containing the constants used by the ALU and all of its
--  sub-modules.
--

library ieee;
use ieee.std_logic_1164.all;

package  ALUConstants  is

--  Adder carry in select constants
--     may be freely changed

   constant CinCmd_ZERO   : std_logic_vector(1 downto 0) := "00";
   constant CinCmd_ONE    : std_logic_vector(1 downto 0) := "01";
   constant CinCmd_CIN    : std_logic_vector(1 downto 0) := "10";
   constant CinCmd_CINBAR : std_logic_vector(1 downto 0) := "11";


--  Shifter command constants
--     may be freely changed except a single bit pattern (currently high bit)
--     must distinguish between left shifts and rotates and right shifts and
--     rotates

   constant SCmd_LEFT  : std_logic_vector(2 downto 0) := "0--";
   constant SCmd_LSL   : std_logic_vector(2 downto 0) := "000";
   constant SCmd_SWAP  : std_logic_vector(2 downto 0) := "001";
   constant SCmd_ROL   : std_logic_vector(2 downto 0) := "010";
   constant SCmd_RLC   : std_logic_vector(2 downto 0) := "011";
   constant SCmd_RIGHT : std_logic_vector(2 downto 0) := "1--";
   constant SCmd_LSR   : std_logic_vector(2 downto 0) := "100";
   constant SCmd_ASR   : std_logic_vector(2 downto 0) := "101";
   constant SCmd_ROR   : std_logic_vector(2 downto 0) := "110";
   constant SCmd_RRC   : std_logic_vector(2 downto 0) := "111";


--  ALU command constants
--     may be freely changed

   constant ALUCmd_FBLOCK  : std_logic_vector(1 downto 0) := "00";
   constant ALUCmd_ADDER   : std_logic_vector(1 downto 0) := "01";
   constant ALUCmd_SHIFT   : std_logic_vector(1 downto 0) := "10";


end package;



--
--  FBlockBit
--
--  This is a bit of the F-Block for doing logical operations in the ALU.  The
--  operations available are:
--     FCmd    Operation
--     0000    0
--     0001    A nor B
--     0010    not A and B
--     0011    not A
--     0100    A and not B
--     0101    not B
--     0110    A xor B
--     0111    A nand B
--     1000    A and B
--     1001    A xnor B
--     1010    B
--     1011    not A or B
--     1100    A
--     1101    A or not B
--     1110    A or B
--     1111    1
--
--  Inputs:
--    A    - first operand bit (bus A)
--    B    - second operand bit (bus B)
--    FCmd - operation to perform (4 bits)
--
--  Outputs:
--    F    - F-Block output (based on input busses and command)
--

library ieee;
use ieee.std_logic_1164.all;

entity  FBlockBit  is

    port(
        A    : in   std_logic;                      -- first operand
        B    : in   std_logic;                      -- second operand
        FCmd : in   std_logic_vector(3 downto 0);   -- operation to perform
        F    : out  std_logic                       -- result
    );

end  FBlockBit;


architecture  dataflow  of  FBlockBit  is
begin

    F  <=  FCmd(3)  when  ((A = '1') and (B = '1'))  else
           FCmd(2)  when  ((A = '1') and (B = '0'))  else
           FCmd(1)  when  ((A = '0') and (B = '1'))  else
           FCmd(0)  when  ((A = '0') and (B = '0'))  else
           'X';

end  dataflow;



--
--  AdderBit
--
--  This is a bit of the adder for doing addition in the ALU.
--
--  Inputs:
--    A  - first operand bit (bus A)
--    B  - second operand bit (bus B)
--    Ci - carry in (from previous bit)
--
--  Outputs:
--    S  - sum for this bit
--    Co - carry out for this bit
--

library ieee;
use ieee.std_logic_1164.all;

entity  AdderBit  is

    port(
        A  : in   std_logic;        -- first operand
        B  : in   std_logic;        -- second operand
        Ci : in   std_logic;        -- carry in from previous bit
        S  : out  std_logic;        -- sum (result)
        Co : out  std_logic         -- carry out to next bit
    );

end  AdderBit;


architecture  dataflow  of  AdderBit is
begin

    S  <=  A  xor  B  xor  Ci;
    Co <=  (A  and  B)  or  (A  and Ci)  or  (B  and  Ci);

end  dataflow;



--
--  FBlock
--
--  This is the F-Block for doing logical operations in the ALU.  The F-Block
--  operations are:
--     FCmd    Operation
--     0000    0
--     0001    FBOpA nor FBOpB
--     0010    not FBOpA and FBOpB
--     0011    not FBOpA
--     0100    FBOpA and not FBOpB
--     0101    not FBOpB
--     0110    FBOpA xor FBOpB
--     0111    FBOpA nand FBOpB
--     1000    FBOpA and FBOpB
--     1001    FBOpA xnor FBOpB
--     1010    FBOpB
--     1011    not FBOpA or FBOpB
--     1100    FBOpA
--     1101    FBOpA or not FBOpB
--     1110    FBOpA or FBOpB
--     1111    1
--
--  Generics:
--    wordsize - width of the F-Block in bits (default 8)
--
--  Inputs:
--    FBOpA   - first operand
--    FBOpB   - second operand
--    FCmd    - operation to perform (4 bits)
--
--  Outputs:
--    FResult - F-Block result (based on input busses and command)
--

library ieee;
use ieee.std_logic_1164.all;

entity  FBlock  is

    generic (
        wordsize : integer := 8      -- default width is 8-bits
    );

    port(
        FBOpA   : in   std_logic_vector(wordsize - 1 downto 0); -- first operand
        FBOpB   : in   std_logic_vector(wordsize - 1 downto 0); -- second operand
        FCmd    : in   std_logic_vector(3 downto 0);            -- operation to perform
        FResult : out  std_logic_vector(wordsize - 1 downto 0)  -- result
    );

end  FBlock;



architecture  structural  of  FBlock  is

    component  FBlockBit
        port(
            A    : in   std_logic;                      -- first operand
            B    : in   std_logic;                      -- second operand
            FCmd : in   std_logic_vector(3 downto 0);   -- operation to perform
            F    : out  std_logic                       -- result
        );
    end component;

begin

    F1:  for  i  in  FResult'Range  generate        -- make enough FBlockBits
    begin
        FBx: FBlockBit  port map  (FBOpA(i), FBOpB(i), FCmd, FResult(i));
    end generate;

end  structural;



--
--  Adder
--
--  This is the adder for doing addition in the ALU.
--
--  Generics:
--    wordsize - width of the adder in bits (default 8)
--
--  Inputs:
--    AddOpA - first operand
--    AddOpB - second operand
--    Cin    - carry in (from status register)
--    CinCmd - operation for carry in (2 bits)
--
--  Outputs:
--    AddResult - sum
--    Cout      - carry out for the addition
--    HalfCOut  - half carry out for the addition
--    Overflow  - signed overflow
--

library ieee;
use ieee.std_logic_1164.all;
use work.ALUConstants.all;

entity  Adder  is

    generic (
        wordsize : integer := 8      -- default width is 8-bits
    );

    port(
        AddOpA    : in   std_logic_vector(wordsize - 1 downto 0);   -- first operand
        AddOpB    : in   std_logic_vector(wordsize - 1 downto 0);   -- second operand
        Cin       : in   std_logic;                                 -- carry in
        CinCmd    : in   std_logic_vector(1 downto 0);              -- carry in operation
        AddResult : out  std_logic_vector(wordsize - 1 downto 0);   -- sum (result)
        Cout      : out  std_logic;                                 -- carry out
        HalfCout  : out  std_logic;                                 -- half carry out
        Overflow  : out  std_logic                                  -- signed overflow
    );

end  Adder;


architecture  structural  of  Adder  is

    component  AdderBit
        port(
            A  : in   std_logic;        -- first operand
            B  : in   std_logic;        -- second operand
            Ci : in   std_logic;        -- carry in from previous bit
            S  : out  std_logic;        -- sum (result)
            Co : out  std_logic         -- carry out to next bit
        );
    end component;

    signal  carry : std_logic_vector(wordsize downto 0);        -- intermediate carry results

begin

    -- get the carry in based on CinCmd
    carry(0)  <=  '0'      when  CinCmd = CinCmd_ZERO  else
                  '1'      when  CinCmd = CinCmd_ONE  else
                  Cin      when  CinCmd = CinCmd_CIN  else
                  not Cin  when  CinCmd = CinCmd_CINBAR  else
                  'X';

    A1:  for  i  in  AddResult'Range  generate      -- make enough AdderBits
    begin
        ABx: AdderBit  port map  (AddOpA(i), AddOpB(i), carry(i),
                                  AddResult(i), carry(i + 1));
    end generate;

    Cout     <= carry(wordsize);        -- compute carry out
    HalfCout <= carry(4);               -- half carry (carry into high nibble)
    -- overflow if carry into sign bit doesn't match the carry out
    Overflow <= carry(wordsize - 1) xor carry(wordsize);

end  structural;



--
--  Shifter
--
--  This is the shifter for doing shift/rotate operations in the ALU.  The
--  shift operations are defined by the constants SCmd_LEFT, SCmd_RIGHT,
--  SCmd_SWAP, SCmd_LSL, SCmd_ROL, SCmd_RLC, SCmd_LSR, SCmd_ASR, SCmd_ROR,
--  and SCmd_RRC.
--
--  Generics:
--    wordsize - width of the shifter in bits (default 8)
--               must be an even number of bits
--
--  Inputs:
--    SOp     - operand
--    Cin     - carry in (from status register)
--    SCmd    - operation to perform (3 bits)
--
--  Outputs:
--    SResult - shift result
--    Cout    - carry out from the shift (link)
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.std_match;
use work.ALUConstants.all;

entity  Shifter  is

    generic (
        wordsize : integer := 8      -- default width is 8-bits
    );

    port(
        SOp     : in   std_logic_vector(wordsize - 1 downto 0); -- operand
        Cin     : in   std_logic;                               -- carry in
        SCmd    : in   std_logic_vector(2 downto 0);            -- shift operation
        SResult : out  std_logic_vector(wordsize - 1 downto 0); -- sum (result)
        Cout    : out  std_logic                                -- carry out
    );

end  Shifter;


architecture  dataflow  of  Shifter  is
begin

    -- middle bits get either bit to the left or bit to the right or upper
    --   half bit when swapping
    SResult(wordsize - 2 downto 1)  <= 

        -- do swap first because it is a special case of SCmd
        (SOp(wordsize/2 - 2 downto 0) & SOp(wordsize - 1 downto wordsize/2 + 1))  when  SCmd = SCmd_SWAP  else

        -- right shift
        SOp(wordsize - 1 downto 2)                                                when  std_match(SCmd, SCmd_RIGHT)  else

        -- left shift
        SOp(wordsize - 3 downto 0)                                                when  std_match(SCmd, SCmd_LEFT)  else

        -- unknown command
        (others => 'X');


    -- high bit gets low bit, high bit, bit to the right, 0, or Cin depending
    --   on shift mode, note that swap is a special case that has to be first
    --   due to encoding (overlaps left shifts)
    SResult(wordsize - 1)  <=
        SOp(wordsize/2 - 1)  when  SCmd = SCmd_SWAP  else  -- swap
        SOp(wordsize - 2)    when  std_match(SCmd, SCmd_LEFT)  else  -- shift/rotate left
        Cin                  when  SCmd = SCmd_ROR   else  -- rotate right
        SOp(wordsize - 1)    when  SCmd = SCmd_ASR   else  -- arithmetic shift right
        '0'                  when  SCmd = SCmd_LSR   else  -- logical shift right
        Cin                  when  SCmd = SCmd_RRC   else  -- rotate right w/carry
        'X';                                               -- anything else is illegal


    -- low bit gets high bit, bit to the left, 0, or Cin depending on mode
    SResult(0)  <=
        SOp(wordsize/2)   when  SCmd = SCmd_SWAP  else  -- swap
        SOp(1)            when  std_match(SCmd, SCmd_RIGHT)  else  -- shift/rotate right
        '0'               when  SCmd = SCmd_LSL   else  -- shift left
        SOp(wordsize - 1) when  SCmd = SCmd_ROL   else  -- rotate left
        Cin               when  SCmd = SCmd_RLC   else  -- rotate left w/carry
        'X';                                            -- anything else is illegal


    -- compute the carry out, it is low bit when shifting right and high bit
    --    when shifting left (don't care about swap)
    Cout  <=  SOp(0)             when  std_match(SCmd, SCmd_RIGHT)  else
              SOp(wordsize - 1)  when  std_match(SCmd, SCmd_LEFT)   else
              'X';

end  dataflow;

--
--  Multiplier
--
--  This is the multiplier for doing the MUL operation in the ALU.  The
--  MUL operation takes a total of 2 cycles to execute. 
--
--  Generics:
--    wordsize - width of the shifter in bits (default 8)
--               must be an even number of bits
--
--  Inputs:
--    MulOpA   - first operand
--    MulOpB   - second operand
--    Mulcycle - cycle (0/1) of multiplication process 
--
--  Outputs:
--    Result   - multiplication result
--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.ALUConstants.all;

entity  Multiplier  is

    generic (
        wordsize : integer := 8      -- default width is 8-bits
    );

    port(
        MulOpA    : in     std_logic_vector(wordsize - 1 downto 0); -- first operand
        MulOpB    : in     std_logic_vector(wordsize - 1 downto 0); -- second operand
        Mulcycle  : in     std_logic;                               -- cycle (0/1) of multiplication process
        MulResult : out    std_logic_vector(wordsize - 1 downto 0); -- multiply result
        MulCout   : out    std_logic                                -- multiply carry out
    );

end  Multiplier;


architecture  dataflow  of  Multiplier  is
    signal sum1 : std_logic_vector(13 downto 0); -- partial sum on cycle1 will be 14 bits
    signal halfcarry : std_logic_vector(5 downto 0); -- carry after first partial sum will be 6 bits
    signal sum2 : std_logic_vector(8 downto 0); -- partial sum on cycle2 will be 9 bits (9th is carry out)
begin
    --
    -- To compute the multiplication in 2 clock cycles, we split up the low and high nibbles of each operand. 
    -- A * B = (AH+AL)*(BH+BL)
    --       = AH * BH + AH * BL + AL * BH + AL * BL
    --
    -- On the first cycle, we compute and output the low byte of the result 
    --         The low byte result comes from: AH * BL + AL * BH + AL * BL
    -- On the second cycle, we compute and output the high byte of the result
    --         The high byte result comes from: AH * BH
    -- 

    -- leave extra bits at the end for carries. 
    sum1 <= std_logic_vector(("00" & unsigned(MulOpA(7 downto 4)) * unsigned(MulOpB(3 downto 0)) & "0000")
                            +("00" & unsigned(MulOpA(3 downto 0)) * unsigned(MulOpB(7 downto 4)) & "0000")
                            +("000000" & unsigned(MulOpA(3 downto 0)) * unsigned(MulOpB(3 downto 0))));
    halfcarry <= sum1(13 downto 8); 
    sum2 <= std_logic_vector(("0" & unsigned(MulOpA(7 downto 4)) * unsigned(MulOpB(7 downto 4)))
                            +(unsigned("000" & halfcarry))); 
    -- compute results
    MulCout <= sum2(7); -- Cout is last bit of high byte of result 
    MulResult <= sum1(7 downto 0) when Mulcycle = '0' else
                 sum2(7 downto 0); 

end  dataflow;

--
--  ALU
--
--  This is the actual ALU model for the CPU.  It includes the FBlock,
--  Adder, and Shifter modules.  It also outputs a number of status bits.
--
--  Generics:
--    wordsize - width of the ALU in bits (default 8)
--
--  Inputs:
--    ALUOpA   - first operand
--    ALUOpB   - second operand
--    Cin      - carry in (from status register)
--    FCmd     - F-Block operation to perform (4 bits)
--    CinCmd   - adder carry in operation for carry in (2 bits)
--    SCmd     - shift operation to perform (3 bits)
--    ALUCmd   - ALU operation to perform - selects result (2 bits)
--    NOp      - control signal indicating N flag should be 0 
--    NegOp    - control signal indicating NEG operation (has special flag calculations)
--    Mulcycle - cycle (0/1) of multiplication operation
--
--  Outputs:
--    Result   - ALU result
--    Cout     - carry out flag for ALU
--    HalfCOut - half carry out flag for ALU
--    Overflow - signed overflow flag for ALU
--    Zero     - zero flag for ALU 
--    Signed   - result sign flag for ALU (1 negative, 0 positive)
--    Negative - negaive flag for ALU

library ieee;
use ieee.std_logic_1164.all;
use work.ALUConstants.all;

entity  ALU  is

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
        Overflow : buffer  std_logic;                                 -- signed overflow
        Zero     : out     std_logic;                                 -- result is zero
        Signed   : out     std_logic;                                 -- adjusted sign of result               
        Negative : buffer  std_logic                                  -- sign of result
    );

end  ALU;


architecture  structural  of  ALU  is

    component  FBlock
        generic(
            wordsize : integer
        );
        port(
            FBOpA   : in   std_logic_vector(wordsize - 1 downto 0);
            FBOpB   : in   std_logic_vector(wordsize - 1 downto 0);
            FCmd    : in   std_logic_vector(3 downto 0);
            FResult : out  std_logic_vector(wordsize - 1 downto 0)
        );
    end component;

    component  Adder
        generic(
            wordsize : integer
        );
        port(
            AddOpA    : in   std_logic_vector(wordsize - 1 downto 0);
            AddOpB    : in   std_logic_vector(wordsize - 1 downto 0);
            Cin       : in   std_logic;
            CinCmd    : in   std_logic_vector(1 downto 0);
            AddResult : out  std_logic_vector(wordsize - 1 downto 0);
            Cout      : out  std_logic;
            HalfCout  : out  std_logic;
            Overflow  : out  std_logic
        );
    end component;

    component  Shifter
        generic(
            wordsize : integer
        );
        port(
            SOp     : in   std_logic_vector(wordsize - 1 downto 0);
            Cin     : in   std_logic;
            SCmd    : in   std_logic_vector(2 downto 0);
            SResult : out  std_logic_vector(wordsize - 1 downto 0);
            Cout    : out  std_logic
        );
    end component;

    component  Multiplier
        generic(
            wordsize : integer
        );
        port(
            MulOpA    : in  std_logic_vector(wordsize - 1 downto 0); 
            MulOpB    : in  std_logic_vector(wordsize - 1 downto 0); 
            Mulcycle  : in  std_logic; 
            MulResult : out std_logic_vector(wordsize - 1 downto 0); 
            MulCout   : out std_logic
        );
    end component;

    signal  FBRes   : std_logic_vector(wordsize - 1 downto 0);  -- F-Block result
    signal  AddRes  : std_logic_vector(wordsize - 1 downto 0);  -- adder result
    signal  ShRes   : std_logic_vector(wordsize - 1 downto 0);  -- shifter result
    signal  MulRes  : std_logic_vector(wordsize - 1 downto 0);  -- multiplier result

    signal  AddCout : std_logic;                                -- adder carry out
    signal  ShCout  : std_logic;                                -- shifter carry out
    signal  MulCout : std_logic;                                -- multiplier carry out
    signal  AdderOFlow : std_logic;                             -- overflow from adder
    signal  FCout   : std_logic;                                -- f block carry out
    signal  AddHalfCout : std_logic;                            -- adder half carry out

begin

    -- wire up the blocks
    FB1:   FBlock   generic map (wordsize)
                    port map  (ALUOpA, ALUOpB, FCmd, FBRes);
    Add1:  Adder    generic map (wordsize)
                    port map  (ALUOpA, FBRes, Cin, CinCmd,
                               AddRes, AddCout, AddHalfCout, AdderOFlow);
    Sh1:   Shifter  generic map (wordsize)
                    port map  (ALUOpA, Cin, SCmd, ShRes, ShCout);
    Mul1:  Multiplier generic map (wordsize)
                      port map(ALUOpA, ALUOpB, Mulcycle, MulRes, MulCout);

    -- figure out the result
    Result  <=  FBRes   when  ALUCmd = ALUCmd_FBLOCK  else  -- want F-Block
                AddRes  when  ALUCmd = ALUCmd_ADDER   else  -- want adder
                ShRes   when  ALUCmd = ALUCmd_SHIFT   else  -- want shifter
                MulRes;                                     -- want multiplier

    --------------------------------------------------------------------------------------
    -- here we calculate the half carry flag for the ALU
    -- the H flag is traditionally calculated in the Adder component except for the NEG 
    -- operation in which the H flag is NOT OpB. 
    HalfCout <= AddHalfCout when NegOp = '0' else
                not ALUOpB(3);
    --------------------------------------------------------------------------------------
    -- this process calculated the carry flag for the ALU
    -- the C flag is 0 when doing F block operations except for COM where it is 1. 
    -- the C flag is the carry out of bit 7 for adder operations. 
    -- the C flag is the low/high bit for right/left shift operations. 
    -- for the NEG operation, the C flag is bit-wise OR of OpB. 
    process(FCmd,ALUCmd,NegOp,ALUOpB,FCout,AddCout,ShCout,MulCout)
    begin
        -- figure out the carry out for F block
        if FCmd = "0011" then 
            FCout <= '1'; 
        else
            FCout <= '0';
        end if; 

        if NegOp = '0' then 
            case (ALUCmd) is
            when ALUCmd_FBLOCK => 
                Cout <= FCout;
            when ALUCmd_ADDER =>
                Cout <= AddCout;
            when ALUCmd_SHIFT =>
                Cout <= ShCout;
            when others =>
                Cout <= MulCout;
            end case;
        else
            Cout <= ALUOpB(0) or ALUOpB(1) or ALUOpB(2) or ALUOpB(3) or 
                    ALUOpB(4) or ALUOpB(5) or ALUOpB(6) or ALUOpB(7);
        end if;
    end process;

    --------------------------------------------------------------------------------------
    -- here we calculate the zero flag for the ALU
    -- the Z flag is 1 only when the Result of the ALU is 0, otherwise the Z flag is 0. 
    Zero  <=  '1'  when  Result = (Result'range => '0')  else
              '0';

    --------------------------------------------------------------------------------------
    -- this process calculates the overflow flag for the ALU
    -- the V flag is 0 for F block operations. 
    -- the V flag is R(7) xor OpA(0) for shift operations. 
    -- the V flag is carry out of bit 6 doesnt match carry out of bit 7 for adder operations. 
    -- for the NEG operation, the V flag is bit-wise AND of ALUOpB(6:0) and NOT ALUOpB(7). 
    process(ALUCmd,NegOp,ALUOpB,Result,NOp,AdderOFlow,ALUOpA)
    begin
        if NegOp = '0' then 
            case (ALUCmd) is 
            when ALUCmd_FBLOCK =>
                Overflow <= '0';
            when ALUCmd_SHIFT => 
                Overflow <= (Result(7) and NOp) xor ALUOpA(0);
            when others =>
                Overflow <= AdderOFlow;
            end case;
        else
            Overflow <= ALUOpB(0) and ALUOpB(1) and ALUOpB(2) and ALUOpB(3) and 
                        ALUOpB(4) and ALUOpB(5) and ALUOpB(6) and not ALUOpB(7);
        end if;
    end process;

    --------------------------------------------------------------------------------------
    -- compute the negative flag value
    -- the N flag is the high sign bit of the ALU result. 
    Negative  <=  Result(wordsize - 1) and NOp;

    --------------------------------------------------------------------------------------
    -- compute the signed flag value
    -- the S flag is the N flag xor V flag. 
    Signed <= Negative xor Overflow;

end  structural;



--
--  StatusReg
--
--  This is the ALU status register.  It is a generic status register of the
--  size given in bits.  It allows any subset of bits to be written on each
--  clock. 
--
--  Generics:
--    wordsize - width of the status register in bits (default 8)
--
--  Inputs:
--    RegIn    - data to write to the status register
--    RegMask  - mask for updating status register bits
--    HCHoldOp - indicates if H flag is held
--    ZHoldOp  - indicates if Z flag is held
--    CHoldOp  - indicates if C flag is held
--    clock    - system clock
--    reset    - system reset for SREG. 
--
--  Outputs:
--    RegOut   - current values in the status register
--

library ieee;
use ieee.std_logic_1164.all;

entity  StatusReg  is

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
        reset    : in      std_logic;
        RegOut   : buffer  std_logic_vector(wordsize - 1 downto 0)    -- current register value
    );

end  StatusReg;

architecture  behavioral  of  StatusReg  is
    signal RegInf : std_logic_Vector(wordsize - 1 downto 0); 
    signal C,Z,N,V,S,HC : std_logic; 
    signal RegOuti : std_logic_vector(wordsize - 1 downto 0);
begin
    -- intermediate SREG output node (used to read output)
    RegOuti <= RegOut; 

    -- Determine if certain flags are being held for this update. 
    HC <= RegIn(5) when HCHoldOp = '0' else
          RegOuti(5); 
    S <= RegIn(4) when SHoldOp = '0' else
         RegOuti(4);
    V <= RegIn(3) when VHoldOp = '0' else
         RegOuti(3);
    N <= RegIn(2) when NHoldOp = '0' else
         RegOuti(2);
    Z <= RegIn(1) when ZHoldOp = '0' else
         RegOuti(1); 
    C <= RegIn(0) when CHoldOp = '0' else
         RegOuti(0);

    -- Data that will be written to the SREG. 
    RegInf <= RegIn(7 downto 6) & HC & S & V & N & Z & C; 

    -- this process updates the SREG on the rising edge of clock and includes 
    -- a synchronous system reset. 
    process (clock)
    begin

        -- on the rising edge of the clock write the new values to the status
        --    register if the associated mask bit is 1, otherwise keep the
        --    current value for that bit
        if  rising_edge(clock)  then
            if reset = '0' then 
                RegOut <= (RegInf and RegMask) or (RegOuti and not RegMask);
            else
                RegOut <= (others => '0');
            end if;
        end if;

    end process;

end  behavioral;