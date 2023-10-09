---------------------------------------------------------------------------------------------------------------------------------
--
--  Control Unit
--
--  This is the Control Unit (CU) for the Pipelined ATMEL AVR CPU. The CU contains the instruction register (IR) for the CPU. Each clock 
--  cycle the CU updates the IR with the Program Data Bus (unless skipping or performing multi-clock instruction). The CU used the 
--  current IR to generate various control signals sent to other parts of the CPU. There are inline comments for each control
--  signal assignment explaining what it is doing for the operation below. 
-- 
--  After pipelining the CPU in the top level, certain instructions have data hazards. In order to compensate for this, bubbles have 
--  been inserted for conditional branch instructions as well as all ALU instructions. These bubbles prevent data hazards from happening
--  in the pipelined CPU. 
--
--
--  Revision History:
--     02 Feb 23  Hector Wilson       Initial revision
--     06 Feb 23  Hector Wilson       Completed Load/Store instructions
--     08 Feb 23  Hector Wilson       Completed unconditional branch ops
--     15 Feb 23  Hector Wilson       Added decoding for skip ops
--     16 Feb 23  Hector Wilson       Added decoding for conditional branch ops
--     25 Feb 23  Hector Wilson       Began working on ALU instructions
--     18 Mar 23  Hector Wilson       Completed decoding for ALU ops
--     19 Mar 23  Hector Wilson       Finalized unpipelined CPU design, added comments
--     20 Mar 23  Hector Wilson       Added MUL function
--     23 Mar 23  Hector Wilson       Began pipelining --> Added pipeline bubbles
--     26 Mar 23  Hector Wilson       Added pipelined bubbles
--     27 Mar 23  Hector Wilson       Edited calculation of ALU flags during bubbled/hazardous instructions
--     31 Mar 23  Hector Wilson       Completed pipelined CPU, passes test vectors
--     01 Apr 23  Hector Wilson       Updated comments
--
----------------------------------------------------------------------------------------------------------------------------------
library opcodes;
use opcodes.opcodes.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity  ControlUnit  is

    generic (
        regcnt   : integer := 32;    -- default number of registers is 32
        addrsize : integer := 16;    -- address width is 16-bits
        regsize  : integer := 8;     -- register width is 8-bits

        dausrccnt: integer := 4;     -- DAU has 3 sources RegD, SP, and ProgDB
        dauoffcnt: integer := 2;	 -- DAU has 1 offset (0, 6-bit)
        pausrccnt: integer := 1;	 -- PAU has 1 source PC
        pauoffcnt: integer := 3      -- PAU has 3 offsets (0, 12-bit, 7-bit)
    );

    port(
    	-- generic outputs
        DataBus   : in   std_logic_vector(addrsize-1 downto 0);      -- Incoming Program Data Bus 
        Flags     : in   std_logic_vector(7 downto 0);		   	 	 -- ALU Flags
        SREG      : in   std_logic_vector(7 downto 0);               -- SREG Flags
        clock     : in   std_logic;							 		 -- Clock
        Reset     : in   std_logic;									 -- Reset signal
        Wr        : out  std_logic;									 -- Write signal
        Rd        : out  std_logic;									 -- Read signal
        NoWrite   : out  std_logic;									 -- Cancel Write for single cycle 
        NoWrite2  : out  std_logic;                                  -- Cancel Write for double cycle instruction
        IR        : buffer  std_logic_vector(addrsize-1 downto 0);		 -- Current Instruction Register

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
        DOFS      : out   std_logic;				     -- top level data-address offset-select mux control
        												 -- selects what to feed into the DataAB.
        POFS      : out   std_logic;				     -- top level program-address offset-select mux control
        												 -- selects what to feed into the PrgAB. 
        DDBS      : out   std_logic_vector(1 downto 0);  -- top level data-data-bus mux control selects what is being
        												 -- output to the data data bus. 
        HoldPC    : out   std_logic;                     -- indicates if holding value of PC for multiple cycles
        ALUOpASel : out   std_logic_vector(1 downto 0);
        ALUOpBSel : out   std_logic_vector(1 downto 0);
        SPU       : out   std_logic;                     -- indicates if loading to stack pointer
        PCU       : out   std_logic;					 -- indicates if loading to program counter
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

end  ControlUnit;



architecture Behavioral of ControlUnit is
	-- F BLOCK OPERATIONS
	constant zeros    : std_logic_vector(3 downto 0) := "0000";
	constant AnorB    : std_logic_vector(3 downto 0) := "0001";
	constant notAandB : std_logic_vector(3 downto 0) := "0010";
	constant notA     : std_logic_vector(3 downto 0) := "0011";
	constant AandnotB : std_logic_vector(3 downto 0) := "0100";
	constant notB     : std_logic_vector(3 downto 0) := "0101";
	constant AxorB    : std_logic_vector(3 downto 0) := "0110";
	constant AnandB   : std_logic_vector(3 downto 0) := "0111";
	constant AandB    : std_logic_vector(3 downto 0) := "1000";
	constant AxnorB   : std_logic_vector(3 downto 0) := "1001";
	constant B        : std_logic_vector(3 downto 0) := "1010";
	constant notAorB  : std_logic_vector(3 downto 0) := "1011";
	constant A        : std_logic_vector(3 downto 0) := "1100";
	constant AornotB  : std_logic_vector(3 downto 0) := "1101";
	constant AorB     : std_logic_vector(3 downto 0) := "1110";
	constant ones     : std_logic_vector(3 downto 0) := "1111";

	-- SHIFT BLOCK OPERATIONS
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


    -- The CU state machine is used to implement multi-clock instructions. 
    -- For this pipelined CPU implementation the most number of cycles is 5 (used by bubbled ADIW/SBIW)
	type states is (
					cycle1, cycle2, cycle3, cycle4, cycle5
					);
	signal CycleState, NewCycleState : states; -- current/next state

	-- internal Control Unit signals indicating if IR is being held for a multi-clock or skip instruction. 
	signal HoldIR2 : std_logic; -- held for 2 cycles
	signal HoldIR : std_logic;  -- held for 1 cycle. 
	signal IR2,IR3,IR4 : std_logic_vector(15 downto 0); -- delayed IRs
	
	signal NoWrite_i : std_logic;
	signal NoWrite2_i : std_logic;


begin
	------------------------------------------------------------------------------------------------------------
	-- This clocked process implements the instruction register for the CU and CPU.
	-- Additionally, it handles updating the state machine which determines cycle count for an instruction.
	-- If HoldIR = 0 (IR not being held)
	--    IR updates with Program Data Bus fed into CU
	-- If HoldIR = 1 (IR being held)
	--    IR held
	-- 
	-- If system reset = 1 
	--    CycleState is reset to cycle1. 
	-- If system reset = 0 
	--    CycleState is set to the NextCycleState
	--
	NoWrite <= NoWrite_i;
	NoWrite2 <= NoWrite2_i;
	process(clock) 
	begin
		if rising_edge(clock) then
			IR2 <= IR; 
			IR3 <= IR2;
			IR4 <= IR3;

			if HoldIR = '0' then 
				IR <= DataBus; -- load program data bus into instruction register on rising clock edge
			else
				if NoWrite_i = '1' and NoWrite2_i = '0' then 
					IR <= DataBus;
				else
					IR <= IR;
				end if;
			end if;

			if Reset = '1' then 
				CycleState <= cycle1; 
			else 
				CycleState <= NewCycleState;
			end if;
		end if;

	end process;

	----------------------------------------------------------------------------------------------------------
	----------------------------------------------------------------------------------------------------------
	-- This process implements instruction (IR)-based decoding to generate control signals for the rest of the CPU. 
	--
	process(IR,Flags,SREG,DataBus,CycleState,clock) 
	begin
		if std_match(IR4,OpSBRC) then
			if Flags(1) = '1' then                                      -- if zero flag set, bit was clear so skip 
				-- set next up control signals to a NOP
				NoWrite_i <= '1';

				if std_match(IR3, OpLDS) or std_match(IR3, OpSTS) or std_match(IR3, OpCALL) or std_match(IR3, OpJMP) then 
					NoWrite2_i <= '1'; 
				else
					NoWrite2_i <= '0';
				end if;
			else 														-- if zero flag not set, bit was not clear so continue
				NoWrite_i <= '0';  
				NoWrite2_i <= '0';
			end if;  
		elsif std_match(IR4,OpSBRS) then 
			if Flags(1) = '0' then                                      -- if zero flag clear, bit was set so skip 
				-- set next up control signals to a NOP
				NoWrite_i <= '1';

				if std_match(IR3, OpLDS) or std_match(IR3, OpSTS) or std_match(IR3, OpCALL) or std_match(IR3, OpJMP) then 
					NoWrite2_i <= '1'; 
				else
					NoWrite2_i <= '0';
				end if;
			else 														-- if zero flag not set, bit was not clear so continue
				NoWrite_i <= '0';  
				NoWrite2_i <= '0';
			end if;  
		elsif std_match(IR4,OpCPSE) then 
			if Flags(1) = '1' then                                      -- if zero flag set, registers were equal so skip 
				-- set next up control signals to a NOP
				NoWrite_i <= '1';

				if std_match(IR3, OpLDS) or std_match(IR3, OpSTS) or std_match(IR3, OpCALL) or std_match(IR3, OpJMP) then 
					NoWrite2_i <= '1'; 
				else
					NoWrite2_i <= '0';
				end if;
			else 														-- if zero flag not set, bit was not clear so continue
				NoWrite_i <= '0';  
				NoWrite2_i <= '0';
			end if;  
		else
			NoWrite_i <= '0';
			NoWrite2_i <= '0';
		end if;

		if Reset = '1' then
			SUBR <= '0'; 
			HoldPC <= '1';
			PCU <= '0'; 
			POFS <= '0';
			DOFS <= '1';
			IncDecBitDAU <= 0;
			PrePostSelDAU <= '1';
			IncDecSelDAU <= '0';
			OffsetSelDAU <= 1; 
			SrcSelDAU <= 2;

			IncDecBitPAU <= 0; 
			PrePostSelPAU <= '1';
			IncDecSelPAU <= '0';
			OffsetSelPAU <= 2;
			SrcSelPAU <= 0;
			Rd <= '1'; 
			Wr <= '1'; 
			DDBS <= "00";

			-- REG ARRAY Controls ---
			RegInSel <= 0;        -- default is select Reg 0
			RegStore <= '0';      -- default is not writing to any register
			RegASel <= 0;         -- default is read Reg 0 onto bus A 
			RegBSel <= 0;         -- default is read Reg 0 onto bus B
			RegDInSel <= 0;       -- default is select Reg 0&1
			RegDStore <= '0';     -- default is not writing to any double register
			RegDSel <= 0;         -- default is read Reg0&1 onto bus D

			RegLoadSel <= "01";

		else 
			-- We initialize the CU control signals to perform a NOP instruction
			-- ALU Controls ---
			FCmd <= zeros;        -- default is pass 0 through F Block
			SCmd <= "000";        -- default is do LSL in the shift block
			ALUCmd <= "01";       -- default is ALU outputs adder result
			CinCmd <= "00";       -- default is 0 carry in 
			Mulcycle <= '0';      -- default is first cycle of multiplication

			-- REG ARRAY Controls ---
			RegInSel <= 0;        -- default is select Reg 0
			RegStore <= '0';      -- default is not writing to any register
			RegASel <= 0;         -- default is read Reg 0 onto bus A 
			RegBSel <= 0;         -- default is read Reg 0 onto bus B
			RegDInSel <= 0;       -- default is select Reg 0&1
			RegDStore <= '0';     -- default is not writing to any double register
			RegDSel <= 0;         -- default is read Reg0&1 onto bus D

			-- PAU Controls --- 
			SrcSelPAU <= 0;       -- default is select PC as PAU source
			OffsetSelPAU <= 2;    -- default is select 0 offset
			IncDecSelPAU <= '0';  -- default is increment
			IncDecBitPAU <= 0;    -- default is increment bit 0
			PrePostSelPAU <= '1'; -- default is post-increment 

			-- DAU Controls --- 
			SrcSelDAU <= 2; 	  -- default is select ProgDB as DAU source
			OffsetSelDAU <= 1;    -- default is select 0 offset
			IncDecSelDAU <= '0';  -- default is increment
			IncDecBitDAU <= 0;    -- default is increment bit 0
			PrePostSelDAU <= '1'; -- default is post-increment 

			-- TOP LEVEL Controls ---
			POFS <= '0'; 		 -- load post incremented PC to ProgAB
			DOFS <= '1'; 		 -- load offset-adjusted data address to DataAB

			DDBS <= "00"; 		 -- default is hold Data Data Bus
			HoldIR <= '0'; 		 -- default is dont hold IR (default is not multiclock instruction)
			ALUOpASel <= "00";   -- default is load ALU OpA with RegA bus 
			ALUOpBSel <= "00";   -- default is load ALU OpB with RegB bus
			RegLoadSel <= "00";  -- default is load in ALU result to Reg Array input
			SPU <= '0';			 -- default is dont update Stack Pointer
			PCU <= '0';	         -- default is update PC with inc/dec PAU source
			HoldPC <= '0'; 		 -- default is dont hold PC (default is not multiclock instruction)
			SUBR <= '0';         -- default is dont write DataDB to PC
			SOp <= "00";         -- default is not writing anything to Status Register SREG
			ZOp <= '1';          -- default is dont clear Z flag
			NOp <= '1';          -- default is dont clear N flag
			HCHoldOp <= '0';     -- default is to not hold H flag
			SHoldOp <= '0';      -- default is to not hold S flag
			VHoldOp <= '0';      -- default is to not hold V flag
			NHoldOp <= '0';      -- default is to not hold N flag
			ZHoldOp <= '0';      -- default is to not hold Z flag
			CHoldOp <= '0';      -- default is to not hold C flag
			SubOp <= '0';        -- default is no subtraction operation
			NegOp <= '0';        -- default is no NEG operation

			Wr <= '1';           -- default is not writing
			Rd <= '1';           -- default is not reading
			---------------------------------------------------------------------------------------------------------------------------
			-- LOAD/STORE INSTRUCTIONS
			---------------------------------------------------------------------------------------------------------------------------
			if std_match(IR, OpLDI) then 
				case (CycleState) is
				when cycle1 =>
					RegLoadSel <= "01"; 									  -- pass direct IR bits as input data to Reg Array
					RegStore <= '1'; 										  -- storing data into a register
					RegInSel <= to_integer(unsigned(("1" & IR(7 downto 4)))); -- select which register we're storing data to 
				when others =>
				end case;

		    elsif std_match(IR, OpMOV) then 
		    	case (CycleState) is 
		    	when cycle1 => 
		    		RegASel <= to_integer(unsigned(IR(9) & IR(3 downto 0)));  -- load selected register data onto RegA bus
		    		RegLoadSel <= "11";                                       -- load RegA bus into Reg Array input
		    		RegStore <= '1';                                          -- storing data into a register
		    		RegInSel <= to_integer(unsigned(IR(8 downto 4)));         -- select which register we're storing data to 
		    	when others =>
		    	end case;

			elsif std_match(IR, OpSTS) then  
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        -- STS is 3 clock cycle instruction
					HoldPC <= '1';
					NewCycleState <= cycle2; -- go to second cycle of STS

				when cycle2 =>
					HoldIR <= '1';
					HoldPC <= '0';
					NewCycleState <= cycle3; -- go to third cycle of STS

					RegASel <= to_integer(unsigned(IR(8 downto 4)));  		  -- load selected register data onto RegA bus
					DDBS <= "01";						 					  -- write register contents from RegA to DataDB
					Wr <= '0';			                                  -- writing data

					SrcSelDAU <= 2; 										  -- select ProgDB as source for DAU 
					PrePostSelDAU <= '1';                                     -- select post inc/dec for DAU
					OffsetSelDAU <= 1;										  -- select 0 offset for DAU
					DOFS <= '1'; 											  -- output non inc/dec DAU source to DataAB (direct load)

				when cycle3 =>
					HoldIR <= '0';
					NewCycleState <= cycle1; -- end of instruction


				when others =>
				end case;

			elsif std_match(IR, OpLDS) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- LDS is 3 clock cycle instruction
					HoldPC <= '1';

					NewCycleState <= cycle2; -- go to second cycle of LDS

				when cycle2 =>
					HoldIR <= '1';
					HoldPC <= '0';
					NewCycleState <= cycle3; -- go to third cycle of LDS

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					Rd <= '0';			             							   -- reading data 

					SrcSelDAU <= 2; 										   -- select ProgDB as source for DAU 
					PrePostSelDAU <= '1';                                      -- select post inc/dec for DAU
					OffsetSelDAU <= 1;										   -- select 0 offset for DAU
					DOFS <= '1'; 											   -- output non inc/dec DAU source to DataAB (direct load)                                       

				when cycle3 =>
					HoldIR <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction

				when others =>
				end case;

			------------------------------------------
			-- X POINTER INSTRUCTIONS
			elsif std_match(IR, OpSTX) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- STX is 2 clock cycle instruction
					HoldPC <= '1';

					RegDSel <= 13;                                             -- load X pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- select post inc/dec for DAU
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '1';                                               -- output non inc/dec DAU source to DataAB (direct load)

					NewCycleState <= cycle2;								   -- go to second cycle of STS

				when cycle2 =>
					RegASel <= to_integer(unsigned(IR(8 downto 4)));           -- load selected register onto RegA bus
					DDBS <= "01";                                              -- write register contents from RegA to DataDB
					Wr <= '0';			                                            -- writing data

					RegDSel <= 13;                                             -- load X pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- select post inc/dec for DAU
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '1';                                               -- output non inc/dec DAU source to DataAB (direct load)

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpLDX) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- LDX is 2 clock cycle instruction
					HoldPC <= '1';

					RegDSel <= 13;                                             -- load X pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- select post inc/dec for DAU
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '1';                                               -- output non inc/dec DAU source to DataAB (direct load)

					NewCycleState <= cycle2; -- go to second cycle of STS

				when cycle2 =>
					Rd <= '0';			                                            -- reading data

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					RegDSel <= 13;                                             -- load X pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- select post inc/dec for DAU
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '1';                                               -- output non inc/dec DAU source to DataAB (direct load)

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpSTXI) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- STXI is 2 clock cycle instruction
					HoldPC <= '1';
					NewCycleState <= cycle2; 								   -- go to second cycle of STXI

					RegDSel <= 13;                                             -- load X pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output current X pointer to DataAB

				when cycle2 =>
					RegASel <= to_integer(unsigned(IR(8 downto 4)));           -- load selected register onto RegA bus
					DDBS <= "01";                                              -- write register contents from RegA to DataDB
					Wr <= clock;			                                   -- writing data

					RegDSel <= 13;                                             -- load X pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output non inc/dec source to DataAB (post inc)

					RegDStore <= '1'; 										   -- also updating with incremented X pointer
					RegDInSel <= 13;  

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpLDXI) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- LDXI is 2 clock cycle instruction
					HoldPC <= '1';

					RegDSel <= 13;                                             -- load X pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output current X pointer to DataAB

					NewCycleState <= cycle2; -- go to second cycle of STS

				when cycle2 =>
					Rd <= clock;			                                   -- reading data 

					RegDSel <= 13;                                             -- load X pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output non inc/dec DAU source to DataAB (post inc)

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					RegDStore <= '1'; 										   -- also updating with incremented X pointer
					RegDInSel <= 13;  

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpSTXD) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- STXD is 2 clock cycle instruction
					HoldPC <= '1';
					NewCycleState <= cycle2; 								   -- go to second cycle of STXD

					RegDSel <= 13;                                             -- load X pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					IncDecSelDAU <= '1';                                       -- switch to decrement
					DOFS <= '0';                                               -- output pre decremented X pointer to DataAB (pre dec)

					RegDStore <= '1'; 										   -- also updating with decremented X pointer
					RegDInSel <= 13; 

				when cycle2 =>
					RegASel <= to_integer(unsigned(IR(8 downto 4)));           -- load selected register onto RegA bus
					DDBS <= "01";                                              -- write register contents from RegA to DataDB
					Wr <= clock;			                                   -- writing data

					RegDSel <= 13;                                             -- load X pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output pre deremented X pointer to DataAB (pre dec)

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;


			elsif std_match(IR, OpLDXD) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- LDXD is 2 clock cycle instruction
					HoldPC <= '1';

					RegDSel <= 13;                                             -- load X pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					IncDecSelDAU <= '1';                                       -- switch to decrement
					DOFS <= '0';                                               -- output pre decremented X pointer to DataAB (pre dec)

					RegDStore <= '1'; 										   -- also updating with decremented X pointer
					RegDInSel <= 13;  

					NewCycleState <= cycle2; -- go to second cycle of STS

				when cycle2 =>
					Rd <= clock;			                                   -- reading data 

					RegDSel <= 13;                                             -- load X pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					IncDecSelDAU <= '1';                                       -- switch to decrement
					DOFS <= '1';                                               -- output pre decremented X pointer to DataAB (pre dec)

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			------------------------------------------
			-- Y POINTER INSTRUCTIONS
			elsif std_match(IR, OpSTYI) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- STYI is 2 clock cycle instruction
					HoldPC <= '1';
					NewCycleState <= cycle2; 								   -- go to second cycle of STYI

					RegDSel <= 14;                                             -- load Y pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output current Y pointer to DataAB

				when cycle2 =>
					RegASel <= to_integer(unsigned(IR(8 downto 4)));           -- load selected register onto RegA bus
					DDBS <= "01";                                              -- write register contents from RegA to DataDB
					Wr <= clock;			                                   -- writing data

					RegDSel <= 14;                                             -- load Y pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output non inc/dec source to DataAB (post inc)

					RegDStore <= '1'; 										   -- also updating with incremented Y pointer
					RegDInSel <= 14;  

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpLDYI) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- LDYI is 2 clock cycle instruction
					HoldPC <= '1';

					RegDSel <= 14;                                             -- load Y pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output current Y pointer to DataAB

					NewCycleState <= cycle2; -- go to second cycle of STS

				when cycle2 =>
					Rd <= clock;			                                   -- reading data 

					RegDSel <= 14;                                             -- load Y pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output non inc/dec DAU source to DataAB (post inc)

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					RegDStore <= '1'; 										   -- also updating with incremented Y pointer
					RegDInSel <= 14;  

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpSTYD) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- STYD is 2 clock cycle instruction
					HoldPC <= '1';
					NewCycleState <= cycle2; 								   -- go to second cycle of STXD

					RegDSel <= 14;                                             -- load Y pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					IncDecSelDAU <= '1';                                       -- switch to decrement
					DOFS <= '0';                                               -- output pre decremented Y pointer to DataAB (pre dec)

					RegDStore <= '1'; 										   -- also updating with decremented Y pointer
					RegDInSel <= 14; 

				when cycle2 =>
					RegASel <= to_integer(unsigned(IR(8 downto 4)));           -- load selected register onto RegA bus
					DDBS <= "01";                                              -- write register contents from RegA to DataDB
					Wr <= clock;			                                   -- writing data

					RegDSel <= 14;                                             -- load Y pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output pre deremented Y pointer to DataAB (pre dec)

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;


			elsif std_match(IR, OpLDYD) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- LDYD is 2 clock cycle instruction
					HoldPC <= '1';

					RegDSel <= 14;                                             -- load 4 pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					IncDecSelDAU <= '1';                                       -- switch to decrement
					DOFS <= '0';                                               -- output pre decremented Y pointer to DataAB (pre dec)

					RegDStore <= '1'; 										   -- also updating with decremented Y pointer
					RegDInSel <= 14;  

					NewCycleState <= cycle2; -- go to second cycle of STS

				when cycle2 =>
					Rd <= clock;			                                   -- reading data 

					RegDSel <= 14;                                             -- load Y pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					IncDecSelDAU <= '1';                                       -- switch to decrement
					DOFS <= '1';                                               -- output pre decremented Y pointer to DataAB (pre dec)

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpSTDY) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1'; 											   -- STDY is 2 clock cycle instruction
					HoldPC <= '1';	

					RegDSel <= 14;                                             -- load Y pointer contents onto RegD
					OffsetSelDAU <= 0;                                         -- choose 6-bit direct offset from IR
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- add offset to DAU source
					DOFS <= '1';											   -- output offset adjusted Y pointer to DataAB (Y+q)

					NewCycleState <= cycle2;

				when cycle2 => 
					RegASel <= to_integer(unsigned(IR(8 downto 4)));           -- load selected register onto RegA bus
					DDBS <= "01";                                              -- write register contents from RegA to DataDB
					Wr <= clock;			                                            -- writing data

					RegDSel <= 14;                                             -- load Y pointer contents onto RegD
					OffsetSelDAU <= 0;                                         -- choose 6-bit direct offset from IR
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- add offset to DAU source
					DOFS <= '1';											   -- output offset adjusted Y pointer to DataAB (Y+q)

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1;
				when others =>
				end case;

			elsif std_match(IR, OpLDDY) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- LDDY is 2 clock cycle instruction
					HoldPC <= '1';

					RegDSel <= 14;                                             -- load Y pointer contents onto RegD bus
					OffsetSelDAU <= 0;                                         -- choose 6-bit direct offset from IR
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- add offset to DAU source
					DOFS <= '1';                                               -- output offset adjusted Y pointer to DataAB (Y+q)

					NewCycleState <= cycle2; 								   -- go to second cycle of LDDY

				when cycle2 =>
					Rd <= clock;			                                                         -- reading data 

					RegDSel <= 14;                                             -- load Y pointer contents onto RegD bus
					OffsetSelDAU <= 0;                                         -- choose 6-bit direct offset from IR
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- add offset to DAU source
					DOFS <= '1';                                               -- output offset adjusted Y pointer to DataAB (Y+q)

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			------------------------------------------
			-- Z POINTER INSTRUCTIONS
			elsif std_match(IR, OpSTZI) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- STZI is 2 clock cycle instruction
					HoldPC <= '1';
					NewCycleState <= cycle2; 								   -- go to second cycle of STZI

					RegDSel <= 15;                                             -- load Z pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output current Z pointer to DataAB

				when cycle2 =>
					RegASel <= to_integer(unsigned(IR(8 downto 4)));           -- load selected register onto RegA bus
					DDBS <= "01";                                              -- write register contents from RegA to DataDB
					Wr <= clock;			                                   -- writing data

					RegDSel <= 15;                                             -- load Z pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output non inc/dec source to DataAB (post inc)

					RegDStore <= '1'; 										   -- also updating with incremented Z pointer
					RegDInSel <= 15;  

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpLDZI) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- LDZI is 2 clock cycle instruction
					HoldPC <= '1';

					RegDSel <= 15;                                             -- load Z pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output current Z pointer to DataAB

					NewCycleState <= cycle2; 								   -- go to second cycle of LDZI

				when cycle2 =>
					Rd <= clock;			                                   -- reading data 

					RegDSel <= 15;                                             -- load Z pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output non inc/dec DAU source to DataAB (post inc)

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					RegDStore <= '1'; 										   -- also updating with incremented Z pointer
					RegDInSel <= 15;  

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpSTZD) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- STZD is 2 clock cycle instruction
					HoldPC <= '1';
					NewCycleState <= cycle2; 								   -- go to second cycle of STZD

					RegDSel <= 15;                                             -- load Z pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					IncDecSelDAU <= '1';                                       -- switch to decrement
					DOFS <= '0';                                               -- output pre decremented Z pointer to DataAB (pre dec)

					RegDStore <= '1'; 										   -- also updating with decremented Z pointer
					RegDInSel <= 15; 

				when cycle2 =>
					RegASel <= to_integer(unsigned(IR(8 downto 4)));           -- load selected register onto RegA bus
					DDBS <= "01";                                              -- write register contents from RegA to DataDB
					Wr <= clock;			                                   -- writing data

					RegDSel <= 15;                                             -- load Z pointer onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					DOFS <= '1';                                               -- output pre deremented Z pointer to DataAB (pre dec)

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;


			elsif std_match(IR, OpLDZD) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- LDZD is 2 clock cycle instruction
					HoldPC <= '1';

					RegDSel <= 15;                                             -- load 4 pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					IncDecSelDAU <= '1';                                       -- switch to decrement
					DOFS <= '0';                                               -- output pre decremented Z pointer to DataAB (pre dec)

					RegDStore <= '1'; 										   -- also updating with decremented Z pointer
					RegDInSel <= 15;  

					NewCycleState <= cycle2; -- go to second cycle of STS

				when cycle2 =>
					Rd <= clock;			                                   -- reading data 

					RegDSel <= 15;                                             -- load Z pointer contents onto RegD bus
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					IncDecSelDAU <= '1';                                       -- switch to decrement
					DOFS <= '1';                                               -- output pre decremented Z pointer to DataAB (pre dec)

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpSTDZ) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1'; 											   -- STDZ is 2 clock cycle instruction
					HoldPC <= '1';	

					RegDSel <= 15;                                             -- load Z pointer contents onto RegD
					OffsetSelDAU <= 0;                                         -- choose 6-bit direct offset from IR
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- add offset to DAU source
					DOFS <= '1';											   -- output offset adjusted Z pointer to DataAB (Z+q)

					NewCycleState <= cycle2;

				when cycle2 => 
					RegASel <= to_integer(unsigned(IR(8 downto 4)));           -- load selected register onto RegA bus
					DDBS <= "01";                                              -- write register contents from RegA to DataDB
					Wr <= clock;			                                   -- writing data

					RegDSel <= 15;                                             -- load Z pointer contents onto RegD
					OffsetSelDAU <= 0;                                         -- choose 6-bit direct offset from IR
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- add offset to DAU source
					DOFS <= '1';											   -- output offset adjusted Z pointer to DataAB (Z+q)

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1;
				when others =>
				end case;

			elsif std_match(IR, OpLDDZ) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- LDDY is 2 clock cycle instruction
					HoldPC <= '1';

					RegDSel <= 15;                                             -- load Y pointer contents onto RegD bus
					OffsetSelDAU <= 0;                                         -- choose 6-bit direct offset from IR
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- add offset to DAU source
					DOFS <= '1';                                               -- output offset adjusted Y pointer to DataAB (Z+q)

					NewCycleState <= cycle2; 								   -- go to second cycle of LDDY

				when cycle2 =>
					Rd <= clock;			                                   -- reading data 

					RegDSel <= 15;                                             -- load Z pointer contents onto RegD bus
					OffsetSelDAU <= 0;                                         -- choose 6-bit direct offset from IR
					SrcSelDAU <= 0;                                            -- load RegD into DAU input
					PrePostSelDAU <= '1';                                      -- add offset to DAU source
					DOFS <= '1';                                               -- output offset adjusted Z pointer to DataAB (Z+q)

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			------------------------------------------
			-- PUSH/POP Instructions
			elsif std_match(IR, OpPUSH) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- PUSH is 2 clock cycle instruction
					HoldPC <= '1';

					SrcSelDAU <= 1;                                            -- load Stack Pointer into DAU input
					PrePostSelDAU <= '1';                                      -- select post inc/dec for DAU
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '1';                                               -- output non inc/dec DAU source to DataAB (post dec)

					NewCycleState <= cycle2;								   -- go to second cycle of STS

				when cycle2 =>
					RegASel <= to_integer(unsigned(IR(8 downto 4)));           -- load selected register onto RegA bus
					DDBS <= "01";                                              -- write register contents from RegA to DataDB
					Wr <= clock;			                                   -- writing data

					SrcSelDAU <= 1;                                            -- load Stack Pointer into DAU input
					PrePostSelDAU <= '1';                                      -- select post inc/dec for DAU
					IncDecSelDAU <= '1';                                       -- decrement the DAU source (SP)
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '1';                                               -- output non inc/dec DAU source to DataAB (post dec)

					SPU <= '1';                                                -- updating stack pointer with decremented value

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			elsif std_match(IR, OpPOP) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';        									   -- POP is 2 clock cycle instruction
					HoldPC <= '1';

					SrcSelDAU <= 1;                                            -- load Stack Pointer into DAU input
					PrePostSelDAU <= '0';                                      -- select pre inc/dec for DAU
					IncDecSelDAU <= '0';                                       -- increment the DAU source (SP)
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '0';                                               -- output inc/dec DAU source to DataAB (pre inc)

					SPU <= '1';                                                -- updating stack pointer with incremented value

					NewCycleState <= cycle2;								   -- go to second cycle of STS

				when cycle2 =>
					Rd <= clock;			                                   -- reading data

					RegLoadSel <= "10"; 								       -- pass DataDB as input data into Reg Array
					RegStore <= '1'; 										   -- storing data into a register
					RegInSel <= to_integer(unsigned(IR(8 downto 4))); 		   -- select which register we're storing data to 

					SrcSelDAU <= 1;                                            -- load Stack Pointer into DAU input
					PrePostSelDAU <= '1';                                      -- select post inc/dec for DAU
					IncDecSelDAU <= '1';                                       -- increment the DAU source (SP)
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '1';                                               -- output non inc/dec DAU source to DataAB (pre inc)

					HoldIR <= '0';
					HoldPC <= '0';
					NewCycleState <= cycle1; 								   -- end of instruction 
				when others =>
				end case;

			---------------------------------------------------------------------------------------------------------------------------
			-- SKIP INSTRUCTIONS
			---------------------------------------------------------------------------------------------------------------------------
			-- for SBRC we send the register and the bit in question to the ALU and do a bit comparison (AND). If the zero flag is not set from 
			-- the comparison, then we dont skip (PC+1). If the zero flag is set (bit clear) then we skip (PC+2/3).
			elsif std_match(IR, OpSBRC) then                                      
				case (CycleState) is 
				when cycle1 => 
					SrcSelDAU <= 3;                                             -- select 0 as DAU source 
					IncDecBitDAU <= to_integer(unsigned(IR(2 downto 0)));       -- use the DAU to create bit mask 
					PrePostSelDAU <= '0';                                       -- pre-sel

					RegBSel <= to_integer(unsigned(IR(8 downto 4)));            -- pass wanted register onto RegB Bus
					ALUOpASel <= "01";                                          -- pass Bit Mask from DAU into Operand A of ALU
					ALUOpBSel <= "00";                                          -- pass RegB bus into Operand B of ALU

					FCmd <= AandB;												-- AND OpA and OpB in the ALU to see if bit set or clear
					ALUCmd <= "00";                                             -- ALU is doing F Block Operation

					NewCycleState <= cycle2; 
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 => 
					SrcSelDAU <= 3;                                             -- select 0 as DAU source 
					IncDecBitDAU <= to_integer(unsigned(IR(2 downto 0)));       -- use the DAU to create bit mask 
					PrePostSelDAU <= '0';                                       -- pre-sel

					RegBSel <= to_integer(unsigned(IR(8 downto 4)));            -- pass wanted register onto RegB Bus
					ALUOpASel <= "01";                                          -- pass Bit Mask from DAU into Operand A of ALU
					ALUOpBSel <= "00";                                          -- pass RegB bus into Operand B of ALU

					FCmd <= AandB;												-- AND OpA and OpB in the ALU to see if bit set or clear
					ALUCmd <= "00";                                             -- ALU is doing F Block Operation

					NewCycleState <= cycle1; 
					HoldIR <= '0';
				when others =>
				end case;

			-- for SBRS we send the register and the bit in question to the ALU and do a bit comparison (AND). If the zero flag is not set from 
			-- the comparison, then we skip (PC+2/3). If the zero flag is set (bit clear) then we dont skip (PC+1).
			elsif std_match(IR, OpSBRS) then                                      
				case (CycleState) is 
				when cycle1 => 
					SrcSelDAU <= 3;                                             -- select 0 as DAU source 
					IncDecBitDAU <= to_integer(unsigned(IR(2 downto 0)));       -- use the DAU to create bit mask 
					PrePostSelDAU <= '0';                                       -- post-sel

					RegBSel <= to_integer(unsigned(IR(8 downto 4)));            -- pass wanted register onto RegB Bus
					ALUOpASel <= "01";                                          -- pass Bit Mask from DAU into Operand A of ALU
					ALUOpBSel <= "00";                                          -- pass RegB bus into Operand B of ALU

					FCmd <= AandB;												-- AND OpA and OpB in the ALU to see if bit set or clear
					ALUCmd <= "00";                                             -- ALU is doing F Block Operation

					NewCycleState <= cycle2; 
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					SrcSelDAU <= 3;                                             -- select 0 as DAU source 
					IncDecBitDAU <= to_integer(unsigned(IR(2 downto 0)));       -- use the DAU to create bit mask 
					PrePostSelDAU <= '0';                                       -- pre-sel

					RegBSel <= to_integer(unsigned(IR(8 downto 4)));            -- pass wanted register onto RegB Bus
					ALUOpASel <= "01";                                          -- pass Bit Mask from DAU into Operand A of ALU
					ALUOpBSel <= "00";                                          -- pass RegB bus into Operand B of ALU

					FCmd <= AandB;												-- AND OpA and OpB in the ALU to see if bit set or clear
					ALUCmd <= "00";                                             -- ALU is doing F Block Operation

					NewCycleState <= cycle1; 
					HoldIR <= '0';
				when others =>
				end case;

			-- for CPSE we send both registers in question into OpA and OpB of the ALU and do a subtraction to compare. If the zero flag is set from 
			-- the comparison (Rr=Rd), then we skip (PC+2/3). If the zero flag is not set (Rr != Rd) then we dont skip (PC+1).
			elsif std_match(IR, OpCPSE) then                                      
				case (CycleState) is 
				when cycle1 => 
					RegBSel <= to_integer(unsigned(IR(8 downto 4)));            -- pass first register onto RegB Bus
					RegASel <= to_integer(unsigned(IR(9) & IR(3 downto 0)));     -- pass second register onto RegA Bus

					ALUOpASel <= "00";                                          -- pass RegA bus into Operand A of ALU
					ALUOpBSel <= "00";                                          -- pass RegB bus into Operand B of ALU

					FCmd <= notB;                                               -- do NOT B in order to subtract it
					CinCmd <= "01";                                             -- carry in = 1 since doing subtraction
					ALUCmd <= "01";                                             -- ALU is doing Adder (SUB) Operation

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					RegBSel <= to_integer(unsigned(IR(8 downto 4)));            -- pass first register onto RegB Bus
					RegASel <= to_integer(unsigned(IR(9) & IR(3 downto 0)));     -- pass second register onto RegA Bus

					ALUOpASel <= "00";                                          -- pass RegA bus into Operand A of ALU
					ALUOpBSel <= "00";                                          -- pass RegB bus into Operand B of ALU

					FCmd <= notB;                                               -- do NOT B in order to subtract it
					CinCmd <= "01";                                             -- carry in = 1 since doing subtraction
					ALUCmd <= "01";                                             -- ALU is doing Adder (SUB) Operation

					NewCycleState <= cycle1;
					HoldIR <= '0';

				when others =>
				end case;

			---------------------------------------------------------------------------------------------------------------------------
			-- UNCONDITIONAL BRANCH INSTRUCTIONS
			---------------------------------------------------------------------------------------------------------------------------
			elsif std_match(IR, OpJMP) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1'; 											    -- JMP is a 3 clock cycle instruction
					NewCycleState <= cycle2;							        -- nothing happens here

				when cycle2 => 
					HoldIR <= '1'; 
					PCU <= '1';                                                 -- load ProgDB into the Program Counter
					NewCycleState <= cycle3; 

				when cycle3 =>
					NewCycleState <= cycle1; 								    -- end of instruction
					HoldIR <= '0';

				when others =>
				end case;

			elsif std_match(IR, OpRJMP) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1'; 											    -- RJMP is a 2 clock cycle instruction

					SrcSelPAU <= 0;                                             -- get PC into PAU
					PrePostSelPAU <= '1';                                       -- do a pre inc/dec
					IncDecSelPAU <= '1';                                        -- pre decrement PAU source (PC)
					OffsetSelPAU <= 0;                                          -- load sign extended 12-bit offset
					POFS <= '1';                                                -- update PC with offset adjusted value from PAU

					NewCycleState <= cycle2;							        -- nothing happens here

				when cycle2 => 
					NewCycleState <= cycle1;
					HoldIR <= '0';                                              -- end of instruction 

				when others =>
				end case;

			elsif std_match(IR, OpIJMP) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';												-- IJMP is a 2 clock cycle instruction
					RegDSel <= 15;                                              -- put Z pointer onto the RegD Bus
					HoldPC <= '1'; 
					PCU <= '1';                                                 -- store RegD Bus into PC

					NewCycleState <= cycle2; 

				when cycle2 => 
					NewCycleState <= cycle1; 
					HoldIR <= '0';											    -- end of instruction

				when others => 
				end case;

			elsif std_match(IR, OpCALL) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';												-- CALL is a 4 clock cycle instruction    
					HoldPC <= '1';                                              -- Hold PC here so writing correct values to DataDB next cycle                                      
					NewCycleState <= cycle2; 

				when cycle2 => 
					HoldIR <= '1'; 
					Wr <= clock;                                                -- writing data

					SrcSelDAU <= 1;                                             -- select SP as DAU source
					PrePostSelDAU <= '1';                                       -- do a post inc/dec
					IncDecSelDAU <= '1';                                        -- post decrement SP as DAU source
					SPU <= '1';                                                 -- store post decremented SP back into SP 
					DDBS <= "10";                                               -- load in high byte of PC to DataDB

					NewCycleState <= cycle3;

				when cycle3 =>
					HoldIR <= '1';  
					Wr <= clock;                                                -- writing data

					SrcSelDAU <= 1;                                             -- select SP as DAU source
					PrePostSelDAU <= '1';                                       -- do a post inc/dec
					IncDecSelDAU <= '1';                                        -- post decrement SP as DAU source
					SPU <= '1';                                                 -- store post decremented SP back into SP 
					DDBS <= "11";                                               -- load in low byte of PC to DataDB

					PCU <= '1';                                                 -- pass ProgDB into PC (direct load)
					NewCycleState <= cycle4;

				when cycle4 => 
					NewCycleState <= cycle1; 
					HoldIR <= '0';											    -- end of instruction 

				when others => 
				end case;

			elsif std_match(IR, OpRET) then 
				case (CycleState) is 
				when cycle1 =>
					HoldIR <= '1'; 
					NewCycleState <= cycle2;

				when cycle2 =>
					HoldIR <= '1'; 
					Rd <= clock; 											   -- reading data

					SrcSelDAU <= 1;                                            -- load Stack Pointer into DAU input
					PrePostSelDAU <= '0';                                      -- select pre inc/dec for DAU
					IncDecSelDAU <= '0';                                       -- increment the DAU source (SP)
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '0';                                               -- output inc/dec DAU source to DataAB (pre inc)

					SUBR <= '1';                                               -- returning from subroutine (put DataDB into PC)
					PCU <= '0'; 

					SPU <= '1';                                                -- updating stack pointer with incremented value
					NewCycleState <= cycle3;

				when cycle3 =>
					HoldIR <= '1'; 
					Rd <= clock; 											   -- reading data

					SrcSelDAU <= 1;                                            -- load Stack Pointer into DAU input
					PrePostSelDAU <= '0';                                      -- select pre inc/dec for DAU
					IncDecSelDAU <= '0';                                       -- increment the DAU source (SP)
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '0';                                               -- output inc/dec DAU source to DataAB (pre inc)

					SUBR <= '1';                                               -- returning from subroutine (put DataDB into PC)
					PCU <= '1'; 

					SPU <= '1';                                                -- updating stack pointer with incremented value
					NewCycleState <= cycle4;

				when cycle4 =>
					NewCycleState <= cycle1;
					HoldIR <= '0'; 									           -- end of instruction

				when others =>
				end case;

			elsif std_match(IR, OpRCALL) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';												-- RCALL is a 3 clock cycle instruction   
					Wr <= clock;                                                -- writing data  
					HoldPC <= '1';                                              -- hold PC while writing to DataDB

					NewCycleState <= cycle2; 

					SrcSelDAU <= 1;                                             -- select SP as DAU source
					PrePostSelDAU <= '1';                                       -- do a post inc/dec
					IncDecSelDAU <= '1';                                        -- post decrement SP as DAU source
					SPU <= '1';                                                 -- store post decremented SP back into SP 
					DDBS <= "10";                                               -- load in high byte of PC to DataDB

				when cycle2 => 
					HoldIR <= '1'; 
					Wr <= clock;                                                -- writing data

					SrcSelDAU <= 1;                                             -- select SP as DAU source
					PrePostSelDAU <= '1';                                       -- do a post inc/dec
					IncDecSelDAU <= '1';                                        -- post decrement SP as DAU source
					SPU <= '1';                                                 -- store post decremented SP back into SP 
					DDBS <= "11";                                               -- load in low byte of PC to DataDB

					SrcSelPAU <= 0;                                             -- get PC into PAU
					PrePostSelPAU <= '1';                                       -- do a pre inc/dec
					IncDecSelPAU <= '1';                                        -- pre decrement PAU source (PC)
					OffsetSelPAU <= 0;                                          -- load sign extended 12-bit offset
					POFS <= '1';                                                -- update PC with offset adjusted value from PAU

					NewCycleState <= cycle3;

				when cycle3 => 

					NewCycleState <= cycle1;
					HoldIR <= '0';											    -- end of instruction

				when others => 
				end case;

			elsif std_match(IR, OpICALL) then 
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';												-- RCALL is a 3 clock cycle instruction 
					Wr <= clock;                                                -- writing data
					HoldPC <= '1';                                              -- hold PC while writing to DataDB

					NewCycleState <= cycle2; 

					SrcSelDAU <= 1;                                             -- select SP as DAU source
					PrePostSelDAU <= '1';                                       -- do a post inc/dec
					IncDecSelDAU <= '1';                                        -- post decrement SP as DAU source
					SPU <= '1';                                                 -- store post decremented SP back into SP 
					DDBS <= "10";                                               -- load in high byte of PC to DataDB

				when cycle2 => 
					HoldIR <= '1'; 
					Wr <= clock;                                                -- writing data

					SrcSelDAU <= 1;                                             -- select SP as DAU source
					PrePostSelDAU <= '1';                                       -- do a post inc/dec
					IncDecSelDAU <= '1';                                        -- post decrement SP as DAU source
					SPU <= '1';                                                 -- store post decremented SP back into SP 
					DDBS <= "11";                                               -- load in low byte of PC to DataDB

					RegDSel <= 15;                                              -- put Z pointer onto the RegD Bus
					HoldPC <= '1'; 
					PCU <= '1';                                                 -- store RegD Bus into PC

					NewCycleState <= cycle3;

				when cycle3 => 

					NewCycleState <= cycle1;
					HoldIR <= '0';											    -- end of instruction

				when others => 
				end case;

			elsif std_match(IR, OpRETI) then 
				case (CycleState) is 
				when cycle1 =>
					SOp <= "01"; 											   --
					SrcSelDAU <= 3;                                            -- select 0 as DAU source
					IncDecBitDAU <= 7;									       -- use the DAU to create bit mask for 7th flag (I)
					PrePostSelDAU <= '0';                                      -- select pre since writing to SREG

					HoldIR <= '1'; 
					NewCycleState <= cycle2;

				when cycle2 =>
					HoldIR <= '1'; 
					Rd <= clock; 											   -- reading data

					SrcSelDAU <= 1;                                            -- load Stack Pointer into DAU input
					PrePostSelDAU <= '0';                                      -- select pre inc/dec for DAU
					IncDecSelDAU <= '0';                                       -- increment the DAU source (SP)
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '0';                                               -- output inc/dec DAU source to DataAB (pre inc)

					SUBR <= '1';                                               -- returning from subroutine (put DataDB into PC)
					PCU <= '0'; 

					SPU <= '1';                                                -- updating stack pointer with incremented value
					NewCycleState <= cycle3;

				when cycle3 =>
					HoldIR <= '1'; 
					Rd <= clock; 											   -- reading data

					SrcSelDAU <= 1;                                            -- load Stack Pointer into DAU input
					PrePostSelDAU <= '0';                                      -- select pre inc/dec for DAU
					IncDecSelDAU <= '0';                                       -- increment the DAU source (SP)
					OffsetSelDAU <= 1;                                         -- select 0 offset for DAU
					DOFS <= '0';                                               -- output inc/dec DAU source to DataAB (pre inc)

					SUBR <= '1';                                               -- returning from subroutine (put DataDB into PC)
					PCU <= '1'; 

					SPU <= '1';                                                -- updating stack pointer with incremented value
					NewCycleState <= cycle4;

				when cycle4 =>
					NewCycleState <= cycle1;
					HoldIR <= '0'; 									           -- end of instruction

				when others =>
				end case;

			---------------------------------------------------------------------------------------------------------------------------
			-- CONDITIONAL BRANCH INSTRUCTIONS
			---------------------------------------------------------------------------------------------------------------------------
			elsif std_match(IR, OpBRBC) then 
				case (CycleState) is 
				when cycle1 => 
					if SREG(to_integer(unsigned(IR(2 downto 0))))='0' then 
						HoldIR <= '1'; 											    -- RJMP is a 2 clock cycle instruction
						SrcSelPAU <= 0;                                             -- get PC into PAU
						PrePostSelPAU <= '1';                                       -- do a pre inc/dec
						IncDecSelPAU <= '1';                                        -- pre decrement PAU source (PC)
						OffsetSelPAU <= 1;                                          -- load sign extended 12-bit offset
						POFS <= '1';                                                -- update PC with offset adjusted value from PAU

						NewCycleState <= cycle2; 
						HoldIR <= '1'; 
					end if; 

				when cycle2 => 
					NewCycleState <= cycle1; 
					HoldIR <= '0'; 
				when others => 
				end case;

			elsif std_match(IR, OpBRBS) then 
				case (CycleState) is 
				when cycle1 => 
					if SREG(to_integer(unsigned(IR(2 downto 0))))='1' then 
						HoldIR <= '1'; 											    -- RJMP is a 2 clock cycle instruction
						SrcSelPAU <= 0;                                             -- get PC into PAU
						PrePostSelPAU <= '1';                                       -- do a pre inc/dec
						IncDecSelPAU <= '1';                                        -- pre decrement PAU source (PC)
						OffsetSelPAU <= 1;                                          -- load sign extended 12-bit offset
						POFS <= '1';                                                -- update PC with offset adjusted value from PAU

						NewCycleState <= cycle2; 
						HoldIR <= '1'; 
					end if; 

				when cycle2 => 
					NewCycleState <= cycle1; 
					HoldIR <= '0';
				when others => 
				end case;

			---------------------------------------------------------------------------------------------------------------------------
			-- ALU INSTRUCTIONS
			---------------------------------------------------------------------------------------------------------------------------
			elsif std_match(IR, OpADC) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= B; -- pass B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "10"; -- pass in carry flag to adder

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpADD) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= B; -- pass B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "00"; -- no carry in

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpADIW) then
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';    -- adiw is 2 cycle instruction
					HoldPC <= '1'; 
					NewCycleState <= cycle2; 

					-- ALU Controls --- 
					FCmd <= B; -- pass B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "00"; -- no carry in

					-- REG ARRAY Controls --- 
					RegASel <= to_integer("11" & unsigned(IR(5 downto 4)) & '0'); -- pass Rd (24,26,28,30)

					RegInSel <= to_integer("11" & unsigned(IR(5 downto 4)) & '0'); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "10"; -- send 6-bit immediate value from IR into ALU

					SOp <= "11";   -- updating SREG with ALU flags

				when cycle2 => 
					HoldIR <= '0';
					NewCycleState <= cycle1;

					-- ALU Controls --- 
					FCmd <= B; -- pass B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "10"; -- pass in carry flag to adder

					-- REG ARRAY Controls --- 
					RegASel <= to_integer("11" & unsigned(IR(5 downto 4)) & '1'); -- pass Rd+1 (25,26,28,30)

					RegInSel <= to_integer("11" & unsigned(IR(5 downto 4)) & '1'); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "11"; -- 0 into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag

					NewCycleState <= cycle3;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle5;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle5 =>
					ZOp <= SREG(1);     -- load first byte Zero flag into ZOp (want Zero flag for both bytes to be 0 to set)
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpAND) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= AandB; -- AND op
					ALUCmd <= "00"; -- ALU outputs F block result

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag
					CHoldOp <= '1'; -- hold value of C flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpANDI) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= AandB; -- AND op
					ALUCmd <= "00"; -- ALU outputs F block result

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned('1' & IR(7 downto 4))); -- pass Rd (16+)

					RegInSel <= to_integer(unsigned('1' & IR(7 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "01"; -- send 8-bit immediate value from IR into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag
					CHoldOp <= '1'; -- hold value of C flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpASR) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					ALUCmd <= "10"; -- ALU outputs shift block result
					SCmd <= SCmd_ASR; -- ALU doing ASR op

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "01"; -- send 8-bit immediate value from IR into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpBSET) then 
				case (CycleState) is 
				when cycle1 =>
					SOp <= "01"; 
					SrcSelDAU <= 3;                                             -- select 0 as DAU source
					IncDecBitDAU <= to_integer(unsigned(IR(6 downto 4)));       -- use the DAU to create bit mask
					PrePostSelDAU <= '0';                                       -- pre-set 

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpBCLR) then 
				case(CycleState) is 
				when cycle1 => 
					SOp <= "01"; 
					SrcSelDAU <= 3;                                             -- select 0 as DAU source
					IncDecBitDAU <= to_integer(unsigned(IR(6 downto 4)));       -- use the DAU to create bit mask

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;
				when others =>
				end case;

			elsif std_match(IR, OpBST) then 
				case (CycleState) is 
				when cycle1 =>
					SOp <= "10"; 
					SrcSelDAU <= 3;                                             -- select 0 as DAU source
					IncDecBitDAU <= 6;       									-- use the DAU to create bit mask
					PrePostSelDAU <= '0';                                       -- pre-set 

					TflOp <= to_integer(unsigned(IR(2 downto 0)));
					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd onto RegA Bus

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpBLD) then 
				case (CycleState) is 
				when cycle1 => 
					-- REG ARRAY Controls --- 
					RegBSel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- updating register with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					SrcSelDAU <= 3;                                             -- select 0 as DAU source
					IncDecBitDAU <= to_integer(unsigned(IR(2 downto 0)));       -- use the DAU to create bit mask
					PrePostSelDAU <= '0';                                       -- post-sel

					ALUOpASel <= "01"; -- pass mask from DAU
					ALUOpBSel <= "00"; -- pass RegB Bus
					ALUCmd <= "00"; -- F block operation

					-- if T flag is set then we want to 
					if SREG(6) = '1' then 
						FCmd <= AorB;               -- setting bit
					else  
						FCmd <= notAandB;           -- clearing bit
					end if;

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpCOM) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= notA; -- AND op
					ALUCmd <= "00"; -- ALU outputs F block result

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpCP) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= notB; -- pass B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "01"; -- carry in 1
					SubOp <= '1'; -- doing a subtraction

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpCPC) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= notB; -- pass B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "11"; -- carry in not C flag
					SubOp <= '1'; -- doing a subtraction

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					if Flags(1) = '1' then 
						ZHoldOp <= '1'; -- if zero then hold value of Z flag otherwise clear
					else 
						ZHoldOp <= '0'; 
					end if; 

					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpCPI) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= notB; -- pass B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "01"; -- carry in 1
					SubOp <= '1'; -- doing a subtraction

					-- REG ARRAY Controls --- 
					RegASel <= to_integer('1' & unsigned(IR(7 downto 4))); -- pass Rd

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "01"; -- send immediate value from IR into ALU

					SOp <= "11";   -- updating SREG with ALU flags

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpINC) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= zeros; -- pass B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "01"; -- carry in 1 for increment

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag
					CHoldOp <= '1'; -- hold value of C flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpDEC) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= ones; -- pass ones through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "00"; -- carry in 0 for decrement

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag
					CHoldOp <= '1'; -- hold value of C flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpEOR) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= AxorB; -- XOR op
					ALUCmd <= "00"; -- ALU outputs F block result

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag
					CHoldOp <= '1'; -- hold value of C flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpLSR) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					ALUCmd <= "10"; -- ALU outputs shift block result
					SCmd <= SCmd_LSR; -- ALU doing ASR op

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "01"; -- send 8-bit immediate value from IR into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					NOp <= '0';    -- negative flag is zero'd 
					HCHoldOp <= '1'; -- hold value of H flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpNEG) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= notB; -- NOT B op
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "01"; -- carry in 1 for negation

					-- REG ARRAY Controls --- 
					RegBSel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegB selection with result from ALU
					RegStore <= '1'; -- storing back into RegB

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "10"; -- send 0 into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					NegOp <= '1'; -- specify NEG flags from ALU

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpOR) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= AorB; -- or op
					ALUCmd <= "00"; -- ALU outputs F block result

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag
					CHoldOp <= '1'; -- hold value of C flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpORI) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= AorB; -- or op
					ALUCmd <= "00"; -- ALU outputs F block result

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned('1' & IR(7 downto 4))); -- pass Rd (16+)

					RegInSel <= to_integer(unsigned('1' & IR(7 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "01"; -- send 8-bit immediate value from IR into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag
					CHoldOp <= '1'; -- hold value of C flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpROR) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					ALUCmd <= "10"; -- ALU outputs shift block result
					SCmd <= SCmd_ROR; -- ALU doing ASR op
					CinCmd <= "10"; -- carry in C flag

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "01"; -- send 8-bit immediate value from IR into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpSBC) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= notB; -- pass not B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "11"; -- pass in not C flag to adder

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					SubOp <= '1';  -- doing a subtraction

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpSBCI) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= notB; -- pass not B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "11"; -- pass in not C flag to adder

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned('1' & IR(7 downto 4))); -- pass Rd (16+)

					RegInSel <= to_integer(unsigned('1' & IR(7 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "01"; -- send 8-bit immediate value from IR into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					SubOp <= '1';  -- doing a subtraction

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpSUB) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= notB; -- pass not B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "01"; -- carry in 1

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					SubOp <= '1';  -- doing a subtraction

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpSUBI) then
				case (CycleState) is 
				when cycle1 => 
					-- ALU Controls --- 
					FCmd <= notB; -- pass not B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "01"; -- carry in 1

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned('1' & IR(7 downto 4))); -- pass Rd (16+)

					RegInSel <= to_integer(unsigned('1' & IR(7 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "01"; -- send 8-bit immediate value from IR into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					SubOp <= '1';  -- doing a subtraction

					NewCycleState <= cycle2;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle2 =>
					NewCycleState <= cycle3;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpSBIW) then
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1';    -- sbiw is 2 cycle instruction
					HoldPC <= '1'; 
					NewCycleState <= cycle2; 

					-- ALU Controls --- 
					FCmd <= notB; -- pass not B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "01"; -- carry in 1 on first cycle

					-- REG ARRAY Controls --- 
					RegASel <= to_integer("11" & unsigned(IR(5 downto 4)) & '0'); -- pass Rd (24,26,28,30)

					RegInSel <= to_integer("11" & unsigned(IR(5 downto 4)) & '0'); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "10"; -- send 6-bit immediate value from IR into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					SubOp <= '1';  -- doing a subtraction
					HCHoldOp <= '1'; -- hold value of H flag

				when cycle2 => 
					HoldIR <= '0';
					NewCycleState <= cycle1;

					-- ALU Controls --- 
					FCmd <= notB; -- pass not B through F Block
					ALUCmd <= "01"; -- ALU outputs adder result
					CinCmd <= "11"; -- pass in not C flag to adder

					-- REG ARRAY Controls --- 
					RegASel <= to_integer("11" & unsigned(IR(5 downto 4)) & '1'); -- pass Rd+1 (25,26,28,30)

					RegInSel <= to_integer("11" & unsigned(IR(5 downto 4)) & '1'); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "11"; -- 0 into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					SubOp <= '1';  -- doing a subtraction
					HCHoldOp <= '1'; -- hold value of H flag

					NewCycleState <= cycle3;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle5;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle5 =>
					ZOp <= SREG(1);     -- load first byte Zero flag into ZOp (want Zero flag for both bytes to be 1 to set)
					NewCycleState <= cycle1;

				when others =>
				end case;

			elsif std_match(IR, OpSWAP) then 
				case (CycleState) is 
				when cycle1 =>
					-- ALU Controls --- 
					ALUCmd <= "10"; -- ALU outputs shift block result
					SCmd <= SCmd_SWAP; -- ALU doing ASR op

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd

					RegInSel <= to_integer(unsigned(IR(8 downto 4))); -- update RegA selection with result from ALU
					RegStore <= '1'; -- storing back into RegA

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU

				when others =>
				end case;

			elsif std_match(IR, OpMUL) then
				case (CycleState) is 
				when cycle1 => 
					HoldIR <= '1'; 
					HoldPC <= '1'; 
					NewCycleState <= cycle2;  -- MUL is 2 cycle instruction

					-- ALU Controls --- 
					ALUCmd <= "11"; -- ALU outputs multiplier result
					Mulcycle <= '0'; -- first cycle of multiplication

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd 
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					RegInSel <= 0; -- storing low byte of result into register 0
					RegStore <= '1'; -- storing back into single register

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";   -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag
					SHoldOp <= '1';  -- hold value of S flag
					VHoldOp <= '1';  -- hold value of V flag
					NHoldOp <= '1';  -- hold value of N flag

				when cycle2 =>
					NewCycleState <= cycle1; -- end of MUL instruction

					-- ALU Controls --- 
					ALUCmd <= "11"; -- ALU outputs multiplier result
					Mulcycle <= '1'; -- second cycle of multiplication

					-- REG ARRAY Controls --- 
					RegASel <= to_integer(unsigned(IR(8 downto 4))); -- pass Rd 
					RegBSel <= to_integer(unsigned(IR(9) & IR(3 downto 0))); -- pass Rr

					RegInSel <= 1; -- storing high byte of result into register 1
					RegStore <= '1'; -- storing back into single register

					RegLoadSel <= "00"; -- loading register with result from ALU

					-- TOP LEVEL Controls --- 
					ALUOpASel <= "00"; -- send RegA from RegArray into ALU
					ALUOpBSel <= "00"; -- send RegB from RegArray into ALU

					SOp <= "11";     -- updating SREG with ALU flags
					HCHoldOp <= '1'; -- hold value of H flag
					SHoldOp <= '1';  -- hold value of S flag
					VHoldOp <= '1';  -- hold value of V flag
					NHoldOp <= '1';  -- hold value of N flag

					NewCycleState <= cycle3;
					HoldIR <= '1'; 
					HoldPC <= '1'; 
				when cycle3 =>
					NewCycleState <= cycle4;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle4 =>
					NewCycleState <= cycle5;
					HoldIR <= '1';
					HoldPC <= '1'; 
				when cycle5 =>
					ZOp <= SREG(1);  -- load first byte zero flag into ZOp (want Zero flag for both bytes to be 1 to set)
					NewCycleState <= cycle1;

				when others =>
				end case;

			end if;

			if NoWrite_i = '1' and NoWrite2_i = '0' then 
				NewCycleState <= cycle1;
			end if;
		end if;

	end process;


end Behavioral;

