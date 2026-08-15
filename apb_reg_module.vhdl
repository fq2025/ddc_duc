--------------------------------------------------------------------------------
-- APB Register Module
-- Description: APB-accessible register block containing all specified registers
--              organized into 5 groups with full read/write capability.
-- Address Space: 0x000 ~ 0x824 (total 2085 words), 12-bit byte address
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

--------------------------------------------------------------------------------
-- Package: Array types for bulk register ports
--------------------------------------------------------------------------------
package apb_reg_pkg is
    -- Array type for 5-bit registers (Group 4: s32x512, pingpong selection codes)
    type t_slv5_array   is array (natural range <>) of std_logic_vector(4 downto 0);
    -- Array type for 512-bit registers (Group 5: s512x32, per-channel selection codes)
    type t_slv512_array is array (natural range <>) of std_logic_vector(511 downto 0);
	type t_slv8_array is array (natural range <>) of std_logic_vector(7 downto 0);
end package apb_reg_pkg;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.apb_reg_pkg.all;

--------------------------------------------------------------------------------
-- Entity: apb_reg_module
--------------------------------------------------------------------------------
entity apb_reg_module is
    generic (
        CAL_CHL_NUM   : integer := 512;
        J204B_LANE_NUM : integer := 32;
        CNT_WIDTH     : integer := 32   -- Width for dbf_204b_rx_num / dbf_204b_tx_num
    );
    port (
        -----------------------------------------------------------------------
        -- APB Interface (AMBA 3 APB)
        -----------------------------------------------------------------------
        pclk    : in  std_logic;                      -- APB Clock
        presetn : in  std_logic;                      -- Active-low reset
        paddr   : in  std_logic_vector(13 downto 0);  -- 16KB address space
        psel    : in  std_logic;                      -- Peripheral select
        penable : in  std_logic;                      -- Enable strobe
        pwrite  : in  std_logic;                      -- Write (1) / Read (0)
        pwdata  : in  std_logic_vector(31 downto 0);  -- Write data
        prdata  : out std_logic_vector(31 downto 0);  -- Read data
        pready  : out std_logic;                      -- Transfer ready
        pslverr : out std_logic;                      -- Transfer error

        -----------------------------------------------------------------------
        -- Group 1: Basic Control Registers
        -- dbf_dir, wei_pq_flag, mps_pq_flag (1-bit each)
        -- dbf_204b_rx_num, dbf_204b_tx_num (32-bit each)
        -----------------------------------------------------------------------
        dbf_dir         : out std_logic;
        wei_pq_flag       : out std_logic;
        mps_pq_flag       : out std_logic;
        dbf_204b_rx_num : out std_logic_vector(CNT_WIDTH-1 downto 0);
        dbf_204b_tx_num : out std_logic_vector(CNT_WIDTH-1 downto 0);

        -----------------------------------------------------------------------
        -- Group 2: dbf_rx_hlf_sel_0 ~ dbf_rx_hlf_sel_511
        -- CAL_CHL_NUM single-bit registers. Access individual bit via index:
        -- e.g., dbf_rx_hlf_sel(0) corresponds to dbf_rx_hlf_sel_0
        -----------------------------------------------------------------------
        dbf_rx_hlf_sel  : out std_logic_vector(CAL_CHL_NUM-1 downto 0);

        -----------------------------------------------------------------------
        -- Group 3: dbf_tx_hlf_sel_0 ~ dbf_tx_hlf_sel_511
        -- CAL_CHL_NUM single-bit registers. Access individual bit via index:
        -- e.g., dbf_tx_hlf_sel(0) corresponds to dbf_tx_hlf_sel_0
        -----------------------------------------------------------------------
        dbf_tx_hlf_sel  : out std_logic_vector(CAL_CHL_NUM-1 downto 0);

        -----------------------------------------------------------------------
        -- Group 4: s32x512_m0~m511 sel_code0 / sel_code1 [4:0]
        -- CAL_CHL_NUM channels, each channel has two 5-bit selection codes for pingpong:
        --   sel_code0 = ping selection code  [4:0]
        --   sel_code1 = pong selection code  [4:0]
        -- s32x512_sel_code0(i)  <=> s32x512_m<i>_sel_code0[4:0]
        -- s32x512_sel_code1(i)  <=> s32x512_m<i>_sel_code1[4:0]
        -----------------------------------------------------------------------
        s32x512_sel_code0 : out t_slv5_array(0 to CAL_CHL_NUM-1);
        s32x512_sel_code1 : out t_slv5_array(0 to CAL_CHL_NUM-1);

        -----------------------------------------------------------------------
        -- Group 5: s512x32_m0~m31 sel_code0 / sel_code1 [511:0]
        -- 32 channels, each channel has a 512-bit selection code.
        -- Two such codes per channel for pingpong operation:
        --   sel_code0(m) = ping selection code [511:0]
        --                 (APB: written via 16 x 32-bit at _0~_15)
        --   sel_code1(m) = pong selection code [511:0]
        --                 (APB: written via 16 x 32-bit at _0~_15)
        -- s512x32_sel_code0(m) <=> s512x32_m<m>_sel_code0[511:0]
        -- s512x32_sel_code1(m) <=> s512x32_m<m>_sel_code1[511:0]
        -- where m = 0..31
        -----------------------------------------------------------------------
        s512x32_sel_code0 : out t_slv512_array(0 to J204B_LANE_NUM-1);
        s512x32_sel_code1 : out t_slv512_array(0 to J204B_LANE_NUM-1);

        -----------------------------------------------------------------------
        -- Group 6: Switch Matrix Selection Codes
        -- s4x3_sel_code : 6-bit selection code for 4-to-3 switch matrix
        -- s3x4_sel_code : 12-bit selection code for 3-to-4 switch matrix
        -----------------------------------------------------------------------
        s4x3_sel_code : out std_logic_vector(5 downto 0);
        s3x4_sel_code : out std_logic_vector(11 downto 0);

        -----------------------------------------------------------------------
        -- Group 7: Read-only status registers (来自 inc_rd 模块)
        -----------------------------------------------------------------------
        wei_bram_inc_wring : in  std_logic;
        mps_bram_inc_wring : in  std_logic
    );
end entity apb_reg_module;

--------------------------------------------------------------------------------
-- Architecture: RTL
--------------------------------------------------------------------------------
architecture rtl of apb_reg_module is

    ---------------------------------------------------------------------------
    -- Help function: min_int (clamp integer to upper bound)
    ---------------------------------------------------------------------------
    function min_int(a, b : integer) return integer is
    begin
        if a < b then return a; else return b; end if;
    end function;

    -- Number of 32-bit APB words needed for CAL_CHL_NUM half-select bits
    constant HLF_SEL_WORD_NUM : integer := (CAL_CHL_NUM + 31) / 32;
    -- Number of 32-bit APB words needed for one s512x32 sel_code entry
    constant SEG_PER_CHAN : integer := (CAL_CHL_NUM + 31) / 32;

    ---------------------------------------------------------------------------
    -- Internal register signals
    ---------------------------------------------------------------------------
    -- Group 1
    signal reg_dbf_dir         : std_logic;
    signal reg_wei_pq_flag       : std_logic;
    signal reg_mps_pq_flag       : std_logic;
    signal reg_dbf_204b_rx_num : std_logic_vector(CNT_WIDTH-1 downto 0);
    signal reg_dbf_204b_tx_num : std_logic_vector(CNT_WIDTH-1 downto 0);

    -- Group 2 & 3 (width linked to CAL_CHL_NUM)
    signal reg_dbf_rx_hlf_sel  : std_logic_vector(CAL_CHL_NUM-1 downto 0);
    signal reg_dbf_tx_hlf_sel  : std_logic_vector(CAL_CHL_NUM-1 downto 0);

    -- Group 4 (pingpong, size linked to CAL_CHL_NUM)
    signal reg_s32x512_code0   : t_slv5_array(0 to CAL_CHL_NUM-1);
    signal reg_s32x512_code1   : t_slv5_array(0 to CAL_CHL_NUM-1);

    -- Group 5 (pingpong, 512-bit per channel, 32 channels)
    signal reg_s512x32_code0   : t_slv512_array(0 to J204B_LANE_NUM-1);
    signal reg_s512x32_code1   : t_slv512_array(0 to J204B_LANE_NUM-1);

    -- Group 6 (switch matrix selection codes)
    signal reg_dbf_s4x3_sel_code : std_logic_vector(5 downto 0);
    signal reg_dbf_s3x4_sel_code : std_logic_vector(11 downto 0);

    -- APB control
    signal apb_wr    : std_logic;
    signal word_addr : integer range 0 to 4095;

    -- Read data
    signal rd_data   : std_logic_vector(31 downto 0);

    ---------------------------------------------------------------------------
    -- Address Map Constants (word addresses, paddr[11:2])
    -- All base offsets computed from CAL_CHL_NUM for correct APB address layout
    ---------------------------------------------------------------------------
    -- Group 1: 5 registers (fixed)
    constant ADDR_DBF_DIR         : integer := 0;
    constant ADDR_WEI_RPQ_FLAG       : integer := 1;
    constant ADDR_MPS_RPQ_FLAG       : integer := 2;
    constant ADDR_DBF_204B_RX_NUM : integer := 3;
    constant ADDR_DBF_204B_TX_NUM : integer := 4;

    -- Group 2: dbf_rx_hlf_sel, HLF_SEL_WORD_NUM words x 32 bits
    constant BASE_RX_HLF_SEL      : integer := 5;

    -- Group 3: dbf_tx_hlf_sel, HLF_SEL_WORD_NUM words x 32 bits
    constant BASE_TX_HLF_SEL      : integer := 5 + HLF_SEL_WORD_NUM;

    -- Group 4: s32x512, CAL_CHL_NUM channels x 5-bit x 2 codes (pingpong)
    constant BASE_S32X512_CODE0   : integer := 5 + 2 * HLF_SEL_WORD_NUM;
    constant BASE_S32X512_CODE1   : integer := 5 + 2 * HLF_SEL_WORD_NUM + CAL_CHL_NUM;

    -- Group 5: s512x32, 32 channels x 16 segments x 2 codes (pingpong)
    constant BASE_S512X32_CODE0   : integer := 5 + 2 * HLF_SEL_WORD_NUM + 2 * CAL_CHL_NUM;
    constant BASE_S512X32_CODE1   : integer := BASE_S512X32_CODE0 + J204B_LANE_NUM * SEG_PER_CHAN;

    -- Group 6: Switch matrix selection codes
    constant ADDR_DBF_S4X3_SEL_CODE : integer := BASE_S512X32_CODE1 + J204B_LANE_NUM * SEG_PER_CHAN;
    constant ADDR_DBF_S3X4_SEL_CODE : integer := ADDR_DBF_S4X3_SEL_CODE + 1;

    -- Group 7: Read-only incremental write status registers
    constant ADDR_WEI_BRAM_INC_WRING : integer := ADDR_DBF_S3X4_SEL_CODE + 1;
    constant ADDR_MPS_BRAM_INC_WRING : integer := ADDR_DBF_S3X4_SEL_CODE + 2;

begin

    ---------------------------------------------------------------------------
    -- APB Control Signals
    ---------------------------------------------------------------------------
    apb_wr    <= psel and penable and pwrite;
    word_addr <= to_integer(unsigned(paddr(13 downto 2)));

    pready  <= '1';
    pslverr <= '0';
    prdata  <= rd_data;

    ---------------------------------------------------------------------------
    -- Output Assignments (Register -> Port)
    ---------------------------------------------------------------------------
    dbf_dir         <= reg_dbf_dir;
    wei_pq_flag       <= reg_wei_pq_flag;
    mps_pq_flag       <= reg_mps_pq_flag;
    dbf_204b_rx_num <= reg_dbf_204b_rx_num;
    dbf_204b_tx_num <= reg_dbf_204b_tx_num;

    dbf_rx_hlf_sel  <= reg_dbf_rx_hlf_sel;
    dbf_tx_hlf_sel  <= reg_dbf_tx_hlf_sel;

    gen_s32x512_out: for i in 0 to CAL_CHL_NUM-1 generate
        s32x512_sel_code0(i) <= reg_s32x512_code0(i);
        s32x512_sel_code1(i) <= reg_s32x512_code1(i);
    end generate gen_s32x512_out;

    s512x32_sel_code0 <= reg_s512x32_code0;
    s512x32_sel_code1 <= reg_s512x32_code1;

    s4x3_sel_code <= reg_dbf_s4x3_sel_code;
    s3x4_sel_code <= reg_dbf_s3x4_sel_code;

    ---------------------------------------------------------------------------
    -- Group 1 Write: Basic control registers (addr 0~4)
    ---------------------------------------------------------------------------
    process(pclk, presetn)
    begin
        if presetn = '0' then
            reg_dbf_dir         <= '0';
            reg_wei_pq_flag       <= '0';
            reg_mps_pq_flag       <= '0';
            reg_dbf_204b_rx_num <= (others => '0');
            reg_dbf_204b_tx_num <= (others => '0');
        elsif rising_edge(pclk) then
            if apb_wr = '1' then
                case word_addr is
                    when ADDR_DBF_DIR         => reg_dbf_dir         <= pwdata(0);
                    when ADDR_WEI_RPQ_FLAG       => reg_wei_pq_flag       <= pwdata(0);
                    when ADDR_MPS_RPQ_FLAG       => reg_mps_pq_flag       <= pwdata(0);
                    when ADDR_DBF_204B_RX_NUM => reg_dbf_204b_rx_num <= pwdata(CNT_WIDTH-1 downto 0);
                    when ADDR_DBF_204B_TX_NUM => reg_dbf_204b_tx_num <= pwdata(CNT_WIDTH-1 downto 0);
                    when others => null;
                end case;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Group 2 Write: dbf_rx_hlf_sel
    -- HLF_SEL_WORD_NUM words x 32 bits = CAL_CHL_NUM single-bit registers
    -- Last word clamps bit range via min_int if not multiple of 32
    ---------------------------------------------------------------------------
    gen_rx_hlf_wr: for i in 0 to HLF_SEL_WORD_NUM-1 generate
        constant C_LO : integer := i * 32;
        constant C_HI : integer := min_int((i + 1) * 32 - 1, CAL_CHL_NUM - 1);
    begin
        process(pclk, presetn)
        begin
            if presetn = '0' then
                reg_dbf_rx_hlf_sel(C_HI downto C_LO) <= (others => '0');
            elsif rising_edge(pclk) then
                if apb_wr = '1' and word_addr = BASE_RX_HLF_SEL + i then
                    reg_dbf_rx_hlf_sel(C_HI downto C_LO) <= pwdata(C_HI - C_LO downto 0);
                end if;
            end if;
        end process;
    end generate gen_rx_hlf_wr;

    ---------------------------------------------------------------------------
    -- Group 3 Write: dbf_tx_hlf_sel
    -- HLF_SEL_WORD_NUM words x 32 bits = CAL_CHL_NUM single-bit registers
    ---------------------------------------------------------------------------
    gen_tx_hlf_wr: for i in 0 to HLF_SEL_WORD_NUM-1 generate
        constant C_LO : integer := i * 32;
        constant C_HI : integer := min_int((i + 1) * 32 - 1, CAL_CHL_NUM - 1);
    begin
        process(pclk, presetn)
        begin
            if presetn = '0' then
                reg_dbf_tx_hlf_sel(C_HI downto C_LO) <= (others => '0');
            elsif rising_edge(pclk) then
                if apb_wr = '1' and word_addr = BASE_TX_HLF_SEL + i then
                    reg_dbf_tx_hlf_sel(C_HI downto C_LO) <= pwdata(C_HI - C_LO downto 0);
                end if;
            end if;
        end process;
    end generate gen_tx_hlf_wr;

    ---------------------------------------------------------------------------
    -- Group 4 Write: s32x512_m0~m(CAL_CHL_NUM-1) sel_code0 / ping
    -- CAL_CHL_NUM x 5-bit registers, stored in pwdata(4 downto 0)
    ---------------------------------------------------------------------------
    gen_s32x512_c0_wr: for i in 0 to CAL_CHL_NUM-1 generate
        process(pclk, presetn)
        begin
            if presetn = '0' then
                reg_s32x512_code0(i) <= (others => '0');
            elsif rising_edge(pclk) then
                if apb_wr = '1' and word_addr = BASE_S32X512_CODE0 + i then
                    reg_s32x512_code0(i) <= pwdata(4 downto 0);
                end if;
            end if;
        end process;
    end generate gen_s32x512_c0_wr;

    ---------------------------------------------------------------------------
    -- Group 4 Write: s32x512_m0~m(CAL_CHL_NUM-1) sel_code1 / pong
    -- CAL_CHL_NUM x 5-bit registers, stored in pwdata(4 downto 0)
    ---------------------------------------------------------------------------
    gen_s32x512_c1_wr: for i in 0 to CAL_CHL_NUM-1 generate
        process(pclk, presetn)
        begin
            if presetn = '0' then
                reg_s32x512_code1(i) <= (others => '0');
            elsif rising_edge(pclk) then
                if apb_wr = '1' and word_addr = BASE_S32X512_CODE1 + i then
                    reg_s32x512_code1(i) <= pwdata(4 downto 0);
                end if;
            end if;
        end process;
    end generate gen_s32x512_c1_wr;

    ---------------------------------------------------------------------------
    -- Group 5 Write: s512x32_m0~m(J204B_LANE_NUM-1) sel_code0 / ping [511:0]
    -- APB writes 32-bit segments: addr = BASE + m*16 + s
    -- Each segment maps to bits [(s+1)*32-1 : s*32] of the 512-bit register
    ---------------------------------------------------------------------------
    gen_s512x32_c0_wr: for m in 0 to J204B_LANE_NUM-1 generate
        gen_s512x32_c0_seg: for s in 0 to SEG_PER_CHAN-1 generate
            constant C_LO : integer := s * 32;
            constant C_HI : integer := min_int((s+1)*32 - 1, CAL_CHL_NUM - 1);
        begin
            process(pclk, presetn)
            begin
                if presetn = '0' then
                    reg_s512x32_code0(m)(C_HI downto C_LO) <= (others => '0');
                elsif rising_edge(pclk) then
                    if apb_wr = '1' and word_addr = BASE_S512X32_CODE0 + m*SEG_PER_CHAN + s then
                        reg_s512x32_code0(m)(C_HI downto C_LO) <= pwdata(C_HI - C_LO downto 0);
                    end if;
                end if;
            end process;
        end generate gen_s512x32_c0_seg;
    end generate gen_s512x32_c0_wr;

    ---------------------------------------------------------------------------
    -- Group 5 Write: s512x32_m0~m(J204B_LANE_NUM-1) sel_code1 / pong [511:0]
    -- APB writes 32-bit segments: addr = BASE + m*16 + s
    -- Each segment maps to bits [(s+1)*32-1 : s*32] of the 512-bit register
    ---------------------------------------------------------------------------
    gen_s512x32_c1_wr: for m in 0 to J204B_LANE_NUM-1 generate
        gen_s512x32_c1_seg: for s in 0 to SEG_PER_CHAN-1 generate
            constant C_LO : integer := s * 32;
            constant C_HI : integer := min_int((s+1)*32 - 1, CAL_CHL_NUM - 1);
        begin
            process(pclk, presetn)
            begin
                if presetn = '0' then
                    reg_s512x32_code1(m)(C_HI downto C_LO) <= (others => '0');
                elsif rising_edge(pclk) then
                    if apb_wr = '1' and word_addr = BASE_S512X32_CODE1 + m*SEG_PER_CHAN + s then
                        reg_s512x32_code1(m)(C_HI downto C_LO) <= pwdata(C_HI - C_LO downto 0);
                    end if;
                end if;
            end process;
        end generate gen_s512x32_c1_seg;
    end generate gen_s512x32_c1_wr;

    ---------------------------------------------------------------------------
    -- Group 6 Write: s4x3_sel_code (6-bit) and s3x4_sel_code (12-bit)
    ---------------------------------------------------------------------------
    process(pclk, presetn)
    begin
        if presetn = '0' then
            reg_dbf_s4x3_sel_code <= (others => '0');
            reg_dbf_s3x4_sel_code <= (others => '0');
        elsif rising_edge(pclk) then
            if apb_wr = '1' then
                case word_addr is
                    when ADDR_DBF_S4X3_SEL_CODE => reg_dbf_s4x3_sel_code <= pwdata(5 downto 0);
                    when ADDR_DBF_S3X4_SEL_CODE => reg_dbf_s3x4_sel_code <= pwdata(11 downto 0);
                    when others => null;
                end case;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Read Data Multiplexer (combinational)
    ---------------------------------------------------------------------------
    process(word_addr,
            reg_dbf_dir, reg_wei_pq_flag, reg_mps_pq_flag,
            reg_dbf_204b_rx_num, reg_dbf_204b_tx_num,
            reg_dbf_rx_hlf_sel, reg_dbf_tx_hlf_sel,
            reg_s32x512_code0, reg_s32x512_code1,
            reg_s512x32_code0, reg_s512x32_code1,
            reg_dbf_s4x3_sel_code, reg_dbf_s3x4_sel_code,
            wei_bram_inc_wring, mps_bram_inc_wring)
        variable v_data : std_logic_vector(31 downto 0);
        variable v_off  : integer range 0 to 4095;
        variable v_chan : integer range 0 to 31;
        variable v_seg  : integer range 0 to 15;
        variable v_lo   : integer range 0 to 511;
        variable v_hi   : integer range 0 to 511;
    begin
        v_data := (others => '0');

        if word_addr = ADDR_DBF_DIR then
            v_data(0) := reg_dbf_dir;
        elsif word_addr = ADDR_WEI_RPQ_FLAG then
            v_data(0) := reg_wei_pq_flag;
        elsif word_addr = ADDR_MPS_RPQ_FLAG then
            v_data(0) := reg_mps_pq_flag;
        elsif word_addr = ADDR_DBF_204B_RX_NUM then
            v_data := std_logic_vector(resize(unsigned(reg_dbf_204b_rx_num), 32));
        elsif word_addr = ADDR_DBF_204B_TX_NUM then
            v_data := std_logic_vector(resize(unsigned(reg_dbf_204b_tx_num), 32));

        -- Group 2 Read: dbf_rx_hlf_sel (HLF_SEL_WORD_NUM words)
        elsif word_addr >= BASE_RX_HLF_SEL and word_addr < BASE_RX_HLF_SEL + HLF_SEL_WORD_NUM then
            v_lo := (word_addr - BASE_RX_HLF_SEL) * 32;
            v_hi := v_lo + 31;
            if v_hi >= CAL_CHL_NUM then
                v_hi := CAL_CHL_NUM - 1;
            end if;
            v_data(v_hi - v_lo downto 0) := reg_dbf_rx_hlf_sel(v_hi downto v_lo);

        -- Group 3 Read: dbf_tx_hlf_sel (HLF_SEL_WORD_NUM words)
        elsif word_addr >= BASE_TX_HLF_SEL and word_addr < BASE_TX_HLF_SEL + HLF_SEL_WORD_NUM then
            v_lo := (word_addr - BASE_TX_HLF_SEL) * 32;
            v_hi := v_lo + 31;
            if v_hi >= CAL_CHL_NUM then
                v_hi := CAL_CHL_NUM - 1;
            end if;
            v_data(v_hi - v_lo downto 0) := reg_dbf_tx_hlf_sel(v_hi downto v_lo);

        -- Group 4 Read: s32x512_code0 / ping (CAL_CHL_NUM words)
        elsif word_addr >= BASE_S32X512_CODE0 and word_addr < BASE_S32X512_CODE0 + CAL_CHL_NUM then
            v_data(4 downto 0) := reg_s32x512_code0(word_addr - BASE_S32X512_CODE0);

        -- Group 4 Read: s32x512_code1 / pong (CAL_CHL_NUM words)
        elsif word_addr >= BASE_S32X512_CODE1 and word_addr < BASE_S32X512_CODE1 + CAL_CHL_NUM then
            v_data(4 downto 0) := reg_s32x512_code1(word_addr - BASE_S32X512_CODE1);

        -- Group 5 Read: s512x32_code0 / ping
        -- Extract 32-bit segment from per-entry register
        elsif word_addr >= BASE_S512X32_CODE0 and word_addr < BASE_S512X32_CODE0 + J204B_LANE_NUM * SEG_PER_CHAN then
            v_off  := word_addr - BASE_S512X32_CODE0;
            v_chan := v_off / SEG_PER_CHAN;
            v_seg  := v_off mod SEG_PER_CHAN;
            v_data := reg_s512x32_code0(v_chan)((v_seg+1)*32-1 downto v_seg*32);

        -- Group 5 Read: s512x32_code1 / pong
        -- Extract 32-bit segment from per-entry register
        elsif word_addr >= BASE_S512X32_CODE1 and word_addr < BASE_S512X32_CODE1 + J204B_LANE_NUM * SEG_PER_CHAN then
            v_off  := word_addr - BASE_S512X32_CODE1;
            v_chan := v_off / SEG_PER_CHAN;
            v_seg  := v_off mod SEG_PER_CHAN;
            v_data := reg_s512x32_code1(v_chan)((v_seg+1)*32-1 downto v_seg*32);

        -- Group 6 Read: s4x3_sel_code
        elsif word_addr = ADDR_DBF_S4X3_SEL_CODE then
            v_data(5 downto 0) := reg_dbf_s4x3_sel_code;

        -- Group 6 Read: s3x4_sel_code
        elsif word_addr = ADDR_DBF_S3X4_SEL_CODE then
            v_data(11 downto 0) := reg_dbf_s3x4_sel_code;

        -- Group 7 Read: incremental write status (read-only)
        elsif word_addr = ADDR_WEI_BRAM_INC_WRING then
            v_data(0) := wei_bram_inc_wring;
        elsif word_addr = ADDR_MPS_BRAM_INC_WRING then
            v_data(0) := mps_bram_inc_wring;

        else
            v_data := (others => '0');
        end if;

        rd_data <= v_data;
    end process;

end architecture rtl;
