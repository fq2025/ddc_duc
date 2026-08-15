--------------------------------------------------------------------------------
-- APB Split Module
-- Description: APB address decoder that fans out a single APB master bus to
--              three sub-modules:
--                - apb_reg_module   (registers)
--                - apb_weibram_wr   (WEI BRAM write controller)
--                - apb_mpsbram_wr   (MPS BRAM write controller, multi-segment buffered)
--
-- Address Decode (paddr[27:26]):
--   "00" �? apb_reg_module   (base 0x0000_0000, uses paddr[13:0] = 16KB)
--   "01" �? apb_weibram_wr   (base 0x0800_0000)
--   "10" �? apb_mpsbram_wr   (base 0x1000_0000)
--   "11" �? No slave (APB error response)
--
--   All sub-modules share pclk, presetn, penable, pwrite, pwdata.
--   psel is individually decoded from paddr high bits.
--   prdata/pready/pslverr are muxed back from the selected slave.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.apb_reg_pkg.all;

entity apb_split is
    generic (
        CAL_CHL_NUM    : integer := 512; -- Number of calculation channels for apb_reg_module
        CNT_WIDTH      : integer := 32;  -- Width for dbf_204b_rx_num / dbf_204b_tx_num
        WEI_ADDR_WIDTH : integer := 12;  -- Address width for apb_weibram_wr
        MPS_ADDR_WIDTH : integer := 12;  -- Address width for apb_mpsbram_wr
        MPS_BRAM_WIDTH : integer := 160; -- Per-BRAM data width for apb_mpsbram_wr
        MPS_BRAM_NUM   : integer := 8;    -- Number of MPS BRAMs
        J204B_LANE_NUM  : integer := 32    -- Number of s512x32 output channels
    );
    port (
        -----------------------------------------------------------------------
        -- APB Slave Interface (from master)
        -----------------------------------------------------------------------
        pclk    : in  std_logic;
        presetn : in  std_logic;
        paddr   : in  std_logic_vector(31 downto 0);
        psel    : in  std_logic;
        penable : in  std_logic;
        pwrite  : in  std_logic;
        pwdata  : in  std_logic_vector(31 downto 0);
        prdata  : out std_logic_vector(31 downto 0);
        pready  : out std_logic;
        pslverr : out std_logic;

        -----------------------------------------------------------------------
        -- apb_reg_module outputs (pass-through)
        -----------------------------------------------------------------------
        dbf_dir         : out std_logic;
        wei_pq_flag    : out std_logic;
        mps_pq_flag    : out std_logic;
        dbf_204b_rx_num : out std_logic_vector(CNT_WIDTH-1 downto 0);
        dbf_204b_tx_num : out std_logic_vector(CNT_WIDTH-1 downto 0);
        dbf_rx_hlf_sel  : out std_logic_vector(CAL_CHL_NUM-1 downto 0);
        dbf_tx_hlf_sel  : out std_logic_vector(CAL_CHL_NUM-1 downto 0);
        s32x512_sel_code0 : out t_slv5_array(0 to CAL_CHL_NUM-1);
        s32x512_sel_code1 : out t_slv5_array(0 to CAL_CHL_NUM-1);
        s512x32_sel_code0 : out t_slv512_array(0 to J204B_LANE_NUM-1);
        s512x32_sel_code1 : out t_slv512_array(0 to J204B_LANE_NUM-1);
        s4x3_sel_code : out std_logic_vector(5 downto 0);
        s3x4_sel_code : out std_logic_vector(11 downto 0);
        -----------------------------------------------------------------------
        -- apb_weibram_wr outputs (pass-through)
        -----------------------------------------------------------------------
		
        wei_bram_rct_en    : out std_logic_vector(CAL_CHL_NUM-1 downto 0);
        wei_bram_rct_wen   : out std_logic_vector(3 downto 0);
        wei_bram_rct_addr  : out std_logic_vector(WEI_ADDR_WIDTH-1 downto 0);
        wei_bram_rct_wdata : out std_logic_vector(31 downto 0);
        wei_bram_rct_rdata : in  std_logic_vector(31 downto 0);

        -----------------------------------------------------------------------
        -- apb_mpsbram_wr outputs (pass-through)
        -----------------------------------------------------------------------
        mps_bram_rct_wen   : out std_logic_vector(159 downto 0);
        mps_bram_rct_addr  : out std_logic_vector(MPS_ADDR_WIDTH-1 downto 0);
        mps_bram_rct_wdata : out std_logic_vector(MPS_BRAM_NUM * MPS_BRAM_WIDTH - 1 downto 0);
        mps_bram_rct_rdata : in  std_logic_vector(MPS_BRAM_NUM * MPS_BRAM_WIDTH - 1 downto 0);

        -----------------------------------------------------------------------
        -- inc_wring status inputs (from inc_rd modules, read-only via apb_reg_module)
        -----------------------------------------------------------------------
        wei_bram_inc_wring : in  std_logic;
        mps_bram_inc_wring : in  std_logic
    );
end entity apb_split;

--------------------------------------------------------------------------------
-- Architecture: RTL
--------------------------------------------------------------------------------
architecture rtl of apb_split is

    -- Calculate minimum BRAM_IDX_W to cover CAL_CHL_NUM channels
    function calc_wei_bram_idx_w(chl_num : integer) return integer is
        variable v_w : integer;
    begin
        v_w := 0;
        while 2**v_w < chl_num loop
            v_w := v_w + 1;
        end loop;
        return v_w;
    end function;

    constant C_WEI_BRAM_IDX_W : integer := calc_wei_bram_idx_w(CAL_CHL_NUM);

    -- Address decode
    signal sel_reg : std_logic;  -- paddr[27:26] = "00"
    signal sel_wei : std_logic;  -- paddr[27:26] = "01"
    signal sel_mps : std_logic;  -- paddr[27:26] = "10"
    signal sel_none : std_logic; -- paddr[27:26] = "11"

    -- Per-module psel
    signal reg_psel : std_logic;
    signal wei_psel : std_logic;
    signal mps_psel : std_logic;

    -- Per-module prdata
    signal reg_prdata : std_logic_vector(31 downto 0);
    signal wei_prdata : std_logic_vector(31 downto 0);
    signal mps_prdata : std_logic_vector(31 downto 0);

    -- Per-module pready
    signal reg_pready : std_logic;
    signal wei_pready : std_logic;
    signal mps_pready : std_logic;

    -- Per-module pslverr
    signal reg_pslverr : std_logic;
    signal wei_pslverr : std_logic;
    signal mps_pslverr : std_logic;

    -- Registered output signals (internal)
    signal prdata_int  : std_logic_vector(31 downto 0);
    signal pready_int  : std_logic;
    signal pslverr_int : std_logic;

    -- Full-width WEI BRAM write enable (internal)
    signal wei_bram_wr_en_full : std_logic_vector(2**C_WEI_BRAM_IDX_W - 1 downto 0);

    ---------------------------------------------------------------------------
    -- Component Declarations
    ---------------------------------------------------------------------------
    component apb_reg_module is
        generic (
            CAL_CHL_NUM   : integer := 512;
            J204B_LANE_NUM : integer := 32;
            CNT_WIDTH     : integer := 32
        );
        port (
            pclk    : in  std_logic;
            presetn : in  std_logic;
            paddr   : in  std_logic_vector(13 downto 0);
            psel    : in  std_logic;
            penable : in  std_logic;
            pwrite  : in  std_logic;
            pwdata  : in  std_logic_vector(31 downto 0);
            prdata  : out std_logic_vector(31 downto 0);
            pready  : out std_logic;
            pslverr : out std_logic;
            dbf_dir         : out std_logic;
            wei_pq_flag       : out std_logic;
            mps_pq_flag       : out std_logic;
            dbf_204b_rx_num : out std_logic_vector(CNT_WIDTH-1 downto 0);
            dbf_204b_tx_num : out std_logic_vector(CNT_WIDTH-1 downto 0);
            dbf_rx_hlf_sel  : out std_logic_vector(CAL_CHL_NUM-1 downto 0);
            dbf_tx_hlf_sel  : out std_logic_vector(CAL_CHL_NUM-1 downto 0);
            s32x512_sel_code0 : out t_slv5_array(0 to CAL_CHL_NUM-1);
            s32x512_sel_code1 : out t_slv5_array(0 to CAL_CHL_NUM-1);
            s512x32_sel_code0 : out t_slv512_array(0 to J204B_LANE_NUM-1);
            s512x32_sel_code1 : out t_slv512_array(0 to J204B_LANE_NUM-1);
            s4x3_sel_code : out std_logic_vector(5 downto 0);
            s3x4_sel_code : out std_logic_vector(11 downto 0);
            wei_bram_inc_wring : in  std_logic;
            mps_bram_inc_wring : in  std_logic
        );
    end component;

    component apb_weibram_wr is
        generic (
            BRAM_IDX_W : integer := 9;
            ADDR_WIDTH : integer := 32
        );
        port (
            pclk    : in  std_logic;
            presetn : in  std_logic;
            paddr   : in  std_logic_vector(31 downto 0);
            psel    : in  std_logic;
            penable : in  std_logic;
            pwrite  : in  std_logic;
            pwdata  : in  std_logic_vector(31 downto 0);
            prdata  : out std_logic_vector(31 downto 0);
            pready  : out std_logic;
            pslverr : out std_logic;
            bram_wr_en   : out std_logic_vector(2**BRAM_IDX_W - 1 downto 0);
            bram_wr_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);
            bram_wr_data : out std_logic_vector(31 downto 0)
        );
    end component;

    component apb_mpsbram_wr is
        generic (
            ADDR_WIDTH      : integer := 32;
            MPS_BRAM_WIDTH  : integer := 160;
            MPS_BRAM_NUM    : integer := 8
        );
        port (
            pclk    : in  std_logic;
            presetn : in  std_logic;
            paddr   : in  std_logic_vector(31 downto 0);
            psel    : in  std_logic;
            penable : in  std_logic;
            pwrite  : in  std_logic;
            pwdata  : in  std_logic_vector(31 downto 0);
            prdata  : out std_logic_vector(31 downto 0);
            pready  : out std_logic;
            pslverr : out std_logic;
            bram_rct_wen   : out std_logic;
            bram_rct_addr  : out std_logic_vector(ADDR_WIDTH-1 downto 0);
            bram_rct_wdata : out std_logic_vector(MPS_BRAM_WIDTH * MPS_BRAM_NUM - 1 downto 0);
            bram_rct_rdata : in  std_logic_vector(MPS_BRAM_WIDTH * MPS_BRAM_NUM - 1 downto 0)
        );
    end component;

begin

    ---------------------------------------------------------------------------
    -- Address Decode (combinational)
    ---------------------------------------------------------------------------
    sel_reg  <= '1' when paddr(27 downto 26) = "00" else '0';
    sel_wei  <= '1' when paddr(27 downto 26) = "01" else '0';
    sel_mps  <= '1' when paddr(27 downto 26) = "10" else '0';
    sel_none <= '1' when paddr(27 downto 26) = "11" else '0';

    -- Generate individual psel (qualified by master psel)
    reg_psel <= psel and sel_reg;
    wei_psel <= psel and sel_wei;
    mps_psel <= psel and sel_mps;

    -- Generate global write strobe for wei_bram_ctrl
    wei_bram_rct_wen <= (others => psel and sel_wei and penable and pwrite);

    ---------------------------------------------------------------------------
    -- Read Data Mux (prdata from selected slave) - internal combinational
    ---------------------------------------------------------------------------
    prdata_int <= reg_prdata when sel_reg = '1' else
                  wei_prdata when sel_wei = '1' else
                  mps_prdata when sel_mps = '1' else
                  (others => '0');

    -- PREADY: mux from selected slave - internal combinational
    pready_int <= reg_pready when sel_reg = '1' else
                  wei_pready when sel_wei = '1' else
                  mps_pready when sel_mps = '1' else
                  '1';

    -- PSLVERR: mux from selected slave, or error if no slave selected - internal combinational
    pslverr_int <= reg_pslverr when sel_reg = '1' else
                   wei_pslverr when sel_wei = '1' else
                   mps_pslverr when sel_mps = '1' else
                   (psel and penable);

    ---------------------------------------------------------------------------
    -- Sub-module: apb_reg_module
    ---------------------------------------------------------------------------
    u_reg : apb_reg_module
        generic map (
            CAL_CHL_NUM   => CAL_CHL_NUM,
            J204B_LANE_NUM => J204B_LANE_NUM,
            CNT_WIDTH     => CNT_WIDTH
        )
        port map (
            pclk    => pclk,
            presetn => presetn,
            paddr   => paddr(13 downto 0),
            psel    => reg_psel,
            penable => penable,
            pwrite  => pwrite,
            pwdata  => pwdata,
            prdata  => reg_prdata,
            pready  => reg_pready,
            pslverr => reg_pslverr,

            dbf_dir           => dbf_dir,
            wei_pq_flag         => wei_pq_flag,
            mps_pq_flag         => mps_pq_flag,
            dbf_204b_rx_num   => dbf_204b_rx_num,
            dbf_204b_tx_num   => dbf_204b_tx_num,
            dbf_rx_hlf_sel    => dbf_rx_hlf_sel,
            dbf_tx_hlf_sel    => dbf_tx_hlf_sel,
            s32x512_sel_code0 => s32x512_sel_code0,
            s32x512_sel_code1 => s32x512_sel_code1,
            s512x32_sel_code0 => s512x32_sel_code0,
            s512x32_sel_code1 => s512x32_sel_code1,
			s4x3_sel_code     => s4x3_sel_code,
			s3x4_sel_code     => s3x4_sel_code,
			wei_bram_inc_wring => wei_bram_inc_wring,
			mps_bram_inc_wring => mps_bram_inc_wring
        );

    ---------------------------------------------------------------------------
    -- Sub-module: apb_weibram_wr
    ---------------------------------------------------------------------------
    u_wei : apb_weibram_wr
        generic map (
            BRAM_IDX_W     => C_WEI_BRAM_IDX_W,
            ADDR_WIDTH     => WEI_ADDR_WIDTH			
        )
        port map (
            pclk    => pclk,
            presetn => presetn,
            paddr   => paddr,
            psel    => wei_psel,
            penable => penable,
            pwrite  => pwrite,
            pwdata  => pwdata,
            prdata  => wei_prdata,
            pready  => wei_pready,
            pslverr => wei_pslverr,

            bram_wr_en   => wei_bram_wr_en_full,
            bram_wr_addr => wei_bram_rct_addr,
            bram_wr_data => wei_bram_rct_wdata
        );

    -- Truncate full-width enable to CAL_CHL_NUM channels
    wei_bram_rct_en <= wei_bram_wr_en_full(CAL_CHL_NUM-1 downto 0);

    ---------------------------------------------------------------------------
    -- Sub-module: apb_mpsbram_wr
    ---------------------------------------------------------------------------
    u_mps : apb_mpsbram_wr
        generic map (
            ADDR_WIDTH      => MPS_ADDR_WIDTH,
            MPS_BRAM_WIDTH  => MPS_BRAM_WIDTH,
            MPS_BRAM_NUM    => MPS_BRAM_NUM
        )
        port map (
            pclk    => pclk,
            presetn => presetn,
            paddr   => paddr,
            psel    => mps_psel,
            penable => penable,
            pwrite  => pwrite,
            pwdata  => pwdata,
            prdata  => mps_prdata,
            pready  => mps_pready,
            pslverr => mps_pslverr,

            bram_rct_wen   => mps_bram_rct_wen,
            bram_rct_addr  => mps_bram_rct_addr,
            bram_rct_wdata => mps_bram_rct_wdata,
            bram_rct_rdata => mps_bram_rct_rdata
        );

    ---------------------------------------------------------------------------
    -- Output Pipeline Registers (prdata/pready/pslverr, 1-cycle delay)
    -- Registered to reduce combinational depth on the readback path and
    -- improve timing closure for the APB read return path.
    ---------------------------------------------------------------------------
    process(pclk, presetn)
    begin
        if presetn = '0' then
            prdata  <= (others => '0');
            pready  <= '0';
            pslverr <= '0';
        elsif rising_edge(pclk) then
            prdata  <= prdata_int;
            pready  <= pready_int;
            pslverr <= pslverr_int;
        end if;
    end process;

end architecture rtl;
