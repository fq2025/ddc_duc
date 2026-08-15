--------------------------------------------------------------------------------
-- APB BRAM Write Module (Write-Only)
-- Description: APB-to-BRAM write controller for 512 independent BRAM modules.
--              Each BRAM: 32-bit data width, configurable address width.
--              APB writes are decoded into per-BRAM write enables with shared
--              address/data buses. BRAM is write-only, no read-back.
--
-- Address Mapping (paddr = 32-bit, BRAM address = 32-bit):
--   paddr[27:26]                            = Peripheral select (01=wei)
--   paddr[25 : 26-BRAM_IDX_W]              = BRAM index (upper bits, after peripheral select)
--   paddr[25-BRAM_IDX_W : 2]               = BRAM address (lower portion)
--   paddr[1:0]                             = Byte offset (unused)
--   bram_wr_addr[25-BRAM_IDX_W-1 : 0]      = Driven from paddr lower bits
--   bram_wr_addr[31 : 26-BRAM_IDX_W]       = Zero-padded
--
-- Generics:
--   BRAM_IDX_W : BRAM index width (default 9 => 512 BRAMs)
--   ADDR_WIDTH : BRAM address width (default 32)
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity apb_weibram_wr is
    generic (
        BRAM_IDX_W : integer := 9;   -- BRAM index width (num BRAMs = 2^BRAM_IDX_W)
        ADDR_WIDTH : integer := 32   -- BRAM address width
    );
    port (
        -----------------------------------------------------------------------
        -- APB Interface (AMBA 3 APB, lowercase naming)
        -----------------------------------------------------------------------
        pclk    : in  std_logic;                                        -- APB Clock
        presetn : in  std_logic;                                        -- Active-low reset
        paddr   : in  std_logic_vector(31 downto 0);                    -- 32-bit byte address
        psel    : in  std_logic;                                        -- Peripheral select
        penable : in  std_logic;                                        -- Enable strobe
        pwrite  : in  std_logic;                                        -- Write (1) / Read (0)
        pwdata  : in  std_logic_vector(31 downto 0);                    -- Write data
        prdata  : out std_logic_vector(31 downto 0);                    -- Read data (always 0)
        pready  : out std_logic;                                        -- Transfer ready
        pslVERR : out std_logic;                                        -- Transfer error

        -----------------------------------------------------------------------
        -- BRAM Write Interface (output to 2^BRAM_IDX_W BRAM modules)
        -- All BRAMs share the same address and data bus.
        -- Only the selected BRAM receives write enable (1-hot).
        -----------------------------------------------------------------------
        bram_wr_en   : out std_logic_vector(2**BRAM_IDX_W - 1 downto 0); -- Per-BRAM write enable
        bram_wr_addr : out std_logic_vector(ADDR_WIDTH-1 downto 0);     -- Shared write address
        bram_wr_data : out std_logic_vector(31 downto 0)                -- Shared write data (32-bit)
    );
end entity apb_weibram_wr;

--------------------------------------------------------------------------------
-- Architecture: RTL
--------------------------------------------------------------------------------
architecture rtl of apb_weibram_wr is

    -- Derived constants
    constant BRAM_NUM : integer := 2**BRAM_IDX_W;

    -- APB control
    signal apb_wr    : std_logic;
    signal bram_idx  : integer range 0 to BRAM_NUM-1;
    signal bram_addr : std_logic_vector(ADDR_WIDTH-1 downto 0);

    -- Registered write signals (for clean BRAM timing)
    signal reg_wr_en   : std_logic_vector(2**BRAM_IDX_W - 1 downto 0);
    signal reg_wr_addr : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal reg_wr_data : std_logic_vector(31 downto 0);

begin

    ---------------------------------------------------------------------------
    -- APB Control & Address Decode
    ---------------------------------------------------------------------------
    apb_wr <= psel and penable and pwrite;

    -- BRAM index from paddr bits [25 : 26-BRAM_IDX_W] (after peripheral select bits [27:26])
    bram_idx <= to_integer(unsigned(paddr(25 downto 26 - BRAM_IDX_W)));

    -- BRAM address: from paddr excluding BRAM index, peripheral select, and byte offset bits
    bram_addr <= std_logic_vector(resize(unsigned(paddr(25 - BRAM_IDX_W downto 2)), ADDR_WIDTH));

    -- Write-only: read always returns 0
    prdata  <= (others => '0');
    pready  <= '1';
    pslVERR <= '0';

    ---------------------------------------------------------------------------
    -- BRAM Write Logic (registered outputs)
    ---------------------------------------------------------------------------
    process(pclk, presetn)
    begin
        if presetn = '0' then
            reg_wr_en   <= (others => '0');
            reg_wr_addr <= (others => '0');
            reg_wr_data <= (others => '0');
        elsif rising_edge(pclk) then
            -- Default: all write enables deasserted
            reg_wr_en <= (others => '0');

            if apb_wr = '1' then
                -- Decode 1-hot write enable for selected BRAM
                reg_wr_en(bram_idx) <= '1';
                reg_wr_addr         <= bram_addr;
                reg_wr_data         <= pwdata;
            end if;
        end if;
    end process;

    -- Drive BRAM write outputs
    bram_wr_en   <= reg_wr_en;
    bram_wr_addr <= reg_wr_addr;
    bram_wr_data <= reg_wr_data;

end architecture rtl;
