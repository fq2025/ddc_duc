--------------------------------------------------------------------------------
-- APB MPS BRAM Write & Read Module (Full-width buffered, single-shot write)
-- Description: APB-to-BRAM controller for all MPS BRAMs.
--              Write: APB writes 32-bit segments into a full-width buffer.
--                     When all segments are complete, bram_rct_wen asserts
--                     and full-width data is committed to BRAMs.
--              Read:  APB drives bram_rct_wen='0' with target address,
--                     waits 1 cycle, then reads a 32-bit segment from
--                     the returned full-width data.
--
-- Address Mapping (paddr = 32-bit):
--   paddr[1:0]   = Byte offset (unused)
--   paddr[7:2]   = Word index (0 to SEGMENT_NUM-1)
--   paddr[25:8]  = BRAM row address (shared by all BRAMs)
--
-- Data width = MPS_BRAM_WIDTH * MPS_BRAM_NUM
--   SEGMENT_NUM = (MPS_BRAM_WIDTH * MPS_BRAM_NUM) / 32
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity apb_mpsbram_wr is
    generic (
        ADDR_WIDTH      : integer := 32;
        MPS_BRAM_WIDTH  : integer := 160;    -- Data width per BRAM (must be multiple of 32)
        MPS_BRAM_NUM    : integer := 8
    );
    port (
        -----------------------------------------------------------------------
        -- APB Interface (AMBA 3 APB)
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
        -- BRAM RCT Interface (full-width)
        --   bram_rct_wen = '1' -> write, '0' -> read
        -----------------------------------------------------------------------
        --bram_rct_en    : out std_logic;    -- single-bit enable
        bram_rct_wen   : out std_logic_vector(159 downto 0);    -- byte-level write enables (20 bytes x 8 BRAMs)
        bram_rct_addr  : out std_logic_vector(ADDR_WIDTH-1 downto 0);
        bram_rct_wdata : out std_logic_vector(MPS_BRAM_WIDTH * MPS_BRAM_NUM - 1 downto 0);
        bram_rct_rdata : in  std_logic_vector(MPS_BRAM_WIDTH * MPS_BRAM_NUM - 1 downto 0)
    );
end entity apb_mpsbram_wr;

--------------------------------------------------------------------------------
-- Architecture: RTL
--------------------------------------------------------------------------------
architecture rtl of apb_mpsbram_wr is

    constant C_DATA_WIDTH : integer := MPS_BRAM_WIDTH * MPS_BRAM_NUM;
    constant SEGMENT_NUM  : integer := C_DATA_WIDTH / 32;

    -- APB control
    signal apb_wr     : std_logic;
    signal apb_rd     : std_logic;
    signal word_index : integer range 0 to SEGMENT_NUM-1;
    signal bram_addr  : std_logic_vector(ADDR_WIDTH-1 downto 0);

    -- Write buffer (full-width)
    signal wr_buf     : std_logic_vector(C_DATA_WIDTH-1 downto 0);
    signal seg_done   : std_logic_vector(SEGMENT_NUM-1 downto 0);

    -- Registered BRAM write outputs (internal)
    signal reg_wr_en   : std_logic_vector(C_DATA_WIDTH/8 - 1 downto 0);
    signal reg_wr_addr : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal reg_wr_data : std_logic_vector(C_DATA_WIDTH-1 downto 0);

    -----------------------------------------------------------------------
    -- Read FSM signals
    -----------------------------------------------------------------------
    type t_rd_fsm is (S_IDLE, S_WAIT, S_DONE);
    signal rd_state    : t_rd_fsm;
    signal rd_seg_reg  : integer range 0 to SEGMENT_NUM-1;
    signal rd_pready   : std_logic;
    signal rd_en_reg   : std_logic;
    signal rd_addr_reg : std_logic_vector(ADDR_WIDTH-1 downto 0);
    signal rd_data_reg : std_logic_vector(31 downto 0);

begin

    -----------------------------------------------------------------------
    -- APB Control & Address Decode
    -----------------------------------------------------------------------
    apb_wr <= psel and penable and pwrite;
    apb_rd <= psel and penable and (not pwrite);

    -- BRAM row address: paddr byte-address bits [25:8]
    bram_addr <= std_logic_vector(resize(unsigned(paddr(25 downto 8)), ADDR_WIDTH));

    -- Word index from paddr[7:2] (clamped to valid range)
    word_index <= to_integer(unsigned(paddr(7 downto 2))) when 
                  to_integer(unsigned(paddr(7 downto 2))) < SEGMENT_NUM else 0;

    -- Error output: always 0
    pslverr <= '0';

    -----------------------------------------------------------------------
    -- BRAM Write Logic (full-width buffered, one-shot write)
    -----------------------------------------------------------------------
    process(pclk, presetn)
        variable v_seg_done : std_logic_vector(SEGMENT_NUM-1 downto 0);
    begin
        if presetn = '0' then
            wr_buf      <= (others => '0');
            seg_done    <= (others => '0');
            reg_wr_en   <= (others => '0');
            reg_wr_addr <= (others => '0');
            reg_wr_data <= (others => '0');
        elsif rising_edge(pclk) then
            -- Default: write enable low (one-cycle pulse)
            reg_wr_en <= (others => '0');

            v_seg_done := seg_done;
            
            ----------------------------------------------------------------
            -- APB write: update buffer and segment tracking
            ----------------------------------------------------------------
            if apb_wr = '1' then
                -- Update the corresponding 32-bit segment in the buffer
                for i in 0 to SEGMENT_NUM-1 loop
                    if word_index = i then
                        wr_buf((i + 1) * 32 - 1 downto i * 32) <= pwdata;
                    end if;
                end loop;
            
                -- Mark segment as done
                v_seg_done(word_index) := '1';
            end if;
            
            ----------------------------------------------------------------
            -- When all segments are complete, trigger write
            -- Check AFTER APB update so the last segment triggers commit
            ----------------------------------------------------------------
            if v_seg_done = (v_seg_done'range => '1') then
                reg_wr_en   <= (others => '1');
                reg_wr_addr <= bram_addr;
                reg_wr_data <= wr_buf;
                -- Clear buffer and segment tracking for next round
                v_seg_done  := (others => '0');
                wr_buf      <= (others => '0');
            end if;

            -- Update segment tracking registers
            seg_done <= v_seg_done;
        end if;
    end process;

    -----------------------------------------------------------------------
    -- APB Read Logic (FSM with 2-cycle wait for BRAM read latency)
    --   S_IDLE -> (apb_rd) -> S_WAIT -> S_DONE -> S_IDLE
    -----------------------------------------------------------------------
    process(pclk, presetn)
    begin
        if presetn = '0' then
            rd_state    <= S_IDLE;
            rd_en_reg   <= '0';
            rd_addr_reg <= (others => '0');
            rd_seg_reg  <= 0;
            rd_data_reg <= (others => '0');
            rd_pready   <= '1';
        elsif rising_edge(pclk) then
            case rd_state is
                when S_IDLE =>
                    rd_en_reg <= '0';
                    rd_pready <= '1';
                    if apb_rd = '1' then
                        rd_en_reg   <= '1';
                        rd_addr_reg <= bram_addr;
                        rd_seg_reg  <= word_index;
                        rd_pready   <= '0';
                        rd_state    <= S_WAIT;
                    end if;

                when S_WAIT =>
                    -- Keep rd_en high; sample BRAM read data (now valid)
                    rd_en_reg   <= rd_en_reg;
                    rd_pready   <= '0';
                    rd_data_reg <= bram_rct_rdata((rd_seg_reg + 1) * 32 - 1 downto rd_seg_reg * 32);
                    rd_state    <= S_DONE;

                when S_DONE =>
                    rd_en_reg <= '0';
                    rd_pready <= '1';
                    rd_state  <= S_IDLE;

                when others =>
                    rd_state <= S_IDLE;
            end case;
        end if;
    end process;

    -----------------------------------------------------------------------
    -- Output assignment
    -----------------------------------------------------------------------
    -- Write enable: only during write commit; '0' indicates read
    bram_rct_wen <= reg_wr_en;

    -- Mux address: read addr during read, write addr otherwise
    bram_rct_addr <= rd_addr_reg when rd_en_reg = '1' else reg_wr_addr;

    -- Write data passthrough
    bram_rct_wdata <= reg_wr_data;

    -- prdata: write returns 0, read returns the selected segment
    prdata <= (others => '0') when pwrite = '1' else rd_data_reg;

    -- pready: writes always ready, reads controlled by FSM
    pready <= '1' when pwrite = '1' else rd_pready;

end architecture rtl;
