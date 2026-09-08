-- all signals in this module have either a prefix
--      "wr_*" = write domain 
--      "rd_*" = read domain  

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use IEEE.math_real.all;

entity async_fifo is
    generic (
        C_WIDTH     : integer := 16;
        C_DEPTH     : integer := 16
    );
    port (
        -- Write domain 
        i_wr_clk    : in  std_logic;
        i_wr_rst    : in  std_logic;
        i_wr_en     : in  std_logic; 
        i_wr_data   : in  std_logic_vector(C_WIDTH-1 downto 0);
        o_wr_full   : out std_logic;

        -- Read domain 
        i_rd_clk    : in  std_logic;
        i_rd_rst    : in  std_logic;
        i_rd_en     : in  std_logic; 
        o_rd_data   : out std_logic_vector(C_WIDTH-1 downto 0);
        o_rd_empty  : out std_logic
    );
end entity async_fifo;

architecture rtl of async_fifo is
    constant C_PTR_BITS : natural := integer(ceil(log2(real(C_DEPTH))));

    type t_async_fifo is array(0 to C_DEPTH - 1) of std_logic_vector(C_WIDTH-1 downto 0);
    signal async_fifo   : t_async_fifo := (others => (others => '0'));

    subtype t_ptr is unsigned(C_PTR_BITS downto 0);

    signal wr_ptr_bin   : t_ptr; -- binary pointer
    signal wr_ptr_gray  : t_ptr; -- combinational conversion of wr_ptr_bin (gray pointer)

    signal rd_ptr_bin   : t_ptr; 
    signal rd_ptr_gray  : t_ptr;

    signal wr_rd_ptr_gray   : t_ptr;
    signal rd_wr_ptr_gray   : t_ptr;

    signal wr_rd_ptr_gray_std   : std_logic_vector(C_PTR_BITS downto 0);

    signal full_flag    : boolean;
    signal empty_flag   : boolean;

    function get_gray(bin : t_ptr) return t_ptr is
    begin 
        return bin xor shift_right(bin, 1);
    end function get_gray;

begin

    assert (2**C_PTR_BITS = C_DEPTH) 
        report "C_DEPTH must be a power of 2 for Gray code pointers to wrap safely." 
        severity failure;

    full_flag   <= (wr_ptr_gray(C_PTR_BITS) /= wr_rd_ptr_gray(C_PTR_BITS)) and 
                (wr_ptr_gray(C_PTR_BITS-1) /= wr_rd_ptr_gray(C_PTR_BITS-1)) and 
                (wr_ptr_gray(C_PTR_BITS-2 downto 0) = wr_rd_ptr_gray(C_PTR_BITS-2 downto 0));
    empty_flag  <= (rd_ptr_gray = rd_wr_ptr_gray);

    o_wr_full   <= '1' when full_flag  else '0';
    o_rd_empty  <= '1' when empty_flag else '0';

    wr_rd_ptr_gray <= unsigned(wr_rd_ptr_gray_std);

    u_ff_sync_0 : entity work.ff_sync
        generic map (
            C_NUM_STAGES    => 2,
            C_NUM_BITS      => C_PTR_BITS + 1
        ) port map (
            i_bit_unsync    => wr_ptr_gray,
            i_rst           => i_rd_rst,
            i_dest_clk      => i_rd_clk,
            o_bit_synced    => unsigned(rd_wr_ptr_gray)
        );

    u_ff_sync_1 : entity work.ff_sync 
        generic map (
            C_NUM_STAGES    => 2,
            C_NUM_BITS      => C_PTR_BITS + 1
        ) port map (
            i_bit_unsync    => rd_ptr_gray,
            i_rst           => i_wr_rst,
            i_dest_clk      => i_wr_clk,
            o_bit_synced    => wr_rd_ptr_gray_std
        );
    
    read : process(i_rd_clk)
    begin
        if rising_edge(i_rd_clk) then 
            if i_rd_rst = '1' then 
                rd_ptr_bin  <= (others => '0');
                rd_ptr_gray <= (others => '0');
            elsif i_rd_en = '1' and (not empty_flag) then 
                o_rd_data   <= async_fifo(to_integer(rd_ptr_bin(C_PTR_BITS-1 downto 0)));
                rd_ptr_bin  <= rd_ptr_bin + 1;
                rd_ptr_gray <= get_gray(rd_ptr_bin + 1);
            end if;
        end if; 
    end process read;

    write : process(i_wr_clk)
    begin 
        if rising_edge(i_wr_clk) then 
            if i_wr_rst = '1' then 
                wr_ptr_bin  <= (others => '0');
                wr_ptr_gray <= (others => '0');
            elsif i_wr_en = '1' and (not full_flag) then
                async_fifo(to_integer(wr_ptr_bin(C_PTR_BITS-1 downto 0))) <= i_wr_data;
                wr_ptr_bin  <= wr_ptr_bin + 1; 
                wr_ptr_gray <= get_gray(wr_ptr_bin + 1);
            end if;
        end if;
    end process write;
    
end architecture rtl;