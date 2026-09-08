library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.spi_pkg.all;

entity as5048a_spi_master is
    port (
        i_clk       : in std_logic;
        i_rst       : in std_logic;
        i_start     : in std_logic;
        i_miso      : in std_logic;
        o_mosi      : out std_logic;
        o_n_cs      : out std_logic;
        o_sclk      : out std_logic;
        o_vld       : out std_logic;
        o_raw_angle : out unsigned(13 downto 0)
    );
end entity as5048a_spi_master;

architecture rtl of as5048a_spi_master is

    type t_state is (S_IDLE, S_CS_HIGH, S_CS_SETUP, S_SHIFT, S_CS_HOLD, S_CHECK, S_DONE);
    type t_step is (CMD_PHASE, READ_PHASE, CLEAR_PHASE);

    function even_parity_ok(x : std_logic_vector(15 downto 0)) return boolean is
        variable v_xor : std_logic := '0';
    begin
        for i in x'range loop
            v_xor := v_xor xor x(i);
        end loop;
        return v_xor = '0';
    end function;

    signal r_state      : t_state := S_IDLE;
    signal r_step       : t_step := CMD_PHASE;
    signal r_sclk       : std_logic := '0';
    signal r_tx_shift   : std_logic_vector(15 downto 0) := C_READ_ANGLE_CMD;
    signal r_rx_shift   : std_logic_vector(15 downto 0) := (others => '0');
    signal r_angle      : unsigned(13 downto 0) := (others => '0');
    signal r_phase_cnt  : natural range 0 to C_WAIT_MAX := 0;
    signal r_half_cnt   : natural range 0 to C_SCLK_SWTCH_CNT - 1 := 0;
    signal r_bit_cnt    : natural range 0 to 15 := 0;

begin

    o_sclk      <= r_sclk;
    o_mosi      <= r_tx_shift(15);
    o_n_cs      <= '1' when r_state = S_IDLE or r_state = S_CS_HIGH or r_state = S_CHECK or r_state = S_DONE else '0';
    o_vld       <= '1' when r_state = S_DONE else '0';
    o_raw_angle <= r_angle;

    process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_state     <= S_IDLE;
                r_step      <= CMD_PHASE;
                r_sclk      <= '0';
                r_tx_shift  <= C_READ_ANGLE_CMD;
                r_rx_shift  <= (others => '0');
                r_angle     <= (others => '0');
                r_phase_cnt <= 0;
                r_half_cnt  <= 0;
                r_bit_cnt   <= 0;
            else
                case r_state is
                    when S_IDLE =>
                        r_sclk <= '0';
                        if i_start = '1' then
                            r_step      <= CMD_PHASE;
                            r_tx_shift  <= C_READ_ANGLE_CMD;
                            r_rx_shift  <= (others => '0');
                            r_phase_cnt <= 0;
                            r_state     <= S_CS_HIGH;
                        end if;

                    when S_CS_HIGH =>
                        r_sclk <= '0';
                        if r_phase_cnt = C_WAIT_MAX - 1 then
                            r_phase_cnt <= 0;
                            r_state     <= S_CS_SETUP;
                        else
                            r_phase_cnt <= r_phase_cnt + 1;
                        end if;

                    when S_CS_SETUP =>
                        r_sclk <= '0';
                        if r_phase_cnt = C_WAIT_MAX - 1 then
                            r_phase_cnt <= 0;
                            r_half_cnt  <= 0;
                            r_bit_cnt   <= 0;
                            r_rx_shift  <= (others => '0');
                            r_state     <= S_SHIFT;
                        else
                            r_phase_cnt <= r_phase_cnt + 1;
                        end if;

                    when S_SHIFT =>
                        if r_half_cnt = C_SCLK_SWTCH_CNT - 1 then
                            r_half_cnt <= 0;

                            if r_sclk = '0' then
                                r_sclk <= '1';
                            else
                                r_sclk     <= '0';
                                r_rx_shift <= r_rx_shift(14 downto 0) & i_miso;

                                if r_bit_cnt = 15 then
                                    r_phase_cnt <= 0;
                                    r_state     <= S_CS_HOLD;
                                else
                                    r_bit_cnt  <= r_bit_cnt + 1;
                                    r_tx_shift <= r_tx_shift(14 downto 0) & '0';
                                end if;
                            end if;
                        else
                            r_half_cnt <= r_half_cnt + 1;
                        end if;

                    when S_CS_HOLD =>
                        r_sclk <= '0';
                        if r_phase_cnt = C_SCLK_SWTCH_CNT - 1 then
                            r_phase_cnt <= 0;
                            r_state     <= S_CHECK;
                        else
                            r_phase_cnt <= r_phase_cnt + 1;
                        end if;

                    when S_CHECK =>
                        if r_step = CMD_PHASE then
                            r_step      <= READ_PHASE;
                            r_tx_shift  <= C_READ_ANGLE_CMD;
                            r_phase_cnt <= 0;
                            r_state     <= S_CS_HIGH;

                        elsif r_step = READ_PHASE then
                            if not even_parity_ok(r_rx_shift) then
                                r_step      <= CMD_PHASE;
                                r_tx_shift  <= C_READ_ANGLE_CMD;
                                r_phase_cnt <= 0;
                                r_state     <= S_CS_HIGH;
                            elsif r_rx_shift(14) = '1' then
                                r_step      <= CLEAR_PHASE;
                                r_tx_shift  <= C_CLR_ERR_FL_CMD;
                                r_phase_cnt <= 0;
                                r_state     <= S_CS_HIGH;
                            else
                                r_angle <= unsigned(r_rx_shift(13 downto 0));
                                r_state <= S_DONE;
                            end if;

                        else
                            r_step      <= CMD_PHASE;
                            r_tx_shift  <= C_READ_ANGLE_CMD;
                            r_phase_cnt <= 0;
                            r_state     <= S_CS_HIGH;
                        end if;

                    when S_DONE =>
                        r_state <= S_IDLE;
                end case;
            end if;
        end if;
    end process;

end architecture rtl;
