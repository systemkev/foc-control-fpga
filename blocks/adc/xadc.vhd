library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.fixed_pkg.all;

library work;
use work.common_pkg.all;

entity motor_current_reader is
    port (
        i_clk                : in  std_logic;
        i_reset              : in  std_logic;
        i_pwm_center_trigger : in  std_logic;
        
        i_top_pin_vauxp4   : in  std_logic;
        i_top_pin_vauxn4   : in  std_logic;
        i_top_pin_vauxp6   : in  std_logic;
        i_top_pin_vauxn6   : in  std_logic;
        
        o_phases            : out t_abc_phase
    );
end motor_current_reader;

architecture behavioral of motor_current_reader is

    component xadc_wiz_0 is
        port (
            daddr_in        : in  std_logic_vector (6 downto 0);
            den_in          : in  std_logic;
            di_in           : in  std_logic_vector (15 downto 0);
            dwe_in          : in  std_logic;
            do_out          : out std_logic_vector (15 downto 0);
            drdy_out        : out std_logic;
            dclk_in         : in  std_logic;
            reset_in        : in  std_logic;
            convst_in       : in  std_logic;
            vauxp4          : in  std_logic;
            vauxn4          : in  std_logic;
            vauxp6          : in  std_logic;
            vauxn6          : in  std_logic;
            busy_out        : out std_logic;
            channel_out     : out std_logic_vector (4 downto 0);
            eoc_out         : out std_logic;
            eos_out         : out std_logic;
            alarm_out       : out std_logic;
            vp_in           : in  std_logic;
            vn_in           : in  std_logic
        );
    end component;

    signal daddr_signal    : std_logic_vector(6 downto 0);
    signal den_signal      : std_logic;
    signal data_out_signal : std_logic_vector(15 downto 0);
    signal drdy_signal     : std_logic;
    signal channel_signal  : std_logic_vector(4 downto 0);
    signal eoc_signal      : std_logic;

    signal adc_val_a       : unsigned(15 downto 0) := (others => '0');
    signal adc_val_b       : unsigned(15 downto 0) := (others => '0');
    
    signal calc_a          : unsigned(31 downto 0) := (others => '0');
    signal calc_b          : unsigned(31 downto 0) := (others => '0');
    
    signal current_a       : signed(15 downto 0)   := (others => '0');
    signal current_b       : signed(15 downto 0)   := (others => '0');
    signal current_c       : signed(15 downto 0)   := (others => '0');

begin
    
    u_xadc : xadc_wiz_0
    port map (
        daddr_in        => daddr_signal,
        den_in          => den_signal,
        di_in           => (others => '0'),
        dwe_in          => '0',
        do_out          => data_out_signal,
        drdy_out        => drdy_signal,
        dclk_in         => i_clk,
        reset_in        => i_reset,
        convst_in       => i_pwm_center_trigger,
        vauxp4          => i_top_pin_vauxp4,
        vauxn4          => i_top_pin_vauxn4,
        vauxp6          => i_top_pin_vauxp6,
        vauxn6          => i_top_pin_vauxn6,
        busy_out        => open,
        channel_out     => channel_signal,
        eoc_out         => eoc_signal,
        eos_out         => open,
        alarm_out       => open,
        vp_in           => '0',
        vn_in           => '0'
    );

    daddr_signal <= "00" & channel_signal;
    den_signal   <= eoc_signal;

    o_phases.A <= to_sfixed(std_logic_vector(current_a), 3, -12);
    o_phases.B <= to_sfixed(std_logic_vector(current_b), 3, -12);
    o_phases.C <= to_sfixed(std_logic_vector(current_c), 3, -12);

    process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_reset = '1' then
                adc_val_a <= (others => '0');
                adc_val_b <= (others => '0');
                current_a <= (others => '0');
                current_b <= (others => '0');
                current_c <= (others => '0');
            else
                if drdy_signal = '1' then
                    if daddr_signal = "0010100" then      
                        adc_val_a <= unsigned(data_out_signal);
                    elsif daddr_signal = "0010110" then   
                        adc_val_b <= unsigned(data_out_signal);
                    end if;
                end if;
                
                calc_a <= adc_val_a * to_unsigned(27034, 16);
                calc_b <= adc_val_b * to_unsigned(27034, 16);
                
                current_a <= signed(calc_a(31 downto 16)) - to_signed(13517, 16);
                current_b <= signed(calc_b(31 downto 16)) - to_signed(13517, 16);
                
                current_c <= to_signed(0, 16) - current_a - current_b;
                
            end if;
        end if;
    end process;

end architecture behavioral;