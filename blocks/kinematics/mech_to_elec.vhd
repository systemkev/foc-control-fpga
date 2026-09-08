library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.common_pkg.all;

entity mech_to_elec is
    port (
        -- System
        i_clk           : in std_logic;
        i_rst           : in std_logic;

        -- Inputs 
        i_vld           : in std_logic;
        i_mech_pos      : in t_angl_raw_unwr;   
        i_offset        : in t_angl_raw_unwr;

        -- Outputs 
        o_vld           : out std_logic;
        o_elec_pos      : out t_angl_raw_unwr;
        o_elec_cordic   : out t_cordic_inputs
    );
end entity mech_to_elec;

architecture rtl of mech_to_elec is
    signal r_elec_pos   : t_angl_raw_unwr;
    signal r_elec_vld   : std_logic;
begin
    
    -- elec = (mech * POLE PAIRS + offset)
    mech_to_elec: process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                r_elec_pos <= (others => '0');
                r_elec_vld <= '0';
            elsif i_vld = '1' then 
                r_elec_pos <= resize(to_unsigned(C_NUM_POLE_PAIRS, 4) * i_mech_pos + i_offset, r_elec_pos);
                r_elec_vld <= '1';
            else 
                r_elec_vld <= '0';
            end if;
        end if;
    end process mech_to_elec;

    -- currently, the angle is stored in the form
    -- 0 => 2**14 - 1 (0 => 360 deg)
    -- For the cordic, we must convert to
    -- -2**15 => 2**15-1 (-180 => 179.99 deg)
    -- To do this, subtract 2**13 (to center it)
    -- then multiply by 4 (shift left by 2)
    elec_to_cordic: process(i_clk)
    begin
        if rising_edge(i_clk) then
            if i_rst = '1' then
                o_vld         <= '0';
            elsif r_elec_vld = '1' then 
                o_elec_cordic <= signed((not r_elec_pos(13)) & r_elec_pos(12 downto 0) & "00");
                o_elec_pos    <= r_elec_pos;
                o_vld         <= '1';
            else 
                o_vld         <= '0';
            end if;
        end if;
    end process elec_to_cordic;
    
end architecture rtl;