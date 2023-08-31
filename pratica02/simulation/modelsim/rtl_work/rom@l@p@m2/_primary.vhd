library verilog;
use verilog.vl_types.all;
entity romLPM2 is
    port(
        address         : in     vl_logic_vector(4 downto 0);
        clock           : in     vl_logic;
        q               : out    vl_logic_vector(15 downto 0)
    );
end romLPM2;
