library verilog;
use verilog.vl_types.all;
entity RegIR is
    port(
        Clock           : in     vl_logic;
        Rin             : in     vl_logic;
        \Bus\           : in     vl_logic_vector(15 downto 0);
        R_output        : out    vl_logic_vector(9 downto 0)
    );
end RegIR;
