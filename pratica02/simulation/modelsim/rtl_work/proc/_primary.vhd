library verilog;
use verilog.vl_types.all;
entity proc is
    port(
        Clock           : in     vl_logic;
        DIN             : in     vl_logic_vector(15 downto 0);
        Reset           : in     vl_logic;
        R0_output       : out    vl_logic_vector(15 downto 0);
        R1_output       : out    vl_logic_vector(15 downto 0);
        R2_output       : out    vl_logic_vector(15 downto 0);
        R3_output       : out    vl_logic_vector(15 downto 0);
        R4_output       : out    vl_logic_vector(15 downto 0);
        R5_output       : out    vl_logic_vector(15 downto 0);
        R6_output       : out    vl_logic_vector(15 downto 0);
        R7_output       : out    vl_logic_vector(15 downto 0);
        BusWires        : out    vl_logic_vector(15 downto 0);
        Done            : out    vl_logic
    );
end proc;
