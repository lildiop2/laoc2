library verilog;
use verilog.vl_types.all;
entity CounterPC is
    port(
        Clock           : in     vl_logic;
        Clear           : in     vl_logic;
        Counter         : out    vl_logic_vector(4 downto 0)
    );
end CounterPC;
