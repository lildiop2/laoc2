module memory_hierarchy (
    input clk,
    input rst,
    input [7:0] addr_in,
    input [7:0] data_in,
    output reg [7:0] data_out,
    output reg hit,
    output reg miss,
    output reg dirty
);

    parameter N = 4;
    parameter M = 32;

    reg [7:0] cache [0:N-1];
    reg [7:0] mem [0:M-1];
    reg [7:0] addr_cache;
    reg [7:0] data_cache;
    reg [7:0] data_mem;
    reg [7:0] dirty_cache;
    reg [7:0] valid_cache;
    reg [7:0] lru_cache;
    reg [7:0] lru_mem;

    always @(posedge clk) begin
        if (rst) begin
            addr_cache <= 0;
            data_cache <= 0;
            dirty_cache <= 0;
            valid_cache <= 0;
            lru_cache <= 0;
        end else begin
            if (hit) begin
                data_out <= data_cache;
            end else begin
                addr_mem <= addr_cache;
                data_mem <= mem[addr_mem];
                dirty_cache <= valid_cache;
                valid_cache <= 0;
                lru_cache <= 0;
                for (int i = 0; i < N; i++) begin
                    if (cache[i] == addr_cache) begin
                        lru_cache <= i;
                        break;
                    end
                end
                mem[addr_mem] <= data_in;
            end
        end
    end

    always @(posedge clk) begin
        if (hit) begin
            hit <= 1;
            miss <= 0;
        end else begin
            hit <= 0;
            miss <= 1;
        end
        if (dirty_cache) begin
            dirty <= 1;
        end else begin
            dirty <= 0;
        end
    end

    assign data_out = cache[lru_cache];

endmodule
