module key_filter #(
    parameter integer CNT_MAX = 1000000
)(
    input  wire        sys_clk,
    input  wire        sys_rst_n,
    input  wire        key_in,
    output reg         key_flag
);
    reg [1:0]  sync;
    reg [19:0] count;
    reg        held;

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            sync <= 2'b11;
            count <= 0;
            held <= 1'b1;
            key_flag <= 0;
        end else begin
            sync <= {sync[0], key_in};
            key_flag <= 0;
            if (sync[1]) begin
                count <= 0;
                held  <= 1'b0;
            end else if (!held) begin
                if (count == CNT_MAX-1) begin
                    key_flag <= 1'b1;
                    held     <= 1'b1;
                end else
                    count <= count + 1'b1;
            end
        end
    end
endmodule
