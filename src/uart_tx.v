module uart_tx #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD     = 115200
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  data,
    input  wire        send,
    output reg         tx,
    output wire        busy
);
    localparam integer DIV = CLK_FREQ / BAUD;

    reg [12:0] cnt;
    reg [3:0]  bit_idx;
    reg        active;
    reg [7:0]  shifter;

    assign busy = active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt     <= 0;
            bit_idx <= 0;
            active  <= 0;
            shifter <= 0;
            tx      <= 1'b1;
        end else begin
            if (!active) begin
                tx <= 1'b1;
                if (send) begin
                    active  <= 1'b1;
                    shifter <= data;
                    bit_idx <= 0;
                    cnt     <= 0;
                end
            end else begin
                if (cnt == DIV-1) begin
                    cnt <= 0;
                    bit_idx <= bit_idx + 1'b1;
                    case (bit_idx)
                        4'd0:      tx <= 1'b0;
                        4'd1,4'd2,4'd3,4'd4,
                        4'd5,4'd6,4'd7,4'd8: tx <= shifter[bit_idx-1];
                        4'd9: begin tx <= 1'b1; active <= 0; end
                        default:  tx <= 1'b1;
                    endcase
                end else
                    cnt <= cnt + 1'b1;
            end
        end
    end
endmodule
