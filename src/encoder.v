module encoder(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enc_a,
    input  wire        enc_b,
    output reg  [15:0] pos
);
    reg [1:0] a_sync, b_sync;
    reg [1:0] state_now, state_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 0;
            b_sync <= 0;
            state_now <= 0;
        end else begin
            a_sync <= {a_sync[0], enc_a};
            b_sync <= {b_sync[0], enc_b};
            state_now <= {a_sync[1], b_sync[1]};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_last <= 0;
            pos <= 16'd0;
        end else begin
            state_last <= state_now;
            case ({state_last, state_now})

                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: pos <= pos - 1'b1;

                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: pos <= pos + 1'b1;
                default: pos <= pos;
            endcase
        end
    end
endmodule
