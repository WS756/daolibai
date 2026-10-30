module adc #(
    parameter integer SAMPLE_PERIOD = 3125,
    parameter integer AVG_N         = 32
)(
    input  wire        clk_50m,
    input  wire        rst_n,
    input  wire [9:0]  adc_data_in,
    output wire        adc_clk,
    output wire        adc_oe_n,
    output reg  [11:0] theta,
    output reg         sample_valid
);
    reg        clk_div;
    reg [9:0]  captured;
    reg [11:0] sample_timer;
    reg [4:0]  cnt;
    reg [16:0] sum;
    reg        sample_ready;

    assign adc_clk  = clk_div;
    assign adc_oe_n = 1'b0;

    wire [16:0] sum_next = sum + {7'd0, captured};

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            clk_div      <= 0;
            captured     <= 0;
            sample_timer <= 0;
            cnt          <= 0;
            sum          <= 0;
            theta        <= 0;
            sample_valid <= 0;
            sample_ready <= 0;
        end else begin
            clk_div <= ~clk_div;
            sample_ready <= 0;

            sample_valid <= sample_ready;

            if (!clk_div)
                captured <= adc_data_in;

            if (sample_timer == SAMPLE_PERIOD-1) begin
                sample_timer <= 0;
                if (cnt == AVG_N-1) begin
                    cnt   <= 0;
                    sum   <= 0;
                    theta <= {sum_next[16:5], 2'b00};
                    sample_ready <= 1;
                end else begin
                    cnt <= cnt + 1'b1;
                    sum <= sum_next;
                end
            end else begin
                sample_timer <= sample_timer + 1'b1;
            end
        end
    end
endmodule
