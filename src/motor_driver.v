module motor_driver #(
    parameter integer       PWM_PERIOD = 2500,
    parameter signed [15:0] CMD_LIMIT  = 16'sd255
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              enable,
    input  wire signed [15:0] command,
    output wire              in1,
    output wire              in2,
    output wire              pwm
);
    reg [11:0] counter;
    reg        dir;
    reg [7:0]  duty;

    wire signed [15:0] cmd_lim = (command >  CMD_LIMIT) ? CMD_LIMIT :
                                 (command < -CMD_LIMIT) ? -CMD_LIMIT : command;
    wire signed [15:0] mag = (cmd_lim < 0) ? -cmd_lim : cmd_lim;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 0;
            dir     <= 0;
            duty    <= 0;
        end else if (!enable) begin
            counter <= 0;
            dir     <= 0;
            duty    <= 0;
        end else begin
            if (counter == PWM_PERIOD-1) begin
                counter <= 0;
                dir  <= (cmd_lim >= 0);
                duty <= mag[7:0];
            end else
                counter <= counter + 1'b1;
        end
    end

    wire        stopped   = (duty == 8'd0);
    wire [31:0] product   = duty * PWM_PERIOD;
    wire [11:0] threshold = product[19:8];

    assign in1 = enable && dir     && !stopped;
    assign in2 = enable && (!dir)  && !stopped;
    assign pwm = enable && !stopped && (counter < threshold);
endmodule
