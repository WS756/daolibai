module top(
    input  wire        clk_50m,
    input  wire        rst_n,
    input  wire [3:0]  key_sw,
    output wire        uart_tx,
    input  wire        uart_rx,
    output wire        adc_clk,
    output wire        adc_oe_n,
    input  wire [9:0]  adc_data_in,
    input  wire        enc1_a,
    input  wire        enc1_b,
    input  wire        enc2_a,
    input  wire        enc2_b,
    output wire        AN1,
    output wire        AN2,
    output wire        PWMA,
    output wire        BN1,
    output wire        BN2,
    output wire        PWMB,
    output wire [3:0]  led
);

    wire [11:0] theta;
    wire        sample_valid;
    wire [15:0] enc_pos;
    wire        key1_pulse, key2_pulse, key3_pulse, key4_pulse;
    wire        running;
    wire signed [15:0] command;
    wire signed [15:0] angle_error, pos_error;

    key_filter key1(.sys_clk(clk_50m), .sys_rst_n(rst_n), .key_in(key_sw[0]), .key_flag(key1_pulse));
    key_filter key2(.sys_clk(clk_50m), .sys_rst_n(rst_n), .key_in(key_sw[1]), .key_flag(key2_pulse));
    key_filter key3(.sys_clk(clk_50m), .sys_rst_n(rst_n), .key_in(key_sw[2]), .key_flag(key3_pulse));
    key_filter key4(.sys_clk(clk_50m), .sys_rst_n(rst_n), .key_in(key_sw[3]), .key_flag(key4_pulse));

    adc angle_adc(
        .clk_50m(clk_50m), .rst_n(rst_n),
        .adc_data_in(adc_data_in),
        .adc_clk(adc_clk), .adc_oe_n(adc_oe_n),
        .theta(theta), .sample_valid(sample_valid)
    );

    encoder arm_encoder(
        .clk(clk_50m), .rst_n(rst_n),
        .enc_a(enc1_a), .enc_b(enc1_b),
        .pos(enc_pos)
    );

    pendulum_ctrl controller(
        .clk_50m(clk_50m), .rst_n(rst_n),
        .start(key1_pulse), .calibrate(key2_pulse), .rotate_toggle(key3_pulse), .stop(key4_pulse),
        .sample_valid(sample_valid),
        .theta_in(theta), .enc_pos(enc_pos),
        .running(running), .command(command),
        .angle_error_debug(angle_error), .pos_error_debug(pos_error)
    );

    motor_driver motor(
        .clk(clk_50m), .rst_n(rst_n),
        .enable(running), .command(command),
        .in1(BN1), .in2(BN2), .pwm(PWMB)
    );

    assign AN1  = 1'b0;
    assign AN2  = 1'b0;
    assign PWMA = 1'b0;

    wire [15:0] displayed_angle = {4'd0, theta} + 16'd135;
    telemetry monitor(
        .clk(clk_50m), .rst_n(rst_n),
        .angle(displayed_angle), .encoder({enc_pos, 16'd0}),
        .tx(uart_tx)
    );

    reg [24:0] heartbeat;
    always @(posedge clk_50m or negedge rst_n)
        if (!rst_n) heartbeat <= 0;
        else        heartbeat <= heartbeat + 1'b1;

    reg [11:0] theta_prev;
    reg [23:0] alive_timer;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            theta_prev  <= 0;
            alive_timer <= 0;
        end else begin
            if ((theta > theta_prev + 12'd20) || (theta < theta_prev - 12'd20)) begin
                theta_prev  <= theta;
                alive_timer <= 24'd5000000;
            end else if (alive_timer != 0)
                alive_timer <= alive_timer - 1'b1;
        end
    end

    assign led[0] = running;
    assign led[1] = (alive_timer != 0);
    assign led[2] = running && (command != 0);
    assign led[3] = heartbeat[24];
endmodule
