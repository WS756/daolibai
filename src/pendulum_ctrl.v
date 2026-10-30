module pendulum_ctrl #(

    parameter signed [15:0] ANG_KP = 16'sd384,
    parameter signed [15:0] ANG_KI = 16'sd0,
    parameter signed [15:0] ANG_KD = 16'sd3072,

    parameter signed [15:0] POS_KP = 16'sd32,
    parameter signed [15:0] POS_KI = 16'sd0,
    parameter signed [15:0] ARM_DAMP = 16'sd64,

    parameter [15:0] ONE_REV     = 16'd1040,
    parameter [15:0] ROTATE_STEP = 16'd1,
    parameter [15:0] DECEL_COUNTS = 16'd200,

    parameter [11:0] ANGLE_SETPOINT = 12'd2925,

    parameter signed [15:0] ANGLE_POLARITY = 16'sd1,

    parameter signed [15:0] OUT_LIMIT    = 16'sd120,
    parameter signed [15:0] POS_OUT_LIMIT = 16'sd150,
    parameter signed [31:0] ANG_I_LIMIT  = 32'sd24,
    parameter signed [31:0] POS_I_LIMIT  = 32'sd24,
    parameter signed [15:0] FALL_ANGLE   = 16'sd1500,
    parameter integer       FALL_TICKS   = 50
)(
    input  wire        clk_50m,
    input  wire        rst_n,
    input  wire        start,
    input  wire        calibrate,
    input  wire        stop,
    input  wire        rotate_toggle,
    input  wire        sample_valid,
    input  wire [11:0] theta_in,
    input  wire [15:0] enc_pos,
    output wire        running,
    output reg  signed [15:0] command,
    output wire signed [15:0] angle_error_debug,
    output wire signed [15:0] pos_error_debug
);

    reg [11:0] setpoint;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            setpoint <= ANGLE_SETPOINT;
        else if (calibrate && !running)
            setpoint <= theta_in;
    end

    localparam IDLE = 1'b0, BALANCE = 1'b1;
    reg        state;
    reg        rotate_mode;
    reg [15:0] spin_remaining;
    reg        spin_phase;
    reg        rot_phase;
    reg [15:0] pos_setpoint;
    reg [15:0] fall_count;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            rotate_mode <= 0;
            spin_remaining <= 0;
            spin_phase <= 0;
            rot_phase <= 0;
            pos_setpoint <= 0;
            fall_count <= 0;
        end else begin
            if (stop) begin
                state <= IDLE;
                rotate_mode <= 0;
                spin_remaining <= 0;
                spin_phase <= 0;
                rot_phase <= 0;
                fall_count <= 0;
            end else if (start) begin
                state <= BALANCE;
                rotate_mode <= 0;
                spin_remaining <= 0;
                spin_phase <= 0;
                rot_phase <= 0;
                pos_setpoint <= enc_pos;
                fall_count <= 0;
            end else if (state == BALANCE) begin

                if (calibrate)
                    spin_remaining <= ONE_REV;

                if (rotate_toggle)
                    rotate_mode <= ~rotate_mode;
                if (sample_valid) begin

                    rot_phase <= ~rot_phase;

                    if (spin_remaining > 0) begin
                        if (spin_remaining > DECEL_COUNTS) begin

                            if (rot_phase) begin
                                pos_setpoint <= pos_setpoint + ROTATE_STEP;
                                spin_remaining <= spin_remaining - ROTATE_STEP;
                            end
                            spin_phase <= 0;
                        end else begin

                            spin_phase <= ~spin_phase;
                            if (spin_phase && rot_phase) begin
                                pos_setpoint <= pos_setpoint + ROTATE_STEP;
                                spin_remaining <= spin_remaining - ROTATE_STEP;
                            end
                        end
                    end else if (rotate_mode) begin
                        if (rot_phase)
                            pos_setpoint <= pos_setpoint + ROTATE_STEP;
                    end
                    if (falling) begin
                        if (fall_count >= FALL_TICKS-1) begin
                            state <= IDLE;
                            rotate_mode <= 0;
                            spin_remaining <= 0;
                            spin_phase <= 0;
                            rot_phase <= 0;
                            fall_count <= 0;
                        end else
                            fall_count <= fall_count + 1'b1;
                    end else
                        fall_count <= 0;
                end
            end
        end
    end

    assign running = (state == BALANCE);

    wire signed [31:0] pos_raw = $signed({16'd0, pos_setpoint}) - $signed({16'd0, enc_pos});
    wire signed [31:0] pos_err = (pos_raw >  32'sd32768) ? pos_raw - 32'sd65536 :
                                 (pos_raw < -32'sd32768) ? pos_raw + 32'sd65536 : pos_raw;

    wire signed [15:0] pos_u;
    pid_position #(
        .KP(POS_KP), .KI(POS_KI), .KD(16'sd0),
        .I_LIMIT(POS_I_LIMIT), .OUT_LIMIT(POS_OUT_LIMIT)
    ) position_pid (
        .clk(clk_50m), .rst_n(rst_n),
        .tick(sample_valid), .enable(running),
        .e(pos_err[15:0]), .u(pos_u)
    );

    wire signed [17:0] tilt_target = $signed({6'd0, setpoint}) + pos_u;

    wire signed [31:0] ang_raw = tilt_target - $signed({20'd0, theta_in});
    wire signed [31:0] ang_err = (ang_raw >  32'sd2048) ? ang_raw - 32'sd4096 :
                                 (ang_raw < -32'sd2048) ? ang_raw + 32'sd4096 : ang_raw;

    wire signed [31:0] ang_abs = (ang_err < 0) ? -ang_err : ang_err;
    wire falling = (ang_abs > FALL_ANGLE);

    wire signed [15:0] ang_u;
    pid_position #(
        .KP(ANG_KP), .KI(ANG_KI), .KD(ANG_KD),
        .I_LIMIT(ANG_I_LIMIT), .OUT_LIMIT(OUT_LIMIT)
    ) angle_pid (
        .clk(clk_50m), .rst_n(rst_n),
        .tick(sample_valid), .enable(running),
        .e(ang_err[15:0]), .u(ang_u)
    );

    reg [15:0] enc_prev;
    reg signed [31:0] vel_q8;
    wire signed [31:0] enc_vel_now = $signed({enc_pos[15], enc_pos}) - $signed({enc_prev[15], enc_prev});
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            enc_prev <= 0;
            vel_q8   <= 0;
        end else if (sample_valid) begin
            vel_q8   <= vel_q8 + (((enc_vel_now << 8) - vel_q8) / 32'sd8);
            enc_prev <= enc_pos;
        end
    end

    wire signed [16:0] ang_polar = (ANGLE_POLARITY < 0) ? -$signed({ang_u[15], ang_u})
                                                      :  $signed({ang_u[15], ang_u});
    wire signed [16:0] arm_damp  = - (($signed(ARM_DAMP) * vel_q8) >>> 8);
    wire signed [16:0] cmd_sum   = ang_polar + arm_damp;
    wire signed [16:0] cmd_clip  = (cmd_sum >  OUT_LIMIT) ? OUT_LIMIT :
                                   (cmd_sum < -OUT_LIMIT) ? -OUT_LIMIT : cmd_sum;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)        command <= 0;
        else if (!running) command <= 0;
        else               command <= cmd_clip[15:0];
    end

    assign angle_error_debug = ang_err[15:0];
    assign pos_error_debug   = pos_err[15:0];
endmodule
