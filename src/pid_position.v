module pid_position #(
    parameter signed [15:0] KP       = 16'sd256,
    parameter signed [15:0] KI       = 16'sd1,
    parameter signed [15:0] KD       = 16'sd4096,
    parameter signed [31:0] I_LIMIT   = 32'sd24,
    parameter signed [31:0] ACC_LIMIT = 32'sd1048576,
    parameter signed [15:0] OUT_LIMIT = 16'sd180
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire              tick,
    input  wire              enable,
    input  wire signed [15:0] e,
    output reg  signed [15:0] u
);

    reg signed [31:0] acc;
    reg signed [15:0] last_e;

    wire signed [16:0] de = $signed({e[15], e}) - $signed({last_e[15], last_e});

    wire signed [31:0] p_term = (KP * e) >>> 8;

    localparam signed [31:0] ACC_HARD = (KI > 0) ? ((I_LIMIT << 8) / KI) : ACC_LIMIT;

    wire signed [31:0] acc_next = acc + e;
    wire signed [31:0] acc_clip = (acc_next >  ACC_HARD) ?  ACC_HARD :
                                  (acc_next < -ACC_HARD) ? -ACC_HARD : acc_next;

    wire signed [47:0] i_full = ($signed(KI) * acc_clip) >>> 8;
    wire signed [31:0] i_term = (i_full >  I_LIMIT) ?  I_LIMIT :
                                (i_full < -I_LIMIT) ? -I_LIMIT : i_full[31:0];

    wire signed [31:0] d_term = ($signed(KD) * de) >>> 8;

    wire signed [31:0] u_raw  = p_term + i_term + d_term;
    wire signed [31:0] u_clip = (u_raw >  OUT_LIMIT) ?  OUT_LIMIT :
                                (u_raw < -OUT_LIMIT) ? -OUT_LIMIT : u_raw;

    wire freeze = ((u_raw >  OUT_LIMIT) && (e > 0)) ||
                  ((u_raw < -OUT_LIMIT) && (e < 0));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc    <= 0;
            last_e <= 0;
            u      <= 0;
        end else if (!enable) begin
            acc    <= 0;
            last_e <= e;
            u      <= 0;
        end else if (tick) begin
            if (!freeze) acc <= acc_clip;
            last_e <= e;
            u      <= u_clip[15:0];
        end
    end
endmodule
