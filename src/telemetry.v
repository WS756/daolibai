module telemetry #(
    parameter integer FRAME_PERIOD = 1_000_000,
    parameter integer BYTE_GAP     = 6000
)(
    input  wire             clk,
    input  wire             rst_n,
    input  wire             running,
    input  wire [11:0]      angle,
    input  wire signed [15:0] error,
    input  wire signed [15:0] pos_error,
    input  wire signed [15:0] command,
    output wire             tx
);
    reg [19:0] frame_timer;
    reg [15:0] byte_timer;
    reg [3:0]  byte_idx;
    reg        sending;
    reg [15:0] angle_l, err_l, perr_l, cmd_l;
    reg        run_l;
    reg [7:0]  byte_data;
    reg        byte_send;
    wire       busy;

    wire [7:0] sel_byte =
        (byte_idx == 0)  ? 8'hAA :
        (byte_idx == 1)  ? 8'h55 :
        (byte_idx == 2)  ? angle_l[11:8] :
        (byte_idx == 3)  ? angle_l[7:0] :
        (byte_idx == 4)  ? err_l[15:8] :
        (byte_idx == 5)  ? err_l[7:0] :
        (byte_idx == 6)  ? perr_l[15:8] :
        (byte_idx == 7)  ? perr_l[7:0] :
        (byte_idx == 8)  ? cmd_l[15:8] :
        (byte_idx == 9)  ? cmd_l[7:0] :
        (byte_idx == 10) ? {7'd0, run_l} :
        (byte_idx == 11) ? 8'h0A : 8'h0D;

    uart_tx sender(
        .clk(clk), .rst_n(rst_n),
        .data(byte_data), .send(byte_send),
        .tx(tx), .busy(busy)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_timer <= 0;
            byte_timer  <= 0;
            byte_idx    <= 0;
            sending     <= 0;
            angle_l     <= 0;
            err_l       <= 0;
            perr_l      <= 0;
            cmd_l       <= 0;
            run_l       <= 0;
            byte_data   <= 0;
            byte_send   <= 0;
        end else begin
            byte_send <= 0;
            if (!sending) begin
                byte_timer <= 0;
                if (frame_timer == FRAME_PERIOD-1) begin
                    frame_timer <= 0;
                    sending  <= 1;
                    byte_idx <= 0;
                    angle_l  <= angle;
                    err_l    <= error;
                    perr_l   <= pos_error;
                    cmd_l    <= command;
                    run_l    <= running;
                end else
                    frame_timer <= frame_timer + 1'b1;
            end else begin
                if (byte_timer == 0) begin
                    byte_data  <= sel_byte;
                    byte_send  <= 1;
                    byte_timer <= 1;
                end else if (byte_timer == BYTE_GAP-1) begin
                    byte_timer <= 0;
                    if (byte_idx == 11)
                        sending <= 0;
                    else
                        byte_idx <= byte_idx + 1'b1;
                end else
                    byte_timer <= byte_timer + 1'b1;
            end
        end
    end
endmodule
