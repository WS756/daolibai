module telemetry #(
    parameter integer FRAME_PERIOD = 50_000_000,
    parameter integer BYTE_GAP     = 60000
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] angle,
    input  wire [31:0] encoder,
    output wire        tx
);
    reg [25:0] frame_timer;
    reg [16:0] byte_timer;
    reg [3:0]  byte_idx;
    reg        sending;
    reg [15:0] angle_l;
    reg [31:0] enc_l;
    reg [7:0]  byte_data;
    reg        byte_send;
    wire       busy;

    wire [7:0] sel_byte =
        (byte_idx == 0) ? 8'hAA :
        (byte_idx == 1) ? 8'h55 :
        (byte_idx == 2) ? angle_l[15:8] :
        (byte_idx == 3) ? angle_l[7:0] :
        (byte_idx == 4) ? enc_l[31:24] :
        (byte_idx == 5) ? enc_l[23:16] :
        (byte_idx == 6) ? enc_l[15:8] :
        (byte_idx == 7) ? enc_l[7:0] :
        (byte_idx == 8) ? 8'h0D : 8'h0A;

    uart_tx #(.CLK_FREQ(50_000_000), .BAUD(9600)) sender(
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
            enc_l       <= 0;
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
                    enc_l    <= encoder;
                end else
                    frame_timer <= frame_timer + 1'b1;
            end else begin
                if (byte_timer == 0) begin
                    byte_data <= sel_byte;
                    byte_send <= 1;
                    byte_timer <= 1;
                end else if (byte_timer == BYTE_GAP-1) begin
                    byte_timer <= 0;
                    if (byte_idx == 9)
                        sending <= 0;
                    else
                        byte_idx <= byte_idx + 1'b1;
                end else
                    byte_timer <= byte_timer + 1'b1;
            end
        end
    end
endmodule
