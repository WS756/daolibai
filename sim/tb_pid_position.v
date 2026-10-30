`timescale 1ns/1ns

module tb_pid_position;

    reg clk, rst_n, tick, enable;
    reg signed [15:0] e1, e2;
    wire signed [15:0] u1, u2;

    pid_position #(
        .KP(16'sd256), .KI(16'sd256), .KD(16'sd0),
        .I_LIMIT(32'sd50), .ACC_LIMIT(32'sd1048576), .OUT_LIMIT(16'sd180)
    ) dut_pi(
        .clk(clk), .rst_n(rst_n), .tick(tick), .enable(enable),
        .e(e1), .u(u1)
    );

    pid_position #(
        .KP(16'sd256), .KI(16'sd0), .KD(16'sd256),
        .I_LIMIT(32'sd0), .ACC_LIMIT(32'sd1048576), .OUT_LIMIT(16'sd180)
    ) dut_pd(
        .clk(clk), .rst_n(rst_n), .tick(tick), .enable(enable),
        .e(e2), .u(u2)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    integer errs;

    task tick_pulse;
        begin
            tick = 1; #10; tick = 0;
        end
    endtask

    initial begin
        errs = 0;
        clk = 0; rst_n = 0; tick = 0; enable = 0; e1 = 0; e2 = 0;
        #30; rst_n = 1; enable = 1;

        e1 = 50; tick_pulse; #10;
        if (u1 !== 100) begin $display("PI t1 FAIL: u=%0d expect 100", u1); errs = errs+1; end

        tick_pulse; #10;
        if (u1 !== 100) begin $display("PI t2 FAIL: u=%0d expect 100", u1); errs = errs+1; end

        e1 = 0; tick_pulse; #10;
        if (u1 !== 50) begin $display("PI t3 FAIL: u=%0d expect 50", u1); errs = errs+1; end

        e1 = -50; tick_pulse; #10;
        if (u1 !== 0) begin $display("PI t4 FAIL: u=%0d expect 0", u1); errs = errs+1; end

        enable = 0; #10;
        if (u1 !== 0) begin $display("PI disable FAIL: u=%0d expect 0", u1); errs = errs+1; end
        enable = 1;
        e1 = 50; tick_pulse; #10;

        if (u1 !== 100) begin $display("PI re-enable FAIL: u=%0d expect 100", u1); errs = errs+1; end

        enable = 0; #10; enable = 1;

        e2 = 100; tick_pulse; #10;
        if (u2 !== 180) begin $display("PD p1 FAIL: u=%0d expect 180", u2); errs = errs+1; end

        e2 = 50; tick_pulse; #10;
        if (u2 !== 0) begin $display("PD p2 FAIL: u=%0d expect 0", u2); errs = errs+1; end

        enable = 0;
        e2 = -1926; #100;
        e2 = 0;     #100;
        enable = 1;
        tick_pulse; #10;

        if (u2 !== 0) begin $display("PD tracking FAIL: u=%0d expect 0", u2); errs = errs+1; end

        if (errs == 0) $display("tb_pid_position: ALL PASS");
        else           $display("tb_pid_position: %0d CHECK(S) FAILED", errs);
        $finish;
    end
endmodule
