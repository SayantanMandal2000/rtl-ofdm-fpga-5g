//==============================================================================
// tb_ofdm_top.v  -  system testbench for ofdm_top
//
//  ofdm_top.tx_*  ->  channel_model (per-symbol phase, noise, unknown delay)
//                 ->  ofdm_top.rx_*
//
//  The testbench plays the role of the ARM processor: it computes the 14
//  phase-compensation coefficients and writes them through AXI-Lite.
//
//  Test 1 : chaos ON , phase compensation OFF  -> constellation spins, BER > 0
//           (paper Sec. III-A: EVM too high without phase correction)
//  Test 2 : chaos ON , phase compensation ON   -> BER = 0, low EVM
//  Test 3 : RX key != TX key (eavesdropper)    -> BER ~ 50 %   (security)
//  Test 4 : chaos OFF, phase compensation ON   -> BER = 0 (plain OFDM)
//==============================================================================
`timescale 1ns / 1ps
module tb_ofdm_top;
    localparam N = 128, CP = 32, W = 16, NSYM = 14;
    localparam real PI = 3.14159265358979;

    reg clk = 1'b0, rst_n = 1'b0;
    always #5 clk = ~clk;                       // 100 MHz

    // AXI-Lite
    reg  [7:0]  awaddr = 0, araddr = 0;
    reg         awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
    reg  [31:0] wdata = 0;
    reg  [3:0]  wstrb = 4'hF;
    wire        awready, wready, bvalid, arready, rvalid;
    wire [1:0]  bresp, rresp;
    wire [31:0] rdata;

    // streams
    wire                tx_valid, tx_sof, tx_sos;
    wire signed [W-1:0] tx_i, tx_q;
    wire [3:0]          tx_sym_idx;
    wire                rx_valid;
    wire signed [W-1:0] rx_i, rx_q;
    wire                led_locked, led_ok, dbg_valid;
    wire signed [W-1:0] dbg_i, dbg_q;
    reg                 idle_noise_en = 1'b0;

    ofdm_top #(.N(N), .LOGN(7), .CP(CP), .W(W), .NSYM(NSYM), .SLOTS(20)) dut (
        .clk(clk), .rst_n(rst_n),
        .s_axil_awaddr(awaddr), .s_axil_awvalid(awvalid), .s_axil_awready(awready),
        .s_axil_wdata(wdata), .s_axil_wstrb(wstrb), .s_axil_wvalid(wvalid), .s_axil_wready(wready),
        .s_axil_bresp(bresp), .s_axil_bvalid(bvalid), .s_axil_bready(bready),
        .s_axil_araddr(araddr), .s_axil_arvalid(arvalid), .s_axil_arready(arready),
        .s_axil_rdata(rdata), .s_axil_rresp(rresp), .s_axil_rvalid(rvalid), .s_axil_rready(rready),
        .tx_valid(tx_valid), .tx_i(tx_i), .tx_q(tx_q), .tx_sof(tx_sof), .tx_sos(tx_sos),
        .tx_sym_idx(tx_sym_idx),
        .rx_valid(rx_valid), .rx_i(rx_i), .rx_q(rx_q),
        .led_rx_locked(led_locked), .led_ber_ok(led_ok),
        .dbg_valid(dbg_valid), .dbg_i(dbg_i), .dbg_q(dbg_q));

    channel_model #(.W(W), .NOISE(8), .NSYM(NSYM)) u_ch (
        .clk(clk), .idle_noise_en(idle_noise_en),
        .in_valid(tx_valid), .in_re(tx_i), .in_im(tx_q), .in_sym_idx(tx_sym_idx),
        .out_valid(rx_valid), .out_re(rx_i), .out_im(rx_q));

    // ------------------------------------------------------------ AXI tasks --
    task axi_write(input [7:0] a, input [31:0] d);
        begin
            @(negedge clk);
            awaddr = a; awvalid = 1; wdata = d; wvalid = 1; bready = 1;
            fork
                begin wait (awready); @(negedge clk); awvalid = 0; end
                begin wait (wready);  @(negedge clk); wvalid  = 0; end
            join
            wait (bvalid); @(negedge clk); bready = 0;
        end
    endtask

    task axi_read(input [7:0] a, output [31:0] d);
        begin
            @(negedge clk);
            araddr = a; arvalid = 1; rready = 1;
            wait (arready); @(negedge clk); arvalid = 0;
            wait (rvalid); d = rdata; @(negedge clk); rready = 0;
        end
    endtask

    // --------------------------------------------------------- EVM monitor --
    real evm_err, evm_ref;
    integer evm_n;
    integer fd_const = 0, test_no = 0;   // constellation dump: test,I,Q
    function real nearest; input real v;
        begin
            if      (v < -4096.0) nearest = -6144.0;
            else if (v <  0.0)    nearest = -2048.0;
            else if (v <  4096.0) nearest =  2048.0;
            else                  nearest =  6144.0;
        end
    endfunction
    always @(posedge clk) if (dbg_valid) begin : evm_acc
        real ri, rq;
        ri = nearest(dbg_i); rq = nearest(dbg_q);
        evm_err = evm_err + (dbg_i - ri) * (dbg_i - ri) + (dbg_q - rq) * (dbg_q - rq);
        evm_ref = evm_ref + ri * ri + rq * rq;
        evm_n   = evm_n + 1;
        if (fd_const != 0 && evm_n <= 1024) $fdisplay(fd_const, "%0d,%0d,%0d", test_no, dbg_i, dbg_q);
    end

    // ----------------------------------------------------- helper routines --
    reg [31:0] rd, slots0, bits, errs, peak;
    reg [31:0] ctrl_shadow;
    integer    fails = 0;

    task wait_slots(input integer n);
        begin
            axi_read(8'h20, slots0);
            rd = slots0;
            while (rd < slots0 + n) begin
                repeat (500) @(posedge clk);
                axi_read(8'h20, rd);
            end
        end
    endtask

    // measure over n complete slots: flush one slot, clear counters, run n slots
    task measure(input [8*64-1:0] name, input integer n);
        real evm_pct, ber;
        begin
            wait_slots(1);
            axi_write(8'h00, ctrl_shadow | 32'h8);          // BER clear pulse
            evm_err = 0.0; evm_ref = 0.0; evm_n = 0;
            wait_slots(n);
            axi_read(8'h18, bits);
            axi_read(8'h1C, errs);
            axi_read(8'h24, peak);
            evm_pct = (evm_ref > 0.0) ? 100.0 * $sqrt(evm_err / evm_ref) : 0.0;
            ber     = (bits > 0) ? (1.0 * errs) / bits : 0.0;
            $display("[%0t ns] %0s", $time, name);
            $display("        bits=%0d  errors=%0d  BER=%e  EVM(vs nearest)=%0.2f %%  sync peak=%0d/126",
                     bits, errs, ber, evm_pct, peak);
        end
    endtask

    integer l, cq, sq;
    real    ph;

    // ------------------------------------------------------------- stimulus --
    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("tb_ofdm_top.vcd");
            $dumpvars(0, tb_ofdm_top);
        end
        evm_err = 0.0; evm_ref = 0.0; evm_n = 0;
        fd_const = $fopen("constellation.csv", "w");
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // "ARM": keys and 14 phase-compensation coefficients  c_l = e^{-j phi_l}
        axi_write(8'h08, 32'hCAFE_F00D); axi_write(8'h0C, 32'h0BAD_5EED);
        axi_write(8'h10, 32'hCAFE_F00D); axi_write(8'h14, 32'h0BAD_5EED);
        for (l = 0; l < NSYM; l = l + 1) begin
            ph = -(23.0 + 47.0 * l) * PI / 180.0;
            cq = $rtoi($cos(ph) * 16384.0 + ($cos(ph) >= 0 ? 0.5 : -0.5));
            sq = $rtoi($sin(ph) * 16384.0 + ($sin(ph) >= 0 ? 0.5 : -0.5));
            axi_write(8'h40 + 4*l, {cq[15:0], sq[15:0]});
        end
        axi_read(8'h40 + 4*3, rd);
        $display("coef[3] read back = 0x%08h", rd);

        // unknown arrival time: 77 idle noise samples before the first slot
        idle_noise_en = 1'b1;
        repeat (77) @(posedge clk);
        idle_noise_en = 1'b0;

        // ---------------- Test 1 : no phase compensation ----------------
        ctrl_shadow = 32'h5;                      // tx_en | chaos_en
        axi_write(8'h00, ctrl_shadow);
        test_no = 1;
        measure("TEST 1: chaos ON,  phase compensation OFF", 2);
        if (errs == 0) begin fails = fails + 1; $display("        -> FAIL (expected errors)"); end
        else $display("        -> OK  (constellation rotated, data corrupted as expected)");

        // ---------------- Test 2 : phase compensation ON ----------------
        ctrl_shadow = 32'h7;                      // tx_en | pc_en | chaos_en
        axi_write(8'h00, ctrl_shadow);
        test_no = 2;
        measure("TEST 2: chaos ON,  phase compensation ON ", 3);
        if (errs != 0 || bits == 0) begin fails = fails + 1; $display("        -> FAIL"); end
        else $display("        -> PASS (error-free, encrypted link)");

        // ---------------- Test 3 : wrong RX key ----------------
        axi_write(8'h10, 32'hCAFE_F00C);          // one key bit different
        test_no = 3;
        measure("TEST 3: RX key differs by 1 bit (eavesdropper)", 2);
        if (errs < bits / 10) begin fails = fails + 1; $display("        -> FAIL (expected ~50%% BER)"); end
        else $display("        -> OK  (cannot decrypt without the exact key)");
        axi_write(8'h10, 32'hCAFE_F00D);

        // ---------------- Test 4 : chaos OFF ----------------
        ctrl_shadow = 32'h3;                      // tx_en | pc_en
        axi_write(8'h00, ctrl_shadow);
        test_no = 4;
        measure("TEST 4: chaos OFF, phase compensation ON  (plain OFDM)", 2);
        if (errs != 0 || bits == 0) begin fails = fails + 1; $display("        -> FAIL"); end
        else $display("        -> PASS");

        axi_read(8'h20, rd);
        $display("RX slots synchronised: %0d", rd);
        $fclose(fd_const);
        if (fails == 0) $display("*** ALL TESTS PASSED ***");
        else            $display("*** %0d TEST(S) FAILED ***", fails);
        $finish;
    end

    // watchdog
    initial begin
        #20_000_000;
        $display("*** TIMEOUT ***");
        $finish;
    end
endmodule
