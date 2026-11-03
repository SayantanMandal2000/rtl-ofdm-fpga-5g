//==============================================================================
// tb_fft_core.v  -  unit test: IFFT (scaled) followed by FFT (unscaled)
// must return the original 16-QAM frequency-domain symbols.
//==============================================================================
`timescale 1ns / 1ps
module tb_fft_core;
    localparam N = 128, LOGN = 7, W = 16;
    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    reg                 i_valid, f_valid;
    reg  signed [W-1:0] i_re, i_im, f_re, f_im;
    wire                i_ready, i_done, f_ready, f_done;
    reg  [LOGN-1:0]     i_addr, f_addr;
    wire signed [W-1:0] i_ore, i_oim, f_ore, f_oim;
    reg                 i_rel, f_rel;

    fft_core #(.N(N),.LOGN(LOGN),.W(W),.SCALE(1)) u_ifft (
        .clk(clk),.rst(rst),.inverse(1'b1),.in_valid(i_valid),.in_re(i_re),.in_im(i_im),
        .in_ready(i_ready),.busy(),.done(i_done),.rd_addr(i_addr),.rd_re(i_ore),.rd_im(i_oim),
        .out_release(i_rel));
    fft_core #(.N(N),.LOGN(LOGN),.W(W),.SCALE(0)) u_fft (
        .clk(clk),.rst(rst),.inverse(1'b0),.in_valid(f_valid),.in_re(f_re),.in_im(f_im),
        .in_ready(f_ready),.busy(),.done(f_done),.rd_addr(f_addr),.rd_re(f_ore),.rd_im(f_oim),
        .out_release(f_rel));

    reg signed [W-1:0] ref_re [0:N-1];
    reg signed [W-1:0] ref_im [0:N-1];
    integer k, maxerr, e, seed;
    function signed [W-1:0] lvl; input [1:0] b;
        case (b) 2'b00: lvl = -6144; 2'b01: lvl = -2048; 2'b11: lvl = 2048; default: lvl = 6144; endcase
    endfunction

    initial begin
        i_valid = 0; f_valid = 0; i_rel = 0; f_rel = 0; i_addr = 0; f_addr = 0; seed = 7;
        repeat (4) @(posedge clk); rst = 0;
        for (k = 0; k < N; k = k + 1) begin
            ref_re[k] = lvl($random(seed)); ref_im[k] = lvl($random(seed));
        end
        // IFFT load
        for (k = 0; k < N; k = k + 1) begin
            @(negedge clk); i_valid = 1; i_re = ref_re[k]; i_im = ref_im[k];
        end
        @(negedge clk); i_valid = 0;
        wait (i_done);
        // move time samples into FFT
        for (k = 0; k < N; k = k + 1) begin
            @(negedge clk); i_addr = k; #1; f_valid = 1; f_re = i_ore; f_im = i_oim;
        end
        @(negedge clk); f_valid = 0; i_rel = 1; @(negedge clk); i_rel = 0;
        wait (f_done);
        maxerr = 0;
        for (k = 0; k < N; k = k + 1) begin
            f_addr = k; #1;
            e = f_ore - ref_re[k]; if (e < 0) e = -e; if (e > maxerr) maxerr = e;
            e = f_oim - ref_im[k]; if (e < 0) e = -e; if (e > maxerr) maxerr = e;
        end
        $display("FFT round-trip max |error| = %0d LSB (decision margin 2048)", maxerr);
        if (maxerr < 200) $display("TB_FFT PASS"); else $display("TB_FFT FAIL");
        $finish;
    end
endmodule
