//==============================================================================
// fft_core.v
// Memory-based radix-2 DIT FFT / IFFT (one butterfly every 2 clocks).
//
//  * Samples are written in natural order and stored at bit-reversed addresses,
//    the in-place butterflies then leave the spectrum in natural order.
//  * inverse = 1 -> IFFT (conjugate twiddles).
//  * SCALE   = 1 -> every stage divides by 2 (total 1/N) : used for the TX IFFT,
//               so x[n] = (1/N) * sum X[k] e^{+j2pi nk/N}  exactly.
//    SCALE   = 0 -> unscaled with saturation             : used for the RX FFT,
//               which then returns the original QAM levels.
//  * The result is read back through a random-access port, which lets the TX
//    insert the cyclic prefix directly (read N-CP..N-1 then 0..N-1).
//
// Handshake:  IDLE/LOAD : in_ready=1, push N samples with in_valid
//             CALC      : log2(N) * N/2 butterflies
//             DONE      : done=1, read rd_addr -> rd_re/rd_im (combinational),
//                         pulse out_release to return to IDLE.
//
// NOTE: The paper (Sec. III-C) uses the Xilinx FFT IP core (128-point,
// natural order, convergent rounding). This core keeps the same ordering and
// scaling so the project simulates in any simulator without IP generation.
//==============================================================================
`timescale 1ns / 1ps
module fft_core #(
    parameter N     = 128,
    parameter LOGN  = 7,
    parameter W     = 16,
    parameter SCALE = 1
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 inverse,
    // load interface
    input  wire                 in_valid,
    input  wire signed [W-1:0]  in_re,
    input  wire signed [W-1:0]  in_im,
    output wire                 in_ready,
    // result interface
    output wire                 busy,
    output wire                 done,
    input  wire [LOGN-1:0]      rd_addr,
    output wire signed [W-1:0]  rd_re,
    output wire signed [W-1:0]  rd_im,
    input  wire                 out_release
);
    localparam S_IDLE = 2'd0, S_LOAD = 2'd1, S_CALC = 2'd2, S_DONE = 2'd3;
    localparam TW_FRAC = 14;

    reg [1:0] state;
    reg       inv_r;
    reg signed [W-1:0] mem_re [0:N-1];
    reg signed [W-1:0] mem_im [0:N-1];

    reg [LOGN-1:0] ld_cnt;
    reg [3:0]      stage;          // 0 .. LOGN-1 (LOGN <= 15)
    reg [LOGN-2:0] bf;             // butterfly index 0 .. N/2-1
    reg            ph;             // 0: fetch, 1: compute+write

    assign in_ready = (state == S_IDLE) || (state == S_LOAD);
    assign busy     = (state == S_CALC);
    assign done     = (state == S_DONE);
    assign rd_re    = mem_re[rd_addr];
    assign rd_im    = mem_im[rd_addr];

    // ---------------- bit reversal ----------------
    function [LOGN-1:0] bitrev;
        input [LOGN-1:0] a;
        integer i;
        begin
            for (i = 0; i < LOGN; i = i + 1) bitrev[i] = a[LOGN-1-i];
        end
    endfunction

    // ---------------- butterfly addressing ----------------
    //  half   = 2^stage
    //  k      = bf mod half       grp = bf / half
    //  top    = grp*2*half + k    bot = top + half
    //  twiddle index = k * N/(2*half) = k << (LOGN-1-stage)
    wire [LOGN-1:0] half    = {{(LOGN-1){1'b0}},1'b1} << stage;
    wire [LOGN-1:0] k_in    = {1'b0, bf} & (half - 1'b1);
    wire [LOGN-1:0] grp_off = ({1'b0, bf} - k_in) << 1;     // grp*2*half
    wire [LOGN-1:0] a_top   = grp_off + k_in;
    wire [LOGN-1:0] a_bot   = a_top + half;
    wire [LOGN-1:0] tw_full = k_in << (LOGN-1-stage);
    wire [LOGN-2:0] tw_addr = tw_full[LOGN-2:0];

    wire signed [15:0] tw_c, tw_s;
    twiddle_rom #(.LOGN(LOGN)) u_tw (.addr(tw_addr), .cos_o(tw_c), .sin_o(tw_s));

    // fetch registers
    reg signed [W-1:0]  ar, ai, br, bi;
    reg signed [15:0]   wc, ws;
    reg [LOGN-1:0]      top_r, bot_r;

    // ---------------- butterfly datapath ----------------
    //  t = W * b ;  W = c - j*s (forward) / c + j*s (inverse)  -> ws carries sign
    wire signed [W+15:0] m1 = br * wc;
    wire signed [W+15:0] m2 = bi * ws;
    wire signed [W+15:0] m3 = br * ws;
    wire signed [W+15:0] m4 = bi * wc;
    wire signed [W+16:0] t_re_f = m1 - m2 + (1 <<< (TW_FRAC-1));
    wire signed [W+16:0] t_im_f = m3 + m4 + (1 <<< (TW_FRAC-1));
    wire signed [W+2:0]  t_re   = t_re_f >>> TW_FRAC;
    wire signed [W+2:0]  t_im   = t_im_f >>> TW_FRAC;

    wire signed [W+3:0] s0_re = ar + t_re;
    wire signed [W+3:0] s0_im = ai + t_im;
    wire signed [W+3:0] s1_re = ar - t_re;
    wire signed [W+3:0] s1_im = ai - t_im;

    function signed [W-1:0] sat_scale;
        input signed [W+3:0] v;
        input                sc;
        reg   signed [W+3:0] r;
        begin
            r = sc ? ((v + 1) >>> 1) : v;           // round-half-up when scaling
            if (r > $signed({{4{1'b0}}, 1'b0, {(W-1){1'b1}}}))
                sat_scale = {1'b0, {(W-1){1'b1}}};
            else if (r < $signed({{4{1'b1}}, 1'b1, {(W-1){1'b0}}}))
                sat_scale = {1'b1, {(W-1){1'b0}}};
            else
                sat_scale = r[W-1:0];
        end
    endfunction

    wire sc = (SCALE != 0);

    // ---------------- control ----------------
    always @(posedge clk) begin
        if (rst) begin
            state  <= S_IDLE;
            ld_cnt <= 0;
            stage  <= 0;
            bf     <= 0;
            ph     <= 1'b0;
            inv_r  <= 1'b0;
        end else begin
            case (state)
            S_IDLE, S_LOAD: begin
                if (in_valid) begin
                    if (state == S_IDLE) inv_r <= inverse;
                    mem_re[bitrev(ld_cnt)] <= in_re;
                    mem_im[bitrev(ld_cnt)] <= in_im;
                    if (ld_cnt == N-1) begin
                        ld_cnt <= 0;
                        stage  <= 0;
                        bf     <= 0;
                        ph     <= 1'b0;
                        state  <= S_CALC;
                    end else begin
                        ld_cnt <= ld_cnt + 1'b1;
                        state  <= S_LOAD;
                    end
                end
            end
            S_CALC: begin
                if (!ph) begin
                    ar    <= mem_re[a_top];
                    ai    <= mem_im[a_top];
                    br    <= mem_re[a_bot];
                    bi    <= mem_im[a_bot];
                    wc    <= tw_c;
                    ws    <= inv_r ? tw_s : -tw_s;
                    top_r <= a_top;
                    bot_r <= a_bot;
                    ph    <= 1'b1;
                end else begin
                    mem_re[top_r] <= sat_scale(s0_re, sc);
                    mem_im[top_r] <= sat_scale(s0_im, sc);
                    mem_re[bot_r] <= sat_scale(s1_re, sc);
                    mem_im[bot_r] <= sat_scale(s1_im, sc);
                    ph <= 1'b0;
                    if (bf == N/2-1) begin
                        bf <= 0;
                        if (stage == LOGN-1) state <= S_DONE;
                        else                 stage <= stage + 1'b1;
                    end else begin
                        bf <= bf + 1'b1;
                    end
                end
            end
            S_DONE: if (out_release) state <= S_IDLE;
            endcase
        end
    end
endmodule
