//==============================================================================
// ofdm_tx.v  -  OFDM transmitter (baseband)
//
// Slot structure (NR-like, normal CP): NSYM = 14 OFDM symbols per slot
//   symbol 0      : conjugate-symmetric training symbol (timing sync, Sec. III-C)
//   symbols 1..13 : chaos-encrypted 16-QAM data symbols (Sec. III-B)
//
// Training symbol (paper Fig. 8): M-sequence -> 16-QAM -> imaginary branch set
// to 0 ("Void signal changed to 0") -> IFFT -> CP insertion. A real-valued
// spectrum gives a conjugate-symmetric time sequence
//      s(0) s(1) .. s(N/2-1) s*(N/2) s*(N/2-1) .. s*(1)        (paper Fig. 2)
//
// Data symbol: PRBS-15 -> XOR with chaos ks[3:0] -> 16-QAM map ->
//              chaotic quadrant rotation ks[5:4] -> IFFT -> CP insertion.
//
// Output stream is valid-qualified (sample-enable). tx_sos marks the first
// sample of a slot (0.5 ms trigger), tx_sof the first sample of a frame
// (10 ms trigger, every SLOTS slots).
//==============================================================================
`timescale 1ns / 1ps
module ofdm_tx #(
    parameter N     = 128,
    parameter LOGN  = 7,
    parameter CP    = 32,
    parameter W     = 16,
    parameter NSYM  = 14,
    parameter SLOTS = 20
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 enable,
    input  wire                 chaos_en,
    input  wire [63:0]          key,
    output reg                  tx_valid,
    output reg  signed [W-1:0]  tx_re,
    output reg  signed [W-1:0]  tx_im,
    output reg                  tx_sos,
    output reg                  tx_sof
);
    localparam S_IDLE = 3'd0, S_MAP = 3'd1, S_WAIT = 3'd2, S_OUT = 3'd3, S_REL = 3'd4;

    reg [2:0]      state;
    reg [3:0]      sym;          // symbol index inside slot
    reg [4:0]      slot;         // slot index inside frame
    reg [LOGN:0]   k;            // sub-carrier / sample counter
    reg [8:0]      n_out;        // output sample counter 0..N+CP-1

    // ---------------- PRBS-15 data source (x^15 + x^14 + 1) ----------------
    reg  [14:0] prbs;
    function [18:0] prbs_step4;       // returns {4 bits, next state}
        input [14:0] s;
        reg   [14:0] t;
        reg   [3:0]  b;
        integer i;
        begin
            t = s;
            for (i = 3; i >= 0; i = i - 1) begin
                b[i] = t[14] ^ t[13];
                t    = {t[13:0], b[i]};
            end
            prbs_step4 = {b, t};
        end
    endfunction
    wire [18:0] prbs_nx = prbs_step4(prbs);

    // ---------------- M-sequence for training (x^9 + x^5 + 1) ----------------
    reg  [8:0] mseq;
    wire       m_b1 = mseq[8] ^ mseq[4];
    wire [8:0] mseq_1 = {mseq[7:0], m_b1};
    wire       m_b0 = mseq_1[8] ^ mseq_1[4];
    wire [8:0] mseq_2 = {mseq_1[7:0], m_b0};

    // ---------------- digital chaos key stream ----------------
    reg  ks_reseed;
    wire [7:0] ks;
    wire       ks_avail;
    wire       is_train = (sym == 0);
    wire       need_ks  = chaos_en && !is_train;
    wire       fft_in_ready;
    wire       map_go   = (state == S_MAP) && fft_in_ready && (!need_ks || ks_avail);

    chaos_keystream #(.AW(8)) u_chaos (
        .clk(clk), .rst(rst), .reseed(ks_reseed), .key(key),
        .pop(map_go && need_ks), .ks(ks), .ks_avail(ks_avail));

    // ---------------- QAM mapping + encryption ----------------
    wire [3:0] data_bits = prbs_nx[18:15] ^ (need_ks ? ks[3:0] : 4'b0);
    wire signed [W-1:0] qi, qq, ri, rq;
    qam16_mapper #(.W(W)) u_map (.bits(data_bits), .i_o(qi), .q_o(qq));
    qam_rotate   #(.W(W)) u_rot (.rot(need_ks ? ks[5:4] : 2'd0),
                                  .i_i(qi), .q_i(qq), .i_o(ri), .q_o(rq));

    // training value: 16-QAM real branch from 2 M-sequence bits, imag = 0
    wire signed [W-1:0] tr_i, tr_q_unused;
    qam16_mapper #(.W(W)) u_map_tr (.bits({m_b1, m_b0, 2'b00}), .i_o(tr_i), .q_o(tr_q_unused));

    wire signed [W-1:0] fft_in_re = is_train ? tr_i : ri;
    wire signed [W-1:0] fft_in_im = is_train ? {W{1'b0}} : rq;

    // ---------------- IFFT ----------------
    wire               fft_done;
    reg  [LOGN-1:0]    fft_rd_addr;
    wire signed [W-1:0] fft_rd_re, fft_rd_im;
    reg                fft_release;

    fft_core #(.N(N), .LOGN(LOGN), .W(W), .SCALE(1)) u_ifft (
        .clk(clk), .rst(rst), .inverse(1'b1),
        .in_valid(map_go), .in_re(fft_in_re), .in_im(fft_in_im), .in_ready(fft_in_ready),
        .busy(), .done(fft_done),
        .rd_addr(fft_rd_addr), .rd_re(fft_rd_re), .rd_im(fft_rd_im),
        .out_release(fft_release));

    // ---------------- control FSM ----------------
    reg out_rd;            // read issued this cycle -> output next cycle
    reg rd_sos, rd_sof;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE; sym <= 0; slot <= 0; k <= 0; n_out <= 0;
            prbs <= 15'h7FFF; mseq <= 9'h1FF;
            ks_reseed <= 1'b0; fft_release <= 1'b0; out_rd <= 1'b0;
            fft_rd_addr <= 0; rd_sos <= 1'b0; rd_sof <= 1'b0;
        end else begin
            ks_reseed   <= 1'b0;
            fft_release <= 1'b0;
            out_rd      <= 1'b0;
            case (state)
            S_IDLE: begin
                if (enable) begin
                    sym <= 0; k <= 0;
                    mseq <= 9'h1FF;          // identical training every slot
                    ks_reseed <= 1'b1;       // re-key chaos at slot start
                    state <= S_MAP;
                end
            end
            S_MAP: begin
                if (map_go) begin
                    if (is_train) mseq <= mseq_2;
                    else          prbs <= prbs_nx[14:0];
                    if (k == N-1) begin k <= 0; state <= S_WAIT; end
                    else          k <= k + 1'b1;
                end
            end
            S_WAIT: begin
                if (fft_done) begin n_out <= 0; state <= S_OUT; end
            end
            S_OUT: begin
                // cyclic prefix = last CP samples, then the N samples
                fft_rd_addr <= (n_out < CP) ? (N - CP + n_out) : (n_out - CP);
                out_rd      <= 1'b1;
                rd_sos      <= (n_out == 0) && (sym == 0);
                rd_sof      <= (n_out == 0) && (sym == 0) && (slot == 0);
                if (n_out == N + CP - 1) state <= S_REL;
                n_out <= n_out + 1'b1;
            end
            S_REL: begin
                if (!out_rd) begin
                    fft_release <= 1'b1;
                    k <= 0;
                    if (sym == NSYM-1) begin
                        sym  <= 0;
                        slot <= (slot == SLOTS-1) ? 5'd0 : slot + 1'b1;
                        state <= S_IDLE;     // next slot (re-checks enable)
                    end else begin
                        sym   <= sym + 1'b1;
                        state <= S_MAP;
                    end
                end
            end
            default: state <= S_IDLE;
            endcase
        end
    end

    // registered output (combinational FFT read port is sampled here)
    always @(posedge clk) begin
        if (rst) begin
            tx_valid <= 1'b0; tx_re <= 0; tx_im <= 0; tx_sos <= 1'b0; tx_sof <= 1'b0;
        end else begin
            tx_valid <= out_rd;
            tx_sos   <= out_rd & rd_sos;
            tx_sof   <= out_rd & rd_sof;
            if (out_rd) begin
                tx_re <= fft_rd_re;
                tx_im <= fft_rd_im;
            end
        end
    end
endmodule
