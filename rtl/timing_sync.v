//==============================================================================
// timing_sync.v  -  OFDM timing synchronisation with a conjugate-symmetric
//                   training sequence (paper Sec. III-C, Fig. 9, ref [14])
//
// Fig. 9 structure : data input is split into three branches
//   (1) ts_sliding_corr  - "sliding window energy computing module"  -> P(d)
//   (2) ts_energy_acc    - "average energy accumulation module"      -> R(d)
//   (3) ts_data_buffer   - "data buffer module" (delay + CP removal)
//   and ts_frame_find    - "frame finding module" : M(d) peak search,
//                          gives the enable to the data buffer.
//
// Timing metric for a window starting at sample d (training without CP):
//     P(d) = sum_{k=1}^{N/2-1} r(d+k) * r(d+N-k)
// For the conjugate-symmetric training x(N-k) = x*(k) every product equals
// |x(k)|^2, so P(d) peaks only when the window is aligned.
// Hardware approximation (paper: "since hardware implementation of the
// computation is difficult ... approximation"): r is quantised to its sign
// bits (1-bit I/Q), so P needs no multipliers, and
//     M(d) = |Re P(d)| + |Im P(d)|   replaces  |P(d)|^2 / R(d)^2.
// For the ideal aligned training M(d) = 2*(N/2-1) = 126 (N = 128) for any
// carrier phase; random data gives M ~ 10..40.  R(d) (sliding |r| sum) gates
// the detector so that idle noise / silence can never trigger.
//==============================================================================
`timescale 1ns / 1ps

// ------------------------------------------------------------------------------
module ts_sliding_corr #(
    parameter N = 128
)(
    input  wire              clk,
    input  wire              rst,
    input  wire              in_valid,
    input  wire              in_sgn_i,      // sign bit of I
    input  wire              in_sgn_q,      // sign bit of Q
    input  wire [31:0]       in_idx,        // absolute sample index
    output reg               p_valid,
    output reg  signed [10:0] p_re,
    output reg  signed [10:0] p_im,
    output reg  [31:0]       p_d            // window start index d
);
    // buf_i[0] = oldest = r(d) ... buf_i[N-1] = newest = r(d+N-1)
    reg [N-1:0] buf_i, buf_q;
    wire [N-1:0] nb_i = {in_sgn_i, buf_i[N-1:1]};
    wire [N-1:0] nb_q = {in_sgn_q, buf_q[N-1:1]};

    // each pair : (a1 + j b1)(a2 + j b2),  a,b in {+1,-1}  (sign bit 1 -> -1)
    //   Re = a1a2 - b1b2 , Im = a1b2 + b1a2 ;  a1a2 = +1 when sign bits equal
    integer k;
    reg signed [10:0] acc_re, acc_im;
    always @(*) begin
        acc_re = 0; acc_im = 0;
        for (k = 1; k < N/2; k = k + 1) begin
            acc_re = acc_re + ((buf_i[k] ~^ buf_i[N-k]) ? 11'sd1 : -11'sd1)
                            - ((buf_q[k] ~^ buf_q[N-k]) ? 11'sd1 : -11'sd1);
            acc_im = acc_im + ((buf_i[k] ~^ buf_q[N-k]) ? 11'sd1 : -11'sd1)
                            + ((buf_q[k] ~^ buf_i[N-k]) ? 11'sd1 : -11'sd1);
        end
    end

    reg        sh_v;
    reg [31:0] sh_idx;
    always @(posedge clk) begin
        if (rst) begin
            buf_i <= 0; buf_q <= 0; sh_v <= 1'b0; p_valid <= 1'b0;
            p_re <= 0; p_im <= 0; p_d <= 0; sh_idx <= 0;
        end else begin
            sh_v <= in_valid;
            if (in_valid) begin
                buf_i  <= nb_i;
                buf_q  <= nb_q;
                sh_idx <= in_idx - (N - 1);
            end
            // register the adder tree result one clock after the shift
            p_valid <= sh_v;
            if (sh_v) begin
                p_re <= acc_re;
                p_im <= acc_im;
                p_d  <= sh_idx;
            end
        end
    end
endmodule

// ------------------------------------------------------------------------------
module ts_energy_acc #(
    parameter N = 128,
    parameter W = 16
)(
    input  wire                clk,
    input  wire                rst,
    input  wire                in_valid,
    input  wire signed [W-1:0] in_re,
    input  wire signed [W-1:0] in_im,
    output reg                 r_valid,
    output reg  [W+8:0]        r_sum             // sum of |re|+|im| over N samples
);
    wire [W:0] mag = (in_re[W-1] ? -in_re : in_re) + (in_im[W-1] ? -in_im : in_im);
    reg  [W:0] dly [0:N-1];
    integer i;
    reg  [W+8:0] acc;
    always @(posedge clk) begin
        if (rst) begin
            acc <= 0; r_valid <= 1'b0; r_sum <= 0;
            for (i = 0; i < N; i = i + 1) dly[i] <= 0;
        end else begin
            r_valid <= in_valid;
            if (in_valid) begin
                for (i = N-1; i > 0; i = i - 1) dly[i] <= dly[i-1];
                dly[0] <= mag;
                acc    <= acc + mag - dly[N-1];
            end
            r_sum <= acc;
        end
    end
endmodule

// ------------------------------------------------------------------------------
module ts_frame_find #(
    parameter N      = 128,
    parameter TH     = 88,        // metric threshold (ideal peak = N-2)
    parameter E_MIN  = 16384,     // minimum window energy
    parameter HOLD   = 24         // samples without a new max before lock
)(
    input  wire              clk,
    input  wire              rst,
    input  wire              arm,          // re-arm search (slot finished)
    input  wire              p_valid,
    input  wire signed [10:0] p_re,
    input  wire signed [10:0] p_im,
    input  wire [31:0]       p_d,
    input  wire [24:0]       r_sum,
    output reg               lock,         // 1-clock pulse
    output reg  [31:0]       d_hat,        // index of training start (after CP)
    output reg  [10:0]       peak_val,
    output reg               searching
);
    wire [10:0] m = (p_re[10] ? -p_re : p_re) + (p_im[10] ? -p_im : p_im);
    reg  [10:0] best;
    reg  [31:0] best_d;
    reg  [7:0]  since;
    reg         have;

    always @(posedge clk) begin
        if (rst) begin
            searching <= 1'b1; lock <= 1'b0; have <= 1'b0; best <= 0;
            best_d <= 0; since <= 0; d_hat <= 0; peak_val <= 0;
        end else begin
            lock <= 1'b0;
            if (arm && !searching) begin
                searching <= 1'b1; have <= 1'b0; best <= 0; since <= 0;
            end else if (searching && p_valid) begin
                if (m >= TH && r_sum >= E_MIN && (!have || m > best)) begin
                    have <= 1'b1; best <= m; best_d <= p_d; since <= 0;
                end else if (have) begin
                    if (since == HOLD-1) begin
                        lock      <= 1'b1;
                        d_hat     <= best_d;
                        peak_val  <= best;
                        searching <= 1'b0;
                        have      <= 1'b0;
                    end
                    since <= since + 1'b1;
                end
            end
        end
    end
endmodule

// ------------------------------------------------------------------------------
// Data buffer: delays the stream by DBUF samples (SRL-friendly, no reset),
// then, once the frame finder has locked, removes the CP of every data symbol
// and forwards N useful samples per symbol (NDATA symbols per slot) into a FIFO.
// Output word: {first_of_slot, im, re}
module ts_data_buffer #(
    parameter N     = 128,
    parameter CP    = 32,
    parameter W     = 16,
    parameter NDATA = 13,
    parameter DBUF  = 128
)(
    input  wire                clk,
    input  wire                rst,
    input  wire                in_valid,
    input  wire signed [W-1:0] in_re,
    input  wire signed [W-1:0] in_im,
    input  wire [31:0]         in_idx,
    input  wire                lock,
    input  wire [31:0]         d_hat,
    output reg                 slot_done,   // 1-clock pulse
    // FIFO read side
    input  wire                rd_en,
    output wire [2*W:0]        rd_data,
    output wire                rd_avail
);
    // ---- delay line (shift register enabled by valid -> SRL inference) ----
    reg [2*W+31:0] dl [0:DBUF-1];
    integer i;
    always @(posedge clk) begin
        if (in_valid) begin
            dl[0] <= {in_idx, in_im, in_re};
            for (i = 1; i < DBUF; i = i + 1) dl[i] <= dl[i-1];
        end
    end
    wire [2*W+31:0] d_out = dl[DBUF-1];
    wire [31:0]     d_idx = d_out[2*W+31:2*W];

    // valid of delayed sample: becomes true after DBUF inputs
    reg [15:0] fill;
    wire       d_valid = in_valid && (fill >= DBUF);
    always @(posedge clk) begin
        if (rst) fill <= 0;
        else if (in_valid && fill < DBUF) fill <= fill + 1'b1;
    end

    // ---- symbol framing ----
    reg        active;
    reg [31:0] sym_start;      // index of first useful sample of current symbol
    reg [3:0]  sym_cnt;
    reg        first_flag;
    reg        push;
    reg [2*W:0] push_data;

    wire [31:0] off = d_idx - sym_start;

    always @(posedge clk) begin
        if (rst) begin
            active <= 1'b0; sym_start <= 0; sym_cnt <= 0; slot_done <= 1'b0;
            push <= 1'b0; first_flag <= 1'b0; push_data <= 0;
        end else begin
            slot_done <= 1'b0;
            push      <= 1'b0;
            if (lock) begin
                active     <= 1'b1;
                sym_start  <= d_hat + N + CP;     // first data symbol, CP removed
                sym_cnt    <= 0;
                first_flag <= 1'b1;
            end else if (active && d_valid && (off < N)) begin
                // sample (in_valid is the shift strobe: d_out is the sample being
                // shifted out of the delay line in this cycle)
                push      <= 1'b1;
                push_data <= {first_flag && (off == 0), d_out[2*W-1:0]};
                if (off == N-1) begin
                    first_flag <= 1'b0;
                    sym_start  <= sym_start + N + CP;
                    if (sym_cnt == NDATA-1) begin
                        active    <= 1'b0;
                        slot_done <= 1'b1;
                    end
                    sym_cnt <= sym_cnt + 1'b1;
                end
            end
        end
    end

    wire f_full, f_empty;
    wire [8:0] f_cnt;
    sync_fifo #(.DW(2*W+1), .AW(8)) u_fifo (
        .clk(clk), .rst(rst), .flush(1'b0),
        .wr_en(push), .din(push_data),
        .rd_en(rd_en), .dout(rd_data),
        .full(f_full), .empty(f_empty), .count(f_cnt));
    assign rd_avail = !f_empty;
endmodule

// ------------------------------------------------------------------------------
module timing_sync #(
    parameter N     = 128,
    parameter CP    = 32,
    parameter W     = 16,
    parameter NDATA = 13,
    parameter TH    = 88,
    parameter E_MIN = 16384
)(
    input  wire                clk,
    input  wire                rst,
    input  wire                in_valid,
    input  wire signed [W-1:0] in_re,
    input  wire signed [W-1:0] in_im,
    // to the demodulator
    input  wire                rd_en,
    output wire [2*W:0]        rd_data,
    output wire                rd_avail,
    // status
    output wire                lock_pulse,
    output wire                locked,
    output wire [10:0]         peak_val,
    output reg  [31:0]         slot_count
);
    reg [31:0] idx;
    always @(posedge clk) begin
        if (rst) idx <= 0;
        else if (in_valid) idx <= idx + 1'b1;
    end

    wire               p_valid;
    wire signed [10:0] p_re, p_im;
    wire [31:0]        p_d;
    wire               r_valid;
    wire [W+8:0]       r_sum;
    wire [31:0]        d_hat;
    wire               slot_done, searching;

    ts_sliding_corr #(.N(N)) u_corr (
        .clk(clk), .rst(rst), .in_valid(in_valid),
        .in_sgn_i(in_re[W-1]), .in_sgn_q(in_im[W-1]), .in_idx(idx),
        .p_valid(p_valid), .p_re(p_re), .p_im(p_im), .p_d(p_d));

    // energy sum: same 2-clock latency as the correlator -> R(d) aligned with P(d)
    ts_energy_acc #(.N(N), .W(W)) u_energy (
        .clk(clk), .rst(rst), .in_valid(in_valid), .in_re(in_re), .in_im(in_im),
        .r_valid(r_valid), .r_sum(r_sum));

    ts_frame_find #(.N(N), .TH(TH), .E_MIN(E_MIN)) u_find (
        .clk(clk), .rst(rst), .arm(slot_done),
        .p_valid(p_valid), .p_re(p_re), .p_im(p_im), .p_d(p_d), .r_sum(r_sum),
        .lock(lock_pulse), .d_hat(d_hat), .peak_val(peak_val), .searching(searching));

    ts_data_buffer #(.N(N), .CP(CP), .W(W), .NDATA(NDATA), .DBUF(N)) u_buf (
        .clk(clk), .rst(rst), .in_valid(in_valid), .in_re(in_re), .in_im(in_im),
        .in_idx(idx), .lock(lock_pulse), .d_hat(d_hat), .slot_done(slot_done),
        .rd_en(rd_en), .rd_data(rd_data), .rd_avail(rd_avail));

    assign locked = !searching;
    always @(posedge clk) begin
        if (rst) slot_count <= 0;
        else if (lock_pulse) slot_count <= slot_count + 1'b1;
    end
endmodule
