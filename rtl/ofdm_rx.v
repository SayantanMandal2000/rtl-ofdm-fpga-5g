//==============================================================================
// ofdm_rx.v  -  OFDM receiver (baseband)
//
//  rx samples -> timing_sync (training detection, CP removal, data buffer)
//             -> FFT (unscaled, natural order)
//             -> chaotic de-rotation ks[5:4] -> 16-QAM hard demap
//             -> XOR ks[3:0] -> recovered PRBS bits -> self-synchronising
//                PRBS-15 checker (bit / error counters)
//
// The RX chaos generator is re-seeded with the RX key when the first data
// symbol of a slot enters the FFT; the key-stream FIFO pop is driven by the
// FFT-output valid (paper: "valid signal of ... the FFT block").
//==============================================================================
`timescale 1ns / 1ps
module ofdm_rx #(
    parameter N     = 128,
    parameter LOGN  = 7,
    parameter CP    = 32,
    parameter W     = 16,
    parameter NDATA = 13
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 chaos_en,
    input  wire [63:0]          key,
    input  wire                 ber_clear,
    input  wire                 rx_valid,
    input  wire signed [W-1:0]  rx_re,
    input  wire signed [W-1:0]  rx_im,
    // status
    output wire                 locked,
    output wire [31:0]          slot_count,
    output wire [10:0]          sync_peak,
    output reg  [31:0]          ber_bits,
    output reg  [31:0]          ber_errs,
    // debug: equalised / decrypted constellation (before decision)
    output reg                  dbg_valid,
    output reg  signed [W-1:0]  dbg_re,
    output reg  signed [W-1:0]  dbg_im,
    output reg  [3:0]           dbg_bits
);
    // ---------------- timing synchronisation ----------------
    wire          buf_rd;
    wire [2*W:0]  buf_data;
    wire          buf_avail;
    wire          lock_pulse;

    timing_sync #(.N(N), .CP(CP), .W(W), .NDATA(NDATA)) u_sync (
        .clk(clk), .rst(rst), .in_valid(rx_valid), .in_re(rx_re), .in_im(rx_im),
        .rd_en(buf_rd), .rd_data(buf_data), .rd_avail(buf_avail),
        .lock_pulse(lock_pulse), .locked(locked), .peak_val(sync_peak),
        .slot_count(slot_count));

    // ---------------- FFT ----------------
    wire                fft_in_ready, fft_done, fft_busy;
    wire [LOGN-1:0]     fft_rd_addr;
    wire signed [W-1:0] fft_rd_re, fft_rd_im;
    reg                 fft_release;

    assign buf_rd = buf_avail && fft_in_ready;

    fft_core #(.N(N), .LOGN(LOGN), .W(W), .SCALE(0)) u_fft (
        .clk(clk), .rst(rst), .inverse(1'b0),
        .in_valid(buf_rd), .in_re(buf_data[W-1:0]), .in_im(buf_data[2*W-1:W]),
        .in_ready(fft_in_ready), .busy(fft_busy), .done(fft_done),
        .rd_addr(fft_rd_addr), .rd_re(fft_rd_re), .rd_im(fft_rd_im),
        .out_release(fft_release));

    // remember whether the symbol being transformed is the first of a slot
    reg first_pending, busy_d;
    reg ks_reseed;
    always @(posedge clk) begin
        if (rst) begin first_pending <= 1'b0; busy_d <= 1'b0; ks_reseed <= 1'b0; end
        else begin
            busy_d    <= fft_busy;
            ks_reseed <= 1'b0;
            if (buf_rd && buf_data[2*W]) first_pending <= 1'b1;
            // reseed when the FFT starts (previous symbol fully consumed)
            if (fft_busy && !busy_d && first_pending) begin
                ks_reseed     <= 1'b1;
                first_pending <= 1'b0;
            end
        end
    end

    // ---------------- chaos key stream ----------------
    wire [7:0] ks;
    wire       ks_avail;
    reg        rd_active;
    reg [LOGN:0] k;
    wire       rd_go = rd_active && (!chaos_en || ks_avail);

    chaos_keystream #(.AW(8)) u_chaos (
        .clk(clk), .rst(rst), .reseed(ks_reseed), .key(key),
        .pop(rd_go && chaos_en), .ks(ks), .ks_avail(ks_avail));

    // ---------------- read-out FSM ----------------
    reg               s1_v;
    reg signed [W-1:0] s1_re, s1_im;
    reg [7:0]         s1_ks;

    always @(posedge clk) begin
        if (rst) begin
            rd_active <= 1'b0; k <= 0; fft_release <= 1'b0; s1_v <= 1'b0;
            s1_re <= 0; s1_im <= 0; s1_ks <= 0;
        end else begin
            fft_release <= 1'b0;
            s1_v        <= 1'b0;
            if (fft_done && !rd_active && !fft_release) begin
                rd_active <= 1'b1;
                k <= 0;
            end
            if (rd_go) begin
                // combinational FFT read at address k
                s1_v  <= 1'b1;
                s1_re <= fft_rd_re;
                s1_im <= fft_rd_im;
                s1_ks <= chaos_en ? ks : 8'h00;
                if (k == N-1) begin
                    rd_active   <= 1'b0;
                    fft_release <= 1'b1;
                end
                k <= k + 1'b1;
            end
        end
    end
    assign fft_rd_addr = k[LOGN-1:0];

    // ---------------- decryption + demapping ----------------
    wire signed [W-1:0] d_re, d_im;
    wire [3:0]          dm_bits;
    qam_rotate #(.W(W)) u_derot (.rot(2'd0 - s1_ks[5:4]),
                                 .i_i(s1_re), .q_i(s1_im), .i_o(d_re), .q_o(d_im));
    qam16_demapper #(.W(W)) u_demap (.i_i(d_re), .q_i(d_im), .bits(dm_bits));
    wire [3:0] plain_bits = dm_bits ^ s1_ks[3:0];

    always @(posedge clk) begin
        if (rst) begin
            dbg_valid <= 1'b0; dbg_re <= 0; dbg_im <= 0; dbg_bits <= 0;
        end else begin
            dbg_valid <= s1_v;
            if (s1_v) begin dbg_re <= d_re; dbg_im <= d_im; dbg_bits <= plain_bits; end
        end
    end

    // ---------------- self-synchronising PRBS-15 checker ----------------
    // Expected bit = h[14] ^ h[13] (history of received bits). Errors are
    // counted after 32 bits have filled the history.
    reg [14:0] hist;
    reg [5:0]  warm;
    reg [14:0] h_t;
    reg [2:0]  e_t;
    integer b;
    always @(posedge clk) begin
        if (rst || ber_clear) begin
            hist <= 0; warm <= 0; ber_bits <= 0; ber_errs <= 0;
        end else if (dbg_valid) begin
            h_t = hist; e_t = 0;
            for (b = 3; b >= 0; b = b - 1) begin
                if ((h_t[14] ^ h_t[13]) != dbg_bits[b]) e_t = e_t + 1'b1;
                h_t = {h_t[13:0], dbg_bits[b]};
            end
            hist <= h_t;
            if (warm < 6'd8) warm <= warm + 1'b1;   // 8 x 4 = 32 bits warm-up
            else begin
                ber_bits <= ber_bits + 4;
                ber_errs <= ber_errs + e_t;
            end
        end
    end
endmodule
