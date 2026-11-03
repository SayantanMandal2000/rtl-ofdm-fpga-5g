//==============================================================================
// channel_model.v  (simulation only)
//
// Models what sits between the FPGA TX and RX in the paper's test bench
// (Fig. 5: RRU -> mixer -> RF -> VSA):
//   * per-OFDM-symbol carrier phase rotation  phi_l  (the NR up-conversion
//     phase term that the transmitter phase compensation must pre-cancel)
//   * additive uniform noise of +-NOISE LSB on I and Q
//   * idle noise samples before the burst (unknown arrival time for the
//     timing synchroniser)
//==============================================================================
`timescale 1ns / 1ps
module channel_model #(
    parameter W     = 16,
    parameter NOISE = 8,
    parameter NSYM  = 14
)(
    input  wire                 clk,
    input  wire                 idle_noise_en,
    input  wire                 in_valid,
    input  wire signed [W-1:0]  in_re,
    input  wire signed [W-1:0]  in_im,
    input  wire [3:0]           in_sym_idx,
    output reg                  out_valid,
    output reg  signed [W-1:0]  out_re,
    output reg  signed [W-1:0]  out_im
);
    real    phi_deg [0:NSYM-1];
    real    c, s, xr, xi, yr, yi;
    integer l, seed;
    localparam real PI = 3.14159265358979;

    initial begin
        seed = 2026;
        for (l = 0; l < NSYM; l = l + 1) phi_deg[l] = 23.0 + 47.0 * l;
        out_valid = 0; out_re = 0; out_im = 0;
    end

    function integer noise;
        input integer dummy;
        begin
            noise = ($random(seed) % (NOISE + 1));
        end
    endfunction

    function signed [W-1:0] sat;
        input real v;
        integer iv;
        begin
            iv = (v >= 0.0) ? $rtoi(v + 0.5) : -$rtoi(-v + 0.5);
            if (iv >  32767) iv =  32767;
            if (iv < -32768) iv = -32768;
            sat = iv;
        end
    endfunction

    always @(posedge clk) begin
        out_valid <= 1'b0;
        if (in_valid) begin
            c  = $cos(phi_deg[in_sym_idx] * PI / 180.0);
            s  = $sin(phi_deg[in_sym_idx] * PI / 180.0);
            xr = in_re; xi = in_im;
            yr = xr * c - xi * s;
            yi = xr * s + xi * c;
            out_valid <= 1'b1;
            out_re    <= sat(yr + noise(0));
            out_im    <= sat(yi + noise(0));
        end else if (idle_noise_en) begin
            out_valid <= 1'b1;
            out_re    <= noise(0);
            out_im    <= noise(0);
        end
    end
endmodule
