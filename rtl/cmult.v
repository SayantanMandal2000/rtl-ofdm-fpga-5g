//==============================================================================
// cmult.v  -  pipelined complex multiplier (2 clocks), Q1.15 data x Q2.14 coef
//   y = x * c   with rounding and saturation back to W bits.
//   Maps onto 4 DSP48 slices.
//==============================================================================
`timescale 1ns / 1ps
module cmult #(
    parameter W  = 16,
    parameter CW = 16,
    parameter CF = 14          // coefficient fractional bits
)(
    input  wire                 clk,
    input  wire                 en,
    input  wire signed [W-1:0]  a_re,
    input  wire signed [W-1:0]  a_im,
    input  wire signed [CW-1:0] c_re,
    input  wire signed [CW-1:0] c_im,
    output reg  signed [W-1:0]  y_re,
    output reg  signed [W-1:0]  y_im
);
    reg signed [W+CW-1:0] p_rr, p_ii, p_ri, p_ir;

    wire signed [W+CW:0] s_re = p_rr - p_ii + (1 <<< (CF-1));
    wire signed [W+CW:0] s_im = p_ri + p_ir + (1 <<< (CF-1));
    wire signed [W+CW-CF:0] r_re = s_re >>> CF;
    wire signed [W+CW-CF:0] r_im = s_im >>> CF;

    localparam signed [W+CW-CF:0] MAXV =  (1 <<< (W-1)) - 1;
    localparam signed [W+CW-CF:0] MINV = -(1 <<< (W-1));

    always @(posedge clk) begin
        if (en) begin
            p_rr <= a_re * c_re;
            p_ii <= a_im * c_im;
            p_ri <= a_re * c_im;
            p_ir <= a_im * c_re;
            y_re <= (r_re > MAXV) ? MAXV[W-1:0] : (r_re < MINV) ? MINV[W-1:0] : r_re[W-1:0];
            y_im <= (r_im > MAXV) ? MAXV[W-1:0] : (r_im < MINV) ? MINV[W-1:0] : r_im[W-1:0];
        end
    end
endmodule
