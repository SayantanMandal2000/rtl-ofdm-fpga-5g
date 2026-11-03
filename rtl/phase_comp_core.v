//==============================================================================
// phase_comp_core.v  -  real-time 5G-NR OFDM phase compensation (TX side)
//
// Paper Sec. III-A / Fig. 3-4 (ref [11]):
//  * The ARM core computes 14 phase-compensation coefficients (one per OFDM
//    symbol of a slot) and writes them to the FPGA over AXI-Lite.
//  * The FPGA uses the 10 ms (frame) and 0.5 ms (slot) triggers plus the
//    sample-valid to determine which OFDM symbol (0..13) the IQ sample belongs
//    to, selects that coefficient through a 14:1 mux and multiplies the IQ data.
//
//  coef_flat[32*l +: 32] = {cos(phi_l), sin(phi_l)}  signed Q2.14 each,
//  i.e. the multiplier is c_l = cos(phi_l) + j*sin(phi_l).
//  pc_enable = 0 -> coefficient forced to 1+j0 (same latency, bypass).
//
// Latency: 2 clocks (complex multiplier). Side-band signals are delayed to match.
//==============================================================================
`timescale 1ns / 1ps
module phase_comp_core #(
    parameter W       = 16,
    parameter N       = 128,
    parameter CP      = 32,
    parameter NSYM    = 14,
    parameter SLOTS   = 20                     // slots per 10 ms frame
)(
    input  wire                  clk,
    input  wire                  rst,
    input  wire                  pc_enable,
    input  wire [32*NSYM-1:0]    coef_flat,
    // input stream
    input  wire                  in_valid,
    input  wire signed [W-1:0]   in_re,
    input  wire signed [W-1:0]   in_im,
    input  wire                  trig_10ms,    // first sample of a frame
    input  wire                  trig_0p5ms,   // first sample of a slot
    // output stream
    output wire                  out_valid,
    output wire signed [W-1:0]   out_re,
    output wire signed [W-1:0]   out_im,
    output wire                  out_sof,
    output wire                  out_sos,
    output wire [3:0]            out_sym_idx,
    output reg  [4:0]            slot_idx      // slot number inside the frame
);
    localparam SYM_LEN = N + CP;

    // ---------------- symbol position tracking ----------------
    reg [8:0] samp;
    reg [3:0] sym;

    wire [8:0] cur_samp = trig_0p5ms ? 9'd0 : samp;
    wire [3:0] cur_sym  = trig_0p5ms ? 4'd0 : sym;

    always @(posedge clk) begin
        if (rst) begin
            samp <= 0; sym <= 0; slot_idx <= 0;
        end else if (in_valid) begin
            if (cur_samp == SYM_LEN-1) begin
                samp <= 0;
                sym  <= (cur_sym == NSYM-1) ? 4'd0 : cur_sym + 1'b1;
            end else begin
                samp <= cur_samp + 1'b1;
                sym  <= cur_sym;
            end
            if (trig_10ms)       slot_idx <= 0;
            else if (trig_0p5ms) slot_idx <= (slot_idx == SLOTS-1) ? 5'd0 : slot_idx + 1'b1;
        end
    end

    // ---------------- 14:1 coefficient mux ----------------
    wire [31:0] coef_sel = coef_flat[32*cur_sym +: 32];
    wire signed [15:0] c_re = pc_enable ? coef_sel[31:16] : 16'sd16384;
    wire signed [15:0] c_im = pc_enable ? coef_sel[15:0]  : 16'sd0;

    cmult #(.W(W), .CW(16), .CF(14)) u_mult (
        .clk(clk), .en(1'b1),
        .a_re(in_re), .a_im(in_im), .c_re(c_re), .c_im(c_im),
        .y_re(out_re), .y_im(out_im));

    // ---------------- side-band delay (2 clocks) ----------------
    reg [1:0] v_d, sof_d, sos_d;
    reg [3:0] sym_d0, sym_d1;
    always @(posedge clk) begin
        if (rst) begin
            v_d <= 0; sof_d <= 0; sos_d <= 0; sym_d0 <= 0; sym_d1 <= 0;
        end else begin
            v_d   <= {v_d[0],   in_valid};
            sof_d <= {sof_d[0], in_valid & trig_10ms};
            sos_d <= {sos_d[0], in_valid & trig_0p5ms};
            sym_d0 <= cur_sym;
            sym_d1 <= sym_d0;
        end
    end
    assign out_valid   = v_d[1];
    assign out_sof     = sof_d[1];
    assign out_sos     = sos_d[1];
    assign out_sym_idx = sym_d1;
endmodule
