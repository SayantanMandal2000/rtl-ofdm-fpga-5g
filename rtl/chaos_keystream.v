//==============================================================================
// chaos_keystream.v  -  digital-chaos module = Lorenz generator + FIFO buffer
//
// Paper Sec. III-B: "The digital chaos module is first configured with ...
// chaotic sequence generators ... The second is to use buffers to store chaotic
// sequences, using FIFO queuing. The valid signal output by the QAM mapper
// and the FFT block are used to control the read ... to obtain a synchronous
// chaotic sequence."
//
// The generator runs ahead and keeps the FIFO full; the consumer pops one
// byte per encrypted/decrypted sub-carrier (pop is driven by the QAM-mapper
// valid on TX and by the FFT-output valid on RX).
// reseed: flush FIFO and restart the Lorenz system from the key.
//
// Key-stream byte usage per sub-carrier:
//   ks[3:0] : XOR mask for the 4 data bits      (bit-level / PRBS encryption)
//   ks[5:4] : k*90 deg rotation of the QAM point (real/imag encryption)
//==============================================================================
`timescale 1ns / 1ps
module chaos_keystream #(
    parameter AW = 8
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        reseed,
    input  wire [63:0] key,
    input  wire        pop,
    output wire [7:0]  ks,
    output wire        ks_avail
);
    wire       g_valid;
    wire [7:0] g_byte;
    wire       f_full, f_empty;
    wire [AW:0] f_cnt;

    // stop stepping when FIFO is nearly full (2-clock step latency)
    wire step_en = (f_cnt < ((1 << AW) - 2));

    lorenz_chaos u_lorenz (
        .clk(clk), .rst(rst), .seed_load(reseed), .key(key),
        .step_en(step_en), .ks_valid(g_valid), .ks_byte(g_byte));

    sync_fifo #(.DW(8), .AW(AW)) u_fifo (
        .clk(clk), .rst(rst), .flush(reseed),
        .wr_en(g_valid), .din(g_byte),
        .rd_en(pop), .dout(ks),
        .full(f_full), .empty(f_empty), .count(f_cnt));

    assign ks_avail = !f_empty;
endmodule
