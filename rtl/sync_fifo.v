//==============================================================================
// sync_fifo.v  -  single-clock first-word-fall-through FIFO
// Used (a) as the chaotic-sequence buffer of the digital-chaos module
// (paper Sec. III-B: "buffers store chaotic sequences using FIFO queuing")
// and (b) as the timing-sync data buffer in front of the RX FFT.
//==============================================================================
`timescale 1ns / 1ps
module sync_fifo #(
    parameter DW = 8,
    parameter AW = 8                       // depth = 2^AW
)(
    input  wire          clk,
    input  wire          rst,
    input  wire          flush,
    input  wire          wr_en,
    input  wire [DW-1:0] din,
    input  wire          rd_en,
    output wire [DW-1:0] dout,             // valid whenever !empty (FWFT)
    output wire          full,
    output wire          empty,
    output reg  [AW:0]   count
);
    reg [DW-1:0] mem [0:(1<<AW)-1];
    reg [AW-1:0] wp, rp;

    wire do_wr = wr_en && !full;
    wire do_rd = rd_en && !empty;

    assign full  = (count == (1 << AW));
    assign empty = (count == 0);
    assign dout  = mem[rp];

    always @(posedge clk) begin
        if (do_wr) mem[wp] <= din;
    end

    always @(posedge clk) begin
        if (rst || flush) begin
            wp <= 0; rp <= 0; count <= 0;
        end else begin
            if (do_wr) wp <= wp + 1'b1;
            if (do_rd) rp <= rp + 1'b1;
            case ({do_wr, do_rd})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end
endmodule
