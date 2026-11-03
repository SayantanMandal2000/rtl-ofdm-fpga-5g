//==============================================================================
// qam16.v  -  16-QAM Gray mapper, hard-decision demapper and the chaotic
//             quadrant rotation used to encrypt the real/imaginary parts.
//
//   bits[3:2] -> I, bits[1:0] -> Q,  Gray: 00->-3  01->-1  11->+1  10->+3
//   Level unit A = 2048 (so +-1 -> +-2048, +-3 -> +-6144 in Q-format 16 bit)
//==============================================================================
`timescale 1ns / 1ps

// ------------------------------------------------------------------ mapper ---
module qam16_mapper #(
    parameter W = 16,
    parameter A = 2048
)(
    input  wire [3:0]           bits,
    output wire signed [W-1:0]  i_o,
    output wire signed [W-1:0]  q_o
);
    function signed [W-1:0] lvl;
        input [1:0] b;
        case (b)
            2'b00:   lvl = -3 * A;
            2'b01:   lvl = -1 * A;
            2'b11:   lvl =  1 * A;
            default: lvl =  3 * A;   // 2'b10
        endcase
    endfunction
    assign i_o = lvl(bits[3:2]);
    assign q_o = lvl(bits[1:0]);
endmodule

// ---------------------------------------------------------------- demapper ---
module qam16_demapper #(
    parameter W = 16,
    parameter A = 2048
)(
    input  wire signed [W-1:0] i_i,
    input  wire signed [W-1:0] q_i,
    output wire [3:0]          bits
);
    function [1:0] dec;
        input signed [W-1:0] v;
        begin
            if      (v < -2*A) dec = 2'b00;
            else if (v <  0)   dec = 2'b01;
            else if (v <  2*A) dec = 2'b11;
            else               dec = 2'b10;
        end
    endfunction
    assign bits = {dec(i_i), dec(q_i)};
endmodule

// ------------------------------------------------- chaotic quadrant rotate ---
// rot = 0: x1, 1: x(+j), 2: x(-1), 3: x(-j).   Decrypt uses rot' = (4-rot)&3.
module qam_rotate #(
    parameter W = 16
)(
    input  wire [1:0]          rot,
    input  wire signed [W-1:0] i_i,
    input  wire signed [W-1:0] q_i,
    output reg  signed [W-1:0] i_o,
    output reg  signed [W-1:0] q_o
);
    always @(*) begin
        case (rot)
            2'd0: begin i_o =  i_i; q_o =  q_i; end
            2'd1: begin i_o = -q_i; q_o =  i_i; end
            2'd2: begin i_o = -i_i; q_o = -q_i; end
            default: begin i_o = q_i; q_o = -i_i; end
        endcase
    end
endmodule
