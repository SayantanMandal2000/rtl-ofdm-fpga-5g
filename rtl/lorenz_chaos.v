//==============================================================================
// lorenz_chaos.v  -  fixed-point digital Lorenz chaotic system
//
//    dx/dt = sigma*(y - x)        sigma = 10
//    dy/dt = x*(rho - z) - y      rho   = 28
//    dz/dt = x*y - beta*z         beta  = 8/3
//
// Forward-Euler integration, dt = 2^-7, state in signed Q12.20 (32 bit).
// Each step takes 2 clocks (products are registered) and emits one 8-bit
// key-stream byte built from the fractional bits of x, y and z.
// Because the arithmetic is bit-exact integer logic, two instances seeded
// with the same key produce identical sequences -> TX/RX chaos synchronisation
// (paper Sec. II-B / III-B, "Lorenz_DATA" sequences of Fig. 7).
//==============================================================================
`timescale 1ns / 1ps
module lorenz_chaos (
    input  wire        clk,
    input  wire        rst,
    input  wire        seed_load,        // load initial conditions from key
    input  wire [63:0] key,
    input  wire        step_en,          // allow stepping (back-pressure)
    output reg         ks_valid,         // one-cycle strobe per new byte
    output reg  [7:0]  ks_byte
);
    localparam F     = 20;                               // fractional bits
    localparam DT_SH = 7;                                // dt = 1/128
    localparam signed [31:0] RHO  = 32'sd28 <<< F;
    localparam signed [31:0] BETA = 32'sd2796203;        // round(8/3 * 2^20)
    localparam signed [31:0] ONE  = 32'sd1 <<< F;

    reg signed [31:0] x, y, z;
    reg signed [63:0] p_xy, p_xr, p_bz;
    reg               ph;

    // key diffusion: 4 xorshift64 rounds so that a 1-bit key change alters
    // the high-order fractional bits of all three initial conditions
    function [63:0] xs64;
        input [63:0] v;
        reg   [63:0] t;
        integer r;
        begin
            t = v ^ 64'h9E37_79B9_7F4A_7C15;
            for (r = 0; r < 4; r = r + 1) begin
                t = t ^ (t << 13);
                t = t ^ (t >> 7);
                t = t ^ (t << 17);
            end
            xs64 = t;
        end
    endfunction
    wire [63:0] h = xs64(key);

    // seed mapping : x0,y0 in [1,2), z0 in [20,21)
    wire signed [31:0] x0 = ONE + {12'd0, h[19:0]};
    wire signed [31:0] y0 = ONE + {12'd0, h[39:20]};
    wire signed [31:0] z0 = (32'sd20 <<< F) + {12'd0, h[59:40]};

    // Euler increments (wide intermediates)
    wire signed [39:0] dx = 40'sd10 * ($signed({{8{y[31]}}, y}) - $signed({{8{x[31]}}, x}));
    wire signed [63:0] dy = (p_xr >>> F) - y;
    wire signed [63:0] dz = (p_xy >>> F) - (p_bz >>> F);

    always @(posedge clk) begin
        if (rst || seed_load) begin
            x <= x0; y <= y0; z <= z0;
            ph <= 1'b0;
            ks_valid <= 1'b0;
            ks_byte  <= 8'd0;
        end else begin
            ks_valid <= 1'b0;
            if (!ph) begin
                if (step_en) begin
                    p_xy <= x * y;
                    p_xr <= x * (RHO - z);
                    p_bz <= z * BETA;
                    ph   <= 1'b1;
                end
            end else begin
                x  <= x + (dx >>> DT_SH);
                y  <= y + (dy >>> DT_SH);
                z  <= z + (dz >>> DT_SH);
                ph <= 1'b0;
                ks_valid <= 1'b1;
                ks_byte  <= x[11:4] ^ y[13:6] ^ z[9:2] ^ {x[19:16], y[19:16]};
            end
        end
    end
endmodule
