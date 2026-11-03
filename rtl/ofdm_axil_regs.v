//==============================================================================
// ofdm_axil_regs.v  -  AXI4-Lite slave register bank (ARM <-> FPGA interface)
//
// Paper Sec. III-A: the ARM core computes the 14 phase-compensation
// coefficients and sends them to the FPGA through the AXI-Lite protocol.
//
//  Offset  Name        Access  Description
//  0x00    CTRL        RW      [0] tx_enable  [1] pc_enable  [2] chaos_enable
//                              [3] ber_clear (self-clearing pulse)
//  0x04    STATUS      RO      [0] rx_locked
//  0x08    TX_KEY_LO   RW      chaos key (TX) bits 31:0
//  0x0C    TX_KEY_HI   RW      chaos key (TX) bits 63:32
//  0x10    RX_KEY_LO   RW      chaos key (RX) bits 31:0
//  0x14    RX_KEY_HI   RW      chaos key (RX) bits 63:32
//  0x18    BER_BITS    RO      number of checked bits
//  0x1C    BER_ERRS    RO      number of bit errors
//  0x20    RX_SLOTS    RO      number of slots synchronised by the RX
//  0x24    SYNC_PEAK   RO      last timing metric peak value
//  0x40+4l PC_COEF[l]  RW      l = 0..13 : [31:16] cos(phi_l), [15:0] sin(phi_l), Q2.14
//==============================================================================
`timescale 1ns / 1ps
module ofdm_axil_regs #(
    parameter NSYM = 14,
    parameter AW   = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,
    // AXI4-Lite slave
    input  wire [AW-1:0]         s_axil_awaddr,
    input  wire                  s_axil_awvalid,
    output reg                   s_axil_awready,
    input  wire [31:0]           s_axil_wdata,
    input  wire [3:0]            s_axil_wstrb,
    input  wire                  s_axil_wvalid,
    output reg                   s_axil_wready,
    output reg  [1:0]            s_axil_bresp,
    output reg                   s_axil_bvalid,
    input  wire                  s_axil_bready,
    input  wire [AW-1:0]         s_axil_araddr,
    input  wire                  s_axil_arvalid,
    output reg                   s_axil_arready,
    output reg  [31:0]           s_axil_rdata,
    output reg  [1:0]            s_axil_rresp,
    output reg                   s_axil_rvalid,
    input  wire                  s_axil_rready,
    // register outputs
    output wire                  tx_enable,
    output wire                  pc_enable,
    output wire                  chaos_enable,
    output reg                   ber_clear,
    output wire [63:0]           tx_key,
    output wire [63:0]           rx_key,
    output wire [32*NSYM-1:0]    coef_flat,
    // status inputs
    input  wire                  rx_locked,
    input  wire [31:0]           ber_bits,
    input  wire [31:0]           ber_errs,
    input  wire [31:0]           rx_slots,
    input  wire [15:0]           sync_peak
);
    reg [31:0] ctrl;
    reg [31:0] key_tx_lo, key_tx_hi, key_rx_lo, key_rx_hi;
    reg [31:0] coef [0:NSYM-1];

    assign tx_enable    = ctrl[0];
    assign pc_enable    = ctrl[1];
    assign chaos_enable = ctrl[2];
    assign tx_key       = {key_tx_hi, key_tx_lo};
    assign rx_key       = {key_rx_hi, key_rx_lo};

    genvar g;
    generate
        for (g = 0; g < NSYM; g = g + 1) begin : g_coef
            assign coef_flat[32*g +: 32] = coef[g];
        end
    endgenerate

    // ---------------- write channel ----------------
    reg [AW-1:0] awaddr_l;
    reg          aw_ok, w_ok;
    reg [31:0]   wdata_l;
    reg [3:0]    wstrb_l;

    function [31:0] apply_strb;
        input [31:0] old, nw;
        input [3:0]  st;
        integer b;
        begin
            for (b = 0; b < 4; b = b + 1)
                apply_strb[8*b +: 8] = st[b] ? nw[8*b +: 8] : old[8*b +: 8];
        end
    endfunction

    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            s_axil_awready <= 1'b0; s_axil_wready <= 1'b0;
            s_axil_bvalid  <= 1'b0; s_axil_bresp  <= 2'b00;
            aw_ok <= 1'b0; w_ok <= 1'b0;
            ctrl      <= 32'h0000_0004;          // chaos enabled by default
            key_tx_lo <= 32'h1234_5678; key_tx_hi <= 32'h9ABC_DEF0;
            key_rx_lo <= 32'h1234_5678; key_rx_hi <= 32'h9ABC_DEF0;
            ber_clear <= 1'b0;
            for (i = 0; i < NSYM; i = i + 1) coef[i] <= 32'h4000_0000; // 1 + j0
        end else begin
            ber_clear      <= 1'b0;
            s_axil_awready <= 1'b0;
            s_axil_wready  <= 1'b0;

            if (s_axil_awvalid && !aw_ok && !s_axil_bvalid) begin
                s_axil_awready <= 1'b1;
                awaddr_l <= s_axil_awaddr;
                aw_ok    <= 1'b1;
            end
            if (s_axil_wvalid && !w_ok && !s_axil_bvalid) begin
                s_axil_wready <= 1'b1;
                wdata_l <= s_axil_wdata;
                wstrb_l <= s_axil_wstrb;
                w_ok    <= 1'b1;
            end

            if (aw_ok && w_ok) begin
                aw_ok <= 1'b0; w_ok <= 1'b0;
                s_axil_bvalid <= 1'b1;
                s_axil_bresp  <= 2'b00;
                case (awaddr_l)
                    8'h00: begin
                        ctrl      <= apply_strb(ctrl, wdata_l, wstrb_l) & 32'h0000_0007;
                        ber_clear <= wstrb_l[0] & wdata_l[3];
                    end
                    8'h08: key_tx_lo <= apply_strb(key_tx_lo, wdata_l, wstrb_l);
                    8'h0C: key_tx_hi <= apply_strb(key_tx_hi, wdata_l, wstrb_l);
                    8'h10: key_rx_lo <= apply_strb(key_rx_lo, wdata_l, wstrb_l);
                    8'h14: key_rx_hi <= apply_strb(key_rx_hi, wdata_l, wstrb_l);
                    default: begin
                        if (awaddr_l >= 8'h40 && awaddr_l < 8'h40 + 4*NSYM)
                            coef[(awaddr_l - 8'h40) >> 2] <=
                                apply_strb(coef[(awaddr_l - 8'h40) >> 2], wdata_l, wstrb_l);
                    end
                endcase
            end

            if (s_axil_bvalid && s_axil_bready) s_axil_bvalid <= 1'b0;
        end
    end

    // ---------------- read channel ----------------
    always @(posedge clk) begin
        if (!rst_n) begin
            s_axil_arready <= 1'b0;
            s_axil_rvalid  <= 1'b0;
            s_axil_rresp   <= 2'b00;
            s_axil_rdata   <= 32'd0;
        end else begin
            s_axil_arready <= 1'b0;
            if (s_axil_arvalid && !s_axil_rvalid && !s_axil_arready) begin
                s_axil_arready <= 1'b1;
                s_axil_rvalid  <= 1'b1;
                s_axil_rresp   <= 2'b00;
                case (s_axil_araddr)
                    8'h00: s_axil_rdata <= ctrl;
                    8'h04: s_axil_rdata <= {31'd0, rx_locked};
                    8'h08: s_axil_rdata <= key_tx_lo;
                    8'h0C: s_axil_rdata <= key_tx_hi;
                    8'h10: s_axil_rdata <= key_rx_lo;
                    8'h14: s_axil_rdata <= key_rx_hi;
                    8'h18: s_axil_rdata <= ber_bits;
                    8'h1C: s_axil_rdata <= ber_errs;
                    8'h20: s_axil_rdata <= rx_slots;
                    8'h24: s_axil_rdata <= {16'd0, sync_peak};
                    default: begin
                        if (s_axil_araddr >= 8'h40 && s_axil_araddr < 8'h40 + 4*NSYM)
                            s_axil_rdata <= coef[(s_axil_araddr - 8'h40) >> 2];
                        else
                            s_axil_rdata <= 32'hDEAD_BEEF;
                    end
                endcase
            end else if (s_axil_rvalid && s_axil_rready) begin
                s_axil_rvalid <= 1'b0;
            end
        end
    end
endmodule
