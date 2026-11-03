## ofdm_top.xdc
## Timing constraint for the system clock (100 MHz).
## ofdm_top exposes AXI-Lite and IQ streams as ports: on a Zynq board these are
## normally connected inside a block design (PS7 M_AXI_GP0 -> AXI-Lite,
## IQ -> DAC/ADC or loop-back), so no pin LOCs are given here.
## For a stand-alone out-of-context synthesis/timing run this file is enough.

create_clock -period 10.000 -name sys_clk [get_ports clk]
