#!/usr/bin/env bash
# Open-source alternative to the Vivado simulator (Icarus Verilog >= 11).
# Run from the project root:  bash scripts/run_sim_iverilog.sh
set -e
mkdir -p sim_out
iverilog -g2005 -o sim_out/tb_fft.vvp tb/tb_fft_core.v rtl/fft_core.v rtl/twiddle_rom.v
vvp -n sim_out/tb_fft.vvp
iverilog -g2005 -s tb_ofdm_top -o sim_out/tb_top.vvp tb/tb_ofdm_top.v tb/channel_model.v rtl/*.v
(cd sim_out && vvp -n tb_top.vvp)
python3 scripts/plot_constellation.py sim_out/constellation.csv sim_out/constellation.png || true
