#==============================================================================
# create_project.tcl  -  build the Vivado project for the FPGA OFDM system
#
# Usage (from the project root, i.e. the folder that contains rtl/ tb/ scripts/):
#     vivado -mode batch -source scripts/create_project.tcl
#   or in the Vivado Tcl console:
#     cd <path>/ofdm_fpga ; source scripts/create_project.tcl
#
# Then:  Flow Navigator -> Run Simulation -> Run Behavioral Simulation
#        (top = tb_ofdm_top, run time is set to "all")
#==============================================================================
set proj_name ofdm_fpga
set root      [file normalize [file dirname [info script]]/..]
# Zynq-7020 (ARM + FPGA, as in the paper's ARM/FPGA phase-compensation design).
# Change to your board's part if needed.
set part      xc7z020clg400-1

create_project $proj_name $root/vivado/$proj_name -part $part -force
set_property target_language Verilog [current_project]

# ---------------- design sources ----------------
add_files -fileset sources_1 [list \
    $root/rtl/twiddle_rom.v      \
    $root/rtl/fft_core.v         \
    $root/rtl/sync_fifo.v        \
    $root/rtl/lorenz_chaos.v     \
    $root/rtl/chaos_keystream.v  \
    $root/rtl/qam16.v            \
    $root/rtl/cmult.v            \
    $root/rtl/phase_comp_core.v  \
    $root/rtl/ofdm_axil_regs.v   \
    $root/rtl/ofdm_tx.v          \
    $root/rtl/timing_sync.v      \
    $root/rtl/ofdm_rx.v          \
    $root/rtl/ofdm_top.v         ]
set_property top ofdm_top [get_filesets sources_1]

# ---------------- constraints ----------------
add_files -fileset constrs_1 $root/constraints/ofdm_top.xdc

# ---------------- simulation sources ----------------
add_files -fileset sim_1 [list \
    $root/tb/channel_model.v \
    $root/tb/tb_ofdm_top.v   \
    $root/tb/tb_fft_core.v   ]
set_property top tb_ofdm_top [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1

puts "Project created: $root/vivado/$proj_name"
puts "Run behavioural simulation of tb_ofdm_top and check the Tcl console for"
puts "'*** ALL TESTS PASSED ***'."
