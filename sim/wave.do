# ============================================================
# wave.do — ModelSim waveform setup for Lab 8 integration TB
#
# Run from the ModelSim transcript after compiling:
#   do wave.do
# Or load automatically via sim.sh wave
# ============================================================

# ── Clock and reset ──────────────────────────────────────────
add wave -divider "Clock / Reset"
add wave -radix bin  sim:/fifo_top_tb/clk
add wave -radix bin  sim:/fifo_top_tb/rst_n

# ── ARM programming interface ────────────────────────────────
add wave -divider "ARM Programming Interface"
add wave -radix bin  sim:/fifo_top_tb/arm_prog_en
add wave -radix bin  sim:/fifo_top_tb/arm_prog_we
add wave -radix bin  sim:/fifo_top_tb/arm_dmem_sel
add wave -radix hex  sim:/fifo_top_tb/arm_prog_addr
add wave -radix hex  sim:/fifo_top_tb/arm_prog_wdata
add wave -radix hex  sim:/fifo_top_tb/arm_prog_rdata

# ── ARM status ───────────────────────────────────────────────
add wave -divider "ARM Status"
add wave -radix bin  sim:/fifo_top_tb/arm_cpu_done
add wave -radix bin  sim:/fifo_top_tb/arm_cpu_running

# ── ARM internal PCs (for tracking thread progress) ─────────
add wave -divider "ARM Thread PCs"
add wave -radix unsigned sim:/fifo_top_tb/arm_dut/arm_inst/pc[0]
add wave -radix unsigned sim:/fifo_top_tb/arm_dut/arm_inst/pc[1]
add wave -radix unsigned sim:/fifo_top_tb/arm_dut/arm_inst/pc[2]
add wave -radix unsigned sim:/fifo_top_tb/arm_dut/arm_inst/pc[3]

# ── ARM DMEM Port B (programming path) ──────────────────────
add wave -divider "ARM DMEM Port B"
add wave -radix bin  sim:/fifo_top_tb/arm_dut/arm_inst/dmem_inst/web
add wave -radix hex  sim:/fifo_top_tb/arm_dut/arm_inst/dmem_inst/addrb
add wave -radix hex  sim:/fifo_top_tb/arm_dut/arm_inst/dmem_inst/dinb
add wave -radix hex  sim:/fifo_top_tb/arm_dut/arm_inst/dmem_inst/doutb

# ── GPU programming interface ────────────────────────────────
add wave -divider "GPU Programming Interface"
add wave -radix bin  sim:/fifo_top_tb/gpu_prog_en
add wave -radix bin  sim:/fifo_top_tb/gpu_prog_we
add wave -radix bin  sim:/fifo_top_tb/gpu_dmem_sel
add wave -radix hex  sim:/fifo_top_tb/gpu_prog_addr
add wave -radix hex  sim:/fifo_top_tb/gpu_prog_wdata

# ── GPU status ───────────────────────────────────────────────
add wave -divider "GPU Status"
add wave -radix bin  sim:/fifo_top_tb/gpu_done
add wave -radix bin  sim:/fifo_top_tb/gpu_running

# ── GPU FSM state ────────────────────────────────────────────
add wave -divider "GPU FSM"
add wave -radix unsigned sim:/fifo_top_tb/gpu_dut/gpu_core/state
add wave -radix unsigned sim:/fifo_top_tb/gpu_dut/gpu_core/pc

# ── Shared FIFO BRAM (host Port B) ──────────────────────────
add wave -divider "Shared BRAM (host port)"
add wave -radix bin  sim:/fifo_top_tb/bram_we_b
add wave -radix hex  sim:/fifo_top_tb/bram_addr_b
add wave -radix hex  sim:/fifo_top_tb/bram_wdata_b
add wave -radix hex  sim:/fifo_top_tb/bram_rdata_b

# ── Shared FIFO BRAM (GPU port) ─────────────────────────────
add wave -divider "Shared BRAM (GPU port)"
add wave -radix bin  sim:/fifo_top_tb/gpu_bram_we
add wave -radix hex  sim:/fifo_top_tb/gpu_bram_addr
add wave -radix hex  sim:/fifo_top_tb/gpu_bram_wdata
add wave -radix hex  sim:/fifo_top_tb/gpu_bram_rdata

# ── GPU register file (R2=VecA, R3=VecB, R4=VecC, R5=result) 
add wave -divider "GPU Registers"
add wave -radix hex  sim:/fifo_top_tb/gpu_dut/gpu_core/regfile_inst/regs[2]
add wave -radix hex  sim:/fifo_top_tb/gpu_dut/gpu_core/regfile_inst/regs[3]
add wave -radix hex  sim:/fifo_top_tb/gpu_dut/gpu_core/regfile_inst/regs[4]
add wave -radix hex  sim:/fifo_top_tb/gpu_dut/gpu_core/regfile_inst/regs[5]

# ── Tensor unit ──────────────────────────────────────────────
add wave -divider "Tensor Unit"
add wave -radix bin  sim:/fifo_top_tb/gpu_dut/gpu_core/tensor_inst/start
add wave -radix bin  sim:/fifo_top_tb/gpu_dut/gpu_core/tensor_inst/done
add wave -radix hex  sim:/fifo_top_tb/gpu_dut/gpu_core/tensor_inst/result

# Set time resolution and zoom
configure wave -timelineunits ns
WaveRestoreZoom {0 ns} {5000 ns}

run -all
