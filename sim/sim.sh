#!/bin/bash
# ============================================================
# sim.sh — Compile and run Lab 8 integration testbench
#          in ModelSim (vsim / vlog)
#
# Usage:
#   chmod +x sim.sh
#   ./sim.sh          # compile + run + show transcript
#   ./sim.sh wave     # compile + run + open waveform GUI
# ============================================================

set -e

WORK_DIR="./work"
TB_TOP="fifo_top_tb"

# Source files — order matters for dependencies
SOURCES=(
    "isa_defines.vh"        # GPU ISA defines (included by gpu_core_min)
    "dmem_32.v"             # ARM data memory
    "imem.v"                # ARM instruction memory
    "arm_alu.v"             # ARM ALU
    "arm_regfile.v"         # ARM register file
    "arm_pipeline.v"        # ARM 4-thread pipeline
    "arm_cpu_wrapper.v"     # ARM programming interface wrapper
    "gpu_imem.v"            # GPU instruction memory (programmable BRAM)
    "gpu_regfile_min.v"     # GPU register file
    "bf16_mac.v"            # BFloat16 MAC unit
    "tensor_unit.v"         # 4-lane tensor unit
    "gpu_ldst.v"            # GPU load/store unit
    "gpu_pc.v"              # GPU program counter
    "gpu_core_min.v"        # GPU core FSM
    "gpu_net.v"             # GPU adapter (fifo_top interface)
    "fifo_top_tb.v"         # Testbench (must be last)
)

# ── 1. Create / refresh work library ─────────────────────────────────
echo "=== Creating work library ==="
vlib $WORK_DIR
vmap work $WORK_DIR

# ── 2. Compile all sources ────────────────────────────────────────────
echo "=== Compiling sources ==="
for f in "${SOURCES[@]}"; do
    # Skip header files — they are `include'd, not compiled directly
    if [[ "$f" == *.vh ]]; then
        echo "  (header) $f"
        continue
    fi
    echo "  vlog $f"
    vlog -work work \
         -incr \
         +incdir+. \
         "$f"
done

# ── 3. Run simulation ────────────────────────────────────────────────
echo ""
echo "=== Running simulation ==="

if [ "$1" == "wave" ]; then
    # GUI mode — open waveform window
    vsim -do "
        add wave -r /*;
        run -all;
    " work.$TB_TOP
else
    # Batch mode — transcript to stdout, exit on finish
    vsim -c \
         -do "run -all; quit -f" \
         work.$TB_TOP
fi
