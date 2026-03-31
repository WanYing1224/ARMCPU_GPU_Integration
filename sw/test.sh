#!/bin/bash
# =============================================================================
# load_and_test.sh — Lab 8 Network Processor Integration
# Programs ARM IMEM/DMEM and GPU IMEM via the PCI register interface,
# then runs both processors and reads back results.
#
# Mirrors the Lab 6 load_and_test.sh workflow.
#
# Register base address (from ids.xml FIFO_BLOCK_ADDR):
#   Adjust BASE_ADDR to match your ids.xml assignment.
# =============================================================================

BASE_ADDR=0x2000400   # Update to your FIFO_BLOCK_ADDR value

# SW register offsets (must match fifo_top.v NUM_SOFTWARE_REGS ordering)
CMD=$((BASE_ADDR + 0x00))
PROG_ADDR=$((BASE_ADDR + 0x04))
PROG_WDATA=$((BASE_ADDR + 0x08))
PROG_CTRL=$((BASE_ADDR + 0x0c))
PROC_ADDR=$((BASE_ADDR + 0x10))
WDATA_HI=$((BASE_ADDR + 0x14))
WDATA_LO=$((BASE_ADDR + 0x18))
WDATA_CTRL=$((BASE_ADDR + 0x1c))

# HW register offsets
HW_STATUS=$((BASE_ADDR + 0x00))
HW_RDATA_HI=$((BASE_ADDR + 0x04))
HW_RDATA_LO=$((BASE_ADDR + 0x08))
HW_RDATA_CTRL=$((BASE_ADDR + 0x0c))
HW_PROC_STATUS=$((BASE_ADDR + 0x10))
HW_ARM_DMEM_RDATA=$((BASE_ADDR + 0x14))
HW_ARM_PC=$((BASE_ADDR + 0x18))

# CMD bit masks
ARM_PROG_EN=0x20    # bit[5]
GPU_PROG_EN=0x40    # bit[6]
ARM_START=0x08      # bit[3]
GPU_START=0x10      # bit[4]
FIFO_RESET=0x04     # bit[2]

# PROG_CTRL bit masks
ARM_PROG_WE=0x1     # bit[0]: write ARM IMEM or DMEM
ARM_DMEM_SEL=0x2    # bit[1]: 0=IMEM, 1=DMEM
GPU_PROG_WE=0x4     # bit[2]: write GPU IMEM
# (GPU DMEM = shared BRAM, loaded via PROC_ADDR/WDATA path)

# Helper: write a 32-bit value to a register
reg_write() {
    local addr=$1
    local val=$2
    regwrite $addr $val   # NetFPGA regwrite utility
}

# Helper: read a register (returns hex value)
reg_read() {
    local addr=$1
    regread $addr
}

# Helper: write one ARM IMEM word
#   $1 = word address (0-based), $2 = 32-bit instruction (hex)
arm_imem_write() {
    local waddr=$1
    local instr=$2
    local byte_addr=$(( waddr * 4 ))
    reg_write $PROG_ADDR $byte_addr
    reg_write $PROG_WDATA $instr
    reg_write $PROG_CTRL $ARM_PROG_WE      # arm_prog_we=1, arm_dmem_sel=0 (IMEM)
    reg_write $PROG_CTRL 0x0               # de-assert
}

# Helper: write one ARM DMEM word
#   $1 = word address (0-based), $2 = 32-bit data (hex)
arm_dmem_write() {
    local waddr=$1
    local data=$2
    local byte_addr=$(( waddr * 4 ))
    reg_write $PROG_ADDR $byte_addr
    reg_write $PROG_WDATA $data
    reg_write $PROG_CTRL $(( ARM_PROG_WE | ARM_DMEM_SEL ))  # DMEM
    reg_write $PROG_CTRL 0x0
}

# Helper: read one ARM DMEM word
#   $1 = word address (0-based)
arm_dmem_read() {
    local waddr=$1
    local byte_addr=$(( waddr * 4 ))
    reg_write $PROG_ADDR $byte_addr
    # Two reads needed (BRAM synchronous latency) — same as Lab 6
    reg_read  $HW_ARM_DMEM_RDATA
    reg_read  $HW_ARM_DMEM_RDATA
}

# Helper: write one GPU IMEM word
#   $1 = word address (0-based), $2 = 32-bit instruction (hex)
gpu_imem_write() {
    local waddr=$1
    local instr=$2
    local byte_addr=$(( waddr * 4 ))
    reg_write $PROG_ADDR $byte_addr
    reg_write $PROG_WDATA $instr
    reg_write $PROG_CTRL $GPU_PROG_WE      # gpu_prog_we=1, gpu_dmem_sel=0 (IMEM)
    reg_write $PROG_CTRL 0x0
}

# Helper: write 72-bit word to shared FIFO BRAM (GPU data memory)
#   $1 = BRAM word address (0-based)
#   $2 = data[63:32] (hi), $3 = data[31:0] (lo), $4 = ctrl[7:0]
bram_write() {
    local waddr=$1
    local hi=$2
    local lo=$3
    local ctrl=$4
    reg_write $PROC_ADDR   $waddr
    reg_write $WDATA_HI    $hi
    reg_write $WDATA_LO    $lo
    reg_write $WDATA_CTRL  $ctrl   # writing ctrl triggers host_we edge
    reg_write $WDATA_CTRL  0x0
}

# Helper: read 72-bit word from shared FIFO BRAM
#   $1 = BRAM word address (0-based)
bram_read() {
    local waddr=$1
    reg_write $PROC_ADDR $waddr
    echo "HI:   $(reg_read $HW_RDATA_HI)"
    echo "LO:   $(reg_read $HW_RDATA_LO)"
    echo "CTRL: $(reg_read $HW_RDATA_CTRL)"
}

# =============================================================================
# STEP 1: Reset everything
# =============================================================================
echo "=== Step 1: Reset ==="
reg_write $CMD $FIFO_RESET
reg_write $CMD 0x0

# =============================================================================
# STEP 2: Load ARM IMEM
# (Same program as imem.v case-ROM: add 1 to dmem[0..13])
# =============================================================================
echo "=== Step 2: Load ARM IMEM ==="
reg_write $CMD $ARM_PROG_EN   # hold ARM in reset during programming

# Thread 0: add 1 to dmem[0..13]
arm_imem_write  0  0xE3A00000   # MOV r0, #0
arm_imem_write  1  0xE3A01038   # MOV r1, #56  (14*4)
arm_imem_write  2  0xE5903000   # LDR r3, [r0]
arm_imem_write  3  0xE2833001   # ADD r3, r3, #1
arm_imem_write  4  0xE5803000   # STR r3, [r0]
arm_imem_write  5  0xE2800004   # ADD r0, r0, #4
arm_imem_write  6  0xE1500001   # CMP r0, r1
arm_imem_write  7  0xBAFFFFF9   # BLT -7 (back to word 2)
arm_imem_write  8  0xEAFFFFFE   # B . (halt)
# Thread 1,2,3 spin at their start PCs
arm_imem_write 48  0xEAFFFFFE   # B .
arm_imem_write 96  0xEAFFFFFE   # B .
arm_imem_write 144 0xEAFFFFFE   # B .

# =============================================================================
# STEP 3: Load ARM DMEM (test data: integers 0..13)
# =============================================================================
echo "=== Step 3: Load ARM DMEM ==="
for i in $(seq 0 13); do
    arm_dmem_write $i $i
done

# Release ARM from programming mode
reg_write $CMD 0x0

# =============================================================================
# STEP 4: Load GPU IMEM
# (FMA kernel: R2=Vec_A, R3=Vec_B, R4=Vec_C → R5=A*B+C, store to BRAM[0])
# =============================================================================
echo "=== Step 4: Load GPU IMEM ==="
reg_write $CMD $GPU_PROG_EN   # hold GPU in reset

gpu_imem_write 0  0x04400000   # READ_TID  R1
gpu_imem_write 1  0x0C800000   # LD64 R2, R0, 0       (Vec A from BRAM[0])
gpu_imem_write 2  0x0CC00008   # LD64 R3, R0, 8       (Vec B from BRAM[1])
gpu_imem_write 3  0x0D000010   # LD64 R4, R0, 16      (Vec C from BRAM[2])
gpu_imem_write 4  0x8548D000   # BF_MAC R5, R2, R3, R4
gpu_imem_write 5  0x11440000   # ST64 R5, R1, 0       (result → BRAM[0])

# Release GPU from programming mode (keep CMD=0 until start)
reg_write $CMD 0x0

# =============================================================================
# STEP 5: Load GPU operands into shared FIFO BRAM
# Set FIFO to PROC mode (fifo_mode=01) for BRAM access
# Vec A = 0x3F803F803F803F80 (4 x 1.0 in BF16)
# Vec B = 0x40004000400040   (4 x 2.0 in BF16)
# Vec C = 0x3F803F803F803F80 (4 x 1.0 in BF16)
# Expected result D = A*B+C = 4x 1.0*2.0+1.0 = 3.0 = 0x4040
# =============================================================================
echo "=== Step 5: Load GPU operands into BRAM ==="
reg_write $CMD 0x01   # fifo_mode=01 (PROC)

# BRAM[0] = Vec A  (hi=0x3F803F80, lo=0x3F803F80)
bram_write 0  0x3F803F80  0x3F803F80  0x00

# BRAM[1] = Vec B  (hi=0x40004000, lo=0x40004000)
bram_write 1  0x40004000  0x40004000  0x00

# BRAM[2] = Vec C  (hi=0x3F803F80, lo=0x3F803F80)
bram_write 2  0x3F803F80  0x3F803F80  0x00

reg_write $CMD 0x0   # back to IDLE

# =============================================================================
# STEP 6: Run ARM CPU
# =============================================================================
echo "=== Step 6: Run ARM CPU ==="
reg_write $CMD $ARM_START
reg_write $CMD 0x0

echo "Waiting for ARM done..."
while true; do
    status=$(reg_read $HW_PROC_STATUS)
    arm_done=$(( (status & 0x1) ))
    if [ "$arm_done" -eq 1 ]; then break; fi
    sleep 0.01
done
echo "ARM done."

# Read back ARM DMEM results (expect 1..14 after +1)
echo "ARM DMEM results:"
for i in $(seq 0 13); do
    val=$(arm_dmem_read $i)
    echo "  dmem[$i] = $val"
done

# =============================================================================
# STEP 7: Run GPU
# =============================================================================
echo "=== Step 7: Run GPU ==="
reg_write $CMD $GPU_START
reg_write $CMD 0x0

echo "Waiting for GPU done..."
while true; do
    status=$(reg_read $HW_PROC_STATUS)
    gpu_done=$(( (status >> 2) & 0x1 ))
    if [ "$gpu_done" -eq 1 ]; then break; fi
    sleep 0.01
done
echo "GPU done."

# Read GPU result from BRAM[0]
# Expected: 4 x 3.0 in BF16 = 0x4040404040404040
echo "GPU result (BRAM[0]):"
reg_write $CMD 0x01   # PROC mode to read BRAM
bram_read 0
reg_write $CMD 0x0

echo "=== Test complete ==="