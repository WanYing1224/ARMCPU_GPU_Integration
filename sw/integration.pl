#!/usr/bin/perl
# ============================================================
# integration.pl — Lab 8 Network Processor Integration Test
#
# Tests the ARM CPU (via PCI programming interface) and the
# GPU BFloat16 tensor core running on the NetFPGA.
#
# Register map (ids.xml, NUM_SOFTWARE_REGS=6, NUM_HARDWARE_REGS=5):
#
#   SW registers (host writes):
#   0x2000100  CMD         SW0
#   0x2000104  PROC_ADDR   SW1
#   0x2000108  WDATA_HI    SW2
#   0x200010C  WDATA_LO    SW3
#   0x2000110  WDATA_CTRL  SW4
#   0x2000114  RDATA_SEL   SW5
#
#   HW registers (host reads):
#   0x2000118  STATUS      HW0
#   0x200011C  RDATA_HI    HW1
#   0x2000120  RDATA_LO    HW2
#   0x2000124  RDATA_CTRL  HW3
#   0x2000128  POINTERS    HW4
#
# CMD bits:
#   [1:0] = fifo_mode  (00=IDLE, 01=PROC, 10=FIFO_OUT, 11=FIFO_IN)
#   [2]   = fifo_reset (pulse)
#   [3]   = arm_start  (rising edge)
#   [4]   = gpu_start  (rising edge)
#   [5]   = gpu2_start (rising edge, reserved for future use)
#
# RDATA_SEL bits:
#   [1]   = arm_prog_en  — 1 = hold ARM in reset / programming mode
#   [2]   = gpu_prog_en  — 1 = hold GPU in reset / programming mode
#   [3]   = arm_dmem_sel — 0 = target ARM IMEM, 1 = target ARM DMEM
#   [4]   = gpu_dmem_sel — 0 = target GPU IMEM, 1 = target shared BRAM
#   [5]   = arm_prog_we  — rising edge writes proc_addr/wdata_lo to ARM
#   [6]   = gpu_prog_we  — rising edge writes proc_addr/wdata_lo to GPU IMEM
#
# POINTERS bits:
#   [0]    = arm_done    [1]  = arm_running
#   [2]    = gpu_done    [3]  = gpu_running
#   [4]    = gpu2_done   [5]  = gpu2_running
#   [15:8] = tail_addr   [23:16] = head_addr
#
# Test sequence:
#   [1] Reset
#   [2] Program ARM IMEM (add-1-to-dmem loop, 14 words)
#   [3] Program ARM DMEM (initial values 0..13)
#   [4] Release ARM, start, poll done, verify dmem results
#   [5] Program GPU IMEM (BF_MAC kernel)
#   [6] Load GPU operands into shared FIFO BRAM
#   [7] Start GPU, poll done, verify BF16 result
# ============================================================

use strict;
use warnings;

# ── Register addresses ────────────────────────────────────────────────────────
# Base address matches FIFO_BLOCK_ADDR from generated system_defs_h.v.
# The 6 SW regs land at base+0x00..+0x14, 5 HW regs at base+0x18..+0x28.
my $BASE           = 0x2000100;

my $CMD_REG        = $BASE + 0x00;   # SW0
my $PROC_ADDR_REG  = $BASE + 0x04;   # SW1
my $WDATA_HI_REG   = $BASE + 0x08;   # SW2
my $WDATA_LO_REG   = $BASE + 0x0C;   # SW3
my $WDATA_CTRL_REG = $BASE + 0x10;   # SW4
my $RDATA_SEL_REG  = $BASE + 0x14;   # SW5

my $STATUS_REG     = $BASE + 0x18;   # HW0
my $RDATA_HI_REG   = $BASE + 0x1C;   # HW1
my $RDATA_LO_REG   = $BASE + 0x20;   # HW2
my $RDATA_CTRL_REG = $BASE + 0x24;   # HW3
my $POINTERS_REG   = $BASE + 0x28;   # HW4

# ── CMD bit masks ─────────────────────────────────────────────────────────────
my $CMD_MODE_IDLE  = 0x00;
my $CMD_MODE_PROC  = 0x01;
my $CMD_FIFO_RESET = 0x04;
my $CMD_ARM_START  = 0x08;
my $CMD_GPU_START  = 0x10;

# ── RDATA_SEL bit masks ───────────────────────────────────────────────────────
my $SEL_ARM_PROG_EN  = 0x02;   # [1]
my $SEL_GPU_PROG_EN  = 0x04;   # [2]
my $SEL_ARM_DMEM     = 0x08;   # [3]  0=IMEM, 1=DMEM
my $SEL_GPU_DMEM     = 0x10;   # [4]  0=GPU IMEM, 1=shared BRAM
my $SEL_ARM_PROG_WE  = 0x20;   # [5]  rising edge
my $SEL_GPU_PROG_WE  = 0x40;   # [6]  rising edge

# ── POINTERS bit masks ────────────────────────────────────────────────────────
my $PTR_ARM_DONE   = 0x01;
my $PTR_ARM_RUN    = 0x02;
my $PTR_GPU_DONE   = 0x04;
my $PTR_GPU_RUN    = 0x08;

my $TIMEOUT = 2000;   # 2000 * 10ms = 20 seconds max

# ── Pass/fail counters ────────────────────────────────────────────────────────
my ($pass, $fail) = (0, 0);

# =============================================================================
# Low-level register helpers
# =============================================================================

sub regwrite {
    my ($addr, $val) = @_;
    system(sprintf("regwrite 0x%08x 0x%08x", $addr, $val));
}

sub regread_val {
    my ($addr) = @_;
    my $out = `regread 0x${\ sprintf("%08x", $addr)}`;
    my ($v) = $out =~ /:\s*(0x[0-9a-fA-F]+)/i;
    return defined($v) ? hex($v) : 0;
}

# =============================================================================
# BRAM helpers (shared FIFO BRAM, 72-bit words, PROC mode required)
# write_toggle produces a new value each call so fifo_top detects the edge.
# =============================================================================
my $write_toggle = 0;

sub bram_write {
    my ($word_addr, $hi, $lo) = @_;
    $write_toggle ^= 0x100;
    regwrite($PROC_ADDR_REG,  $word_addr);
    regwrite($WDATA_HI_REG,   $hi);
    regwrite($WDATA_LO_REG,   $lo);
    regwrite($WDATA_CTRL_REG, $write_toggle);  # edge triggers host_we
}

sub bram_read {
    my ($word_addr) = @_;
    regwrite($PROC_ADDR_REG, $word_addr);
    my $hi = regread_val($RDATA_HI_REG);
    my $lo = regread_val($RDATA_LO_REG);
    return ($hi, $lo);
}

# =============================================================================
# ARM programming helpers
# All use proc_addr (byte address) + wdata_lo (32-bit word) + rdata_sel edges.
# =============================================================================

sub arm_imem_write {
    my ($word_idx, $instr) = @_;
    my $byte_addr = $word_idx * 4;
    regwrite($PROC_ADDR_REG, $byte_addr);
    regwrite($WDATA_LO_REG,  $instr);
    # pulse arm_prog_we (rising edge), keep arm_prog_en=1, arm_dmem_sel=0 (IMEM)
    regwrite($RDATA_SEL_REG, $SEL_ARM_PROG_EN);
    regwrite($RDATA_SEL_REG, $SEL_ARM_PROG_EN | $SEL_ARM_PROG_WE);
    regwrite($RDATA_SEL_REG, $SEL_ARM_PROG_EN);
}

sub arm_dmem_write {
    my ($word_idx, $data) = @_;
    my $byte_addr = $word_idx * 4;
    regwrite($PROC_ADDR_REG, $byte_addr);
    regwrite($WDATA_LO_REG,  $data);
    # pulse arm_prog_we, keep arm_prog_en=1, arm_dmem_sel=1 (DMEM)
    regwrite($RDATA_SEL_REG, $SEL_ARM_PROG_EN | $SEL_ARM_DMEM);
    regwrite($RDATA_SEL_REG, $SEL_ARM_PROG_EN | $SEL_ARM_DMEM | $SEL_ARM_PROG_WE);
    regwrite($RDATA_SEL_REG, $SEL_ARM_PROG_EN | $SEL_ARM_DMEM);
}

# Two reads to flush synchronous BRAM read latency (same as Lab 6)
sub arm_dmem_read {
    my ($word_idx) = @_;
    my $byte_addr = $word_idx * 4;
    regwrite($PROC_ADDR_REG, $byte_addr);
    regread_val($RDATA_LO_REG);             # flush
    return regread_val($RDATA_LO_REG);      # valid
}

# =============================================================================
# GPU programming helpers
# =============================================================================

sub gpu_imem_write {
    my ($word_idx, $instr) = @_;
    my $byte_addr = $word_idx * 4;
    regwrite($PROC_ADDR_REG, $byte_addr);
    regwrite($WDATA_LO_REG,  $instr);
    # pulse gpu_prog_we, keep gpu_prog_en=1, gpu_dmem_sel=0 (GPU IMEM)
    regwrite($RDATA_SEL_REG, $SEL_GPU_PROG_EN);
    regwrite($RDATA_SEL_REG, $SEL_GPU_PROG_EN | $SEL_GPU_PROG_WE);
    regwrite($RDATA_SEL_REG, $SEL_GPU_PROG_EN);
}

# =============================================================================
# Poll helper — waits up to TIMEOUT*10ms for a POINTERS bit to assert
# =============================================================================
sub poll_done {
    my ($bit, $label) = @_;
    for my $i (1..$TIMEOUT) {
        my $st = regread_val($POINTERS_REG);
        return 1 if ($st & $bit);
        select(undef, undef, undef, 0.01);
    }
    printf("  TIMEOUT waiting for %s\n", $label);
    $fail++;
    return 0;
}

# =============================================================================
# Check helper — compares 64-bit result (hi+lo) against expected
# =============================================================================
sub check64 {
    my ($label, $got_hi, $got_lo, $exp_hi, $exp_lo) = @_;
    if ($got_hi == $exp_hi && $got_lo == $exp_lo) {
        printf("  PASS  %s\n", $label);
        $pass++;
    } else {
        printf("  FAIL  %s\n        expected  %08x_%08x\n        got       %08x_%08x\n",
               $label, $exp_hi, $exp_lo, $got_hi, $got_lo);
        $fail++;
    }
}

sub check32 {
    my ($label, $got, $exp) = @_;
    if ($got == $exp) {
        printf("  PASS  %s\n", $label);
        $pass++;
    } else {
        printf("  FAIL  %s  expected 0x%08x  got 0x%08x\n", $label, $exp, $got);
        $fail++;
    }
}

# =============================================================================
# TEST START
# =============================================================================
printf "\n%s\n", "=" x 64;
print  "  Lab 8 Integration Test: ARM CPU + GPU BFloat16 Tensor Core\n";
printf "%s\n\n", "=" x 64;

# ─────────────────────────────────────────────────────────────────────────────
print "[1] Reset\n";
regwrite($CMD_REG,       $CMD_FIFO_RESET);
regwrite($CMD_REG,       $CMD_MODE_IDLE);
regwrite($RDATA_SEL_REG, 0x00);
print "    done\n\n";

# ─────────────────────────────────────────────────────────────────────────────
print "[2] Program ARM IMEM\n";
print "    Kernel: Thread 0 adds 1 to each of DMEM[0..13]\n";
print "            Threads 1,2,3 spin (B .)\n";

# hold ARM in reset during programming
regwrite($RDATA_SEL_REG, $SEL_ARM_PROG_EN);

# Thread 0: add 1 to dmem[0..13]
#   r0 = 0 (base address)
#   r1 = 56 (= 14 * 4, byte limit)
#   loop: LDR r3,[r0]; ADD r3,r3,#1; STR r3,[r0]; ADD r0,r0,#4; CMP r0,r1; BLT
#   halt: B .
arm_imem_write( 0, 0xE3A00000);  # MOV r0, #0
arm_imem_write( 1, 0xE3A01038);  # MOV r1, #56
arm_imem_write( 2, 0xE5903000);  # LDR r3, [r0]
arm_imem_write( 3, 0xE2833001);  # ADD r3, r3, #1
arm_imem_write( 4, 0xE5803000);  # STR r3, [r0]
arm_imem_write( 5, 0xE2800004);  # ADD r0, r0, #4
arm_imem_write( 6, 0xE1500001);  # CMP r0, r1
arm_imem_write( 7, 0xBAFFFFF9);  # BLT -7  (back to word 2)
arm_imem_write( 8, 0xEAFFFFFE);  # B .  (halt T0)
arm_imem_write(48, 0xEAFFFFFE);  # B .  (halt T1)
arm_imem_write(96, 0xEAFFFFFE);  # B .  (halt T2)
arm_imem_write(144,0xEAFFFFFE);  # B .  (halt T3)

print "    IMEM loaded\n\n";

# ─────────────────────────────────────────────────────────────────────────────
print "[3] Program ARM DMEM (initial values 0..13)\n";

for my $i (0..13) {
    arm_dmem_write($i, $i);
}

print "    DMEM loaded\n\n";

# ─────────────────────────────────────────────────────────────────────────────
print "[4] Release ARM and start execution\n";

regwrite($RDATA_SEL_REG, 0x00);          # release ARM from prog mode
regwrite($CMD_REG, $CMD_ARM_START);      # rising edge → start
regwrite($CMD_REG, $CMD_MODE_IDLE);      # deassert arm_start

print "    waiting for arm_done...\n";
if (poll_done($PTR_ARM_DONE, "arm_done")) {
    print "    arm_done asserted\n\n";

    print "[4b] Verify ARM DMEM results (expect 1..14)\n";
    for my $i (0..13) {
        my $val = arm_dmem_read($i);
        check32(sprintf("dmem[%2d] == %d", $i, $i + 1), $val, $i + 1);
    }
}

# ─────────────────────────────────────────────────────────────────────────────
print "\n[5] Program GPU IMEM\n";
print "    Kernel: READ_TID→R1, LD64 VecA→R2, LD64 VecB→R3, LD64 VecC→R4,\n";
print "            BF_MAC R5=R2*R3+R4, ST64 R5→BRAM[0]\n";

regwrite($RDATA_SEL_REG, $SEL_GPU_PROG_EN);  # hold GPU in reset

gpu_imem_write(0, 0x04400000);  # READ_TID  R1        (R1=0, single thread)
gpu_imem_write(1, 0x0C800000);  # LD64  R2, R0, 0     (VecA @ BRAM word 0, byte 0x00)
gpu_imem_write(2, 0x0CC00008);  # LD64  R3, R0, 8     (VecB @ BRAM word 1, byte 0x08)
gpu_imem_write(3, 0x0D000010);  # LD64  R4, R0, 16    (VecC @ BRAM word 2, byte 0x10)
gpu_imem_write(4, 0x8548D000);  # BF_MAC R5, R2, R3, R4
gpu_imem_write(5, 0x11440000);  # ST64  R5, R1, 0     (result → BRAM word 0, byte 0x00)

regwrite($RDATA_SEL_REG, 0x00);  # release GPU from prog mode
print "    GPU IMEM loaded\n\n";

# ─────────────────────────────────────────────────────────────────────────────
print "[6] Load GPU operands into shared FIFO BRAM\n";
print "    VecA = 4 x 2.0  (BF16=0x4000) → expect 0x4000400040004000\n";
print "    VecB = 4 x 1.5  (BF16=0x3FC0) → expect 0x3FC03FC03FC03FC0\n";
print "    VecC = 4 x 0.5  (BF16=0x3F00) → expect 0x3F003F003F003F00\n";
print "    Expected result: 2.0*1.5+0.5 = 3.5 (BF16=0x4060) → 0x4060406040604060\n";

regwrite($CMD_REG, $CMD_MODE_PROC);

bram_write(0, 0x40004000, 0x40004000);   # BRAM[0] = VecA: 4 x 2.0
bram_write(1, 0x3FC03FC0, 0x3FC03FC0);   # BRAM[1] = VecB: 4 x 1.5
bram_write(2, 0x3F003F00, 0x3F003F00);   # BRAM[2] = VecC: 4 x 0.5

# Verify BRAM writes before running GPU
my ($va_hi, $va_lo) = bram_read(0);
check64("VecA BRAM write verify", $va_hi, $va_lo, 0x40004000, 0x40004000);

my ($vb_hi, $vb_lo) = bram_read(1);
check64("VecB BRAM write verify", $vb_hi, $vb_lo, 0x3FC03FC0, 0x3FC03FC0);

my ($vc_hi, $vc_lo) = bram_read(2);
check64("VecC BRAM write verify", $vc_hi, $vc_lo, 0x3F003F00, 0x3F003F00);

regwrite($CMD_REG, $CMD_MODE_IDLE);

# ─────────────────────────────────────────────────────────────────────────────
print "\n[7] Start GPU and verify BFloat16 FMA result\n";

regwrite($CMD_REG, $CMD_GPU_START);   # rising edge → gpu_start
regwrite($CMD_REG, $CMD_MODE_IDLE);   # deassert gpu_start

print "    waiting for gpu_done...\n";
if (poll_done($PTR_GPU_DONE, "gpu_done")) {
    print "    gpu_done asserted\n";

    # Read result from BRAM[0] — GPU ST64 wrote to byte 0x00 = word 0
    regwrite($CMD_REG, $CMD_MODE_PROC);
    my ($rhi, $rlo) = bram_read(0);
    regwrite($CMD_REG, $CMD_MODE_IDLE);

    # 2.0 * 1.5 + 0.5 = 3.5
    # BF16 3.5 = sign=0, exp=128 (0x80), mant=0x60 → 0x4060
    # 4 lanes packed: 0x4060406040604060
    check64("GPU BF_MAC result (4 x 3.5)", $rhi, $rlo, 0x40604060, 0x40604060);
}

# ─────────────────────────────────────────────────────────────────────────────
printf "\n%s\n", "=" x 64;
printf "  Results:  PASS=%d   FAIL=%d\n", $pass, $fail;
printf "%s\n", "=" x 64;
print  $fail ? "  FAILURES detected — check values above\n\n"
             : "  ALL TESTS PASSED\n\n";