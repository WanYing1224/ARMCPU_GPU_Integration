# ARM CPU & GPU Integration on NetFPGA

This repository contains the Verilog hardware design, simulation testbenches, and software toolchains for a custom System-on-Chip (SoC) deployed on the NetFPGA platform. The architecture integrates a 5-stage pipelined ARM CPU with a custom GPU featuring a BFloat16 Tensor Core for Multiply-Accumulate (MAC) operations.

The two processors communicate and share data through a dual-port Convertible FIFO BRAM, which also acts as the packet buffer interface to the NetFPGA network ports.

## 🏗 Architecture Overview

* **Top-Level Module (`fifo_top.v`)**: Acts as the integration wrapper routing the PCI-style generic registers to the CPU and GPU, and multiplexing access to the shared BRAM.
  
* **ARM CPU (`arm_cpu_wrapper.v`)**: A 32-bit pipelined ARM processor with its own independent Instruction Memory (IMEM) and Data Memory (DMEM).
  
* **GPU (`gpu_net.v` & `gpu_core_min.v`)**: A parallel coprocessor designed for tensor math. It utilizes a structurally synthesized BFloat16 Multiply-Accumulate unit (`bf16_mac.v`) to perform floating-point operations.
  
* **Register Map (`ids.xml`)**: Defines the NetFPGA software/hardware generic register map (offsets `0x00` through `0x28`) for programming the instruction memories, configuring the FIFO, and triggering execution.

## 📁 Repository Structure

* `CPU/` / `GPU/` / `src/` / `verilog/`: Core Verilog RTL files for the processors, memory modules, and the top-level network integration.
  
* `testbench/` & `sim/`: ModelSim simulation files (`.do` scripts, `.vcd` waveform dumps, and `fifo_top_tb.v`) for verifying the design behavior prior to bitfile compilation.
  
* `sw/`: Software host scripts (`load_and_test.sh`, `fifo_reg.pl`) and pre-compiled machine code (`.mem`, `.hex`) for loading instructions and verifying execution on hardware.
  
* `armisatoolchain/`: Python scripts (`asm2bin_v3.py`) to parse and convert ARM assembly into 32-bit machine code.
  
* `kernel/`: Utilities for translating CUDA/PTX instructions into the custom GPU instruction set.
  
* `include/`: NetFPGA XML configurations (`ids.xml`, `project.xml`) for the register system.
  
* `bitfile/`: The final compiled NetFPGA bitfile (`nf2_top_par.bit`) and generated C-headers (`reg_defines_lab10.h`).

## 🛠 Simulation Testing (ModelSim)

To run the full integration testbench simulating the ARM CPU and GPU sharing the BRAM interface:

1. Navigate to the `sim` directory.
   
2. Execute the simulation script: `do wave.do`
  
3. The testbench (`fifo_top_tb.v`) will program both processors, feed them tensor workloads via the BRAM, and report pass/fail status for the computations.

## 🚀 Hardware Deployment (NetFPGA)

Ensure your NetFPGA node is loaded with the nf2_top_par.bit file. Testing relies on the native regread and regwrite utilities.

1. Navigate to the sw directory.

2. Verify that imem.mem, dmem.hex, and gpu_imem.mem are present.

3. Run the host integration script: `load_and_test.sh`

This script handles the complex MUX logic necessary to bypass the ARM execution state machine and write directly to the GPU's memory arrays via the `GPU_PROG_WE` path. It verifies execution completion using hardware polling on the `POINTERS_REG`.
