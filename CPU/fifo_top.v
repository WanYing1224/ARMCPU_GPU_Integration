`timescale 1ns/1ps
///////////////////////////////////////////////////////////////////////////////
// fifo_top.v - Network Processor Integration (Lab 8)
//
// Integrates:
//   - Convertible FIFO (packet capture/send via BRAM)
//   - ARM network processor (reads/modifies packet data)
//   - GPU tensor processor (bf16 SIMD operations on packet data)
//
// BRAM Port B MUX: host(regwrite) / ARM / GPU share access
//
// Register Map:
//   SW reg 0: CMD
//     [1:0] = FIFO mode (00=IDLE/passthrough, 01=PROC, 10=FIFO_OUT, 11=FIFO_IN)
//     [2]   = FIFO reset (pulse)
//     [3]   = ARM start (pulse)
//     [4]   = GPU start (pulse)
//   SW reg 1: PROC_ADDR    - BRAM addr for host read/write
//   SW reg 2: WDATA_HI     - data[63:32]
//   SW reg 3: WDATA_LO     - data[31:0]
//   SW reg 4: WDATA_CTRL   - ctrl[7:0] + write trigger
//
//   HW reg 0: STATUS       - [1:0]=mode, [2]=pkt_ready, [15:8]=pkt_len
//   HW reg 1: RDATA_HI     - data[63:32] at PROC_ADDR
//   HW reg 2: RDATA_LO     - data[31:0]
//   HW reg 3: RDATA_CTRL   - ctrl[7:0]
//   HW reg 4: PROC_STATUS  - [0]=arm_done, [1]=arm_running, [2]=gpu_done, [3]=gpu_running, [15:8]=tail, [23:16]=head
///////////////////////////////////////////////////////////////////////////////

module fifo_top
   #(
      parameter DATA_WIDTH = 64,
      parameter CTRL_WIDTH = DATA_WIDTH/8,
      parameter UDP_REG_SRC_WIDTH = 2
   )
   (
      input  [DATA_WIDTH-1:0]             in_data,
      input  [CTRL_WIDTH-1:0]             in_ctrl,
      input                               in_wr,
      output                              in_rdy,

      output [DATA_WIDTH-1:0]             out_data,
      output [CTRL_WIDTH-1:0]             out_ctrl,
      output                              out_wr,
      input                               out_rdy,

      input                               reg_req_in,
      input                               reg_ack_in,
      input                               reg_rd_wr_L_in,
      input  [`UDP_REG_ADDR_WIDTH-1:0]    reg_addr_in,
      input  [`CPCI_NF2_DATA_WIDTH-1:0]   reg_data_in,
      input  [UDP_REG_SRC_WIDTH-1:0]      reg_src_in,

      output                              reg_req_out,
      output                              reg_ack_out,
      output                              reg_rd_wr_L_out,
      output [`UDP_REG_ADDR_WIDTH-1:0]    reg_addr_out,
      output [`CPCI_NF2_DATA_WIDTH-1:0]   reg_data_out,
      output [UDP_REG_SRC_WIDTH-1:0]      reg_src_out,

      input                               reset,
      input                               clk
   );

   // ============================================================
   // Software Registers
   // ============================================================
   wire [31:0] sw_cmd;
   wire [31:0] sw_proc_addr;
   wire [31:0] sw_wdata_hi;
   wire [31:0] sw_wdata_lo;
   wire [31:0] sw_wdata_ctrl;

   wire [1:0] fifo_mode = sw_cmd[1:0];
   wire       cmd_reset = sw_cmd[2];

   // ============================================================
   // ARM / GPU start pulse edge detection
   // ============================================================
   reg sw_cmd3_prev, sw_cmd4_prev;
   wire arm_start = sw_cmd[3] && !sw_cmd3_prev;
   wire gpu_start = sw_cmd[4] && !sw_cmd4_prev;

   always @(posedge clk) begin
      if (reset) begin
         sw_cmd3_prev <= 1'b0;
         sw_cmd4_prev <= 1'b0;
      end else begin
         sw_cmd3_prev <= sw_cmd[3];
         sw_cmd4_prev <= sw_cmd[4];
      end
   end

   // ============================================================
   // Forward declarations (used before module instantiation)
   // ============================================================
   wire        arm_running;
   wire        gpu_running;
   wire [71:0] proc_rdata;

   // ============================================================
   // Host write trigger (edge detect on sw_wdata_ctrl)
   // ============================================================
   reg [31:0] sw_wdata_ctrl_prev;
   wire host_we = (sw_wdata_ctrl != sw_wdata_ctrl_prev) &&
                  (fifo_mode == 2'b01) && !arm_running && !gpu_running;

   always @(posedge clk) begin
      if (reset)
         sw_wdata_ctrl_prev <= 32'b0;
      else
         sw_wdata_ctrl_prev <= sw_wdata_ctrl;
   end

   // ============================================================
   // ARM Network Processor
   // ============================================================
   wire        arm_done;
   wire [7:0]  arm_bram_addr;
   wire [71:0] arm_bram_wdata;
   wire        arm_bram_we;

   arm_cpu_wrapper arm_inst (
      .clk       (clk),
      .rst_n     (~reset),
      .start     (arm_start),
      .done      (arm_done),
      .running   (arm_running),
      .bram_addr (arm_bram_addr),
      .bram_wdata(arm_bram_wdata),
      .bram_rdata(proc_rdata),     // reads from BRAM Port B
      .bram_we   (arm_bram_we)
   );

   // ============================================================
   // GPU Tensor Processor
   // ============================================================
   wire        gpu_done;
   wire [7:0]  gpu_bram_addr;
   wire [71:0] gpu_bram_wdata;
   wire        gpu_bram_we;

   gpu_net gpu_inst (
      .clk       (clk),
      .rst_n     (~reset),
      .start     (gpu_start),
      .done      (gpu_done),
      .running   (gpu_running),
      .bram_addr (gpu_bram_addr),
      .bram_wdata(gpu_bram_wdata),
      .bram_rdata(proc_rdata),     // reads from BRAM Port B
      .bram_we   (gpu_bram_we)
   );

   // ============================================================
   // BRAM Port B MUX: ARM > GPU > Host
   // ============================================================
   wire [7:0]  mux_addr;
   wire [71:0] mux_wdata;
   wire        mux_we;

   assign mux_addr  = arm_running ? arm_bram_addr :
                      gpu_running ? gpu_bram_addr  :
                      sw_proc_addr[7:0];

   assign mux_wdata = arm_running ? arm_bram_wdata :
                      gpu_running ? gpu_bram_wdata  :
                      {sw_wdata_ctrl[7:0], sw_wdata_hi, sw_wdata_lo};

   assign mux_we    = arm_running ? arm_bram_we :
                      gpu_running ? gpu_bram_we  :
                      host_we;

   // ============================================================
   // Convertible FIFO
   // ============================================================
   wire        pkt_ready;
   wire [7:0]  pkt_len;
   wire [7:0]  head_addr, tail_addr;

   wire [63:0] fifo_out_data;
   wire [7:0]  fifo_out_ctrl;
   wire        fifo_out_wr;
   wire        fifo_in_rdy;

   convertible_fifo fifo_inst (
      .clk          (clk),
      .reset        (reset),

      .net_in_data  (in_data),
      .net_in_ctrl  (in_ctrl),
      .net_in_wr    (in_wr && (fifo_mode == 2'b11)),
      .net_in_rdy   (fifo_in_rdy),

      .net_out_data (fifo_out_data),
      .net_out_ctrl (fifo_out_ctrl),
      .net_out_wr   (fifo_out_wr),
      .net_out_rdy  (out_rdy),

      .proc_addr    (mux_addr),
      .proc_wdata   (mux_wdata),
      .proc_rdata   (proc_rdata),
      .proc_we      (mux_we),

      .mode         (fifo_mode),
      .cmd_send     (1'b0),
      .cmd_reset    (cmd_reset),
      .pkt_ready    (pkt_ready),
      .pkt_len      (pkt_len),
      .head_addr    (head_addr),
      .tail_addr    (tail_addr)
   );

   // ============================================================
   // Passthrough MUX (IDLE mode = direct passthrough)
   // ============================================================
   assign out_data = (fifo_mode == 2'b00) ? in_data      : fifo_out_data;
   assign out_ctrl = (fifo_mode == 2'b00) ? in_ctrl      : fifo_out_ctrl;
   assign out_wr   = (fifo_mode == 2'b00) ? in_wr        : fifo_out_wr;
   assign in_rdy   = (fifo_mode == 2'b00) ? out_rdy      :
                     (fifo_mode == 2'b11) ? fifo_in_rdy  : 1'b0;

   // ============================================================
   // Hardware Registers
   // ============================================================
   wire [31:0] hw_status     = {16'b0, pkt_len, 5'b0, pkt_ready, fifo_mode};
   wire [31:0] hw_rdata_hi   = proc_rdata[63:32];
   wire [31:0] hw_rdata_lo   = proc_rdata[31:0];
   wire [31:0] hw_rdata_ctrl = {24'b0, proc_rdata[71:64]};
   wire [31:0] hw_proc_status = {8'b0, head_addr, tail_addr,
                                  4'b0, gpu_running, gpu_done,
                                  arm_running, arm_done};

   // ============================================================
   // Generic Regs: 5 SW + 5 HW
   // ============================================================
   generic_regs
   #(
      .UDP_REG_SRC_WIDTH   (UDP_REG_SRC_WIDTH),
      .TAG                 (`FIFO_BLOCK_ADDR),
      .REG_ADDR_WIDTH      (`FIFO_REG_ADDR_WIDTH),
      .NUM_COUNTERS        (0),
      .NUM_SOFTWARE_REGS   (5),
      .NUM_HARDWARE_REGS   (5)
   ) fifo_regs (
      .reg_req_in       (reg_req_in),
      .reg_ack_in       (reg_ack_in),
      .reg_rd_wr_L_in   (reg_rd_wr_L_in),
      .reg_addr_in      (reg_addr_in),
      .reg_data_in      (reg_data_in),
      .reg_src_in       (reg_src_in),

      .reg_req_out      (reg_req_out),
      .reg_ack_out      (reg_ack_out),
      .reg_rd_wr_L_out  (reg_rd_wr_L_out),
      .reg_addr_out     (reg_addr_out),
      .reg_data_out     (reg_data_out),
      .reg_src_out      (reg_src_out),

      .counter_updates  (),
      .counter_decrement(),

      .software_regs    ({sw_wdata_ctrl, sw_wdata_lo, sw_wdata_hi,
                          sw_proc_addr, sw_cmd}),

      .hardware_regs    ({hw_proc_status, hw_rdata_ctrl, hw_rdata_lo,
                          hw_rdata_hi, hw_status}),

      .clk              (clk),
      .reset            (reset)
   );

endmodule
