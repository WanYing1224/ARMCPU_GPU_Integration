`timescale 1ns/1ps
///////////////////////////////////////////////////////////////////////////////
// arm_cpu_wrapper.v - BRAM Bridge for Lab 6 ARM Pipeline
//
// Wraps the full 4-thread ARM pipeline (arm_pipeline + arm_alu + arm_regfile
// + imem + dmem_32) with a BRAM bridge for Lab 8 integration.
//
// Workflow:
//   COPY_IN:  BRAM[6..12] → dmem[0..13]  (7x64-bit → 14x32-bit)
//   RUN:      Release ARM from reset, execute for 512 cycles
//   COPY_OUT: dmem[0..13] → BRAM[6..12]  (14x32-bit → 7x64-bit)
//
// Same external interface as arm_net_proc (drop-in replacement in fifo_top).
///////////////////////////////////////////////////////////////////////////////

module arm_cpu_wrapper (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    output reg         done,
    output wire        running,

    // BRAM interface (shared with Convertible FIFO)
    output reg  [7:0]  bram_addr,
    output reg  [71:0] bram_wdata,
    input  wire [71:0] bram_rdata,
    output reg         bram_we
);

    reg active;
    assign running = active;

    // =================================================================
    // Wrapper State Machine
    // =================================================================
    localparam W_IDLE       = 4'd0;
    localparam W_CI_ADDR    = 4'd1;   // COPY_IN: issue BRAM read
    localparam W_CI_WAIT    = 4'd2;   // BRAM latency
    localparam W_CI_HI      = 4'd3;   // write hi32 to dmem
    localparam W_CI_LO      = 4'd4;   // write lo32 to dmem
    localparam W_RUN        = 4'd5;   // ARM executing
    localparam W_CO_RD_HI   = 4'd6;   // COPY_OUT: read hi32 from dmem
    localparam W_CO_HI_LAT  = 4'd7;   // dmem latency
    localparam W_CO_RD_LO   = 4'd8;   // read lo32, latch hi32
    localparam W_CO_LO_LAT  = 4'd9;   // dmem latency
    localparam W_CO_WR_BRAM = 4'd10;  // write packed 64-bit to BRAM
    localparam W_DONE       = 4'd11;

    reg [3:0]  w_state;
    reg [3:0]  w_idx;          // 0..6 for 7 BRAM words
    reg [9:0]  run_counter;
    reg        arm_rst;        // active-high reset for ARM pipeline
    reg [63:0] ci_latch;       // latch BRAM read data
    reg [31:0] co_hi_latch;    // latch dmem hi32 during copy_out

    // dmem Port B signals (wrapper ↔ dmem)
    reg        dmem_web;
    reg [7:0]  dmem_addrb;
    reg [31:0] dmem_dinb;
    wire [31:0] dmem_doutb;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_state     <= W_IDLE;
            w_idx       <= 0;
            run_counter <= 0;
            arm_rst     <= 1'b1;
            active      <= 0;
            done        <= 0;
            bram_addr   <= 0;
            bram_wdata  <= 0;
            bram_we     <= 0;
            dmem_web    <= 0;
            dmem_addrb  <= 0;
            dmem_dinb   <= 0;
            ci_latch    <= 0;
            co_hi_latch <= 0;
        end else begin
            bram_we  <= 1'b0;
            dmem_web <= 1'b0;

            case (w_state)
                W_IDLE: begin
                    if (start) begin
                        active  <= 1;
                        done    <= 0;
                        arm_rst <= 1'b1;  // keep ARM in reset
                        w_idx   <= 0;
                        w_state <= W_CI_ADDR;
                    end
                end

                // ---- COPY_IN: BRAM[6+idx] → dmem[2*idx], dmem[2*idx+1] ----
                W_CI_ADDR: begin
                    bram_addr <= 8'd6 + {4'd0, w_idx};
                    w_state   <= W_CI_WAIT;
                end
                W_CI_WAIT: begin
                    w_state <= W_CI_HI;  // 1-cycle BRAM latency
                end
                W_CI_HI: begin
                    ci_latch   <= bram_rdata[63:0];
                    dmem_web   <= 1'b1;
                    dmem_addrb <= {3'b0, w_idx, 1'b0};  // dmem[2*idx]
                    dmem_dinb  <= bram_rdata[63:32];     // hi32
                    w_state    <= W_CI_LO;
                end
                W_CI_LO: begin
                    dmem_web   <= 1'b1;
                    dmem_addrb <= {3'b0, w_idx, 1'b1};  // dmem[2*idx+1]
                    dmem_dinb  <= ci_latch[31:0];        // lo32
                    if (w_idx == 4'd6) begin
                        arm_rst     <= 1'b0;  // release ARM
                        run_counter <= 0;
                        w_state     <= W_RUN;
                    end else begin
                        w_idx   <= w_idx + 1;
                        w_state <= W_CI_ADDR;
                    end
                end

                // ---- RUN: ARM executes for 512 cycles ----
                W_RUN: begin
                    run_counter <= run_counter + 1;
                    if (run_counter == 10'd512) begin
                        arm_rst <= 1'b1;  // freeze ARM
                        w_idx   <= 0;
                        w_state <= W_CO_RD_HI;
                    end
                end

                // ---- COPY_OUT: dmem[2*idx],dmem[2*idx+1] → BRAM[6+idx] ----
                W_CO_RD_HI: begin
                    dmem_addrb <= {3'b0, w_idx, 1'b0};  // dmem[2*idx]
                    w_state    <= W_CO_HI_LAT;
                end
                W_CO_HI_LAT: begin
                    w_state <= W_CO_RD_LO;  // 1-cycle dmem latency
                end
                W_CO_RD_LO: begin
                    co_hi_latch <= dmem_doutb;            // latch hi32
                    dmem_addrb  <= {3'b0, w_idx, 1'b1};  // dmem[2*idx+1]
                    w_state     <= W_CO_LO_LAT;
                end
                W_CO_LO_LAT: begin
                    w_state <= W_CO_WR_BRAM;  // 1-cycle dmem latency
                end
                W_CO_WR_BRAM: begin
                    bram_addr  <= 8'd6 + {4'd0, w_idx};
                    bram_wdata <= {8'h00, co_hi_latch, dmem_doutb};  // pack 64-bit
                    bram_we    <= 1'b1;
                    if (w_idx == 4'd6)
                        w_state <= W_DONE;
                    else begin
                        w_idx   <= w_idx + 1;
                        w_state <= W_CO_RD_HI;
                    end
                end

                W_DONE: begin
                    active  <= 0;
                    done    <= 1;
                    w_state <= W_IDLE;
                end

                default: w_state <= W_IDLE;
            endcase
        end
    end

    // =================================================================
    // ARM Pipeline Instance (Lab 6 full 4-thread)
    // =================================================================
    arm_pipeline arm_inst (
        .clk             (clk),
        .rst             (arm_rst),
        // dmem Port B exposed for wrapper
        .ext_dmem_web    (dmem_web),
        .ext_dmem_addrb  (dmem_addrb),
        .ext_dmem_dinb   (dmem_dinb),
        .ext_dmem_doutb  (dmem_doutb)
    );

endmodule
