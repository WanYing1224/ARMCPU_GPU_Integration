// imem.v - Instruction Memory (Lab 8 version)
// Same interface as Lab 6, but case ROM for synthesis.
// T0 (PC=0..47): Packet processing - add 1 to each of 14 dmem words
// T1/T2/T3: Spin (B .)
// 4-thread round-robin → no NOPs needed between dependent instructions.
`timescale 1ns/1ps
module imem (
    input  wire [8:0]  addr,
    output wire [31:0] dout,
    input  wire [8:0]  data_addr,
    output wire [31:0] data_dout,
    input  wire        clk,
    input  wire        we,
    input  wire [31:0] din
);
    // Primary read port (instruction fetch)
    reg [31:0] dout_r;
    always @(*) begin
        case (addr)
            // === Thread 0: add 1 to dmem[0..13] ===
            9'd0:  dout_r = 32'hE3A00000; // MOV r0, #0
            9'd1:  dout_r = 32'hE3A01038; // MOV r1, #56  (14*4)
            9'd2:  dout_r = 32'hE5903000; // LDR r3, [r0]
            9'd3:  dout_r = 32'hE2833001; // ADD r3, r3, #1
            9'd4:  dout_r = 32'hE5803000; // STR r3, [r0]
            9'd5:  dout_r = 32'hE2800004; // ADD r0, r0, #4
            9'd6:  dout_r = 32'hE1500001; // CMP r0, r1
            9'd7:  dout_r = 32'hBAFFFFF9; // BLT pc=2
            9'd8:  dout_r = 32'hEAFFFFFE; // B . (halt)
            // === Thread 1: spin ===
            9'd48: dout_r = 32'hEAFFFFFE; // B .
            // === Thread 2: spin ===
            9'd96: dout_r = 32'hEAFFFFFE; // B .
            // === Thread 3: spin ===
            9'd144:dout_r = 32'hEAFFFFFE; // B .
            default: dout_r = 32'hE1A0B00B; // NOP
        endcase
    end
    assign dout = dout_r;

    // Secondary read port (PC-relative LDR)
    reg [31:0] data_dout_r;
    always @(*) begin
        case (data_addr)
            9'd0:  data_dout_r = 32'hE3A00000;
            9'd1:  data_dout_r = 32'hE3A01038;
            9'd2:  data_dout_r = 32'hE5903000;
            9'd3:  data_dout_r = 32'hE2833001;
            9'd4:  data_dout_r = 32'hE5803000;
            9'd5:  data_dout_r = 32'hE2800004;
            9'd6:  data_dout_r = 32'hE1500001;
            9'd7:  data_dout_r = 32'hBAFFFFF9;
            9'd8:  data_dout_r = 32'hEAFFFFFE;
            9'd48: data_dout_r = 32'hEAFFFFFE;
            9'd96: data_dout_r = 32'hEAFFFFFE;
            9'd144:data_dout_r = 32'hEAFFFFFE;
            default: data_dout_r = 32'hE1A0B00B;
        endcase
    end
    assign data_dout = data_dout_r;

endmodule
