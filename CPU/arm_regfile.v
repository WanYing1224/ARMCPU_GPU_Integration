`timescale 1ns/1ps
module arm_regfile (
    input  wire        clk,
    input  wire        rst,
    
    input  wire [1:0]  thread_id,   
    input  wire [1:0]  wb_thread,  
   
    input  wire [3:0]  r0addr,
    output wire [31:0] r0data,
    input  wire [3:0]  r1addr,
    output wire [31:0] r1data,
    
    input  wire        we,
    input  wire [3:0]  waddr,
    input  wire [31:0] wdata,
    
    input  wire        we2,
    input  wire [3:0]  waddr2,
    input  wire [31:0] wdata2
);
  
    reg [31:0] regs [0:63];
    integer i;

    
    wire [5:0] rd_addr0 = {thread_id, r0addr};
    wire [5:0] rd_addr1 = {thread_id, r1addr};
    wire [5:0] wr_addr  = {wb_thread, waddr};
    wire [5:0] wr_addr2 = {wb_thread, waddr2};

    
  
    assign r0data = (we2 && wr_addr2 == rd_addr0) ? wdata2 :
                    (we  && wr_addr  == rd_addr0) ? wdata  : regs[rd_addr0];
    assign r1data = (we2 && wr_addr2 == rd_addr1) ? wdata2 :
                    (we  && wr_addr  == rd_addr1) ? wdata  : regs[rd_addr1];

  
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 64; i = i + 1)
                regs[i] <= 32'b0;
        end else begin
            if (we)
                regs[wr_addr]  <= wdata;
            if (we2)
                regs[wr_addr2] <= wdata2;
        end
    end
endmodule
