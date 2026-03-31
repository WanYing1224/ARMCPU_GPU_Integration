module Tensor_Unit(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,      
    input  wire [63:0] rs1_data, // Vector A
    input  wire [63:0] rs2_data, // Vector B
    input  wire [63:0] rs3_data, // Vector C
    
    output wire        busy,
    output wire        done,
    output reg  [63:0] acc_out
);

    // Single-cycle combinational execution
    assign busy = 1'b0;
    assign done = start;

    // Wires to hold the outputs of the 4 parallel math cores
    wire [15:0] out_lane3, out_lane2, out_lane1, out_lane0;

    // Lane 3 (Bits 63:48)
    bf16_mac mac_lane3 (
        .a(rs1_data[63:48]), .b(rs2_data[63:48]), .c(rs3_data[63:48]), .z(out_lane3)
    );

    // Lane 2 (Bits 47:32)
    bf16_mac mac_lane2 (
        .a(rs1_data[47:32]), .b(rs2_data[47:32]), .c(rs3_data[47:32]), .z(out_lane2)
    );

    // Lane 1 (Bits 31:16)
    bf16_mac mac_lane1 (
        .a(rs1_data[31:16]), .b(rs2_data[31:16]), .c(rs3_data[31:16]), .z(out_lane1)
    );

    // Lane 0 (Bits 15:0)
    bf16_mac mac_lane0 (
        .a(rs1_data[15:0]), .b(rs2_data[15:0]), .c(rs3_data[15:0]), .z(out_lane0)
    );

    // Pack the 4 independent results back into a 64-bit vector
    always @(*) begin
        if (start) begin
            acc_out = {out_lane3, out_lane2, out_lane1, out_lane0};
        end else begin
            acc_out = 64'd0;
        end
    end

endmodule
