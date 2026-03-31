module Execution_Unit(
    input wire [5:0] opcode,
    input wire [63:0] rs1_data,
    input wire [63:0] rs2_data,
	
    output reg [63:0] alu_out,
    output reg cmp_flag
);

    // Unpack the 64-bit registers into four 16-bit lanes
    wire signed [15:0] a0 = rs1_data[15:0], a1 = rs1_data[31:16], a2 = rs1_data[47:32], a3 = rs1_data[63:48];
    wire signed [15:0] b0 = rs2_data[15:0], b1 = rs2_data[31:16], b2 = rs2_data[47:32], b3 = rs2_data[63:48];

    // BFloat16 Multiplier Wires (Assuming external sub-modules)
    wire [15:0] bf_mul0, bf_mul1, bf_mul2, bf_mul3;
    
    // Instantiate 4 parallel BF16 Combinational Multipliers
    // BF16_Mul m0 (.a(a0), .b(b0), .out(bf_mul0)); ... (omitted for brevity)

    always @(*) 
	begin
        alu_out = 64'd0;
        cmp_flag = 1'b0;

        case(opcode)
            6'b010000: // OP_ADD_I16
			begin 
                alu_out[15:0]  = a0 + b0;
                alu_out[31:16] = a1 + b1;
                alu_out[47:32] = a2 + b2;
                alu_out[63:48] = a3 + b3;
            end
			
            6'b010001: // OP_SUB_I16
			begin 
                alu_out[15:0]  = a0 - b0;
                alu_out[31:16] = a1 - b1;
                alu_out[47:32] = a2 - b2;
                alu_out[63:48] = a3 - b3;
            end
			
            6'b010010: // OP_CMP_I16 (Sets flag if Rs1 < Rs2 for branch)
			begin
                cmp_flag = (a0 < b0); // Simplified to evaluate lane 0 for loop bounds
            end
			
            6'b100000: // OP_BF_MUL
			begin
                alu_out = {bf_mul3, bf_mul2, bf_mul1, bf_mul0};
            end
			
            6'b100010: // OP_RELU (INT16 example)
			begin
                // Compute the Rectified Linear Unit: out[i]=max(0,in[i])[cite: 55].
                alu_out[15:0]  = (a0 > 0) ? a0 : 16'd0;
                alu_out[31:16] = (a1 > 0) ? a1 : 16'd0;
                alu_out[47:32] = (a2 > 0) ? a2 : 16'd0;
                alu_out[63:48] = (a3 > 0) ? a3 : 16'd0;
            end
			
            default: alu_out = 64'd0;
			
        endcase
    end

endmodule
