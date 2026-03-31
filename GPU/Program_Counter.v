module Program_Counter(
    input  wire        clk,
    input  wire        rst,
    input  wire        stall_en,     // High when scoreboard or Tensor Unit stalls
    input  wire        branch_taken, // From EX stage (Control Unit)
    input  wire [31:0] branch_target,
    output reg  [31:0] pc
);

    always @(posedge clk or posedge rst) begin
        if(rst)
		begin
            pc <= 32'h00000000;
        end 
		
		else if (!stall_en) 
		begin
            if(branch_taken) 
			begin
                pc <= branch_target;
            end 
			else 
			begin
                pc <= pc + 32'd4; // PC+4 logic
            end
        end
        // If stall_en is high, PC holds its value
    end
endmodule
