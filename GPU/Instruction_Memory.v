module Instruction_Memory #(
    parameter MEM_DEPTH = 1024 // 4KB of Instruction Memory
)(
    input  wire [31:0] pc,
    output wire [31:0] instr
);

    // Inferred BRAM
    reg [31:0] rom [0:MEM_DEPTH-1];
	
	integer i;

    // Initialize from the Python compiler's output
    initial 
	begin
        for(i = 0; i < MEM_DEPTH; i = i + 1)
		begin
            rom[i] = 32'd0;
        end
		
		$readmemh("../hex_file/gpu_program.hex", rom);
    end

    // Fetch interface (Word-aligned addressing)
    assign instr = rom[pc[11:2]]; 

endmodule
