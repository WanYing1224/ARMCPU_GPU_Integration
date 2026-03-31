module Data_Memory #(
    parameter MEM_DEPTH = 1024
)(
    input wire clk,
    input wire we,
    input wire [31:0] addr,       // Calculated Base + Offset
    input wire [63:0] write_data, // From Rs_src (ST64)
	
    output wire [63:0] read_data   // To Rd (LD64)
);

    reg [63:0] ram [0:MEM_DEPTH-1];
	
	initial begin
        $readmemh("../hex_file/data_memory.hex", ram);
    end

    assign read_data = ram[addr[11:3]];
	
	always @(posedge clk) 
	begin
        if(we) 
		begin
            ram[addr[11:3]] <= write_data; // 64-bit aligned addressing
        end
		
    end
	
endmodule
