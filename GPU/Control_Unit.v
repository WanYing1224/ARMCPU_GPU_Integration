module Control_Unit(
    input wire [31:0] instr_id,      // Instruction in ID stage
    input wire tensor_busy,   		 // Signal from multi-cycle Tensor Unit
    input wire branch_eval,  		 // Branch condition evaluated in EX stage
    
    // Scoreboard tracking (Destinations in later stages)
    input wire [3:0] rd_ex, rd_mem, rd_wb, // rd_rr
    input wire we_ex, we_mem, we_wb, // we_rr

    output reg stall_fetch,
    output reg stall_decode,
    output reg flush_decode,
    output reg flush_execute,
    output reg reg_write_en,
    output reg mem_write_en,
	output reg tensor_start
);

    wire [5:0] opcode = instr_id[31:26];
    wire [3:0] rs1 = instr_id[21:18];
    wire [3:0] rs2 = instr_id[17:14];
	wire [3:0] rs3 = instr_id[13:10];

    // Scoreboard / RAW Hazard Detection
    // If Rs1 or Rs2 match an active Rd in a later stage => stall.
	// Take (rs1 == rd_rr && we_rr) off to fix Data Hazard (RAW) 
    wire hazard_rs1 = (rs1 != 0) && (/*(rs1 == rd_rr && we_rr) ||*/ (rs1 == rd_ex && we_ex) || (rs1 == rd_mem && we_mem) || (rs1 == rd_wb && we_wb));
    wire hazard_rs2 = (rs2 != 0) && (/*(rs2 == rd_rr && we_rr) ||*/ (rs2 == rd_ex && we_ex) || (rs2 == rd_mem && we_mem) || (rs2 == rd_wb && we_wb));
	wire hazard_rs3 = (rs3 != 0) && ((rs3 == rd_ex && we_ex) || (rs3 == rd_mem && we_mem) || (rs3 == rd_wb && we_wb));
    
    wire raw_hazard = hazard_rs1 | hazard_rs2 | hazard_rs3;

    always @(*) 
	begin
        // Default assignments
        stall_fetch = 1'b0;
        stall_decode = 1'b0;
        flush_decode = 1'b0;
        flush_execute = 1'b0;
        reg_write_en = 1'b0;
        mem_write_en = 1'b0;
		tensor_start = 1'b0;

        // Handle Tensor Unit Busy (TF_WAIT)
        if(opcode == 6'b100100 && tensor_busy) 
		begin
            stall_fetch  = 1'b1;
            stall_decode = 1'b1;
        end 
        
        // Handle Read-After-Write (RAW) Hazards via Scoreboard
        else if(raw_hazard) 
		begin
            stall_fetch  = 1'b1;
            stall_decode = 1'b1;
            // Flush the RR stage bubble
            flush_execute = 1'b1; 
        end

        // Handle Branch Misprediction / Flush
        if(branch_eval) 
		begin
            // If taken in EX, flush IF & ID
            flush_decode  = 1'b1;
            flush_execute = 1'b1; 
            stall_fetch   = 1'b0;	// Let fetch new target
        end

        // Decode specific write enables
        //if(opcode == 6'b010000 || opcode == 6'b000001 || opcode == 6'b000010) reg_write_en = 1'b1;  // ADD_I16
		if(opcode == 6'b010000 || opcode == 6'b010001 || opcode == 6'b000001 || opcode == 6'b000010 || 
			opcode == 6'b000011 || opcode == 6'b100000 || opcode == 6'b100001 || opcode == 6'b100010)
		begin
			reg_write_en = 1'b1;
		end
		
        if(opcode == 6'b000100) mem_write_en = 1'b1;  // ST64
		// if(opcode == 6'b100011) tensor_start = 1'b1;  // OP_TF_START -> Triggers the Tensor Unit!
		
		if(opcode == 6'b100000 || opcode == 6'b100001 || opcode == 6'b100011) 
		begin
			tensor_start = 1'b1; 
		end
    end

endmodule
