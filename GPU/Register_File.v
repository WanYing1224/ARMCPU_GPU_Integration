module Register_File (
    input  wire        clk,
    input  wire        we,          // Write Enable
    input  wire [3:0]  rs1_addr,    // Read Port 1 Address
    input  wire [3:0]  rs2_addr,    // Read Port 2 Address
    input  wire [3:0]  rs3_addr,    // NEW: Read Port 3 Address
    input  wire [3:0]  rd_addr,     // Write Port Address
    input  wire [63:0] write_data,  // Data to write
    output wire [63:0] rs1_data,    // Read Port 1 Data
    output wire [63:0] rs2_data,    // Read Port 2 Data
    output wire [63:0] rs3_data     // NEW: Read Port 3 Data
);

    reg [63:0] registers [0:15];

    // Asynchronous/Combinational Reads (to prevent 1-cycle latency)
    assign rs1_data = (rs1_addr == 0) ? 64'd0 : registers[rs1_addr];
    assign rs2_data = (rs2_addr == 0) ? 64'd0 : registers[rs2_addr];
    assign rs3_data = (rs3_addr == 0) ? 64'd0 : registers[rs3_addr];

    always @(posedge clk) begin
        if (we && rd_addr != 0) begin
            registers[rd_addr] <= write_data;
        end
    end
endmodule
