// dmem_32.v - Data Memory (Lab 8 version)
// Same as Lab 6 but:
//   - Added Port B write (web, dinb) for wrapper COPY_IN/COPY_OUT
//   - Removed $readmemh (initial data loaded by wrapper from BRAM)
// 256 x 32-bit, true dual-port, synchronous read → Block RAM
`timescale 1ns / 1ps
module dmem_32 (
    // Port A: ARM pipeline
    input  wire        clka,
    input  wire        wea,
    input  wire [7:0]  addra,
    input  wire [31:0] dina,
    output reg  [31:0] douta,
    // Port B: wrapper (Lab 8 addition)
    input  wire        clkb,
    input  wire        web,
    input  wire [7:0]  addrb,
    input  wire [31:0] dinb,
    output reg  [31:0] doutb
);
    reg [31:0] mem [0:255];

    // Port A: synchronous read/write
    always @(posedge clka) begin
        if (wea) mem[addra] <= dina;
        douta <= mem[addra];
    end

    // Port B: synchronous read/write
    always @(posedge clkb) begin
        if (web) mem[addrb] <= dinb;
        doutb <= mem[addrb];
    end
endmodule
