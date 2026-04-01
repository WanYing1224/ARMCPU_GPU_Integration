`timescale 1ns/1ps
// ============================================================
// tensor_unit.v — GPU Tensor Unit (4-lane BF16 MAC)
//
// Lab 10 change: bf16_mac now instantiates MULT18X18S, which
// requires a clock port. clk is passed to each lane instance.
//
// Everything else is unchanged from Lab 9 — the 1-cycle pipeline
// register and handshake signals (busy, done) are the same.
// ============================================================
module tensor_unit (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,       // pulse from FSM EXEC state
    input  wire [63:0] rs1,         // Vec A (4 x BF16)
    input  wire [63:0] rs2,         // Vec B (4 x BF16)
    input  wire [63:0] rs3,         // Vec C (4 x BF16)
    output wire        busy,        // stall FSM while computing
    output wire        done,        // result ready
    output reg  [63:0] result       // packed 4 x BF16 result
);

    // 4 parallel BF16 MAC lanes — one MULT18X18S each
    wire [15:0] mac0, mac1, mac2, mac3;

    bf16_mac lane0 (
        .clk (clk),
        .a   (rs1[15:0]),  .b (rs2[15:0]),  .c (rs3[15:0]),  .z (mac0)
    );
    bf16_mac lane1 (
        .clk (clk),
        .a   (rs1[31:16]), .b (rs2[31:16]), .c (rs3[31:16]), .z (mac1)
    );
    bf16_mac lane2 (
        .clk (clk),
        .a   (rs1[47:32]), .b (rs2[47:32]), .c (rs3[47:32]), .z (mac2)
    );
    bf16_mac lane3 (
        .clk (clk),
        .a   (rs1[63:48]), .b (rs2[63:48]), .c (rs3[63:48]), .z (mac3)
    );

    // 1-cycle pipeline register — unchanged from original
    reg running;
    assign busy = start & ~running;
    assign done = running;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            running <= 1'b0;
            result  <= 64'd0;
        end else begin
            running <= start;
            if (start)
                result <= {mac3, mac2, mac1, mac0};
        end
    end

endmodule
