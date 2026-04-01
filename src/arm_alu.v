// ============================================================
// arm_alu.v — 32-bit Structural ALU (Lab 10)
//
// Structural changes from behavioral original:
//
//   ADD/SUB arithmetic:
//     Replaced behavioral 33-bit + and - operators with an
//     explicit MUXCY + XORCY carry chain (33 stages).
//     MUXCY propagates or generates carry per bit.
//     XORCY produces the sum bit per bit.
//     carry[32] is the carry-out for C flag.
//     V flag = carry[32] XOR carry[31] (standard Xilinx method).
//     SUB is implemented as A + ~B + 1 (two's complement):
//       B_eff[i] = B[i] ^ do_sub  via LUT2
//       carry_in = do_sub
//     This uses 0 LUTs for carry propagation — only slice FAs.
//
//   Bitwise AND/OR/XOR:
//     Replaced behavioral A&B / A|B / A^B with explicit LUT2
//     instantiation per bit. Each LUT2 is one gate.
//
//   MOV:
//     Direct wire assignment — 0 LUTs.
//
//   Result MUX (op select):
//     Left as a small combinational case statement.
//     This is 6 choices across 32 bits = ~32 LUT4s total.
//     XST infers MUXF5/MUXF6 from the case — this is
//     already optimal and not worth manually unrolling.
//
//   Flags:
//     N = result[31]        — wire, 0 LUTs
//     Z = (result==0)       — NOR chain, ~5 LUTs (XST uses carry)
//     C = carry[32]         — wire from carry chain, 0 LUTs
//     V = carry[32]^carry[31] via LUT2 — 1 LUT total
//
// Primitives used: MUXCY, XORCY, LUT2
// Verilog-2001 / Virtex-II Pro (unisim)
// ============================================================
`timescale 1ns/1ps
module arm_alu (
    input  wire [31:0] A,
    input  wire [31:0] B,
    input  wire [2:0]  op,

    output reg  [31:0] result,
    output wire        N,
    output wire        Z,
    output reg         C,
    output reg         V
);

    // ── SUB control: A - B = A + ~B + 1 ─────────────────────────
    wire do_sub = (op == 3'b001);

    // ── B_eff: B XOR do_sub per bit (LUT2 per bit) ───────────────
    wire [31:0] B_eff;
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : b_invert
            // LUT2 INIT=4'h6 implements XOR: O = I0 ^ I1
            LUT2 #(.INIT(4'h6)) b_xor (
                .I0 (B[i]),
                .I1 (do_sub),
                .O  (B_eff[i])
            );
        end
    endgenerate

    // ── MUXCY + XORCY carry chain (33 stages) ────────────────────
    // carry[0] = cin = do_sub  (0 for ADD, 1 for SUB)
    // carry[i+1] = carry out from stage i
    // sum[i]    = result bit i
    wire [32:0] carry;
    wire [31:0] sum;
    wire [31:0] half_sum;   // A[i] XOR B_eff[i]

    assign carry[0] = do_sub;

    generate
        for (i = 0; i < 32; i = i + 1) begin : adder_chain
            // Half-sum via LUT2 XOR
            LUT2 #(.INIT(4'h6)) hs (
                .I0 (A[i]),
                .I1 (B_eff[i]),
                .O  (half_sum[i])
            );
            // MUXCY: carry generate/propagate
            //   S=half_sum: if 1, propagate CI; if 0, generate from DI
            //   DI=A[i]: the generate bit (A AND B_eff implied by S=0)
            MUXCY mcy (
                .O  (carry[i+1]),
                .CI (carry[i]),
                .DI (A[i]),
                .S  (half_sum[i])
            );
            // XORCY: sum bit = half_sum XOR carry_in
            XORCY xcy (
                .O  (sum[i]),
                .CI (carry[i]),
                .LI (half_sum[i])
            );
        end
    endgenerate

    // ── Bitwise outputs via LUT2 ──────────────────────────────────
    wire [31:0] and_out, or_out, xor_out;
    generate
        for (i = 0; i < 32; i = i + 1) begin : bitwise
            LUT2 #(.INIT(4'h8)) and2 (.I0(A[i]), .I1(B[i]), .O(and_out[i]));
            LUT2 #(.INIT(4'hE)) or2  (.I0(A[i]), .I1(B[i]), .O(or_out[i]));
            LUT2 #(.INIT(4'h6)) xor2 (.I0(A[i]), .I1(B[i]), .O(xor_out[i]));
        end
    endgenerate

    // ── MOV: direct wire ─────────────────────────────────────────
    wire [31:0] mov_out = B;

    // ── Result MUX: op select (behavioral — ~32 LUT4s, XST MUXF5) ─
    always @(*) begin
        case (op)
            3'b000: result = sum;      // ADD
            3'b001: result = sum;      // SUB
            3'b010: result = and_out;  // AND
            3'b011: result = or_out;   // OR
            3'b100: result = xor_out;  // XOR
            3'b101: result = mov_out;  // MOV
            default: result = sum;
        endcase
    end

    // ── Flags ─────────────────────────────────────────────────────
    assign N = result[31];
    assign Z = (result == 32'b0);

    // V flag: carry[32] XOR carry[31] — standard Xilinx overflow detect
    // (overflow occurs when carry into sign bit != carry out of sign bit)
    wire v_raw;
    LUT2 #(.INIT(4'h6)) v_lut (
        .I0 (carry[32]),
        .I1 (carry[31]),
        .O  (v_raw)
    );

    always @(*) begin
        case (op)
            3'b000: begin C = carry[32]; V = v_raw; end  // ADD
            3'b001: begin C = carry[32]; V = v_raw; end  // SUB
            default: begin C = 1'b0; V = 1'b0; end
        endcase
    end

endmodule
