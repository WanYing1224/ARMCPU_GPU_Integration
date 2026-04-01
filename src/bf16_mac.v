// ============================================================
// bf16_mac.v — BFloat16 Fused Multiply-Add (Structural, Lab 10)
//
// All three compute-heavy operations are now structural:
//
//   1. Mantissa multiply (8x8):
//      MULT18X18S primitive — uses DSP18 block, 0 LUTs.
//      Inputs zero-padded to 18 bits (unsigned).
//
//   2. Exponent subtraction (exp_diff):
//      MUXCY + XORCY carry chain (8 stages) for both
//      (mul_exp - exp_c) and (exp_c - mul_exp).
//      A subtractor is A + ~B + 1 — same carry-chain trick
//      as arm_alu.v. Uses 0 LUTs for carry propagation.
//
//   3. Mantissa alignment shift (>> exp_diff):
//      exp_diff is 3 bits (0-7 range on 8-bit data).
//      Implemented as 3 stages of MUXF5 per output bit:
//        Stage 0: shift by 0 or 1
//        Stage 1: shift by 0 or 2 (applied on top of stage 0)
//        Stage 2: shift by 0 or 4 (applied on top of stage 1)
//      This is the standard barrel shifter decomposition.
//      Each stage is a 2:1 mux — LUT2 + MUXF5 or just MUXF5.
//
//   Remaining logic (exponent compare, normalization, zero checks,
//   sign select, final output mux) is small ternary wire logic —
//   XST maps these to a handful of LUTs automatically.
//
// clk port added (required by MULT18X18S registered output).
// tensor_unit.v passes clk to each lane.
// ============================================================
`timescale 1ns/1ps
module bf16_mac (
    input  wire        clk,
    input  wire [15:0] a,
    input  wire [15:0] b,
    input  wire [15:0] c,
    output wire [15:0] z
);

    // ── Unpack BF16 ───────────────────────────────────────────────
    wire        sign_a = a[15];
    wire [7:0]  exp_a  = a[14:7];
    wire [6:0]  mant_a = a[6:0];
    wire        sign_b = b[15];
    wire [7:0]  exp_b  = b[14:7];
    wire [6:0]  mant_b = b[6:0];
    wire        sign_c = c[15];
    wire [7:0]  exp_c  = c[14:7];
    wire [6:0]  mant_c = c[6:0];

    // ── 1. Mantissa multiply via MULT18X18S ───────────────────────
    wire [7:0]  mant_a_full = {1'b1, mant_a};
    wire [7:0]  mant_b_full = {1'b1, mant_b};
    wire [35:0] mult_p;

    MULT18X18S mult_inst (
        .C  (clk),
        .CE (1'b1),
        .R  (1'b0),
        .A  ({10'b0, mant_a_full}),
        .B  ({10'b0, mant_b_full}),
        .P  (mult_p)
    );

    wire [15:0] mul_mant_temp = mult_p[15:0];
    wire        mul_norm      = mul_mant_temp[15];
    wire [7:0]  mul_exp       = exp_a + exp_b - 8'd127 + {7'b0, mul_norm};
    wire [6:0]  mul_mant      = mul_norm ? mul_mant_temp[14:8]
                                         : mul_mant_temp[13:7];
    wire        mul_sign      = sign_a ^ sign_b;
    wire [15:0] mul_res       = (a == 16'd0 || b == 16'd0) ? 16'd0
                              : {mul_sign, mul_exp, mul_mant};

    // ── Restore hidden bits ───────────────────────────────────────
    wire [7:0] m_mul = (mul_res == 16'd0) ? 8'd0 : {1'b1, mul_mant};
    wire [7:0] m_c   = (c == 16'd0)       ? 8'd0 : {1'b1, mant_c};

    // ── Exponent comparison ───────────────────────────────────────
    wire mul_is_bigger = (mul_exp > exp_c);
    wire [7:0] final_exp = mul_is_bigger ? mul_exp : exp_c;

    // ── 2. Exponent difference: structural 8-bit subtractor ───────
    // exp_diff = mul_is_bigger ? (mul_exp - exp_c) : (exp_c - mul_exp)
    // We build two subtractors and mux the result.
    // Subtractor A-B = A + ~B + 1 using MUXCY+XORCY chain.

    // Subtractor 1: mul_exp - exp_c
    wire [7:0]  sub1_b_inv;   // ~exp_c
    wire [8:0]  sub1_carry;
    wire [7:0]  sub1_half;
    wire [7:0]  sub1_out;

    // Subtractor 2: exp_c - mul_exp
    wire [7:0]  sub2_b_inv;   // ~mul_exp
    wire [8:0]  sub2_carry;
    wire [7:0]  sub2_half;
    wire [7:0]  sub2_out;

    assign sub1_carry[0] = 1'b1;   // +1 for two's complement
    assign sub2_carry[0] = 1'b1;

    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : exp_sub
            // ~exp_c and ~mul_exp via LUT2 NOT
            LUT2 #(.INIT(4'h5)) inv1 (.I0(exp_c[i]),   .I1(1'b0), .O(sub1_b_inv[i]));
            LUT2 #(.INIT(4'h5)) inv2 (.I0(mul_exp[i]), .I1(1'b0), .O(sub2_b_inv[i]));

            // Subtractor 1: mul_exp + ~exp_c + 1
            LUT2 #(.INIT(4'h6)) hs1 (.I0(mul_exp[i]), .I1(sub1_b_inv[i]), .O(sub1_half[i]));
            MUXCY mc1 (.O(sub1_carry[i+1]), .CI(sub1_carry[i]), .DI(mul_exp[i]), .S(sub1_half[i]));
            XORCY xc1 (.O(sub1_out[i]),     .CI(sub1_carry[i]), .LI(sub1_half[i]));

            // Subtractor 2: exp_c + ~mul_exp + 1
            LUT2 #(.INIT(4'h6)) hs2 (.I0(exp_c[i]),   .I1(sub2_b_inv[i]), .O(sub2_half[i]));
            MUXCY mc2 (.O(sub2_carry[i+1]), .CI(sub2_carry[i]), .DI(exp_c[i]),   .S(sub2_half[i]));
            XORCY xc2 (.O(sub2_out[i]),     .CI(sub2_carry[i]), .LI(sub2_half[i]));
        end
    endgenerate

    // Select correct difference
    wire [7:0] exp_diff = mul_is_bigger ? sub1_out : sub2_out;

    // ── 3. Mantissa alignment shifter (>> exp_diff) ───────────────
    // exp_diff[2:0] determines shift amount 0-7 on 8-bit data.
    // Implemented as 3 cascaded 2:1 mux stages (barrel shift):
    //   Stage 0 (shift bit 0): shift by 0 or 1
    //   Stage 1 (shift bit 1): shift by 0 or 2
    //   Stage 2 (shift bit 2): shift by 0 or 4
    //
    // For right-shift (>>): shifted bits fill with 0 from left.
    // We only need 8-bit output — upper input bits that shift out
    // are discarded.

    // Helper function macro style: barrel_shr(data, amt, width)
    // We implement both m_mul and m_c shifts.

    // --- Shift m_mul right by exp_diff (when !mul_is_bigger) ---
    wire [7:0] sh_mul_s0, sh_mul_s1, sh_mul_s2;

    generate
        for (i = 0; i < 8; i = i + 1) begin : shr_mul_s0
            // Stage 0: shift by exp_diff[0]
            // out[i] = exp_diff[0] ? (i>0 ? in[i-1] : 0) : in[i]
            wire in_hi = (i > 0) ? m_mul[i-1] : 1'b0;
            MUXF5 mux0 (
                .O  (sh_mul_s0[i]),
                .I0 (m_mul[i]),    // exp_diff[0]=0: no shift
                .I1 (in_hi),       // exp_diff[0]=1: shift right 1
                .S  (exp_diff[0])
            );
        end
        for (i = 0; i < 8; i = i + 1) begin : shr_mul_s1
            // Stage 1: shift by exp_diff[1] (shift by 2)
            wire in_hi2 = (i > 1) ? sh_mul_s0[i-2] : 1'b0;
            MUXF5 mux1 (
                .O  (sh_mul_s1[i]),
                .I0 (sh_mul_s0[i]),
                .I1 (in_hi2),
                .S  (exp_diff[1])
            );
        end
        for (i = 0; i < 8; i = i + 1) begin : shr_mul_s2
            // Stage 2: shift by exp_diff[2] (shift by 4)
            wire in_hi4 = (i > 3) ? sh_mul_s1[i-4] : 1'b0;
            MUXF5 mux2 (
                .O  (sh_mul_s2[i]),
                .I0 (sh_mul_s1[i]),
                .I1 (in_hi4),
                .S  (exp_diff[2])
            );
        end
    endgenerate

    // --- Shift m_c right by exp_diff (when mul_is_bigger) ---
    wire [7:0] sh_c_s0, sh_c_s1, sh_c_s2;

    generate
        for (i = 0; i < 8; i = i + 1) begin : shr_c_s0
            wire in_hi = (i > 0) ? m_c[i-1] : 1'b0;
            MUXF5 mux0 (
                .O  (sh_c_s0[i]),
                .I0 (m_c[i]),
                .I1 (in_hi),
                .S  (exp_diff[0])
            );
        end
        for (i = 0; i < 8; i = i + 1) begin : shr_c_s1
            wire in_hi2 = (i > 1) ? sh_c_s0[i-2] : 1'b0;
            MUXF5 mux1 (
                .O  (sh_c_s1[i]),
                .I0 (sh_c_s0[i]),
                .I1 (in_hi2),
                .S  (exp_diff[1])
            );
        end
        for (i = 0; i < 8; i = i + 1) begin : shr_c_s2
            wire in_hi4 = (i > 3) ? sh_c_s1[i-4] : 1'b0;
            MUXF5 mux2 (
                .O  (sh_c_s2[i]),
                .I0 (sh_c_s1[i]),
                .I1 (in_hi4),
                .S  (exp_diff[2])
            );
        end
    endgenerate

    // Select which operand gets shifted
    wire [7:0] aligned_mul_m = mul_is_bigger ? m_mul      : sh_mul_s2;
    wire [7:0] aligned_c_m   = mul_is_bigger ? sh_c_s2    : m_c;

    // ── 4. Mantissa add/subtract (behavioral — small, ~18 LUTs) ──
    wire signs_match = (mul_sign == sign_c);
    wire final_sign  = mul_is_bigger ? mul_sign : sign_c;

    wire [8:0] sum_mant = signs_match
        ? (aligned_mul_m + aligned_c_m)
        : (mul_is_bigger ? (aligned_mul_m - aligned_c_m)
                         : (aligned_c_m   - aligned_mul_m));

    // ── Normalize ────────────────────────────────────────────────
    wire        add_norm  = sum_mant[8];
    wire [7:0]  norm_exp  = final_exp + {7'b0, add_norm};
    wire [6:0]  norm_mant = add_norm ? sum_mant[7:1] : sum_mant[6:0];

    // ── Final output ─────────────────────────────────────────────
    assign z = (mul_res == 16'd0 && c == 16'd0) ? 16'd0
             : (mul_res == 16'd0)                ? c
             : (c == 16'd0)                      ? mul_res
             : {final_sign, norm_exp, norm_mant};

endmodule
