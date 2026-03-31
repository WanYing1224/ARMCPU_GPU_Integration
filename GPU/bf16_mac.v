module bf16_mac (
    input  wire [15:0] a, // Multiplier
    input  wire [15:0] b, // Multiplicand
    input  wire [15:0] c, // Addend
    output wire [15:0] z  // Result
);

    // --------------------------------------------------------
    // STAGE 1: MULTIPLICATION (A * B)
    // --------------------------------------------------------
    wire sign_a = a[15]; wire [7:0] exp_a = a[14:7]; wire [6:0] mant_a = a[6:0];
    wire sign_b = b[15]; wire [7:0] exp_b = b[14:7]; wire [6:0] mant_b = b[6:0];

    // Multiply Mantissas (add hidden 1)
    wire [15:0] mul_mant_temp = {1'b1, mant_a} * {1'b1, mant_b};
    wire mul_norm = mul_mant_temp[15]; // Check if we need to normalize
    
    // Calculate new Exponent and Sign
    wire [7:0] mul_exp = exp_a + exp_b - 8'd127 + mul_norm;
    wire [6:0] mul_mant = mul_norm ? mul_mant_temp[14:8] : mul_mant_temp[13:7];
    wire mul_sign = sign_a ^ sign_b;

    // Intermediate Multiplication Result
    wire [15:0] mul_res = (a == 0 || b == 0) ? 16'd0 : {mul_sign, mul_exp, mul_mant};

    // --------------------------------------------------------
    // STAGE 2: ADDITION (mul_res + C)
    // --------------------------------------------------------
    wire sign_c = c[15]; wire [7:0] exp_c = c[14:7]; wire [6:0] mant_c = c[6:0];
    
    // Append hidden 1s
    wire [7:0] m_mul_full = (mul_res == 0) ? 8'd0 : {1'b1, mul_mant};
    wire [7:0] m_c_full   = (c == 0)       ? 8'd0 : {1'b1, mant_c};

    // Align Exponents
    wire mul_is_bigger = (mul_exp > exp_c);
    wire [7:0] exp_diff = mul_is_bigger ? (mul_exp - exp_c) : (exp_c - mul_exp);
    wire [7:0] final_exp = mul_is_bigger ? mul_exp : exp_c;

    // Shift the smaller mantissa to match the larger exponent
    wire [7:0] aligned_mul_m = mul_is_bigger ? m_mul_full : (m_mul_full >> exp_diff);
    wire [7:0] aligned_c_m   = mul_is_bigger ? (m_c_full >> exp_diff) : m_c_full;

    // Add or Subtract Mantissas based on signs
    wire signs_match = (mul_sign == sign_c);
    wire final_sign = mul_is_bigger ? mul_sign : sign_c;
    
    wire [8:0] sum_mant = signs_match ? (aligned_mul_m + aligned_c_m) : 
                          (mul_is_bigger ? (aligned_mul_m - aligned_c_m) : (aligned_c_m - aligned_mul_m));

    // Normalize Addition Result
    wire add_norm = sum_mant[8];
    wire [7:0] norm_exp = final_exp + add_norm;
    wire [6:0] norm_mant = add_norm ? sum_mant[7:1] : sum_mant[6:0];

    // Final Output (Handle zeros)
    assign z = (mul_res == 0 && c == 0) ? 16'd0 : 
               (mul_res == 0) ? c : 
               (c == 0) ? mul_res : 
               {final_sign, norm_exp, norm_mant};

endmodule
