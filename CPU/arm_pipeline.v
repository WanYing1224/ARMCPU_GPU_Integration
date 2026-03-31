// ============================================================
// arm_pipeline.v - Lab 6 4-Thread ARM Pipeline (modified for Lab 8)
// EE 533 Lab 6 -> Lab 8  Team 10
//
// Changes from Lab 6:
//   - Removed initial block (use rst instead, synthesizable)
//   - Exposed dmem Port B signals for external BRAM bridge
//   - All other logic UNCHANGED from Lab 6
// ============================================================
`timescale 1ns / 1ps

module arm_pipeline (
    input wire clk,
    input wire rst,

    // --- dmem Port B (new for Lab 8: wrapper read/write access) ---
    input  wire        ext_dmem_web,
    input  wire [7:0]  ext_dmem_addrb,
    input  wire [31:0] ext_dmem_dinb,
    output wire [31:0] ext_dmem_doutb
);

    // =========================================================
    // Rotated immediate decode
    // =========================================================
    function [31:0] rot_imm;
        input [7:0]  imm8;
        input [3:0]  rot;
        reg   [31:0] tmp;
        integer      amt;
        begin
            tmp = {24'b0, imm8};
            amt = rot * 2;
            if (amt == 0)
                rot_imm = tmp;
            else
                rot_imm = (tmp >> amt) | (tmp << (32 - amt));
        end
    endfunction

    // =========================================================
    // CPSR flags (per-thread N/Z/C/V)
    // =========================================================
    reg cpsr_N [0:3];
    reg cpsr_Z [0:3];
    reg cpsr_C [0:3];
    reg cpsr_V [0:3];

    // =========================================================
    // Thread scheduler + 4 PCs
    // =========================================================
    reg [1:0] if_thread;
    reg [8:0] pc [0:3];

    wire        branch_taken;
    wire [8:0]  branch_target_pc;

    reg  [1:0]  idex_thread;
    reg [31:0]  exmem_alu_result;
    reg  [1:0]  memwb_thread;

    localparam [8:0] PC_INIT_T0 = 9'd0;
    localparam [8:0] PC_INIT_T1 = 9'd48;
    localparam [8:0] PC_INIT_T2 = 9'd96;
    localparam [8:0] PC_INIT_T3 = 9'd144;

    // NOTE: initial block REMOVED for synthesis (Lab 8 change)
    // rst block below handles initialization

    always @(posedge clk) begin
        if (rst) begin
            if_thread <= 2'b0;
            pc[0] <= PC_INIT_T0;
            pc[1] <= PC_INIT_T1;
            pc[2] <= PC_INIT_T2;
            pc[3] <= PC_INIT_T3;
        end else begin
            if_thread <= if_thread + 1;
            pc[if_thread] <= pc[if_thread] + 1;
            if (branch_taken)
                pc[idex_thread] <= branch_target_pc;
        end
    end

    // =========================================================
    // Stage 1: IF - Instruction Fetch
    // =========================================================
    wire [31:0] if_instr;
    wire [31:0] imem_data_dout;

    imem imem_inst (
        .addr      (pc[if_thread]),
        .dout      (if_instr),
        .data_addr (exmem_alu_result[10:2]),
        .data_dout (imem_data_dout),
        .clk       (clk),
        .we        (1'b0),
        .din       (32'b0)
    );

    // =========================================================
    // Pipeline Register: IF/ID
    // =========================================================
    reg [31:0] ifid_instr;
    reg [8:0]  ifid_pc;
    reg [1:0]  ifid_thread;

    always @(posedge clk) begin
        if (rst) begin
            ifid_instr  <= 32'hE1A0B00B; // NOP
            ifid_pc     <= 9'b0;
            ifid_thread <= 2'b0;
        end else begin
            ifid_instr  <= if_instr;
            ifid_pc     <= pc[if_thread];
            ifid_thread <= if_thread;
        end
    end

    // =========================================================
    // Stage 2: ID - Decode + Register Read
    // (UNCHANGED from Lab 6)
    // =========================================================
    wire id_is_dp  = (ifid_instr[27:26] == 2'b00);
    wire id_is_ls  = (ifid_instr[27:26] == 2'b01);
    wire id_is_br  = (ifid_instr[27:25] == 3'b101);
    wire id_is_bl  = id_is_br && ifid_instr[24];
    wire id_is_bx  = (ifid_instr[27:4] == 24'h12FFF1);

    wire        id_I     = ifid_instr[25];
    wire [3:0]  id_opraw = ifid_instr[24:21];
    wire        id_S     = ifid_instr[20];
    wire [3:0]  id_Rn_dp = ifid_instr[19:16];
    wire [3:0]  id_Rd_dp = ifid_instr[15:12];
    wire [3:0]  id_rot   = ifid_instr[11:8];
    wire [7:0]  id_imm8  = ifid_instr[7:0];
    wire [3:0]  id_Rm    = ifid_instr[3:0];
    wire [31:0] id_dp_imm = rot_imm(id_imm8, id_rot);
    wire id_is_cmp = id_is_dp && (id_opraw == 4'hA);

    wire        id_ls_L   = ifid_instr[20];
    wire        id_ls_U   = ifid_instr[23];
    wire        id_ls_P   = ifid_instr[24];
    wire        id_ls_W   = ifid_instr[21];
    wire [3:0]  id_ls_Rn  = ifid_instr[19:16];
    wire [3:0]  id_ls_Rd  = ifid_instr[15:12];
    wire [11:0] id_ls_off = ifid_instr[11:0];

    wire        id_ls_reg = id_is_ls && ifid_instr[25];

    wire [31:0] id_ls_off32 = id_ls_U ? {20'b0, id_ls_off}
                                       : (~{20'b0, id_ls_off} + 1);
    wire [31:0] id_ls_imm = id_ls_P ? id_ls_off32 : 32'b0;
    wire id_ls_wb = id_is_ls && ((id_ls_P && id_ls_W) || !id_ls_P);

    wire id_ls_pc_rel = id_is_ls && !id_ls_reg && (id_ls_Rn == 4'hF);
    wire [31:0] id_pc_val = {ifid_pc + 9'd2, 2'b00};

    wire [3:0]  id_cond     = ifid_instr[31:28];
    wire [23:0] id_br_off24 = ifid_instr[23:0];
    wire [31:0] id_br_offset = {{6{id_br_off24[23]}}, id_br_off24, 2'b00};

    wire [1:0]  id_shift_type = ifid_instr[6:5];
    wire [4:0]  id_shift_amt  = ifid_instr[11:7];
    reg  [31:0] id_rm_shifted;
    always @(*) begin
        case (id_shift_type)
            2'b00: id_rm_shifted = id_r1_val << id_shift_amt;
            2'b01: id_rm_shifted = id_r1_val >> id_shift_amt;
            2'b10: id_rm_shifted = $signed(id_r1_val) >>> id_shift_amt;
            2'b11: id_rm_shifted = (id_r1_val >> id_shift_amt) |
                                   (id_r1_val << (32 - id_shift_amt));
        endcase
    end

    wire [31:0] id_ls_reg_off32 = id_ls_U ? id_rm_shifted : (~id_rm_shifted + 1);
    wire [31:0] id_ls_off_final = id_ls_reg ? (id_ls_P ? id_ls_reg_off32 : 32'b0)
                                            : id_ls_imm;

    wire [31:0] id_op2_reg   = (id_is_dp && !id_I) ? id_rm_shifted : id_r1_val;

    wire [3:0]  id_rn_addr   = id_is_bx ? id_Rm :
                               id_is_ls  ? id_ls_Rn : id_Rn_dp;
    wire [3:0]  id_r1_addr   = id_ls_reg ? id_Rm :
                               id_is_ls  ? id_ls_Rd : id_Rm;
    wire [31:0] id_imm_val   = id_is_ls ? id_ls_off_final : id_dp_imm;
    wire        id_alu_src   = id_is_ls | id_I;
    wire [31:0] id_op2_final = id_alu_src ? id_imm_val : id_op2_reg;

    reg [2:0] id_alu_op;
    always @(*) begin
        if (id_is_ls || id_is_br)
            id_alu_op = 3'b000;
        else case (id_opraw)
            4'b0100: id_alu_op = 3'b000; // ADD
            4'b0010: id_alu_op = 3'b001; // SUB
            4'b1010: id_alu_op = 3'b001; // CMP
            4'b1101: id_alu_op = 3'b101; // MOV
            4'b0000: id_alu_op = 3'b010; // AND
            4'b1100: id_alu_op = 3'b011; // ORR
            4'b0001: id_alu_op = 3'b100; // XOR
            default: id_alu_op = 3'b101;
        endcase
    end

    wire id_mem_read  = id_is_ls &&  id_ls_L;
    wire id_mem_write = id_is_ls && !id_ls_L;
    wire id_reg_write = (id_is_dp && !id_is_cmp) || id_mem_read;
    wire id_set_flags = id_is_cmp || (id_is_dp && id_S);
    wire [3:0] id_rd_addr = id_is_ls ? id_ls_Rd : id_Rd_dp;

    wire        wb_we, wb2_we;
    wire [3:0]  wb_waddr, wb2_waddr;
    wire [31:0] wb_wdata, wb2_wdata;
    wire [31:0] id_rn_val, id_r1_val;

    arm_regfile regfile_inst (
        .clk       (clk),
        .rst       (rst),
        .thread_id (ifid_thread),
        .wb_thread (memwb_thread),
        .r0addr    (id_rn_addr), .r0data (id_rn_val),
        .r1addr    (id_r1_addr), .r1data (id_r1_val),
        .we        (wb_we),   .waddr  (wb_waddr),  .wdata  (wb_wdata),
        .we2       (wb2_we),  .waddr2 (wb2_waddr), .wdata2 (wb2_wdata)
    );

    // =========================================================
    // Pipeline Register: ID/EX
    // =========================================================
    reg [31:0] idex_rn;
    reg [31:0] idex_op2;
    reg [31:0] idex_store_data;
    reg [3:0]  idex_rd_addr;
    reg [3:0]  idex_rn_addr;
    reg        idex_alu_src;
    reg [2:0]  idex_alu_op;
    reg        idex_mem_read;
    reg        idex_mem_write;
    reg        idex_reg_write;
    reg        idex_set_flags;
    reg        idex_ls_wb;
    reg [31:0] idex_ls_off32;
    reg        idex_pc_rel;
    reg        idex_is_branch;
    reg        idex_is_bx;
    reg        idex_is_bl;
    reg [31:0] idex_bl_ret;
    reg [31:0] idex_bx_target;
    reg [3:0]  idex_cond;
    reg [8:0]  idex_branch_target;
    reg [8:0]  idex_pc;

    always @(posedge clk) begin
        if (rst) begin
            idex_rn            <= 32'b0;
            idex_op2           <= 32'b0;
            idex_store_data    <= 32'b0;
            idex_rd_addr       <= 4'b0;
            idex_rn_addr       <= 4'b0;
            idex_alu_src       <= 1'b1;
            idex_alu_op        <= 3'b101;
            idex_mem_read      <= 1'b0;
            idex_mem_write     <= 1'b0;
            idex_reg_write     <= 1'b0;
            idex_set_flags     <= 1'b0;
            idex_ls_wb         <= 1'b0;
            idex_ls_off32      <= 32'b0;
            idex_pc_rel        <= 1'b0;
            idex_is_branch     <= 1'b0;
            idex_is_bx         <= 1'b0;
            idex_is_bl         <= 1'b0;
            idex_bl_ret        <= 32'b0;
            idex_bx_target     <= 32'b0;
            idex_cond          <= 4'hE;
            idex_branch_target <= 9'b0;
            idex_pc            <= 9'b0;
            idex_thread        <= 2'b0;
        end else begin
            idex_rn            <= id_ls_pc_rel ? id_pc_val : id_rn_val;
            idex_op2           <= id_op2_final;
            idex_store_data    <= id_r1_val;
            idex_rd_addr       <= id_rd_addr;
            idex_rn_addr       <= id_rn_addr;
            idex_alu_src       <= id_alu_src;
            idex_alu_op        <= id_alu_op;
            idex_mem_read      <= id_mem_read;
            idex_mem_write     <= id_mem_write;
            idex_reg_write     <= id_reg_write;
            idex_set_flags     <= id_set_flags;
            idex_ls_wb         <= id_ls_wb;
            idex_ls_off32      <= id_ls_off32;
            idex_pc_rel        <= id_ls_pc_rel;
            idex_is_branch     <= id_is_br || id_is_bx;
            idex_is_bx         <= id_is_bx;
            idex_is_bl         <= id_is_bl;
            idex_bl_ret        <= {ifid_pc + 9'd1, 2'b00};
            idex_bx_target     <= id_rn_val;
            idex_cond          <= id_cond;
            idex_branch_target <= ifid_pc + 9'd2 + id_br_off24[8:0];
            idex_pc            <= ifid_pc;
            idex_thread        <= ifid_thread;
        end
    end

    // =========================================================
    // Stage 3: EX - ALU + Condition + CPSR
    // =========================================================
    wire [31:0] ex_alu_A = idex_rn;
    wire [31:0] ex_alu_B = idex_op2;
    wire [31:0] ex_store_data = idex_store_data;

    wire [31:0] ex_result;
    wire        ex_N, ex_Z, ex_C, ex_V;

    arm_alu alu_inst (
        .A      (ex_alu_A),
        .B      (ex_alu_B),
        .op     (idex_alu_op),
        .result (ex_result),
        .N      (ex_N), .Z(ex_Z), .C(ex_C), .V(ex_V)
    );

    reg cond_met;
    always @(*) begin
        case (idex_cond)
            4'h0: cond_met = cpsr_Z[idex_thread];
            4'h1: cond_met = ~cpsr_Z[idex_thread];
            4'h2: cond_met = cpsr_C[idex_thread];
            4'h3: cond_met = ~cpsr_C[idex_thread];
            4'h4: cond_met = cpsr_N[idex_thread];
            4'h5: cond_met = ~cpsr_N[idex_thread];
            4'h6: cond_met = cpsr_V[idex_thread];
            4'h7: cond_met = ~cpsr_V[idex_thread];
            4'h8: cond_met = cpsr_C[idex_thread] && ~cpsr_Z[idex_thread];
            4'h9: cond_met = ~cpsr_C[idex_thread] || cpsr_Z[idex_thread];
            4'hA: cond_met = (cpsr_N[idex_thread] == cpsr_V[idex_thread]);
            4'hB: cond_met = (cpsr_N[idex_thread] != cpsr_V[idex_thread]);
            4'hC: cond_met = ~cpsr_Z[idex_thread] &&
                             (cpsr_N[idex_thread] == cpsr_V[idex_thread]);
            4'hD: cond_met = cpsr_Z[idex_thread] ||
                             (cpsr_N[idex_thread] != cpsr_V[idex_thread]);
            4'hE: cond_met = 1'b1;
            4'hF: cond_met = 1'b1;
            default: cond_met = 1'b1;
        endcase
    end

    assign branch_taken     = idex_is_branch && cond_met;
    assign branch_target_pc = idex_is_bx ? idex_bx_target[10:2]
                                         : idex_branch_target;

    integer t;
    always @(posedge clk) begin
        if (rst) begin
            for (t = 0; t < 4; t = t + 1) begin
                cpsr_N[t] <= 1'b0;
                cpsr_Z[t] <= 1'b0;
                cpsr_C[t] <= 1'b0;
                cpsr_V[t] <= 1'b0;
            end
        end else if (idex_set_flags) begin
            cpsr_N[idex_thread] <= ex_N;
            cpsr_Z[idex_thread] <= ex_Z;
            cpsr_C[idex_thread] <= ex_C;
            cpsr_V[idex_thread] <= ex_V;
        end
    end

    // =========================================================
    // Pipeline Register: EX/MEM
    // =========================================================
    reg [31:0] exmem_store_data;
    reg [3:0]  exmem_rd_addr;
    reg [3:0]  exmem_rn_addr;
    reg [31:0] exmem_wb_val;
    reg        exmem_mem_read;
    reg        exmem_mem_write;
    reg        exmem_reg_write;
    reg        exmem_cond_met;
    reg        exmem_ls_wb;
    reg        exmem_is_bl;
    reg [31:0] exmem_bl_ret;
    reg        exmem_pc_rel;
    reg [1:0]  exmem_thread;

    always @(posedge clk) begin
        if (rst) begin
            exmem_alu_result <= 32'b0;
            exmem_store_data <= 32'b0;
            exmem_rd_addr    <= 4'b0;
            exmem_rn_addr    <= 4'b0;
            exmem_wb_val     <= 32'b0;
            exmem_mem_read   <= 1'b0;
            exmem_mem_write  <= 1'b0;
            exmem_reg_write  <= 1'b0;
            exmem_cond_met   <= 1'b0;
            exmem_ls_wb      <= 1'b0;
            exmem_is_bl      <= 1'b0;
            exmem_bl_ret     <= 32'b0;
            exmem_pc_rel     <= 1'b0;
            exmem_thread     <= 2'b0;
        end else begin
            exmem_alu_result <= ex_result;
            exmem_store_data <= ex_store_data;
            exmem_rd_addr    <= idex_rd_addr;
            exmem_rn_addr    <= idex_rn_addr;
            exmem_wb_val     <= ex_alu_A + idex_ls_off32;
            exmem_mem_read   <= idex_mem_read;
            exmem_mem_write  <= idex_mem_write;
            exmem_reg_write  <= idex_reg_write;
            exmem_cond_met   <= cond_met;
            exmem_ls_wb      <= idex_ls_wb;
            exmem_is_bl      <= idex_is_bl;
            exmem_bl_ret     <= idex_bl_ret;
            exmem_pc_rel     <= idex_pc_rel;
            exmem_thread     <= idex_thread;
        end
    end

    // =========================================================
    // Stage 4: MEM - Memory Access
    // =========================================================
    wire [7:0]  dmem_addr = ex_result[9:2];
    wire [31:0] dmem_read_data;

    dmem_32 dmem_inst (
        .clka  (clk),
        .wea   (idex_mem_write && cond_met),
        .addra (dmem_addr),
        .dina  (ex_store_data),
        .douta (dmem_read_data),
        // Port B: exposed for Lab 8 wrapper
        .clkb  (clk),
        .web   (ext_dmem_web),
        .addrb (ext_dmem_addrb),
        .dinb  (ext_dmem_dinb),
        .doutb (ext_dmem_doutb)
    );

    wire [31:0] mem_read_data = exmem_pc_rel ? imem_data_dout : dmem_read_data;

    // =========================================================
    // Pipeline Register: MEM/WB
    // =========================================================
    reg [31:0] memwb_alu_result;
    reg [31:0] memwb_mem_data;
    reg [3:0]  memwb_rd_addr;
    reg        memwb_mem_to_reg;
    reg        memwb_reg_write;
    reg        memwb_cond_met;
    reg        memwb_is_bl;
    reg [31:0] memwb_bl_ret;
    reg        memwb_ls_wb;
    reg [3:0]  memwb_rn_addr;
    reg [31:0] memwb_wb_val;

    always @(posedge clk) begin
        if (rst) begin
            memwb_alu_result <= 32'b0;
            memwb_mem_data   <= 32'b0;
            memwb_rd_addr    <= 4'b0;
            memwb_mem_to_reg <= 1'b0;
            memwb_reg_write  <= 1'b0;
            memwb_cond_met   <= 1'b0;
            memwb_is_bl      <= 1'b0;
            memwb_bl_ret     <= 32'b0;
            memwb_ls_wb      <= 1'b0;
            memwb_rn_addr    <= 4'b0;
            memwb_wb_val     <= 32'b0;
            memwb_thread     <= 2'b0;
        end else begin
            memwb_alu_result <= exmem_alu_result;
            memwb_mem_data   <= mem_read_data;
            memwb_rd_addr    <= exmem_rd_addr;
            memwb_mem_to_reg <= exmem_mem_read;
            memwb_reg_write  <= exmem_reg_write;
            memwb_cond_met   <= exmem_cond_met;
            memwb_is_bl      <= exmem_is_bl;
            memwb_bl_ret     <= exmem_bl_ret;
            memwb_ls_wb      <= exmem_ls_wb;
            memwb_rn_addr    <= exmem_rn_addr;
            memwb_wb_val     <= exmem_wb_val;
            memwb_thread     <= exmem_thread;
        end
    end

    // =========================================================
    // Stage 5: WB - Write Back (UNCHANGED from Lab 6)
    // =========================================================
    assign wb_we    = (memwb_reg_write || memwb_is_bl) && memwb_cond_met;
    assign wb_waddr = memwb_is_bl ? 4'd14 : memwb_rd_addr;
    assign wb_wdata = memwb_is_bl      ? memwb_bl_ret  :
                      memwb_mem_to_reg ? memwb_mem_data : memwb_alu_result;

    assign wb2_we    = memwb_ls_wb && memwb_cond_met;
    assign wb2_waddr = memwb_rn_addr;
    assign wb2_wdata = memwb_wb_val;

endmodule
