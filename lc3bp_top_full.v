
`timescale 1ns/1ps
`default_nettype none

//------------------------------------------------------------------------------
// lc3bp_top_full.v
//------------------------------------------------------------------------------
// Full 5-stage-ish Lab6 pipeline integration:
//   FETCH -> DE -> AGEX -> MEM -> SR
//
// Notes:
// - ControlStore is addressed by {IR[15:11], IR[5]} in DecodeStage.
// - This top connects MEM stage's D-cache interface out to the environment.
// - SR stage writeback feeds DecodeStage (RegFile + CC) on the next cycle.
//
// Files required (same directory or include path):
//   fetch_stage_fixed.v
//   decode_stage_fixed2.v
//   agex_stage_fixed_patched.v   (ALUK=11 is PASSA + store-data forward)
//   mem_stage_fixed.v
//   sr_stage_fixed.v
//   control_store.v (copy of control_store_fixed.v)
//   reg_file.v
//   alu.v
//   shifter.v
//------------------------------------------------------------------------------

`include "fetch_stage_fixed.v"
`include "decode_stage_fixed2.v"
`include "agex_stage_fixed.v"
//`include "mem_stage_fixed.v"
//`include "sr_stage_fixed.v"

module LC3BP_TOP_FULL
(
`ifdef TESTING
    output wire [127:0] reg_contents,

    // Optional: expose some pipeline latches for debug
    output wire [15:0] PC_dbg,
    output wire [15:0] DE_IR_dbg,
    output wire [15:0] AGEX_IR_dbg,
    output wire [15:0] MEM_IR_dbg,
    output wire [15:0] SR_IR_dbg,
`endif

    input  wire        clk,
    input  wire        mem_clk,

    // I-cache interface (external IMEM model supplies instr + imem_r)
    input  wire        imem_r,
    input  wire [15:0] instr,
    output wire [15:0] PC_out,

    // D-cache interface (external DMEM model)
    input  wire        dcache_r,
    input  wire [15:0] dcache_dout,
    output wire        dcache_en,
    output wire [1:0]  dcache_we,
    output wire [15:0] dcache_addr,
    output wire [15:0] dcache_din
);

    //--------------------------------------------------------------------------
    // Pipeline latches / architectural state
    //--------------------------------------------------------------------------

    // FETCH stage latch (PC)
    reg [15:0] PC;

    // DE latches
    reg [15:0] DE_NPC;
    reg [15:0] DE_IR;
    reg        DE_V;

    // AGEX latches
    reg [15:0] AGEX_NPC;
    reg [15:0] AGEX_IR;
    reg [15:0] AGEX_SR1;
    reg [15:0] AGEX_SR2;
    reg [2:0]  AGEX_CC;
    reg [2:0]  AGEX_DRID;
    reg [19:0] AGEX_CS;
    reg        AGEX_V;

    // MEM latches
    reg        MEM_V;
    reg [15:0] MEM_NPC;
    reg [15:0] MEM_IR;
    reg [15:0] MEM_ADDRESS;
    reg [15:0] MEM_ALU_RESULT;
    reg [2:0]  MEM_CC;
    reg [2:0]  MEM_DRID;
    reg [10:0] MEM_CS;
    reg [15:0] MEM_STORE_DATA;

    // SR latches
    reg        SR_V;
    reg [15:0] SR_NPC;
    reg [15:0] SR_IR;
    reg [15:0] SR_ADDRESS;
    reg [15:0] SR_ALU_RESULT;
    reg [15:0] SR_DATA;
    reg [2:0]  SR_DRID;
    reg [3:0]  SR_CS;

    //--------------------------------------------------------------------------
    // Wires between stages
    //--------------------------------------------------------------------------

    // FETCH outputs
    wire        ld_pc;
    wire        ld_de;
    wire [15:0] new_pc;
    wire [15:0] de_npc_in;
    wire [15:0] de_ir_in;
    wire        de_v_in;

    // DE outputs
    wire        v_de_br_stall;
    wire        dep_stall;
    wire        ld_agex;

    wire [15:0] agex_sr1_in;
    wire [15:0] agex_sr2_in;
    wire [2:0]  agex_drid_new_in;
    wire [19:0] agex_cs_in;
    wire [2:0]  agex_cc_in;
    wire        agex_v_in;

    // AGEX outputs -> MEM latch inputs
    wire        ld_mem;
    wire        mem_v_in;
    wire [15:0] mem_npc_in;
    wire [15:0] mem_ir_in;
    wire [15:0] mem_alu_result_in;
    wire [15:0] mem_address_in;
    wire [15:0] mem_store_data_in;
    wire [2:0]  mem_cc_in;
    wire [2:0]  mem_drid_in;
    wire [10:0] mem_cs_in;

    // AGEX -> earlier stage valid-gated
    wire v_agex_ld_reg, v_agex_ld_cc, v_agex_br_stall;

    // MEM stage outputs
    wire        mem_stall;
    wire [1:0]  mem_pcmux;
    wire [15:0] target_pc;
    wire [15:0] trap_pc;

    wire        sr_v_in;
    wire [15:0] sr_npc_in;
    wire [15:0] sr_ir_in;
    wire [15:0] sr_address_in;
    wire [15:0] sr_alu_result_in;
    wire [15:0] sr_data_in;
    wire [2:0]  sr_drid_in;
    wire [3:0]  sr_cs_in;

    wire v_mem_ld_reg, v_mem_ld_cc, v_mem_br_stall;

    // SR stage outputs back to DE
    wire        v_sr_ld_reg;
    wire        v_sr_ld_cc;
    wire [2:0]  sr_drid_out;
    wire [15:0] sr_reg_data_out;
    wire [2:0]  sr_cc_data_out;

    //--------------------------------------------------------------------------
    // Stage instances
    //--------------------------------------------------------------------------

    FetchStage u_fetch(
        .pc            (PC),
        .dep_stall     (dep_stall),
        .mem_stall     (mem_stall),
        .v_de_br_stall (v_de_br_stall),
        .v_agex_br_stall(v_agex_br_stall),
        .v_mem_br_stall(v_mem_br_stall),
        .imem_r        (imem_r),
        .mem_pcmux     (mem_pcmux),
        .target_pc     (target_pc),
        .trap_pc       (trap_pc),
        .instr         (instr),
        .ld_pc         (ld_pc),
        .de_npc        (de_npc_in),
        .de_ir         (de_ir_in),
        .de_v          (de_v_in),
        .ld_de         (ld_de),
        .new_pc        (new_pc)
    );

    DecodeStage u_decode(
`ifdef TESTING
        .reg_contents  (reg_contents),
`endif
        .clk           (clk),
        .mem_clk       (mem_clk),
        .de_npc        (DE_NPC),
        .de_ir         (DE_IR),
        .de_v          (DE_V),

        .v_agex_ld_reg (v_agex_ld_reg),
        .v_mem_ld_reg  (v_mem_ld_reg),
        .v_sr_ld_reg   (v_sr_ld_reg),

        .agex_drid_old (AGEX_DRID),
        .mem_drid      (MEM_DRID),
        .sr_drid       (sr_drid_out),
        .sr_reg_data   (sr_reg_data_out),

        .v_agex_ld_cc  (v_agex_ld_cc),
        .v_mem_ld_cc   (v_mem_ld_cc),
        .v_sr_ld_cc    (v_sr_ld_cc),
        .sr_cc_data    (sr_cc_data_out),

        .mem_stall     (mem_stall),

        .v_de_br_stall (v_de_br_stall),
        .dep_stall     (dep_stall),
        .ld_agex       (ld_agex),

        .agex_sr1      (agex_sr1_in),
        .agex_sr2      (agex_sr2_in),
        .agex_drid_new (agex_drid_new_in),
        .agex_cs       (agex_cs_in),
        .agex_cc       (agex_cc_in),
        .agex_v        (agex_v_in)
    );

    AgexStage u_agex(
        .agex_v          (AGEX_V),
        .agex_npc        (AGEX_NPC),
        .agex_ir         (AGEX_IR),
        .agex_sr1        (AGEX_SR1),
        .agex_sr2        (AGEX_SR2),
        .agex_cc         (AGEX_CC),
        .agex_drid       (AGEX_DRID),
        .agex_cs         (AGEX_CS),

        .mem_stall       (mem_stall),

        .ld_mem          (ld_mem),
        .mem_v_in        (mem_v_in),
        .mem_npc_in      (mem_npc_in),
        .mem_ir_in       (mem_ir_in),
        .mem_alu_result_in(mem_alu_result_in),
        .mem_address_in  (mem_address_in),
        .mem_store_data_in(mem_store_data_in),
        .mem_cc_in       (mem_cc_in),
        .mem_drid_in     (mem_drid_in),
        .mem_cs_in       (mem_cs_in),

        .v_agex_ld_reg   (v_agex_ld_reg),
        .v_agex_ld_cc    (v_agex_ld_cc),
        .v_agex_br_stall (v_agex_br_stall)
    );

    mem_stage_fixed u_mem(
        .mem_v          (MEM_V),
        .mem_ir         (MEM_IR),
        .mem_npc        (MEM_NPC),
        .mem_address    (MEM_ADDRESS),
        .mem_alu_result (MEM_ALU_RESULT),
        .mem_cc         (MEM_CC),
        .mem_drid       (MEM_DRID),
        .mem_cs         (MEM_CS),
        .mem_store_data (MEM_STORE_DATA),

        .dcache_r       (dcache_r),
        .dcache_dout    (dcache_dout),
        .dcache_en      (dcache_en),
        .dcache_we      (dcache_we),
        .dcache_addr    (dcache_addr),
        .dcache_din     (dcache_din),

        .mem_stall      (mem_stall),
        .mem_pcmux      (mem_pcmux),
        .target_pc      (target_pc),
        .trap_pc        (trap_pc),

        .sr_v_in        (sr_v_in),
        .sr_npc_in      (sr_npc_in),
        .sr_ir_in       (sr_ir_in),
        .sr_address_in  (sr_address_in),
        .sr_alu_result_in(sr_alu_result_in),
        .sr_data_in     (sr_data_in),
        .sr_drid_in     (sr_drid_in),
        .sr_cs_in       (sr_cs_in),

        .v_mem_ld_reg   (v_mem_ld_reg),
        .v_mem_ld_cc    (v_mem_ld_cc),
        .v_mem_br_stall (v_mem_br_stall)
    );

    sr_stage_fixed u_sr(
        .sr_v           (SR_V),
        .sr_ir          (SR_IR),
        .sr_npc         (SR_NPC),
        .sr_address     (SR_ADDRESS),
        .sr_alu_result  (SR_ALU_RESULT),
        .sr_data        (SR_DATA),
        .sr_drid        (SR_DRID),
        .sr_cs          (SR_CS),

        .v_sr_ld_reg    (v_sr_ld_reg),
        .v_sr_ld_cc     (v_sr_ld_cc),
        .sr_drid_out    (sr_drid_out),
        .sr_reg_data_out(sr_reg_data_out),
        .sr_cc_data_out (sr_cc_data_out)
    );

    //--------------------------------------------------------------------------
    // Sequential logic: latch updates
    //--------------------------------------------------------------------------

    // PC update
    always @(posedge clk) begin
        if (ld_pc) begin
            PC <= new_pc;
        end
    end

    // DE latch update
    always @(posedge clk) begin
        if (ld_de) begin
            DE_NPC <= de_npc_in;
            DE_IR  <= de_ir_in;
            DE_V   <= de_v_in;
        end
    end

    // AGEX latch update
    always @(posedge clk) begin
        if (ld_agex) begin
            AGEX_NPC  <= DE_NPC;
            AGEX_IR   <= DE_IR;
            AGEX_SR1  <= agex_sr1_in;
            AGEX_SR2  <= agex_sr2_in;
            AGEX_CC   <= agex_cc_in;
            AGEX_DRID <= agex_drid_new_in;
            AGEX_CS   <= agex_cs_in;
            AGEX_V    <= agex_v_in;
        end
    end

    // MEM latch update
    always @(posedge clk) begin
        if (ld_mem) begin
            MEM_V         <= mem_v_in;
            MEM_NPC       <= mem_npc_in;
            MEM_IR        <= mem_ir_in;
            MEM_ADDRESS   <= mem_address_in;
            MEM_ALU_RESULT<= mem_alu_result_in;
            MEM_CC        <= mem_cc_in;
            MEM_DRID      <= mem_drid_in;
            MEM_CS        <= mem_cs_in;
            MEM_STORE_DATA<= mem_store_data_in;
        end
    end

    // SR latch update (stall-aware)
    always @(posedge clk) begin
        if (~mem_stall) begin
            SR_V          <= sr_v_in;
            SR_NPC        <= sr_npc_in;
            SR_IR         <= sr_ir_in;
            SR_ADDRESS    <= sr_address_in;
            SR_ALU_RESULT <= sr_alu_result_in;
            SR_DATA       <= sr_data_in;
            SR_DRID       <= sr_drid_in;
            SR_CS         <= sr_cs_in;
        end
    end

    //--------------------------------------------------------------------------
    // Reset/initial values (simulation convenience)
    //--------------------------------------------------------------------------

    initial begin
        PC          = 16'h3000;

        DE_NPC      = 16'h0000;
        DE_IR       = 16'h0000;
        DE_V        = 1'b0;

        AGEX_NPC    = 16'h0000;
        AGEX_IR     = 16'h0000;
        AGEX_SR1    = 16'h0000;
        AGEX_SR2    = 16'h0000;
        AGEX_CC     = 3'b010; // Z
        AGEX_DRID   = 3'd0;
        AGEX_CS     = 20'd0;
        AGEX_V      = 1'b0;

        MEM_V       = 1'b0;
        MEM_NPC     = 16'h0000;
        MEM_IR      = 16'h0000;
        MEM_ADDRESS = 16'h0000;
        MEM_ALU_RESULT = 16'h0000;
        MEM_CC      = 3'b010;
        MEM_DRID    = 3'd0;
        MEM_CS      = 11'd0;
        MEM_STORE_DATA = 16'h0000;

        SR_V        = 1'b0;
        SR_NPC      = 16'h0000;
        SR_IR       = 16'h0000;
        SR_ADDRESS  = 16'h0000;
        SR_ALU_RESULT = 16'h0000;
        SR_DATA     = 16'h0000;
        SR_DRID     = 3'd0;
        SR_CS       = 4'd0;
    end

    assign PC_out = PC;

`ifdef TESTING
    assign PC_dbg     = PC;
    assign DE_IR_dbg  = DE_IR;
    assign AGEX_IR_dbg= AGEX_IR;
    assign MEM_IR_dbg = MEM_IR;
    assign SR_IR_dbg  = SR_IR;
`endif

endmodule

`default_nettype wire
