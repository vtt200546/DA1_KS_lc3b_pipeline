//////////////////////////////////////////////////////////////////////////////////
//
// decode_stage_fixed2.v
//
// Fixes vs previous version:
//  - Control store bit indices now match Lab6 "Table 2" numbering.
//  - BR.STALL, DRMUX, SR1.NEEDED, SR2.NEEDED, BR.OP, and AGEX.CS slicing corrected.
//
// IMPORTANT: This module expects ControlStore to be addressed by {DE.IR[15:11], DE.IR[5]}
//            and to output cs_bits[22:0] where bit 0 is SR1.NEEDED ... bit 22 is LD.CC.
//////////////////////////////////////////////////////////////////////////////////

`ifndef DECODE_STAGE_V
`define DECODE_STAGE_V

//`include "control_store.v"
//`include "reg_file.v"

module DecodeStage(
    `ifdef TESTING
        output [127:0] reg_contents,
    `endif

    input clk,
    input mem_clk,
    input [15:0] de_npc,
    input [15:0] de_ir,
    input de_v,

    input v_agex_ld_reg,
    input v_mem_ld_reg,
    input v_sr_ld_reg,

    input [2:0] agex_drid_old,
    input [2:0] mem_drid,
    input [2:0] sr_drid,
    input [15:0] sr_reg_data,

    input v_agex_ld_cc,
    input v_mem_ld_cc,
    input v_sr_ld_cc,
    input [2:0] sr_cc_data,

    input mem_stall,

    output v_de_br_stall,
    output dep_stall,
    output ld_agex,

    output [15:0] agex_sr1,
    output [15:0] agex_sr2,
    output [2:0]  agex_drid_new,
    output [19:0] agex_cs,
    output [2:0]  agex_cc,
    output        agex_v
);

    // Control-store address per Lab6: {IR[15:11], IR[5]}
    wire [5:0] cs_addr = {de_ir[15:11], de_ir[5]};

    wire [22:0] de_cs;
    ControlStore CS(
        .addr   (cs_addr),
        .cs_bits(de_cs)
    );

    // Register file read addressing per Lab6: SR1 = IR[8:6], SR2 = IR[11:9] or IR[2:0] selected by IR[13]
    wire        sr2_id_mux = de_ir[13];
    wire [2:0]  sr1 = de_ir[8:6];
    wire [2:0]  sr2 = (sr2_id_mux == 1'b0) ? de_ir[2:0] : de_ir[11:9];
    wire [15:0] sr1_data, sr2_data;

    RegFile regFile(
        `ifdef TESTING
            .reg_contents(reg_contents),
        `endif

        .clk    (clk),
        .sr1    (sr1),
        .sr2    (sr2),
        .dr     (sr_drid),
        .data_in(sr_reg_data),
        .we     (v_sr_ld_reg),
        .sr1_out(sr1_data),
        .sr2_out(sr2_data)
    );

    // ---- Control signals (Table 2 bit numbering) ----
    //  0 SR1.NEEDED
    //  1 SR2.NEEDED
    //  2 DRMUX
    // 12 BR.OP
    // 15 BR.STALL
    wire sr1_needed = de_cs[0];
    wire sr2_needed = de_cs[1];
    wire dr_mux     = de_cs[2];
    wire br_op      = de_cs[12];
    wire de_br_stall_bit = de_cs[15];

    assign v_de_br_stall = de_v & de_br_stall_bit;

    // Destination register ID selection (DRMUX): IR[11:9] vs R7
    wire [2:0] dr = (dr_mux == 1'b0) ? de_ir[11:9] : 3'd7;

    // ---- Dependency check (no forwarding) ----
    // Reg dependencies
    wire sr1_dep_stall = sr1_needed & (
        (v_agex_ld_reg & (agex_drid_old == sr1)) |
        (v_mem_ld_reg  & (mem_drid      == sr1)) |
        (v_sr_ld_reg   & (sr_drid       == sr1))
    );

    wire sr2_dep_stall = sr2_needed & (
        (v_agex_ld_reg & (agex_drid_old == sr2)) |
        (v_mem_ld_reg  & (mem_drid      == sr2)) |
        (v_sr_ld_reg   & (sr_drid       == sr2))
    );

    // CC dependency (only for conditional BR)
    wire br_dep_stall = br_op & (v_agex_ld_cc | v_mem_ld_cc | v_sr_ld_cc);

    assign dep_stall = de_v & (sr1_dep_stall | sr2_dep_stall | br_dep_stall);

    // ---- CC register (architectural) ----
    reg [2:0] CC;
    initial CC = 3'b010; // Z=1

    always @(posedge clk) begin
        if (v_sr_ld_cc) begin
            CC <= sr_cc_data;
        end
    end

    // ---- Outputs to AGEX latches ----
    // Insert bubble into AGEX by driving AGEX.V=0 when dep_stall is asserted.
    // Note: ld_agex should still be asserted when dep_stall=1 so AGEX.V actually becomes 0.
    assign ld_agex        = ~mem_stall;
    assign agex_v         = de_v & ~dep_stall;

    assign agex_sr1       = sr1_data;
    assign agex_sr2       = sr2_data;
    assign agex_drid_new  = dr;

    // Lab6: 20 bits from control store are latched into AGEX.CS.
    // Those are the bits used in later stages, i.e., control store bits [22:3].
    assign agex_cs        = de_cs[22:3];

    assign agex_cc        = CC;

    // mem_clk currently unused (kept to match existing module interface)
    wire _unused_mem_clk = mem_clk;

endmodule

`endif // DECODE_STAGE_V
