`timescale 1ns/1ps
`default_nettype none

// SR stage (Store Result / Writeback) - Lab 6 LC-3b pipeline
// 4-input mux selected by DR.VALUEMUX[1:0]:
//   00: SR.ADDRESS
//   01: SR.DATA
//   10: SR.NPC
//   11: SR.ALU.RESULT
// CC ordering: {N,Z,P} = CC[2:0]

module sr_stage_fixed (
    // From SR latches
    input  wire        sr_v,
    input  wire [15:0] sr_ir,
    input  wire [15:0] sr_npc,
    input  wire [15:0] sr_address,
    input  wire [15:0] sr_alu_result,
    input  wire [15:0] sr_data,
    input  wire [2:0]  sr_drid,
    input  wire [3:0]  sr_cs,        // {LD.CC, LD.REG, DR.VALUEMUX[1:0]}

    // To RegFile write port (in your Decode stage)
    output wire        v_sr_ld_reg,
    output wire        v_sr_ld_cc,
    output wire [2:0]  sr_drid_out,
    output wire [15:0] sr_reg_data,
    output wire [2:0]  sr_cc_data
);

    wire       ld_cc  = sr_cs[3];
    wire       ld_reg = sr_cs[2];
    wire [1:0] dr_valuemux = sr_cs[1:0];

    assign v_sr_ld_reg = sr_v & ld_reg;
    assign v_sr_ld_cc  = sr_v & ld_cc;
    assign sr_drid_out = sr_drid;

    reg [15:0] wb_value;
    always @(*) begin
        case (dr_valuemux)
            2'b00: wb_value = sr_address;
            2'b01: wb_value = sr_data;
            2'b10: wb_value = sr_npc;
            2'b11: wb_value = sr_alu_result;
            default: wb_value = 16'h0000;
        endcase
    end

    assign sr_reg_data = wb_value;

    // CC from wb_value
    wire n = wb_value[15];
    wire z = (wb_value == 16'h0000);
    wire p = (~n) & (~z);

    assign sr_cc_data = {n, z, p};

    // sr_ir kept for completeness/debug
    wire _unused = |sr_ir;

endmodule

`default_nettype wire
