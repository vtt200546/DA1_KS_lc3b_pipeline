//////////////////////////////////////////////////////////////////////////////////
//
// Filename: fetch_stage_fixed.v
// Module:   FetchStage
//
// Fixes vs original:
//  - Correct 2-bit MEM.PCMUX decode and add default (avoid latch / illegal value 4)
//  - Prevent PC update when MEM.STALL asserted (robust stall behavior)
//  - Refactor stall conditions for clarity
//
//////////////////////////////////////////////////////////////////////////////////

`ifndef FETCH_STAGE_V
`define FETCH_STAGE_V

module FetchStage(
    input  [15:0] pc,
    input         dep_stall,
    input         mem_stall,
    input         v_de_br_stall,
    input         v_agex_br_stall,
    input         v_mem_br_stall,
    input         imem_r,
    input  [1:0]  mem_pcmux,
    input  [15:0] target_pc,
    input  [15:0] trap_pc,
    input  [15:0] instr,
    output        ld_pc,
    output [15:0] de_npc,
    output [15:0] de_ir,
    output        de_v,
    output        ld_de,
    output reg [15:0] new_pc
);

    wire [15:0] pc_plus_two   = pc + 16'd2;
    wire        any_br_stall  = v_de_br_stall | v_agex_br_stall | v_mem_br_stall;

    // Inputs to DE latches
    assign de_npc = pc_plus_two;
    assign de_ir  = (imem_r) ? instr : 16'h0000;

    // Bubble conditions (DE.V=0): I-cache not ready OR control-instruction stall
    assign de_v   = imem_r & ~any_br_stall;

    // Hold DE latches only for DEP.STALL or MEM.STALL.
    // Otherwise, always load (either a real instruction when de_v=1, or a bubble when de_v=0).
    assign ld_de  = ~(dep_stall | mem_stall);

    // PC update logic:
    // - sequential: PC <- PC+2 only when I-cache ready, no dep/mem stall, and no control-stall.
    // - redirect:   PC <- TARGET.PC or TRAP.PC when MEM.PCMUX != 0 (generated in MEM stage).
    // - never update PC while MEM.STALL asserted.
    wire seq_pc_en = imem_r & ~(dep_stall | mem_stall | any_br_stall);
    wire redirect  = (mem_pcmux != 2'b00);

    assign ld_pc   = (~mem_stall) & (redirect | seq_pc_en);

    // Next-PC mux (MEM.PCMUX)
    always @(*) begin
        case (mem_pcmux)
            2'b00: new_pc = pc_plus_two;
            2'b01: new_pc = target_pc;
            2'b10: new_pc = trap_pc;
            default: new_pc = pc_plus_two; // 2'b11: treat as sequential (safe default)
        endcase
    end

endmodule

`endif // FETCH_STAGE_V
