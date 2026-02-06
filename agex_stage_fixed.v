
//////////////////////////////////////////////////////////////////////////////////
//
// Filename: agex_stage_fixed.v
// Module : AgexStage
// Description:
//   Address Generation / Execute stage (AGEX) for the pipelined LC-3b (Lab 6).
//
//   Implements the muxing + computation described in Lab6 documentation Table 1:
//     - Address adder (ADDR1MUX, ADDR2MUX, LSHF1)
//     - ADDRESSMUX (TRAP vector address vs. adder output)
//     - ALU (SR2MUX, ALUK)
//     - Shifter (IR[5:0])
//     - ALU.RESULTMUX (SHIFTER vs. ALU)
//
//   Also generates the "to-previous-stages" valid-gated signals:
//     V.AGEX.LD.CC, V.AGEX.LD.REG, V.AGEX.BR.STALL
//
//   And generates LD.MEM (load-enable for MEM latches) from MEM.STALL.
//
//////////////////////////////////////////////////////////////////////////////////

`ifndef AGEX_STAGE_V
`define AGEX_STAGE_V

`include "alu.v"
`include "shifter.v"


module AgexStage(
    // -------- Inputs from AGEX pipeline latches --------
    input  wire        agex_v,
    input  wire [15:0] agex_npc,
    input  wire [15:0] agex_ir,
    input  wire [15:0] agex_sr1,
    input  wire [15:0] agex_sr2,
    input  wire [2:0]  agex_cc,
    input  wire [2:0]  agex_drid,
    input  wire [19:0] agex_cs,   // AGEX.CS = ControlStore[22:3] (20 bits)

    // -------- From MEM stage (back-pressure) --------
    input  wire        mem_stall,

    // -------- Outputs to MEM pipeline latches --------
    output wire        ld_mem,        // Generated in AGEX logic (Table 1)
    output wire        mem_v_in,       // Input to MEM.V latch
    output wire [15:0] mem_npc_in,
    output wire [15:0] mem_ir_in,
    output wire [15:0] mem_alu_result_in,
    output wire [15:0] mem_address_in,
    output wire [2:0]  mem_cc_in,
    output wire [2:0]  mem_drid_in,
    output wire [10:0] mem_cs_in,
    output wire [15:0] mem_store_data_in,     // Propagated subset for MEM/SR (bits 12..22)

    // -------- Outputs to earlier stages (valid-gated) --------
    output wire        v_agex_ld_reg,
    output wire        v_agex_ld_cc,
    output wire        v_agex_br_stall
);

    // ------------------------------------------------------------
    // Index map inside AGEX.CS (ControlStore bits [22:3] latched)
    // AGEX_CS[0]  <-> ControlStore bit 3  (ADDR1MUX)
    // AGEX_CS[19] <-> ControlStore bit 22 (LD.CC)
    // ------------------------------------------------------------
    localparam integer IDX_ADDR1MUX      = 0;   // bit 3
    localparam integer IDX_ADDR2MUX1     = 1;   // bit 4
    localparam integer IDX_ADDR2MUX0     = 2;   // bit 5
    localparam integer IDX_LSHF1         = 3;   // bit 6
    localparam integer IDX_ADDRESSMUX    = 4;   // bit 7
    localparam integer IDX_SR2MUX        = 5;   // bit 8
    localparam integer IDX_ALUK1         = 6;   // bit 9
    localparam integer IDX_ALUK0         = 7;   // bit 10
    localparam integer IDX_ALU_RESULTMUX = 8;   // bit 11

    localparam integer IDX_BR_OP         = 9;   // bit 12
    localparam integer IDX_UNCON_OP      = 10;  // bit 13
    localparam integer IDX_TRAP_OP       = 11;  // bit 14
    localparam integer IDX_BR_STALL      = 12;  // bit 15
    localparam integer IDX_DCACHE_EN     = 13;  // bit 16
    localparam integer IDX_DCACHE_RW     = 14;  // bit 17
    localparam integer IDX_DATA_SIZE     = 15;  // bit 18
    localparam integer IDX_DR_VALUEMUX1  = 16;  // bit 19
    localparam integer IDX_DR_VALUEMUX0  = 17;  // bit 20
    localparam integer IDX_LD_REG        = 18;  // bit 21
    localparam integer IDX_LD_CC         = 19;  // bit 22

    // Decode control signals (Table 1 mapping: first listed value => 0)
    wire        addr1mux      = agex_cs[IDX_ADDR1MUX];      // 0:NPC, 1:BaseR(SR1)
    wire [1:0]  addr2mux      = {agex_cs[IDX_ADDR2MUX1], agex_cs[IDX_ADDR2MUX0]}; // 00:0, 01:off6, 10:pc9, 11:pc11
    wire        lshf1         = agex_cs[IDX_LSHF1];
    wire        addressmux    = agex_cs[IDX_ADDRESSMUX];    // 0:trapvec8<<1, 1:adder
    wire        sr2mux        = agex_cs[IDX_SR2MUX];        // 0:SR2, 1:SEXT(imm5)
    wire [1:0]  aluk          = {agex_cs[IDX_ALUK1], agex_cs[IDX_ALUK0]}; // 00 add, 01 and, 10 xor, 11 passb
    wire        alu_resultmux = agex_cs[IDX_ALU_RESULTMUX]; // 0:shifter, 1:alu

    wire        br_stall      = agex_cs[IDX_BR_STALL];
    wire        ld_reg        = agex_cs[IDX_LD_REG];
    wire        ld_cc         = agex_cs[IDX_LD_CC];

    // ------------------------------------------------------------
    // Sign-extensions used by ADDR2MUX and SR2MUX (Table 1)
    // ------------------------------------------------------------
    wire [15:0] sext_imm5  = {{11{agex_ir[4]}},  agex_ir[4:0]};
    wire [15:0] sext_off6  = {{10{agex_ir[5]}},  agex_ir[5:0]};
    wire [15:0] sext_pc9   = {{7{agex_ir[8]}},   agex_ir[8:0]};
    wire [15:0] sext_pc11  = {{5{agex_ir[10]}},  agex_ir[10:0]};

    // ------------------------------------------------------------
    // Address adder path
    //   ADDR1MUX: 0 -> NPC, 1 -> BaseR(SR1)
    //   ADDR2MUX: 00 -> 0, 01 -> off6, 10 -> pc9, 11 -> pc11
    //   LSHF1   : shift left 1 when asserted
    //   ADDRESSMUX: 0 -> LSHF(ZEXT(IR[7:0]),1), 1 -> adder output
    // ------------------------------------------------------------
    wire [15:0] addr1 = (addr1mux == 1'b0) ? agex_npc : agex_sr1;

    reg  [15:0] addr2_raw;
    always @(*) begin
        case (addr2mux)
            2'b00: addr2_raw = 16'h0000;
            2'b01: addr2_raw = sext_off6;
            2'b10: addr2_raw = sext_pc9;
            2'b11: addr2_raw = sext_pc11;
            default: addr2_raw = 16'h0000;
        endcase
    end

    wire [15:0] addr2 = (lshf1 == 1'b1) ? (addr2_raw << 1) : addr2_raw;
    wire [15:0] addr_adder_out = addr1 + addr2;

    wire [15:0] trapvec_addr = ({8'h00, agex_ir[7:0]} << 1);

    wire [15:0] address_out = (addressmux == 1'b0) ? trapvec_addr : addr_adder_out;

    // ------------------------------------------------------------
    // ALU + Shifter path
    //   SR2MUX: 0 -> SR2, 1 -> SEXT(IR[4:0])
    //   ALUK:   00 add, 01 and, 10 xor, 11 passa
    //   ALU.RESULTMUX: 0 -> shifter, 1 -> alu
    // ------------------------------------------------------------
    wire [15:0] alu_b = (sr2mux == 1'b0) ? agex_sr2 : sext_imm5;

    wire [15:0] alu_out;
    ALU u_alu(
        .a  (agex_sr1),
        .b  (alu_b),
        .op (aluk),
        .res(alu_out)
    );

    wire [15:0] shf_out;
    Shifter u_shf(
        .in        (agex_sr1),
        .shift_ctrl(agex_ir[5:0]),
        .out       (shf_out)
    );

    wire [15:0] alu_result_out = (alu_resultmux == 1'b0) ? shf_out : alu_out;

    // ------------------------------------------------------------
    // LD.MEM + propagation into MEM latches
    //   LD.MEM is generated by AGEX stage logic and must be deasserted
    //   when MEM.STALL is asserted (hold MEM latches).
    // ------------------------------------------------------------
    assign ld_mem   = ~mem_stall;
    assign mem_v_in = agex_v;
    assign mem_store_data_in = agex_sr2;

    assign mem_npc_in        = agex_npc;
    assign mem_ir_in         = agex_ir;
    assign mem_alu_result_in = alu_result_out;
    assign mem_address_in    = address_out;
    assign mem_cc_in         = agex_cc;
    assign mem_drid_in       = agex_drid;

    // Propagate only the bits needed in MEM/SR (Table 2: bits 12..22 => 11 bits)
    // mem_cs_in[0]  = BR.OP  (bit12)
    // mem_cs_in[10] = LD.CC  (bit22)
    assign mem_cs_in = {
        agex_cs[IDX_LD_CC],        // [10] bit22
        agex_cs[IDX_LD_REG],       // [9]  bit21
        agex_cs[IDX_DR_VALUEMUX0], // [8]  bit19
        agex_cs[IDX_DR_VALUEMUX1], // [7]  bit20
        agex_cs[IDX_DATA_SIZE],    // [6]  bit18
        agex_cs[IDX_DCACHE_RW],    // [5]  bit17
        agex_cs[IDX_DCACHE_EN],    // [4]  bit16
        agex_cs[IDX_BR_STALL],     // [3]  bit15
        agex_cs[IDX_TRAP_OP],      // [2]  bit14
        agex_cs[IDX_UNCON_OP],     // [1]  bit13
        agex_cs[IDX_BR_OP]         // [0]  bit12
    };

    // "Signals to previous stages" are valid-gated versions of CS bits (Lab 6 text)
    assign v_agex_ld_reg   = agex_v & ld_reg;
    assign v_agex_ld_cc    = agex_v & ld_cc;
    assign v_agex_br_stall = agex_v & br_stall;

endmodule

`endif // AGEX_STAGE_V
