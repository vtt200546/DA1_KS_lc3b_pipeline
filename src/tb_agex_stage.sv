`timescale 1ns/1ps
//==============================================================
// tb_agex_stage.sv - Directed testbench for AgexStage (Lab 6)
//==============================================================
//
// Chạy nhanh (iverilog + gtkwave):
//   iverilog -g2012 -o sim tb_agex_stage.sv agex_stage_fixed.v alu_fixed.v shifter.v
//   vvp sim
//   gtkwave tb_agex_stage.vcd
//
// Mục tiêu kiểm:
//  1) Address adder path: ADDR1MUX/ADDR2MUX/LSHF1
//  2) ADDRESSMUX: trapvect8<<1 vs adder output
//  3) ALU path: SR2MUX + ALUK (ADD/AND/XOR/PASSB)
//  4) SHF path: IR[5:0] decode, ALU.RESULTMUX chọn SHF vs ALU
//  5) LD.MEM, MEM.V input
//  6) V.AGEX.LD.REG / V.AGEX.LD.CC / V.AGEX.BR.STALL = AGEX.V & CSbit
//
// NOTE: Testbench này giả định mapping AGEX.CS = ControlStore[22:3]
//       đúng như comment trong agex_stage_fixed.v.
//

module tb_agex_stage;

  // -----------------------
  // DUT inputs
  // -----------------------
  logic        agex_v;
  logic [15:0] agex_npc;
  logic [15:0] agex_ir;
  logic [15:0] agex_sr1;
  logic [15:0] agex_sr2;
  logic [2:0]  agex_cc;
  logic [2:0]  agex_drid;
  logic [19:0] agex_cs;
  logic        mem_stall;

  // -----------------------
  // DUT outputs
  // -----------------------
  wire        ld_mem;
  wire        mem_v_in;
  wire [15:0] mem_npc_in;
  wire [15:0] mem_ir_in;
  wire [15:0] mem_alu_result_in;
  wire [15:0] mem_address_in;
  wire [2:0]  mem_cc_in;
  wire [2:0]  mem_drid_in;
  wire [10:0] mem_cs_in;

  wire        v_agex_ld_reg;
  wire        v_agex_ld_cc;
  wire        v_agex_br_stall;

  // -----------------------
  // Instantiate DUT
  // -----------------------
  AgexStage dut (
    .agex_v(agex_v),
    .agex_npc(agex_npc),
    .agex_ir(agex_ir),
    .agex_sr1(agex_sr1),
    .agex_sr2(agex_sr2),
    .agex_cc(agex_cc),
    .agex_drid(agex_drid),
    .agex_cs(agex_cs),
    .mem_stall(mem_stall),
    .ld_mem(ld_mem),
    .mem_v_in(mem_v_in),
    .mem_npc_in(mem_npc_in),
    .mem_ir_in(mem_ir_in),
    .mem_alu_result_in(mem_alu_result_in),
    .mem_address_in(mem_address_in),
    .mem_cc_in(mem_cc_in),
    .mem_drid_in(mem_drid_in),
    .mem_cs_in(mem_cs_in),
    .v_agex_ld_reg(v_agex_ld_reg),
    .v_agex_ld_cc(v_agex_ld_cc),
    .v_agex_br_stall(v_agex_br_stall)
  );

  // -----------------------
  // Local copy of DUT index map (must match agex_stage_fixed.v)
  // -----------------------
  localparam int IDX_ADDR1MUX      = 0;  // CS bit 3
  localparam int IDX_ADDR2MUX1     = 1;  // CS bit 4
  localparam int IDX_ADDR2MUX0     = 2;  // CS bit 5
  localparam int IDX_LSHF1         = 3;  // CS bit 6
  localparam int IDX_ADDRESSMUX    = 4;  // CS bit 7
  localparam int IDX_SR2MUX        = 5;  // CS bit 8
  localparam int IDX_ALUK1         = 6;  // CS bit 9
  localparam int IDX_ALUK0         = 7;  // CS bit 10
  localparam int IDX_ALU_RESULTMUX = 8;  // CS bit 11

  localparam int IDX_BR_OP         = 9;  // bit12
  localparam int IDX_UNCON_OP      = 10; // bit13
  localparam int IDX_TRAP_OP       = 11; // bit14
  localparam int IDX_BR_STALL      = 12; // bit15
  localparam int IDX_DCACHE_EN     = 13; // bit16
  localparam int IDX_DCACHE_RW     = 14; // bit17
  localparam int IDX_DATA_SIZE     = 15; // bit18
  localparam int IDX_DR_VALUEMUX1  = 16; // bit19
  localparam int IDX_DR_VALUEMUX0  = 17; // bit20
  localparam int IDX_LD_REG        = 18; // bit21
  localparam int IDX_LD_CC         = 19; // bit22

  // -----------------------
  // Helpers: sign-extend
  // -----------------------
  function automatic [15:0] sext6(input [5:0] x);
    sext6 = {{10{x[5]}}, x};
  endfunction

  function automatic [15:0] sext5(input [4:0] x);
    sext5 = {{11{x[4]}}, x};
  endfunction

  function automatic [15:0] zext8_lshf1(input [7:0] x);
    zext8_lshf1 = ({8'h00, x} << 1);
  endfunction

  // -----------------------
  // Helpers: build AGEX.CS
  // -----------------------
  function automatic [19:0] make_cs(
    input bit addr1mux,
    input [1:0] addr2mux,
    input bit lshf1,
    input bit addressmux,
    input bit sr2mux,
    input [1:0] aluk,
    input bit alu_resultmux,
    input bit br_stall,
    input bit ld_reg,
    input bit ld_cc
  );
    automatic logic [19:0] cs;
    cs = '0;
    cs[IDX_ADDR1MUX]      = addr1mux;
    cs[IDX_ADDR2MUX1]     = addr2mux[1];
    cs[IDX_ADDR2MUX0]     = addr2mux[0];
    cs[IDX_LSHF1]         = lshf1;
    cs[IDX_ADDRESSMUX]    = addressmux;
    cs[IDX_SR2MUX]        = sr2mux;
    cs[IDX_ALUK1]         = aluk[1];
    cs[IDX_ALUK0]         = aluk[0];
    cs[IDX_ALU_RESULTMUX] = alu_resultmux;

    cs[IDX_BR_STALL]      = br_stall;
    cs[IDX_LD_REG]        = ld_reg;
    cs[IDX_LD_CC]         = ld_cc;
    make_cs = cs;
  endfunction

  // -----------------------
  // Check task
  // -----------------------
  task automatic check_eq16(input string tag, input [15:0] got, input [15:0] exp);
    if (got !== exp) $error("[%0t] %s: exp=0x%04h got=0x%04h", $time, tag, exp, got);
  endtask

  task automatic check_eq1(input string tag, input logic got, input logic exp);
    if (got !== exp) $error("[%0t] %s: exp=%0b got=%0b", $time, tag, exp, got);
  endtask

  task automatic settle();
    #1;
  endtask

  // -----------------------
  // Test sequence
  // -----------------------
  initial begin
    $dumpfile("tb_agex_stage.vcd");
    $dumpvars(0, tb_agex_stage);

    // Default init
    agex_v    = 1'b1;
    agex_npc  = 16'h3000;
    agex_ir   = 16'h0000;
    agex_sr1  = 16'h0000;
    agex_sr2  = 16'h0000;
    agex_cc   = 3'b010;
    agex_drid = 3'd0;
    agex_cs   = '0;
    mem_stall = 1'b0;

    // ------------------------------------------------------------
    // (0) LD.MEM and MEM.V propagation
    // ------------------------------------------------------------
    settle();
    check_eq1("LD_MEM_no_stall", ld_mem, 1'b1);
    check_eq1("MEM_V_in", mem_v_in, agex_v);

    mem_stall = 1'b1;
    settle();
    check_eq1("LD_MEM_stall", ld_mem, 1'b0);
    // mem_v_in vẫn = agex_v (input latch); khi stall, latch ngoài sẽ giữ.
    check_eq1("MEM_V_in_stall", mem_v_in, agex_v);
    mem_stall = 1'b0;

    // ------------------------------------------------------------
    // (1) Address adder: ADDR1MUX=0 (NPC), ADDR2MUX=00 (0), ADDRESSMUX=1 (adder)
    // expect: address = NPC
    // ------------------------------------------------------------
    agex_npc = 16'h3002;
    agex_sr1 = 16'h1234;
    agex_ir  = 16'h0000;
    agex_cs  = make_cs(
      /*addr1mux*/0, /*addr2mux*/2'b00, /*lshf1*/0, /*addressmux*/1,
      /*sr2mux*/0, /*aluk*/2'b00, /*alu_resultmux*/1,
      /*br_stall*/0, /*ld_reg*/0, /*ld_cc*/0
    );
    settle();
    check_eq16("ADDR_basic_npc", mem_address_in, 16'h3002);

    // ------------------------------------------------------------
    // (2) Address adder: BaseR + SEXT(off6), LSHF1=0
    //   addr1mux=1 (SR1), addr2mux=01 (off6)
    //   off6 = -4 (6'b111100) => 0xFFFC
    // expect: 0x1000 + 0xFFFC = 0x0FFC
    // ------------------------------------------------------------
    agex_sr1 = 16'h1000;
    agex_ir  = 16'h0000;
    agex_ir[5:0] = 6'b111100; // -4
    agex_cs  = make_cs(
      /*addr1mux*/1, /*addr2mux*/2'b01, /*lshf1*/0, /*addressmux*/1,
      /*sr2mux*/0, /*aluk*/2'b00, /*alu_resultmux*/1,
      /*br_stall*/0, /*ld_reg*/0, /*ld_cc*/0
    );
    settle();
    check_eq16("ADDR_base_off6", mem_address_in, 16'h0FFC);

    // ------------------------------------------------------------
    // (3) Same but LSHF1=1: (SEXT(off6) << 1) => 0xFFF8, expect 0x0FF8
    // ------------------------------------------------------------
    agex_cs[IDX_LSHF1] = 1'b1;
    settle();
    check_eq16("ADDR_base_off6_lshf1", mem_address_in, 16'h0FF8);

    // ------------------------------------------------------------
    // (4) ADDRESSMUX=0 => trapvect8<<1 (ignore adder)
    //   trapvect8 = 8'h20 => 0x0040
    // ------------------------------------------------------------
    agex_ir = 16'h0000;
    agex_ir[7:0] = 8'h20;
    agex_cs[IDX_ADDRESSMUX] = 1'b0; // select trapvec
    settle();
    check_eq16("ADDR_trapvec", mem_address_in, 16'h0040);

    // ------------------------------------------------------------
    // (5) ALU RESULTMUX=1 selects ALU; SR2MUX=0 selects SR2; ALUK=ADD
    //   SR1=3, SR2=4 => 7
    // ------------------------------------------------------------
    agex_sr1 = 16'h0003;
    agex_sr2 = 16'h0004;
    agex_ir  = 16'h0000;
    agex_cs  = make_cs(
      /*addr1mux*/0, /*addr2mux*/2'b00, /*lshf1*/0, /*addressmux*/1,
      /*sr2mux*/0, /*aluk*/2'b00, /*alu_resultmux*/1,
      /*br_stall*/0, /*ld_reg*/0, /*ld_cc*/0
    );
    settle();
    check_eq16("ALU_add_sr2", mem_alu_result_in, 16'h0007);

    // ------------------------------------------------------------
    // (6) SR2MUX=1 selects SEXT(imm5). ALUK=AND.
    //   SR1=0x00F0, imm5 = -1 (11111) => 0xFFFF => AND => 0x00F0
    // ------------------------------------------------------------
    agex_sr1 = 16'h00F0;
    agex_ir  = 16'h0000;
    agex_ir[4:0] = 5'b11111; // -1
    agex_cs  = make_cs(
      /*addr1mux*/0, /*addr2mux*/2'b00, /*lshf1*/0, /*addressmux*/1,
      /*sr2mux*/1, /*aluk*/2'b01, /*alu_resultmux*/1, // AND
      /*br_stall*/0, /*ld_reg*/0, /*ld_cc*/0
    );
    settle();
    check_eq16("ALU_and_imm5", mem_alu_result_in, 16'h00F0);

    // ------------------------------------------------------------
    // (7) PASSA: ALUK=11 (PASSA). SR2MUX=0. RESULTMUX=1
    //   SR2 = 0xABCD => output 0xABCD
    // ------------------------------------------------------------
    agex_sr1 = 16'h5555;
    agex_sr2 = 16'hABCD;
    agex_ir  = 16'h0000;
    agex_cs  = make_cs(
      /*addr1mux*/0, /*addr2mux*/2'b00, /*lshf1*/0, /*addressmux*/1,
      /*sr2mux*/0, /*aluk*/2'b11, /*alu_resultmux*/1,
      /*br_stall*/0, /*ld_reg*/0, /*ld_cc*/0
    );
    settle();
    check_eq16("ALU_passa", mem_alu_result_in, 16'h5555);

    // ------------------------------------------------------------
    // (8) SHIFTER selected: RESULTMUX=0
    //   LSHF: bit4=0, amount4=4. SR1=0x0001 => 0x0010
    // ------------------------------------------------------------
    agex_sr1 = 16'h0001;
    agex_ir  = 16'h0000;
    agex_ir[5:0] = 6'b0_0_0100; // mode=0, dir=0, amt=4
    agex_cs  = make_cs(
      /*addr1mux*/0, /*addr2mux*/2'b00, /*lshf1*/0, /*addressmux*/1,
      /*sr2mux*/0, /*aluk*/2'b00, /*alu_resultmux*/0, // choose shifter
      /*br_stall*/0, /*ld_reg*/0, /*ld_cc*/0
    );
    settle();
    check_eq16("SHF_lshf", mem_alu_result_in, 16'h0010);

    // (9) RSHFL: dir=1 (bit4=1), mode=0 (bit5=0), amt=1. SR1=0x8000 => 0x4000
    agex_sr1 = 16'h8000;
    agex_ir[5:0] = 6'b0_1_0001; // mode=0, dir=1, amt=1
    settle();
    check_eq16("SHF_rshfl", mem_alu_result_in, 16'h4000);

    // (10) RSHFA: dir=1, mode=1, amt=1. SR1=0x8000 => 0xC000
    agex_ir[5:0] = 6'b1_1_0001; // mode=1, dir=1, amt=1
    settle();
    check_eq16("SHF_rshfa", mem_alu_result_in, 16'hC000);

    // ------------------------------------------------------------
    // (11) Valid-gated signals to previous stages
    //   V.AGEX.* = AGEX.V & CSbit
    // ------------------------------------------------------------
    agex_cs = make_cs(
      /*addr1mux*/0, /*addr2mux*/2'b00, /*lshf1*/0, /*addressmux*/1,
      /*sr2mux*/0, /*aluk*/2'b00, /*alu_resultmux*/1,
      /*br_stall*/1, /*ld_reg*/1, /*ld_cc*/1
    );

    agex_v = 1'b1;
    settle();
    check_eq1("V_agex_ld_reg_on", v_agex_ld_reg, 1'b1);
    check_eq1("V_agex_ld_cc_on",  v_agex_ld_cc,  1'b1);
    check_eq1("V_agex_br_on",     v_agex_br_stall, 1'b1);

    agex_v = 1'b0;
    settle();
    check_eq1("V_agex_ld_reg_off", v_agex_ld_reg, 1'b0);
    check_eq1("V_agex_ld_cc_off",  v_agex_ld_cc,  1'b0);
    check_eq1("V_agex_br_off",     v_agex_br_stall, 1'b0);

    $display("All directed AgexStage tests completed.");
    #10;
    $finish;
  end

endmodule
