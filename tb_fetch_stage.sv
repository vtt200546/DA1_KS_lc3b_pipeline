`timescale 1ns/1ps
// Testbench for FetchStage (combinational).
//
// Mục tiêu kiểm chứng theo spec Lab6:
//  - PC chỉ tăng +2 khi ICACHE.R=1 và không DEP.STALL/MEM.STALL/BR.STALL
//  - Nếu ICACHE.R=0 (không stall bởi dep/mem) thì bơm bubble: DE.V=0 và vẫn cho load DE latch
//  - DEP.STALL hoặc MEM.STALL: giữ DE latch (LD.DE=0) và không cập nhật PC
//  - Redirect PC từ MEM.PCMUX (TARGET.PC / TRAP.PC), nhưng vẫn phải bị chặn khi MEM.STALL=1
//
// Cách chạy nhanh (iverilog + gtkwave):
//   iverilog -g2012 -o sim tb_fetch_stage.sv fetch_stage_fixed.v
//   vvp sim
//   gtkwave tb_fetch_stage.vcd
//
module tb_fetch_stage;

  // "Clock" chỉ để mô phỏng PC/DE latch (FetchStage bản thân là combinational)
  logic clk = 0;
  always #5 clk = ~clk;

  // ==== DUT inputs ====
  logic [15:0] pc_r = 16'h3000;          // PC register emulation
  logic        dep_stall;
  logic        mem_stall;
  logic        v_de_br_stall;
  logic        v_agex_br_stall;
  logic        v_mem_br_stall;
  logic        imem_r;
  logic [1:0]  mem_pcmux;
  logic [15:0] target_pc;
  logic [15:0] trap_pc;
  logic [15:0] instr;

  // ==== DUT outputs ====
  wire         ld_pc;
  wire [15:0]  de_npc;
  wire [15:0]  de_ir;
  wire         de_v;
  wire         ld_de;
  wire [15:0]  new_pc;

  // ==== DE latch emulation ====
  logic [15:0] de_npc_r='0;
  logic [15:0] de_ir_r='0;
  logic        de_v_r=1'b0;

  // Connect PC input directly from pc_r
  wire [15:0] pc = pc_r;

  // Instantiate DUT
  FetchStage dut (
    .pc(pc),
    .dep_stall(dep_stall),
    .mem_stall(mem_stall),
    .v_de_br_stall(v_de_br_stall),
    .v_agex_br_stall(v_agex_br_stall),
    .v_mem_br_stall(v_mem_br_stall),
    .imem_r(imem_r),
    .mem_pcmux(mem_pcmux),
    .target_pc(target_pc),
    .trap_pc(trap_pc),
    .instr(instr),
    .ld_pc(ld_pc),
    .de_npc(de_npc),
    .de_ir(de_ir),
    .de_v(de_v),
    .ld_de(ld_de),
    .new_pc(new_pc)
  );

  // Update PC & DE latches on clock edge (giống pipeline)
  always_ff @(posedge clk) begin
    if (ld_pc) pc_r <= new_pc;

    if (ld_de) begin
      de_npc_r <= de_npc;
      de_ir_r  <= de_ir;
      de_v_r   <= de_v;
    end
  end

  // ==== Helpers ====
  function automatic [15:0] pc_plus2(input [15:0] x);
    pc_plus2 = x + 16'd2;
  endfunction

  task automatic settle_and_check(
    input string tag,
    input bit exp_ld_pc,
    input bit exp_ld_de,
    input bit exp_de_v,
    input [15:0] exp_new_pc,
    input [15:0] exp_de_ir
  );
    // cho combinational settle
    #1;
    if (ld_pc !== exp_ld_pc)
      $error("[%0t] %s: ld_pc exp=%0b got=%0b", $time, tag, exp_ld_pc, ld_pc);
    if (ld_de !== exp_ld_de)
      $error("[%0t] %s: ld_de exp=%0b got=%0b", $time, tag, exp_ld_de, ld_de);
    if (de_v !== exp_de_v)
      $error("[%0t] %s: de_v exp=%0b got=%0b", $time, tag, exp_de_v, de_v);
    if (new_pc !== exp_new_pc)
      $error("[%0t] %s: new_pc exp=0x%04h got=0x%04h", $time, tag, exp_new_pc, new_pc);
    if (de_ir !== exp_de_ir)
      $error("[%0t] %s: de_ir exp=0x%04h got=0x%04h", $time, tag, exp_de_ir, de_ir);

    // de_npc luôn là pc+2 theo design
    if (de_npc !== pc_plus2(pc))
      $error("[%0t] %s: de_npc exp=pc+2=0x%04h got=0x%04h", $time, tag, pc_plus2(pc), de_npc);

    $display("[%0t] %s | pc=0x%04h ld_pc=%0b new_pc=0x%04h | ld_de=%0b de_v=%0b de_ir=0x%04h",
             $time, tag, pc, ld_pc, new_pc, ld_de, de_v, de_ir);
  endtask

  task automatic drive_defaults();
    dep_stall      = 0;
    mem_stall      = 0;
    v_de_br_stall  = 0;
    v_agex_br_stall= 0;
    v_mem_br_stall = 0;
    imem_r         = 1;
    mem_pcmux      = 2'b00;
    target_pc      = 16'h4000;
    trap_pc        = 16'h8000;
    instr          = 16'h1234;
  endtask

  // ==== Test sequence ====
  initial begin
    $dumpfile("tb_fetch_stage.vcd");
    $dumpvars(0, tb_fetch_stage);

    // init registers
//    pc_r     = 16'h3000;
//    de_npc_r = '0;
//    de_ir_r  = '0;
//    de_v_r   = 1'b0;

    drive_defaults();

    // (1) Normal sequential fetch: imem_r=1, no stalls -> PC+2, DE loads instr, DE.V=1
    settle_and_check("SEQ_OK",
      /*exp_ld_pc*/ 1,
      /*exp_ld_de*/ 1,
      /*exp_de_v*/  1,
      /*exp_new_pc*/ pc_plus2(pc),
      /*exp_de_ir*/  instr
    );
    @(posedge clk); // commit PC & DE

    // (2) ICACHE miss: imem_r=0 (không dep/mem stall) -> bubble (DE.V=0), PC không update
    imem_r = 0;
    instr  = 16'h9999; // should be ignored
    settle_and_check("ICACHE_MISS_BUBBLE",
      /*exp_ld_pc*/ 0,
      /*exp_ld_de*/ 1,
      /*exp_de_v*/  0,
      /*exp_new_pc*/ pc_plus2(pc),
      /*exp_de_ir*/  16'h0000
    );
    @(posedge clk); // DE latch should capture bubble

    // (3) DEP.STALL: giữ DE latch, không update PC
    drive_defaults();
    dep_stall = 1;
    instr     = 16'h2222;
    settle_and_check("DEP_STALL_HOLD_DE",
      /*exp_ld_pc*/ 0,
      /*exp_ld_de*/ 0,
      /*exp_de_v*/  1,            // combinational vẫn =1 nhưng latch sẽ giữ do ld_de=0
      /*exp_new_pc*/ pc_plus2(pc),
      /*exp_de_ir*/  instr
    );
    @(posedge clk);

    // (4) MEM.STALL: giữ DE latch, không update PC
    drive_defaults();
    mem_stall = 1;
    instr     = 16'h3333;
    settle_and_check("MEM_STALL_HOLD_ALL",
      /*exp_ld_pc*/ 0,
      /*exp_ld_de*/ 0,
      /*exp_de_v*/  1,
      /*exp_new_pc*/ pc_plus2(pc),
      /*exp_de_ir*/  instr
    );
    @(posedge clk);

    // (5) Control-instruction stall (BR stall): DE.V=0 để chèn bubble, PC đứng
    drive_defaults();
    v_de_br_stall = 1;
    instr         = 16'h4444;
    settle_and_check("BR_STALL_INSERT_BUBBLE",
      /*exp_ld_pc*/ 0,
      /*exp_ld_de*/ 1,
      /*exp_de_v*/  0,
      /*exp_new_pc*/ pc_plus2(pc),
      /*exp_de_ir*/  instr  // de_ir vẫn bằng instr khi imem_r=1; bubble nằm ở de_v=0
    );
    @(posedge clk);

    // (6) Redirect TARGET.PC (MEM.PCMUX=01): PC phải nhảy sang target_pc khi mem_stall=0
    drive_defaults();
    mem_pcmux = 2'b01;
    target_pc = 16'h4ABC;
    settle_and_check("REDIRECT_TARGET_PC",
      /*exp_ld_pc*/ 1,
      /*exp_ld_de*/ 1,
      /*exp_de_v*/  1,
      /*exp_new_pc*/ target_pc,
      /*exp_de_ir*/  instr
    );
    @(posedge clk);

    // (7) Redirect TRAP.PC (MEM.PCMUX=10)
    drive_defaults();
    mem_pcmux = 2'b10;
    trap_pc   = 16'h8F00;
    settle_and_check("REDIRECT_TRAP_PC",
      /*exp_ld_pc*/ 1,
      /*exp_ld_de*/ 1,
      /*exp_de_v*/  1,
      /*exp_new_pc*/ trap_pc,
      /*exp_de_ir*/  instr
    );
    @(posedge clk);

    // (8) Redirect nhưng MEM.STALL=1: phải chặn LD.PC (PC không update)
    drive_defaults();
    mem_pcmux = 2'b01;
    target_pc = 16'h5000;
    mem_stall = 1;
    settle_and_check("REDIRECT_BLOCKED_BY_MEM_STALL",
      /*exp_ld_pc*/ 0,
      /*exp_ld_de*/ 0,
      /*exp_de_v*/  1,
      /*exp_new_pc*/ target_pc, // mux vẫn ra target_pc nhưng không được chốt vì ld_pc=0
      /*exp_de_ir*/  instr
    );
    @(posedge clk);

    $display("All directed FetchStage tests completed.");
    #20;
    $finish;
  end

endmodule
