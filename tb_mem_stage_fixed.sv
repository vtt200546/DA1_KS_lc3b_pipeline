`timescale 1ns/1ps
`default_nettype none

//==============================================================
// tb_mem_stage.sv - Directed testbench for MEM stage
//==============================================================
//
// Chạy nhanh (iverilog + gtkwave):
//   iverilog -g2012 -o sim tb_mem_stage.sv mem_stage_fixed.v
//   vvp sim
//   gtkwave tb_mem_stage.vcd
//
// Test trọng tâm:
//  1) WE[1:0] và DIN cho STW/STB
//  2) Byte load: chọn lane theo ADDR[0] và sign-extend
//  3) mem_stall = mem_v & dcache_en & ~dcache_r
//  4) BR logic (BR taken), TRAP logic (PCMUX=10, trap_pc=dcache_dout)
//  5) sr_v_in bubble khi mem_stall=1
//
module tb_mem_stage;

    // DUT inputs
    logic        mem_v;
    logic [15:0] mem_ir, mem_npc, mem_address, mem_alu_result, mem_store_data;
    logic [2:0]  mem_cc, mem_drid;
    logic [10:0] mem_cs;

    logic        dcache_r;
    logic [15:0] dcache_dout;

    // DUT outputs
    wire         dcache_en;
    wire [1:0]   dcache_we;
    wire [15:0]  dcache_addr, dcache_din;

    wire         mem_stall;
    wire [1:0]   mem_pcmux;
    wire [15:0]  target_pc, trap_pc;

    wire         sr_v_in;
    wire [15:0]  sr_npc_in, sr_ir_in, sr_address_in, sr_alu_result_in, sr_data_in;
    wire [2:0]   sr_drid_in;
    wire [3:0]   sr_cs_in;

    wire         v_mem_ld_reg, v_mem_ld_cc, v_mem_br_stall;

    mem_stage_fixed dut (
        .mem_v(mem_v),
        .mem_ir(mem_ir),
        .mem_npc(mem_npc),
        .mem_address(mem_address),
        .mem_alu_result(mem_alu_result),
        .mem_cc(mem_cc),
        .mem_drid(mem_drid),
        .mem_cs(mem_cs),
        .mem_store_data(mem_store_data),

        .dcache_r(dcache_r),
        .dcache_dout(dcache_dout),
        .dcache_en(dcache_en),
        .dcache_we(dcache_we),
        .dcache_addr(dcache_addr),
        .dcache_din(dcache_din),

        .mem_stall(mem_stall),
        .mem_pcmux(mem_pcmux),
        .target_pc(target_pc),
        .trap_pc(trap_pc),

        .sr_v_in(sr_v_in),
        .sr_npc_in(sr_npc_in),
        .sr_ir_in(sr_ir_in),
        .sr_address_in(sr_address_in),
        .sr_alu_result_in(sr_alu_result_in),
        .sr_data_in(sr_data_in),
        .sr_drid_in(sr_drid_in),
        .sr_cs_in(sr_cs_in),

        .v_mem_ld_reg(v_mem_ld_reg),
        .v_mem_ld_cc(v_mem_ld_cc),
        .v_mem_br_stall(v_mem_br_stall)
    );

    // Helpers for packing mem_cs (matching mem_stage_fixed.v comment)
    function automatic [10:0] CS(
        input bit br_op,
        input bit uncond_op,
        input bit trap_op,
        input bit br_stall,
        input bit dcen,
        input bit dc_rw,
        input bit data_size,
        input bit drvm0,
        input bit drvm1,
        input bit ldreg,
        input bit ldcc
    );
        CS = {ldcc, ldreg, drvm1, drvm0, data_size, dc_rw, dcen, br_stall, trap_op, uncond_op, br_op};
    endfunction

    task automatic check_val(string name, bit cond);
        if (!cond) begin
            $display("FAIL: %s", name);
            $fatal(1);
        end else begin
            $display("PASS: %s", name);
        end
    endtask

    initial begin
        $dumpfile("tb_mem_stage.vcd");
        $dumpvars(0, tb_mem_stage);

        // defaults
        mem_v         = 0;
        mem_ir        = 16'h0000;
        mem_npc       = 16'h3002;
        mem_address   = 16'h4000;
        mem_alu_result= 16'h1111;
        mem_store_data= 16'hA1B2;
        mem_cc        = 3'b010;   // Z
        mem_drid      = 3'd2;

        dcache_r      = 1;
        dcache_dout   = 16'h80FF;

        mem_cs        = CS(0,0,0,0,0,0,1,0,0,0,0);

        #1;

        // 1) Word store => WE=11, DIN=store_data
        mem_v         = 1;
        mem_address   = 16'h4000;
        mem_store_data= 16'hA1B2;
        mem_cs        = CS(0,0,0,0,1,1,1,0,0,0,0); // dcen=1, write=1, word=1
        #1;
        check_val("STW dcache_en=1", dcache_en == 1);
        check_val("STW WE=11", dcache_we == 2'b11);
        check_val("STW DIN=store_data", dcache_din == 16'hA1B2);

        // 2) Byte store at even addr => WE0=1, DIN[7:0]=byte
        mem_address   = 16'h4000; // even
        mem_store_data= 16'h005A;
        mem_cs        = CS(0,0,0,0,1,1,0,0,0,0,0); // byte=0
        #1;
        check_val("STB even WE=01", dcache_we == 2'b01);
        check_val("STB even DIN low byte", dcache_din == 16'h005A);

        // 3) Byte store at odd addr => WE1=1, DIN[15:8]=byte
        mem_address = 16'h4001; // odd
        #1;
        check_val("STB odd WE=10", dcache_we == 2'b10);
        check_val("STB odd DIN high byte", dcache_din == 16'h5A00);

        // 4) Byte load sign-extend: addr even selects low byte 0xFF => 0xFFFF
        mem_cs      = CS(0,0,0,0,1,0,0,0,0,0,0); // read, byte
        mem_address = 16'h5000;
        dcache_dout = 16'h80FF;
        #1;
        check_val("LDB even signext FF", sr_data_in == 16'hFFFF);

        // 5) Byte load sign-extend: addr odd selects high byte 0x80 => 0xFF80
        mem_address = 16'h5001;
        #1;
        check_val("LDB odd signext 80", sr_data_in == 16'hFF80);

        // 6) MEM.STALL when dcache_r=0
        dcache_r = 0;
        #1;
        check_val("MEM.STALL asserted", mem_stall == 1);
        check_val("SR bubble when stall", sr_v_in == 0);

        // 7) BR taken => PCMUX=01
        dcache_r = 1;
        mem_cc  = 3'b100;                      // N set
        mem_ir  = 16'b0000_100_000000000;      // BRn (n=1,z=0,p=0)
        mem_cs  = CS(1,0,0,1,0,0,1,0,0,0,0);   // br_op=1, br_stall=1, no dcache
        #1;
        check_val("BR taken PCMUX=01", mem_pcmux == 2'b01);

        // 8) TRAP => PCMUX=10, trap_pc=dcache_dout
        mem_ir      = 16'hF025;   // TRAP x25
        dcache_dout = 16'h1234;
        mem_cs      = CS(0,0,1,1,1,0,1,0,0,0,0); // trap_op=1, dcache read word
        #1;
        check_val("TRAP PCMUX=10", mem_pcmux == 2'b10);
        check_val("TRAP trap_pc=dout", trap_pc == 16'h1234);

        $display("All MEM stage tests passed.");
        $finish;
    end

endmodule

`default_nettype wire
