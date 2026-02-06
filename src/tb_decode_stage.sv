
`timescale 1ns/1ps
`default_nettype none

//==============================================================
// tb_decode_stage.sv (FIXED) - Directed tests for DecodeStage
//==============================================================
// Fixes vs previous TB:
//  - Build IR by assigning explicit bit fields (no ambiguous underscore grouping)
//  - Ensure IR[8:6], IR[13], IR[11:9], IR[2:0], and IR[5] are exactly what we intend
//
// Notes for Vivado/XSim:
//  - Make sure ucode_tb.mem is in the simulation working directory OR added as
//    "Simulation Source" so $readmemb can find it.
//==============================================================

module tb_decode_stage;

    logic clk = 0;
    always #5 clk = ~clk;

    logic mem_clk = 0;
    always #3 mem_clk = ~mem_clk;

    logic [15:0] de_npc;
    logic [15:0] de_ir;
    logic        de_v;

    logic v_agex_ld_reg, v_mem_ld_reg, v_sr_ld_reg;
    logic [2:0] agex_drid_old, mem_drid, sr_drid;
    logic [15:0] sr_reg_data;

    logic v_agex_ld_cc, v_mem_ld_cc, v_sr_ld_cc;
    logic [2:0] sr_cc_data;

    logic mem_stall;

    wire v_de_br_stall;
    wire dep_stall;
    wire ld_agex;

    wire [15:0] agex_sr1;
    wire [15:0] agex_sr2;
    wire [2:0]  agex_drid_new;
    wire [19:0] agex_cs;
    wire [2:0]  agex_cc;
    wire        agex_v;

`ifdef TESTING
    wire [127:0] reg_contents;
`endif

    DecodeStage dut (
`ifdef TESTING
        .reg_contents(reg_contents),
`endif
        .clk(clk),
        .mem_clk(mem_clk),
        .de_npc(de_npc),
        .de_ir(de_ir),
        .de_v(de_v),

        .v_agex_ld_reg(v_agex_ld_reg),
        .v_mem_ld_reg(v_mem_ld_reg),
        .v_sr_ld_reg(v_sr_ld_reg),

        .agex_drid_old(agex_drid_old),
        .mem_drid(mem_drid),
        .sr_drid(sr_drid),
        .sr_reg_data(sr_reg_data),

        .v_agex_ld_cc(v_agex_ld_cc),
        .v_mem_ld_cc(v_mem_ld_cc),
        .v_sr_ld_cc(v_sr_ld_cc),
        .sr_cc_data(sr_cc_data),

        .mem_stall(mem_stall),

        .v_de_br_stall(v_de_br_stall),
        .dep_stall(dep_stall),
        .ld_agex(ld_agex),

        .agex_sr1(agex_sr1),
        .agex_sr2(agex_sr2),
        .agex_drid_new(agex_drid_new),
        .agex_cs(agex_cs),
        .agex_cc(agex_cc),
        .agex_v(agex_v)
    );

    // Use deterministic microcode for tests
    defparam dut.CS.UCODE_FILE = "D:/DA1_KS/DA1_KS/src/ucode.mem";

    // Build IR with explicit bit fields (avoid overlapping mistakes)
    function automatic [15:0] mk_ir(
        input [4:0] ir15_11,
        input bit   ir13,
        input [2:0] ir11_9,
        input [2:0] ir8_6,
        input bit   ir5,
        input [2:0] ir2_0
    );
        reg [15:0] t;
        begin
            t = 16'h0000;
            t[15:11] = ir15_11;
            t[13]    = ir13;
            t[11:9]  = ir11_9;   // NOTE: ir11_9[2] must match ir15_11[0] (bit 11)
            t[8:6]   = ir8_6;
            t[5]     = ir5;
            t[2:0]   = ir2_0;
            mk_ir = t;
        end
    endfunction

    task automatic check_val(string name, bit cond);
        if (!cond) begin
            $display("FAIL: %s", name);
            $fatal(1);
        end else begin
            $display("PASS: %s", name);
        end
    endtask

    // Write a reg through SR writeback input (RegFile writes on posedge clk)
    task automatic wr_reg(input [2:0] r, input [15:0] v);
        begin
            @(negedge clk);
            v_sr_ld_reg = 1'b1;
            sr_drid     = r;
            sr_reg_data = v;
            @(posedge clk);
            // hold 1 more half-cycle then drop
            @(negedge clk);
            v_sr_ld_reg = 1'b0;
        end
    endtask

    task automatic wr_cc(input [2:0] cc);
        begin
            @(negedge clk);
            v_sr_ld_cc = 1'b1;
            sr_cc_data = cc;
            @(posedge clk);
            @(negedge clk);
            v_sr_ld_cc = 1'b0;
        end
    endtask

    initial begin
        $dumpfile("tb_decode_stage.vcd");
        $dumpvars(0, tb_decode_stage);

        // defaults
        de_npc = 16'h3002;
        de_ir  = 16'h0000;
        de_v   = 1'b0;

        v_agex_ld_reg = 0; v_mem_ld_reg = 0; v_sr_ld_reg = 0;
        agex_drid_old = 3'd0; mem_drid = 3'd0; sr_drid = 3'd0; sr_reg_data = 16'h0000;

        v_agex_ld_cc = 0; v_mem_ld_cc = 0; v_sr_ld_cc = 0; sr_cc_data = 3'b010;
        mem_stall = 0;

        // preload registers
        wr_reg(3'd1, 16'h1111);
        wr_reg(3'd2, 16'h2222);
        wr_reg(3'd3, 16'h3333);

        // 1) Read addressing
        // addr=4: {IR[15:11]=00010, IR[5]=0}
        // SR1=R1, IR13=0 => SR2=IR[2:0]=R2
        @(negedge clk);
        de_v  = 1'b1;
        de_ir = mk_ir(5'b00000, /*ir13*/0, /*ir11_9*/3'b000, /*ir8_6*/3'b001, /*ir5*/0, /*ir2_0*/3'b010);
        #1;
        check_val("Read SR1=R1", agex_sr1 == 16'h1111);
        check_val("Read SR2=R2 (IR13=0)", agex_sr2 == 16'h2222);

        // IR13=1 => SR2=IR[11:9]=R3 (011). Ensure bit11 matches ir15_11[0]=0.
        de_ir = mk_ir(5'b00010, /*ir13*/1, /*ir11_9*/3'b011, /*ir8_6*/3'b001, /*ir5*/0, /*ir2_0*/3'b000);
        #1;
        check_val("Read SR2=R3 (IR13=1)", agex_sr2 == 16'h3333);

        // 2) Dependency stall on SR1 hazard (addr=4 has SR1_NEEDED=1, SR2_NEEDED=1 in ucode_tb.mem)
        v_agex_ld_reg = 1'b1;
        agex_drid_old = 3'd1; // matches SR1
        #1;
        check_val("dep_stall asserted (SR1 hazard)", dep_stall == 1'b1);

        v_agex_ld_reg = 1'b0;
        #1;
        check_val("dep_stall deasserted (no hazards)", dep_stall == 1'b0);

        // 3) BR dependency stall (addr=5: {00010,1} has BR.OP=1 and BR.STALL=1 in ucode_tb.mem)
        de_ir = mk_ir(5'b00000, /*ir13*/0, /*ir11_9*/3'b000, /*ir8_6*/3'b000, /*ir5*/1, /*ir2_0*/3'b000);
        v_mem_ld_cc = 1'b1;
        #1;
        check_val("dep_stall asserted (BR waits for CC)", dep_stall == 1'b1);
        v_mem_ld_cc = 1'b0;
        #1;
        check_val("dep_stall deasserted (CC ready)", dep_stall == 1'b0);

        check_val("v_de_br_stall asserted", v_de_br_stall == 1'b1);

        // 4) DRMUX test: addr=18 => {01001,0}, ucode_tb.mem sets DRMUX=1, so DR=R7
        de_ir = mk_ir(5'b01001, /*ir13*/0, /*ir11_9*/3'b100, /*ir8_6*/3'b010, /*ir5*/0, /*ir2_0*/3'b000);
        #1;
        check_val("DRMUX forces R7", agex_drid_new == 3'd7);

        // 5) CC update
        wr_cc(3'b100); // N
        #1;
        check_val("agex_cc updated to N", agex_cc == 3'b100);

        $display("All DecodeStage tests passed.");
        $finish;
    end

endmodule

`default_nettype wire

