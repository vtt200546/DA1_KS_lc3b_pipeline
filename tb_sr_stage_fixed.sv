`timescale 1ns/1ps
`default_nettype none

module tb_sr_stage;

    logic        sr_v;
    logic [15:0] sr_ir;
    logic [15:0] sr_npc, sr_address, sr_alu_result, sr_data;
    logic [2:0]  sr_drid;
    logic [3:0]  sr_cs;

    wire         v_sr_ld_reg, v_sr_ld_cc;
    wire [2:0]   sr_drid_out;
    wire [15:0]  sr_reg_data;
    wire [2:0]   sr_cc_data;

    sr_stage_fixed dut (
        .sr_v(sr_v),
        .sr_ir(sr_ir),
        .sr_npc(sr_npc),
        .sr_address(sr_address),
        .sr_alu_result(sr_alu_result),
        .sr_data(sr_data),
        .sr_drid(sr_drid),
        .sr_cs(sr_cs),
        .v_sr_ld_reg(v_sr_ld_reg),
        .v_sr_ld_cc(v_sr_ld_cc),
        .sr_drid_out(sr_drid_out),
        .sr_reg_data(sr_reg_data),
        .sr_cc_data(sr_cc_data)
    );

    function automatic [3:0] CS(input bit ldcc, input bit ldreg, input [1:0] mux);
        CS = {ldcc, ldreg, mux};
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
        $dumpfile("tb_sr_stage.vcd");
        $dumpvars(0, tb_sr_stage);

        sr_v = 1'b1;
        sr_ir = 16'h0000;
        sr_npc = 16'h3002;
        sr_address = 16'h1234;
        sr_alu_result = 16'hBEEF;
        sr_data = 16'h00F0;
        sr_drid = 3'd5;

        // 1) ADDRESS -> positive => P
        sr_cs = CS(1,1,2'b00);
        #1;
        check_val("WB=ADDRESS", sr_reg_data == 16'h1234);
        check_val("CC=P", sr_cc_data == 3'b001);

        // 2) DATA -> negative => N
        sr_data = 16'h8001;
        sr_cs = CS(1,1,2'b01);
        #1;
        check_val("WB=DATA", sr_reg_data == 16'h8001);
        check_val("CC=N", sr_cc_data == 3'b100);

        // 3) NPC -> zero => Z
        sr_npc = 16'h0000;
        sr_cs = CS(1,1,2'b10);
        #1;
        check_val("WB=NPC", sr_reg_data == 16'h0000);
        check_val("CC=Z", sr_cc_data == 3'b010);

        // 4) ALU.RESULT -> negative => N
        sr_cs = CS(1,1,2'b11);
        #1;
        check_val("WB=ALU.RESULT", sr_reg_data == 16'hBEEF);
        check_val("CC=N2", sr_cc_data == 3'b100);

        // 5) Valid gating
        sr_v = 1'b0;
        sr_cs = CS(1,1,2'b00);
        #1;
        check_val("v_sr_ld_reg gated", v_sr_ld_reg == 1'b0);
        check_val("v_sr_ld_cc gated",  v_sr_ld_cc  == 1'b0);

        // 6) ld_reg/ld_cc off
        sr_v = 1'b1;
        sr_cs = CS(0,0,2'b00);
        #1;
        check_val("v_sr_ld_reg off", v_sr_ld_reg == 1'b0);
        check_val("v_sr_ld_cc off",  v_sr_ld_cc  == 1'b0);

        check_val("DRID passthrough", sr_drid_out == 3'd5);

        $display("All SR stage tests passed.");
        $finish;
    end

endmodule

`default_nettype wire
