
`timescale 1ns/1ps
`default_nettype none

//------------------------------------------------------------------------------
// tb_lc3bp_full_system.sv
//------------------------------------------------------------------------------
// Full-system (core + simple IMEM/DMEM models) testbench.
// It does NOT depend on external ucode.mem being found by the simulator.
// Instead, it seeds a few ControlStore ROM entries via hierarchical assignment.
//
// Program (starting at PC=0x3000):
//   0x3000: ADDI R1, R0, #5        ; R1 <- 5
//   0x3002: STW  R1, R0, #0        ; MEM[0] <- R1
//   0x3004: LDW  R2, R0, #0        ; R2 <- MEM[0] (=5)
//
// Expected:
//   DMEM[0] = 0x0005
//   R1      = 0x0005
//   R2      = 0x0005
//------------------------------------------------------------------------------

// Enable debug outputs in DUT
`define TESTING

//`include "lc3bp_top_full.v"

module tb_lc3bp_full_system;

    // Clock
    logic clk = 1'b0;
    // In this simplified TB, we run mem_clk in phase with clk
    wire  mem_clk = clk;

    // IMEM interface
    logic        imem_r;
    logic [15:0] instr;
    wire  [15:0] PC_out;

    // DMEM / D-cache interface
    wire         dcache_en;
    wire  [1:0]  dcache_we;
    wire  [15:0] dcache_addr;
    wire  [15:0] dcache_din;
    logic        dcache_r;
    logic [15:0] dcache_dout;

    // Debug outputs
    wire [127:0] reg_contents;
    wire [15:0]  PC_dbg, DE_IR_dbg, AGEX_IR_dbg, MEM_IR_dbg, SR_IR_dbg;

    // Simple memories (word-addressed arrays)
    logic [15:0] imem [0:32767];  // 64KB / 2
    logic [15:0] dmem [0:32767];  // 64KB / 2

    // Instantiate DUT
    LC3BP_TOP_FULL dut (
        .clk        (clk),
        .mem_clk    (mem_clk),
        .imem_r     (imem_r),
        .instr      (instr),
        .PC_out     (PC_out),

        .dcache_r   (dcache_r),
        .dcache_dout(dcache_dout),
        .dcache_en  (dcache_en),
        .dcache_we  (dcache_we),
        .dcache_addr(dcache_addr),
        .dcache_din (dcache_din),

        .reg_contents(reg_contents),
        .PC_dbg     (PC_dbg),
        .DE_IR_dbg  (DE_IR_dbg),
        .AGEX_IR_dbg(AGEX_IR_dbg),
        .MEM_IR_dbg (MEM_IR_dbg),
        .SR_IR_dbg  (SR_IR_dbg)
    );

    // Clock generation
    always #5 clk = ~clk;

    // IMEM: combinational read
    always_comb begin
        imem_r = 1'b1;
        instr  = imem[PC_out[15:1]];
    end

    // DMEM handshake: always "ready" (no stalls)
    always_comb begin
        dcache_r = 1'b1;
        if (dcache_en) begin
            dcache_dout = dmem[dcache_addr[15:1]];
        end else begin
            dcache_dout = 16'h0000;
        end
    end

    // DMEM write on posedge clk when WE asserted
    always_ff @(posedge clk) begin
        if (dcache_en && (dcache_we != 2'b00)) begin
            int unsigned idx;
            idx = dcache_addr[15:1];
            if (dcache_we[0]) dmem[idx][7:0]  <= dcache_din[7:0];
            if (dcache_we[1]) dmem[idx][15:8] <= dcache_din[15:8];
        end
    end

    // Simple helper
    task automatic check_eq16(input string name, input logic [15:0] got, input logic [15:0] exp);
        if (got !== exp) $display("FAIL: %s got=%h exp=%h", name, got, exp);
        else             $display("PASS: %s = %h", name, got);
    endtask

    // ControlStore entries we seed (bit map per decode_stage_fixed2 comments)
    // Bits: [0]=SR1.NEEDED [1]=SR2.NEEDED [2]=DRMUX ... [21]=LD.REG [22]=LD.CC
    localparam logic [22:0] CS_ADDI = 23'd7866625; // SR1 needed, SR2MUX=1, ALUK=ADD, ALU.RESULTMUX=1, DR.VALUEMUX=11, LD.REG/CC
    localparam logic [22:0] CS_LDW  = 23'd7667945; // Base+off6<<1, DCACHE.EN RD WORD, DR.VALUEMUX=01(DATA), LD.REG/CC
    localparam logic [22:0] CS_STW  = 23'd458987;  // Base+off6<<1, DCACHE.EN WR WORD, SR2 needed, no writeback

    // Addresses (ControlStore uses {IR[15:11], IR[5]})
    localparam int unsigned ADDR_ADDI = 6'd5;   // for IR=0x1225
    localparam int unsigned ADDR_LDW  = 6'd24;  // for IR=0x6400
    localparam int unsigned ADDR_STW  = 6'd28;  // for IR=0x7200

    // Program words
    localparam logic [15:0] IR_ADDI = 16'h1225; // ADDI R1,R0,#5
    localparam logic [15:0] IR_STW  = 16'h7200; // STW  R1,R0,#0
    localparam logic [15:0] IR_LDW  = 16'h6400; // LDW  R2,R0,#0

    // Simulation
    // Simulation
    integer k;
    initial begin
        // --- TỐI ƯU HÓA TỐC ĐỘ ---
        // Thay vì init toàn bộ 32k ô nhớ (gây lag), ta chỉ init vùng thấp
        // để đảm bảo R0, R1... đọc từ DMEM[0] không bị X.
        for (k = 0; k < 20; k++) begin
            imem[k] = 16'h0000;
            dmem[k] = 16'h0000;
        end
        // --------------------------

        // load program at 0x3000
        // PC 0x3000 >> 1 = 0x1800 (Decimal 6144)
        imem[16'h3000 >> 1] = IR_ADDI;
        imem[16'h3002 >> 1] = IR_STW;
        imem[16'h3004 >> 1] = IR_LDW;

        // Seed a few ControlStore ROM entries
        #1;
        dut.u_decode.CS.rom[ADDR_ADDI] = CS_ADDI;
        dut.u_decode.CS.rom[ADDR_STW]  = CS_STW;
        dut.u_decode.CS.rom[ADDR_LDW]  = CS_LDW;

        // Run simulation
        // Chạy 80 chu kỳ clock (đủ để thực hiện 3 lệnh)
        repeat (80) begin
            @(posedge clk);
            $display("t=%0t PC=%h | DE_IR=%h AGEX_IR=%h MEM_IR=%h SR_IR=%h | we=%b addr=%h din=%h dout=%h",
                     $time, PC_dbg, DE_IR_dbg, AGEX_IR_dbg, MEM_IR_dbg, SR_IR_dbg,
                     dcache_we, dcache_addr, dcache_din, dcache_dout);
        end

        // Check results
        check_eq16("DMEM[0]", dmem[0], 16'h0005);
        
        // Kiểm tra RegFile (R1 và R2)
        // Lưu ý: reg_contents lấy từ DUT debug port
        check_eq16("R1", reg_contents[31:16], 16'h0005);
        check_eq16("R2", reg_contents[47:32], 16'h0005);

        $display("Simulation Finished.");
        $finish;
    end
endmodule

`default_nettype wire
