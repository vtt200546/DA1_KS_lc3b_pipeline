`timescale 1ns/1ps
`default_nettype none

//------------------------------------------------------------------------------
// tb_lc3bp_full_system.sv (OPTIMIZED - METHOD 2)
//------------------------------------------------------------------------------
// Phiên bản tối ưu: KHÔNG DÙNG mảng bộ nhớ lớn.
// Sử dụng logic case/if để giả lập IMEM/DMEM, giúp mô phỏng cực nhanh.
//------------------------------------------------------------------------------

`define TESTING

// Lưu ý: Nếu bạn đã Add file vào Project thì KHÔNG cần include.
// Nếu chạy dòng lệnh (command line) thì mới cần bỏ comment dòng dưới.
// `include "lc3bp_top_full.v"

module tb_lc3bp_full_system;

    // -----------------------------------------------------------
    // 1. Khai báo tín hiệu (Signals)
    // -----------------------------------------------------------
    logic clk = 1'b0;
    wire  mem_clk = clk;
    always #5 clk = ~clk; // Clock period 10ns

    // Interface IMEM
    logic        imem_r;
    logic [15:0] instr;
    wire  [15:0] PC_out;

    // Interface DMEM
    wire         dcache_en;
    wire  [1:0]  dcache_we;
    wire  [15:0] dcache_addr;
    wire  [15:0] dcache_din;
    logic        dcache_r;
    logic [15:0] dcache_dout;

    // Debug Signals
    wire [127:0] reg_contents;
    wire [15:0]  PC_dbg, DE_IR_dbg, AGEX_IR_dbg, MEM_IR_dbg, SR_IR_dbg;

    // --- THAY THẾ MẢNG BỘ NHỚ LỚN BẰNG BIẾN ĐƠN ---
    // Vì bài test chỉ ghi/đọc tại offset 0 (STW ... #0), ta chỉ cần 1 biến này.
    logic [15:0] dmem_addr_0; 

    // -----------------------------------------------------------
    // 2. Instantiate DUT (Device Under Test)
    // -----------------------------------------------------------
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

    // -----------------------------------------------------------
    // 3. MÔ PHỎNG IMEM (Read-Only) - Method 2
    // -----------------------------------------------------------
    // Thay vì tra mảng imem[PC >> 1], ta dùng case để trả về lệnh trực tiếp.
    // Program:
    //   0x3000: ADDI R1, R0, #5  (Machine Code: 1225)
    //   0x3002: STW  R1, R0, #0  (Machine Code: 7200)
    //   0x3004: LDW  R2, R0, #0  (Machine Code: 6400)
    always_comb begin
        imem_r = 1'b1; // Luôn sẵn sàng
        case (PC_out)
            16'h3000: instr = 16'h1225; // ADDI R1, R0, #5
            16'h3002: instr = 16'h7200; // STW  R1, R0, #0
            16'h3004: instr = 16'h6400; // LDW  R2, R0, #0
            default:  instr = 16'h0000; // NOP (Các địa chỉ khác trả về 0)
        endcase
    end

    // -----------------------------------------------------------
    // 4. MÔ PHỎNG DMEM (Read/Write) - Method 2
    // -----------------------------------------------------------
    
    // Logic ĐỌC (Read)
    always_comb begin
        dcache_r = 1'b1; // Luôn sẵn sàng
        // Nếu CPU muốn đọc VÀ địa chỉ là 0
        if (dcache_en && (dcache_addr == 16'h0000)) begin
            dcache_dout = dmem_addr_0; // Trả về biến giả lập
        end else begin
            dcache_dout = 16'h0000;
        end
    end

    // Logic GHI (Write)
    always_ff @(posedge clk) begin
        if (dcache_en && (dcache_we != 2'b00)) begin
            // Nếu CPU muốn ghi vào địa chỉ 0
            if (dcache_addr == 16'h0000) begin
                if (dcache_we[0]) dmem_addr_0[7:0]  <= dcache_din[7:0];  // Ghi byte thấp
                if (dcache_we[1]) dmem_addr_0[15:8] <= dcache_din[15:8]; // Ghi byte cao
            end
        end
    end

    // -----------------------------------------------------------
    // 5. Setup Control Store (Giữ nguyên từ code cũ)
    // -----------------------------------------------------------
    // Các giá trị Control Store (Microcode) cần thiết cho 3 lệnh trên
    localparam logic [22:0] CS_ADDI = 23'd7866625;
    localparam logic [22:0] CS_LDW  = 23'd7667945;
    localparam logic [22:0] CS_STW  = 23'd458987;
    localparam int unsigned ADDR_ADDI = 6'd5;
    localparam int unsigned ADDR_LDW  = 6'd24;
    localparam int unsigned ADDR_STW  = 6'd28;

    // Helper: Hàm kiểm tra kết quả
    task automatic check_eq16(input string name, input logic [15:0] got, input logic [15:0] exp);
        if (got !== exp) $display("FAIL: %s got=%h exp=%h", name, got, exp);
        else             $display("PASS: %s = %h", name, got);
    endtask

    // -----------------------------------------------------------
    // 6. Main Simulation Loop
    // -----------------------------------------------------------
    initial begin
        // Khởi tạo biến nhớ giả lập
        dmem_addr_0 = 16'h0000;

        // Nạp thủ công vi chương trình vào ROM (để không phụ thuộc file ucode.mem)
        #1;
        dut.u_decode.CS.rom[ADDR_ADDI] = CS_ADDI;
        dut.u_decode.CS.rom[ADDR_STW]  = CS_STW;
        dut.u_decode.CS.rom[ADDR_LDW]  = CS_LDW;

        $display("Simulation Started (Optimized Mode)...");

        // Chạy 80 chu kỳ clock (đủ cho 3 lệnh đi qua 5 tầng pipeline)
        repeat (80) begin
            @(posedge clk);
            // In debug log mỗi chu kỳ
            $display("t=%0t PC=%h | DE=%h AG=%h MEM=%h SR=%h | R1=%h | DMEM[0]=%h",
                     $time, PC_dbg, DE_IR_dbg, AGEX_IR_dbg, MEM_IR_dbg, SR_IR_dbg, 
                     reg_contents[31:16], dmem_addr_0);
        end

        // Kiểm tra kết quả cuối cùng
        $display("---------------------------------------------------");
        // 1. Kiểm tra bộ nhớ: Lệnh STW có ghi được số 5 vào địa chỉ 0 không?
        check_eq16("DMEM[0]", dmem_addr_0, 16'h0005);
        
        // 2. Kiểm tra R1: Lệnh ADDI có tính ra 5 không?
        check_eq16("R1", reg_contents[31:16], 16'h0005);
        
        // 3. Kiểm tra R2: Lệnh LDW có đọc được số 5 từ bộ nhớ về R2 không?
        check_eq16("R2", reg_contents[47:32], 16'h0005);
        $display("---------------------------------------------------");

        $finish;
    end

endmodule
`default_nettype wire