`timescale 1ns/1ps
`default_nettype none

// MEM stage (Data Memory Access) for LC-3b Lab 6 style pipeline.
// - Generates D-cache interface signals (EN, WE[1:0], ADDR, DIN)
// - Generates MEM.STALL and BR-logic (PCMUX + TARGET/TRAP PC)
// - Prepares inputs for SR stage latches (SR.DATA includes byte shift + sign-extend)
// - Generates V.MEM.* signals for dependency check / frontend bubbling
//
// Assumes MEM.CS latch contains control-store bits [12..22] packed into mem_cs[10:0] as:
//   mem_cs[0]  = BR.OP      (bit 12)
//   mem_cs[1]  = UNCOND.OP  (bit 13)
//   mem_cs[2]  = TRAP.OP    (bit 14)
//   mem_cs[3]  = BR.STALL   (bit 15)
//   mem_cs[4]  = DCACHE.EN  (bit 16)
//   mem_cs[5]  = DCACHE.RW  (bit 17) : RD=0, WR=1
//   mem_cs[6]  = DATA.SIZE  (bit 18) : BYTE=0, WORD=1
//   mem_cs[7]  = DR.VALUEMUX0 (bit 20)
//   mem_cs[8]  = DR.VALUEMUX1 (bit 19)
//   mem_cs[9]  = LD.REG     (bit 21)
//   mem_cs[10] = LD.CC      (bit 22)

module mem_stage_fixed (
    // From MEM latches
    input  wire        mem_v,
    input  wire [15:0] mem_ir,
    input  wire [15:0] mem_npc,
    input  wire [15:0] mem_address,     // from AGEX address adder / ADDRESSMUX
    input  wire [15:0] mem_alu_result,   // from AGEX ALU
    input  wire [2:0]  mem_cc,           // N,Z,P in [2:0]
    input  wire [2:0]  mem_drid,
    input  wire [10:0] mem_cs,
    input  wire [15:0] mem_store_data,   // SR2 value forwarded for STB/STW

    // D-cache interface
    input  wire        dcache_r,         // access complete
    input  wire [15:0] dcache_dout,      // read data
    output wire        dcache_en,
    output wire [1:0]  dcache_we,        // WE[0]=low byte, WE[1]=high byte
    output wire [15:0] dcache_addr,
    output wire [15:0] dcache_din,

    // To Fetch stage for PC update
    output wire        mem_stall,
    output wire [1:0]  mem_pcmux,        // 00: PC+2, 01: TARGET, 10: TRAP
    output wire [15:0] target_pc,
    output wire [15:0] trap_pc,

    // To SR latches (data path)
    output wire        sr_v_in,
    output wire [15:0] sr_npc_in,
    output wire [15:0] sr_ir_in,
    output wire [15:0] sr_address_in,
    output wire [15:0] sr_alu_result_in,
    output wire [15:0] sr_data_in,
    output wire [2:0]  sr_drid_in,
    output wire [3:0]  sr_cs_in,         // {LD.CC, LD.REG, DR.VALUEMUX[1:0]}

    // To dependency check / frontend bubbling
    output wire        v_mem_ld_reg,
    output wire        v_mem_ld_cc,
    output wire        v_mem_br_stall
);

    // -------- Unpack control bits --------
    wire br_op      = mem_cs[0];
    wire uncond_op  = mem_cs[1];
    wire trap_op    = mem_cs[2];
    wire br_stall   = mem_cs[3];
    wire cs_dcen    = mem_cs[4];
    wire dcache_rw  = mem_cs[5];     // RD=0, WR=1
    wire data_size  = mem_cs[6];     // BYTE=0, WORD=1
    wire [1:0] dr_valuemux = {mem_cs[8], mem_cs[7]}; // [1]=bit19, [0]=bit20
    wire ld_reg     = mem_cs[9];
    wire ld_cc      = mem_cs[10];

    wire is_word  = (data_size == 1'b1);
    wire is_write = (dcache_rw == 1'b1);

    // -------- D-cache signals --------
    assign dcache_en   = mem_v & cs_dcen;
    assign dcache_addr = mem_address;

    // WE generation:
    // - No memory access => 00
    // - Read            => 00
    // - Store word      => 11
    // - Store byte      => address[0] selects low/high byte lane
    assign dcache_we =
        (!dcache_en)                 ? 2'b00 :
        (!is_write)                  ? 2'b00 :
        (is_word)                    ? 2'b11 :
        (mem_address[0] == 1'b0)     ? 2'b01 :  // low byte
                                       2'b10 ;  // high byte

    // Store data alignment:
    // For STB, mem[address] = SR[7:0]. Put SR[7:0] into enabled lane.
    assign dcache_din =
        (is_word) ? mem_store_data :
        (mem_address[0] == 1'b0) ? {8'h00, mem_store_data[7:0]} :
                                   {mem_store_data[7:0], 8'h00};

    // -------- MEM.STALL --------
    // Stall when a *valid* instruction is doing a memory access and DCACHE.R is 0.
    assign mem_stall = dcache_en & ~dcache_r;

    // -------- Read data path (shift + sign-extend for byte loads) --------
    wire [7:0]  byte_sel      = (mem_address[0] == 1'b0) ? dcache_dout[7:0] : dcache_dout[15:8];
    wire [15:0] mem_load_data = (is_word) ? dcache_dout : {{8{byte_sel[7]}}, byte_sel};

    // -------- BR LOGIC: decide PCMUX --------
    wire br_taken = (mem_ir[11] & mem_cc[2]) |
                    (mem_ir[10] & mem_cc[1]) |
                    (mem_ir[9]  & mem_cc[0]);

    // Target PC comes from MEM.ADDRESS (computed in AGEX)
    assign target_pc = mem_address;

    // TRAP PC is the word read from Trap Vector Table
    assign trap_pc = dcache_dout;

    assign mem_pcmux =
        (!mem_v)                        ? 2'b00 :
        (trap_op)                       ? 2'b10 :
        (uncond_op)                     ? 2'b01 :
        (br_op & br_taken)              ? 2'b01 :
                                          2'b00 ;

    // -------- SR latch inputs --------
    // When MEM stalls, insert a bubble into SR latches.
    assign sr_v_in          = (mem_stall) ? 1'b0 : mem_v;
    assign sr_npc_in        = mem_npc;
    assign sr_ir_in         = mem_ir;
    assign sr_address_in    = mem_address;
    assign sr_alu_result_in = mem_alu_result;
    assign sr_data_in       = mem_load_data;
    assign sr_drid_in       = mem_drid;
    assign sr_cs_in         = {ld_cc, ld_reg, dr_valuemux};

    // -------- V.MEM.* (gate with MEM.V) --------
    assign v_mem_ld_reg   = mem_v & ld_reg;
    assign v_mem_ld_cc    = mem_v & ld_cc;
    assign v_mem_br_stall = mem_v & br_stall;

endmodule

`default_nettype wire
