//------------------------------------------------------------------------------
// control_store_fixed.v
//------------------------------------------------------------------------------
// Control Store ROM for the pipelined LC-3b (EE460N Lab 6 style).
//
// Per the Lab 6 documentation:
//   * The DE stage uses a 6-bit address formed by concatenating IR[15:11] and IR[5].
//   * The control store has 64 rows and 23 columns (23 control bits per entry).
//   * Bit numbering / meaning must match Table 2.
//   * Bits [0..2] are used in DE; the remaining 20 bits ([22:3]) are latched into AGEX.CS.
//
// This module implements a 64x23 ROM initialized from a text file with $readmemb.
// File format expectation (recommended): 64 lines, each line is a 23-bit binary string,
// e.g. 0010010... (23 chars), no '0b' prefix.
//
// NOTE: If you prefer the original UT "ucode" file (often spaced columns), create a
//       converted file for $readmemb, or change this module to parse with $fscanf.
//------------------------------------------------------------------------------

`timescale 1ns/1ps

module ControlStore #(
    parameter UCODE_FILE = "D:/DA1_KS/DA1_KS/src/ucode.mem"
) (
    // DE stage provides the 6-bit control-store address.
    // Per the Lab6 docs, this address is formed as: { IR[15:11], IR[5] }.
    input  wire [5:0]  addr,
    output reg  [22:0] cs_bits
);

    // Bit positions (Table 2)
    localparam integer CS_SR1_NEEDED     = 0;
    localparam integer CS_SR2_NEEDED     = 1;
    localparam integer CS_DRMUX          = 2;
    localparam integer CS_ADDR1MUX       = 3;
    localparam integer CS_ADDR2MUX1      = 4;
    localparam integer CS_ADDR2MUX0      = 5;
    localparam integer CS_LSHF1          = 6;
    localparam integer CS_ADDRESSMUX     = 7;
    localparam integer CS_SR2MUX         = 8;
    localparam integer CS_ALUK1          = 9;
    localparam integer CS_ALUK0          = 10;
    localparam integer CS_ALU_RESULTMUX  = 11;
    localparam integer CS_BR_OP          = 12;
    localparam integer CS_UNCON_OP       = 13;
    localparam integer CS_TRAP_OP        = 14;
    localparam integer CS_BR_STALL       = 15;
    localparam integer CS_DCACHE_EN      = 16;
    localparam integer CS_DCACHE_RW      = 17;
    localparam integer CS_DATA_SIZE      = 18;
    localparam integer CS_DR_VALUEMUX1   = 19;
    localparam integer CS_DR_VALUEMUX0   = 20;
    localparam integer CS_LD_REG         = 21;
    localparam integer CS_LD_CC          = 22;

    // 64x23 ROM
    reg [22:0] rom [0:63];

    integer i;
    integer fd;
    initial begin
        // Safe default
        for (i = 0; i < 64; i = i + 1) begin
            rom[i] = 23'b0;
        end

        // Load from file if present
        fd = $fopen(UCODE_FILE, "r");
        if (fd != 0) begin
            $fclose(fd);
            $readmemb(UCODE_FILE, rom);
        end else begin
            $display("[ControlStore] WARNING: cannot open %0s. ROM initialized to all zeros.", UCODE_FILE);
        end
    end

    // Combinational read
    always @(*) begin
        cs_bits = rom[addr];
    end

endmodule

