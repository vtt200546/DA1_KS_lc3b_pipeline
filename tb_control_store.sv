
`timescale 1ns/1ps
`default_nettype none

module tb_control_store;

    logic [5:0]  addr;
    wire  [22:0] cs_bits;

    // Reference ROM loaded in TB
    logic [22:0] ref_rom [0:63];

    ControlStore #(.UCODE_FILE("ucode.mem")) dut (
        .addr(addr),
        .cs_bits(cs_bits)
    );

    integer i;

    initial begin
        $dumpfile("tb_control_store.vcd");
        $dumpvars(0, tb_control_store);

        // Load same file into TB array
        $readmemb("ucode.mem", ref_rom);

        for (i = 0; i < 64; i = i + 1) begin
            addr = i[5:0];
            #1;
            if (cs_bits !== ref_rom[i]) begin
                $display("FAIL: addr=%0d cs_bits=%b ref=%b", i, cs_bits, ref_rom[i]);
                $fatal(1);
            end
        end

        $display("PASS: ControlStore outputs match ucode.mem for all 64 addresses.");
        $finish;
    end

endmodule

`default_nettype wire
