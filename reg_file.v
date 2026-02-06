`timescale 1ns/1ps
`default_nettype none

module RegFile(
`ifdef TESTING
    output wire [127:0] reg_contents,
`endif
    input  wire        clk,
    input  wire [2:0]  sr1,
    input  wire [2:0]  sr2,
    input  wire [2:0]  dr,
    input  wire [15:0] data_in,
    input  wire        we,
    output wire [15:0] sr1_out,
    output wire [15:0] sr2_out
);

    reg [15:0] R [0:7];

    integer i;
    initial begin
        for (i = 0; i < 8; i = i + 1) begin
            R[i] = 16'h0000;
        end
    end

    always @(posedge clk) begin
        if (we) begin
            R[dr] <= data_in;
        end
    end

    assign sr1_out = R[sr1];
    assign sr2_out = R[sr2];

`ifdef TESTING
    assign reg_contents = {R[7], R[6], R[5], R[4], R[3], R[2], R[1], R[0]};
`endif

endmodule

`default_nettype wire
