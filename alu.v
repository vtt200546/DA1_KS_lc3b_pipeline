//////////////////////////////////////////////////////////////////////////////////
//
// Author: 
// Fixed by: Assistant
// Filename: alu.v
// Modules: ALU
// Description: 4-function ALU (ADD, AND, XOR, PASSA)
//
//////////////////////////////////////////////////////////////////////////////////

`ifndef ALU_V
`define ALU_V

module ALU(
    input [15:0] a,
    input [15:0] b,
    input [1:0] op,
    output reg [15:0] res
);

    // Define operation codes based on control signals
    localparam ADD   = 2'b00;
    localparam AND   = 2'b01;
    localparam XOR   = 2'b10;
    localparam PASSA = 2'b11;

    always @(*) begin
        case (op)
            ADD: begin
                res = a + b;
            end
            AND: begin
                res = a & b;
            end
            XOR: begin
                res = a ^ b;
            end
            PASSA: begin
                res = a;
            end
            default: begin
                res = 16'b0;
            end
        endcase
    end

endmodule

`endif // ALU_V