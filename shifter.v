//////////////////////////////////////////////////////////////////////////////////
//
// Author: Assistant
// Filename: shifter.v
// Modules: Shifter
// Description: Implementation of LC-3b Shift instructions (LSHF, RSHFL, RSHFA).
//              Controlled by IR[5:0].
//
//////////////////////////////////////////////////////////////////////////////////

`ifndef SHIFTER_V
`define SHIFTER_V

module Shifter(
    input  [15:0] in,          // Value to be shifted (from SR1)
    input  [5:0]  shift_ctrl,  // Control bits from IR[5:0]
    output reg [15:0] out      // Shifted result
);

    wire [3:0] amount;
    wire       direction; // 0 = Left, 1 = Right
    wire       mode;      // 0 = Logical, 1 = Arithmetic (only valid for Right shift)

    // Parse the control bits based on ISA
    assign amount    = shift_ctrl[3:0];
    assign direction = shift_ctrl[4];
    assign mode      = shift_ctrl[5];

    always @(*) begin
        if (direction == 1'b0) begin
            // LSHF: Logical Left Shift
            // Vacated bits filled with 0 [cite: 116]
            out = in << amount; 
        end
        else begin
            // Right Shift
            if (mode == 1'b0) begin
                // RSHFL: Logical Right Shift
                // Vacated bits filled with 0 [cite: 120]
                out = in >> amount;
            end
            else begin
                // RSHFA: Arithmetic Right Shift
                // Vacated bits filled with sign bit (in[15]) [cite: 120]
                // Using $signed() allows Verilog to perform arithmetic shift (>>>)
                out = $signed(in) >>> amount;
            end
        end
    end

endmodule

`endif // SHIFTER_V