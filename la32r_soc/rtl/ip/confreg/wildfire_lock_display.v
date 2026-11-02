// Wildfire MINI V1.1: six common-anode digits, segment order {DP,G,F,E,D,C,B,A}.
// Show only DIG1, with the normal 1/6 scan duty; the other five digits stay off.
module wildfire_lock_display #(
    parameter integer SLOT_CYCLES = 50000 // 1 ms at the SoC's 50 MHz sys_clk
)(
    input wire clk,
    input wire resetn,
    input wire [3:0] digit,
    output wire [7:0] seg_cathode,
    output wire [5:0] seg_anode
);
    localparam integer DIV_WIDTH = (SLOT_CYCLES > 1) ? $clog2(SLOT_CYCLES) : 1;
    reg [DIV_WIDTH-1:0] divider;
    reg [2:0] slot;
    wire [7:0] glyph = (digit == 4'd1) ? 8'hf9 : 8'hc0;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            divider <= 0;
            slot <= 0;
        end else if (divider == SLOT_CYCLES - 1) begin
            divider <= 0;
            slot <= (slot == 3'd5) ? 3'd0 : slot + 1'b1;
        end else begin
            divider <= divider + 1'b1;
        end
    end

    // One blank clock at each slot boundary lets the segment lines settle.
    assign seg_anode = (resetn && slot == 0 && divider != 0) ? 6'b000001 : 6'b000000;
    assign seg_cathode = (resetn && slot == 0) ? glyph : 8'hff;
endmodule
