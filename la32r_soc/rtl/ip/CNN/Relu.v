

module Relu(
    input [31: 0] d_i,
    output [31: 0] d_o
);
    assign d_o = {32{~d_i[31]}} & d_i;
endmodule