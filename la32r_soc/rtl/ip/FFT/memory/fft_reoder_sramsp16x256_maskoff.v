
module fft_reoder_sramsp16x256_maskoff (
  input clk,
  input ce,
  input rw,
  input [7:0] addr,
  input [15:0] din,
  output [15:0] dout
);

  sramsp_maskoff #(
    .DATA_WIDTH(16),
    .ADDR_WIDTH(8)
  ) sramsp16x256_maskoff  (
    .CLK      (clk ),
    .GWEN     (~rw ),
    .CEN      (~ce ),
    .A        (addr),
    .D        (din ),
    .Q        (dout)
  );

endmodule
