// Author: brimonzzy
// Create Date: 2025/2/13
// Description: Gentleman-Sande algorithm Radix-2 Butterfly unit
// module bf_rdx2 and module bf_rdx2_noW

module bf_rdx2 (
  input               inv,
  input signed [15:0] x0_re,
  input signed [15:0] x0_im,
  input signed [15:0] x1_re, 
  input signed [15:0] x1_im,
  input signed [15:0] w_re,
  input signed [15:0] w_im,
  output signed [15:0] y0_re,
  output signed [15:0] y0_im,
  output signed [15:0] y1_re,
  output signed [15:0] y1_im
);
    function signed [15:0] saturate(input signed [18:0] value);
    begin
        if (value > 32767)        saturate = 16'h7FFF;
        else if (value < -32768)  saturate = 16'h8000;
        else                      saturate = value[15:0];
    end
    endfunction

    wire signed [16:0] add1_re, add1_im;
    wire signed [16:0] sub2_re, sub2_im;
    wire signed [33:0] mul2_re, mul2_im;
    wire signed [18:0] mul2_re_Q15, mul2_im_Q15;
    wire signed [18:0] mul2_re_Q15_inv, mul2_im_Q15_inv;

    assign add1_re = x0_re + x1_re;
    assign add1_im = x0_im + x1_im;

    assign sub2_re = x0_re - x1_re;
    assign sub2_im = x0_im - x1_im;

    // res_re = x0_re * x1_re - x0_im * x1_im;
    // res_im = x0_re * x1_im + x0_im * x1_re;
    assign mul2_re = (sub2_re * w_re) - (sub2_im * w_im);
    assign mul2_im = (sub2_re * w_im) + (sub2_im * w_re);
    assign mul2_re_Q15 = mul2_re[33:15];
    assign mul2_im_Q15 = mul2_im[33:15];
    assign mul2_re_Q15_inv = mul2_re_Q15 >>> 1;
    assign mul2_im_Q15_inv = mul2_im_Q15 >>> 1;

    assign y0_re = (inv == 0) ? add1_re : (add1_re >>> 1);
    assign y0_im = (inv == 0) ? add1_im : (add1_im >>> 1);
    assign y1_re = (inv == 0) ? (mul2_re_Q15) : (mul2_re_Q15_inv);
    assign y1_im = (inv == 0) ? (mul2_im_Q15) : (mul2_im_Q15_inv);

    // assign y0_re = 16'h0;
    // assign y0_im = 16'h0;
    // assign y1_re = 16'h0;
    // assign y1_im = 16'h0;

    // wire overflow = (y1_re == 16'h7FFF) | (y1_re == 16'h8000) | (y1_im == 16'h7FFF) | (y1_im == 16'h8000);

endmodule


module bf_rdx2_noW (
  input               inv,
  input signed [15:0] x0_re,
  input signed [15:0] x0_im,
  input signed [15:0] x1_re, 
  input signed [15:0] x1_im,
  output signed [15:0] y0_re,
  output signed [15:0] y0_im,
  output signed [15:0] y1_re,
  output signed [15:0] y1_im
);

    wire signed [16:0] add1_re, add1_im;
    wire signed [16:0] sub2_re, sub2_im;

    assign add1_re = x0_re + x1_re;
    assign add1_im = x0_im + x1_im;

    assign sub2_re = x0_re - x1_re;
    assign sub2_im = x0_im - x1_im;

    assign y0_re = (inv == 0) ? add1_re : (add1_re >>> 1);
    assign y0_im = (inv == 0) ? add1_im : (add1_im >>> 1);
    assign y1_re = (inv == 0) ? sub2_re : (sub2_re >>> 1);
    assign y1_im = (inv == 0) ? sub2_im : (sub2_im >>> 1);

endmodule
