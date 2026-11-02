module Encoder #(
    parameter RADIX=16,
    parameter WIDTH=$clog2(RADIX)
)(
    input [RADIX-1: 0] in,
    output [WIDTH-1: 0] out
);
genvar i, j;
generate
    for(i=0; i<WIDTH; i=i+1)begin
        localparam STEP = 2 << i;
        localparam STEP_NUM = 1 << i;
        localparam FULL_STEP_NUM = RADIX / STEP;
        localparam REMAIN = RADIX % STEP;
        localparam REMAIN_NUM = REMAIN < STEP_NUM ? 0 : STEP_NUM - REMAIN;
        localparam ALL_NUM = FULL_STEP_NUM * STEP_NUM + REMAIN_NUM;
        wire [ALL_NUM-1: 0] out_t;
        for(j=0; j<FULL_STEP_NUM; j=j+1)begin
            assign out_t[j*STEP_NUM +: STEP_NUM] = in[j*STEP+STEP_NUM +: STEP_NUM];
        end
        for(j=0; j<REMAIN_NUM; j=j+1)begin
            assign out_t[ALL_NUM-1-j] = in[RADIX-1-j];
        end
        assign out[i] = |out_t;
    end
endgenerate
endmodule

module Decoder #(
    parameter RADIX=16,
    parameter WIDTH=$clog2(RADIX)
)(
    input [WIDTH-1: 0] in,
    output [RADIX-1: 0] out
);
genvar i;
generate
    for(i=0; i<RADIX; i=i+1)begin
        assign out[i] = in == i;
    end
endgenerate
endmodule

module RDataGen(
    input [1: 0] size,
    input [1: 0] offset,
    input [31: 0] data,
    output [31: 0] data_o
);
    wire [7: 0] byte_data;
    wire [15: 0] half;
    wire [31: 0] word;
    assign byte_data = data[offset*8 +: 8];
    assign half = {16{offset[1]}} & data[31: 16] |
                  {16{~offset[1]}} & data[15: 0];
    assign word = data;
    assign data_o = {32{size[1]}} & word |
                    {32{size[0]}} & {{16{half[15]}}, half} |
                    {32{~|size}} & {{24{byte_data[7]}}, byte_data};
endmodule

module SplitReg #(
    parameter DATA_SIZE=1
)(
    input                     clk,
    input                     rst,
    input                     req,
    input [DATA_SIZE-1: 0]     d_i,
    output                     valid,
    input                     ready,
    output [DATA_SIZE-1: 0] d_o
);
    reg [DATA_SIZE-1: 0] d;
    reg [DATA_SIZE-1: 0] nxt_d;
    reg d_valid;
    reg nxt_d_valid;
    wire nxt_en;
    wire nxt_data_en;
    wire en;
    wire data_en;


    assign en           = req & ~d_valid | ready & ~req & ~nxt_d_valid;
    assign data_en      = req & (~d_valid | ~nxt_d_valid & ready);
    assign nxt_data_en  = req & (~ready & d_valid | nxt_d_valid);
    assign nxt_en       = req & ~ready & d_valid | ready & ~req & nxt_d_valid;

    always @(posedge clk)begin
        if(rst)begin
            nxt_d_valid <= 0;
            d_valid     <= 0;
        end
        else begin
            if(en) d_valid          <= req;
            if(nxt_en) nxt_d_valid  <= req;
        end
        if(nxt_data_en) nxt_d       <= d_i;
        if(data_en)     d           <= d_i;
    end
    assign valid = d_valid | nxt_d_valid;
    assign d_o = nxt_d_valid ? nxt_d : d;
endmodule