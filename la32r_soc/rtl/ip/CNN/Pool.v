`include "CNNConfig.vh"

module CNNPool(
    input clk,
    input rst,
    input stall,
    input                           conf_refresh,
    input [`KERNEL_SIZE-1: 0]     kernel_height,
    input [`KERNEL_SIZE-1: 0]      kernel_width,

    input [`POOL_MODE_WIDTH-1: 0]   pool_mode,

    input                           window_valid,
    input [`WINDOW_SIZE*32-1: 0]    window,
    output                          window_stall,
    output [31: 0]                  pool_data,
    output                          pool_valid,
    output                          pool_empty
);
    wire [`POOL_MODE-1: 0]          pool_mode_vec;
    reg [`WINDOW_SIZE: 0]           pool_valid_r;
    reg [`WINDOW_SIZE*32-1: 0]      res_r;
    parameter PVW = $clog2(`WINDOW_SIZE+1);
    reg [PVW-1: 0] pool_valid_idx;
    wire [PVW-1: 0] pool_valid_idx_n;
    wire [31: 0] pool_res;

    assign pool_valid_idx_n = {PVW{kernel_height[1] & kernel_width[1]}} & 'd5 |
                              {PVW{kernel_height[2] & kernel_width[2]}} & 'd9;

    always @(posedge clk)begin
        if(rst)begin
            pool_valid_idx <= 0;
        end
        else if(conf_refresh)begin
            pool_valid_idx <= pool_valid_idx_n;
        end
    end

    always @(posedge clk)begin
        if(rst)begin
            pool_valid_r <= 0;
        end
        else if(!stall_all)begin
            pool_valid_r <= {pool_valid_r[`WINDOW_SIZE-1: 0], window_valid};
        end
    end

    Decoder #(`POOL_MODE) decoder_pool_mode(pool_mode, pool_mode_vec);
    wire stall_all = pool_valid & stall;
    assign window_stall = stall_all;
    assign pool_valid = pool_valid_r[pool_valid_idx];
    assign pool_empty = ~(|pool_valid_r);
    wire [PVW-1: 0] pool_data_idx = pool_valid_idx - 1;
    assign pool_res = res_r[pool_data_idx*32 +: 32];
    // 只支持kernel_size == 2的平均池化
    wire [31: 0] pool_res_avg = ~pool_res[31] ? pool_res >> 2 : 0;
    assign pool_data  = pool_mode_vec[2] & kernel_height[1] & kernel_width[1] & ~pool_res[31]? pool_res_avg : pool_res;
    genvar i, j;
generate
    for(i=0; i<`WINDOW_SIZE; i=i+1)begin: gen_pool
        reg [(i+1)*32-1: 0] buffer_i;
        always @(posedge clk)begin
            if(~stall_all)begin
                if(window_valid)begin
                    buffer_i[i*32 +: 32] <= window[i*32 +: 32];
                end
                else begin
                    buffer_i[i*32 +: 32] <= 0;
                end
            end
        end

        for(j=i-1; j>=0; j=j-1)begin
            always @(posedge clk)begin
                if(~stall_all) buffer_i[j*32 +: 32] <= buffer_i[(j+1) * 32 +: 32];
            end
        end
        wire [31: 0] res;
        if(i != 0)begin
            PoolPE pe(
                .pool_mode(pool_mode_vec),
                .pool_data(res_r[(i-1)*32 +: 32]),
                .data(buffer_i[31: 0]),
                .data_o(res)
            );
        end
        else begin
            assign res = buffer_i[31: 0];
        end
        always @(posedge clk)begin
            if(!stall_all) res_r[i*32 +: 32] <= res;
        end
    end
endgenerate
endmodule

module PoolPE (
    input [`POOL_MODE-1: 0] pool_mode,
    input [31: 0] pool_data,
    input [31: 0] data,
    output [31: 0] data_o
);
    wire mode_le    = pool_mode[0];
    wire mode_ge    = pool_mode[1];
    wire mode_mean  = pool_mode[2];

    wire cin;
    wire [31: 0] adder;
    assign adder = mode_mean ? pool_data : ~pool_data;
    assign cin = ~mode_mean;
    wire [32: 0] add_res;
    assign add_res = data + adder + cin;
    assign data_o = mode_mean ? add_res[31: 0] :
                    mode_le & add_res[32] | mode_ge & ~add_res[32] ? data : pool_data;
endmodule