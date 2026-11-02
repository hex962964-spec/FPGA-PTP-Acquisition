`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/03/29
// Design Name:
// Module Name: sample_fifo
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
//   样本数据 FIFO：
//   - 数据宽度：16bit
//   - 深度：256
//   - 作用：缓存样本 payload，解耦 sampler 与 packet_formatter
//
//   说明：
//   - 当前实现为单时钟同步 FIFO
//   - fifo_dout 在 fifo_rd_en 有效后的下一个时钟周期更新
//   - 新增 clear 输入：用于顶层在 cfg_enable=0 时同步清空 FIFO 指针/计数
//
//   实现注意：
//   - clear / reset 时只清空指针、计数和 fifo_dout，不主动清空 mem 数组内容
//   - 因为 FIFO 的有效性由 fifo_data_count 决定，只要计数清零即可保证行为正确
//   - 这样更利于 FPGA RAM / 寄存器资源推断，避免为“清内存”额外生成大规模复位逻辑
//
//   本次修订：
//   - 将 sample_fifo 深度从 128 拓宽到 256
//   - 对应将 wr_ptr / rd_ptr 扩为 8bit
//   - 对应将 fifo_data_count 扩为 9bit，以正确表示 0~256 的占用水位
//   - 修复 full+read 边界语义：当 FIFO 已满且当前拍真实发生读出时，允许同时接受一笔写入
//     这样可避免上层在“满 + 同拍读”边界丢样本，同时保持 FIFO 水位不变
//
// Dependencies:
//
// Revision:
// Revision 0.05 - Allow write on full when a real read happens in the same cycle
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module sample_fifo (
    clk_in,
    resetn,
    clear,

    fifo_din,
    fifo_wr_en,

    fifo_rd_en,
    fifo_dout,

    fifo_full,
    fifo_empty,
    fifo_data_count
);

    input  wire         clk_in;
    input  wire         resetn;
    input  wire         clear;

    input  wire [15:0]  fifo_din;
    input  wire         fifo_wr_en;

    input  wire         fifo_rd_en;
    output  reg [15:0]  fifo_dout;

    output wire         fifo_full;
    output  reg [8:0]   fifo_data_count;
    output wire         fifo_empty;

    (* ram_style = "distributed" *) reg [15:0] mem [0:255];
    reg [7:0]  wr_ptr;
    reg [7:0]  rd_ptr;

    wire do_write;
    wire do_read;
    wire [15:0] rd_data_old_w;

    assign fifo_empty = (fifo_data_count == 9'd0);
    assign fifo_full  = (fifo_data_count == 9'd256);

    // 显式规定同址读写语义：当 full + do_read + do_write 导致 rd_ptr==wr_ptr 时，
    // 本次读出必须返回“旧队头数据”，新写入数据进入新的队尾位置。
    // 这里使用异步旧值抽头 + 时钟沿寄存，避免把该语义交给工具默认 RAM collision 模式。
    assign rd_data_old_w = mem[rd_ptr];

    // 只有在当前拍真正读走一项时，才允许“满 + 读写同拍”继续接收一笔写入。
    // 这里使用 do_read 而不是 fifo_rd_en，避免在空 FIFO 上误判释放了空间。
    assign do_read  = fifo_rd_en && !fifo_empty;
    assign do_write = fifo_wr_en && (!fifo_full || do_read);

    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            wr_ptr          <= 8'd0;
            rd_ptr          <= 8'd0;
            fifo_data_count <= 9'd0;
            fifo_dout       <= 16'd0;
        end
        else if (clear) begin
            wr_ptr          <= 8'd0;
            rd_ptr          <= 8'd0;
            fifo_data_count <= 9'd0;
            fifo_dout       <= 16'd0;
        end
        else begin
            // 写：将 sample_out 写入 FIFO
            if (do_write) begin
                mem[wr_ptr] <= fifo_din;
                wr_ptr      <= wr_ptr + 8'd1;
            end

            // 读：将当前 rd_ptr 位置的数据送出到 fifo_dout
            if (do_read) begin
                fifo_dout <= rd_data_old_w;
                rd_ptr    <= rd_ptr + 8'd1;
            end

            // FIFO 水位统计
            case ({do_write, do_read})
                2'b10: fifo_data_count <= fifo_data_count + 9'd1;
                2'b01: fifo_data_count <= fifo_data_count - 9'd1;
                default: fifo_data_count <= fifo_data_count;
            endcase
        end
    end

endmodule

`default_nettype wire
