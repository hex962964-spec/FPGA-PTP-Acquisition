`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/03/29
// Design Name:
// Module Name: packet_info_fifo
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
//   包元信息 FIFO（描述符 FIFO）：
//   - 数据宽度：128bit
//   - 深度：16
//   - 用于缓存每一包对应的元信息，而不是把 timestamp 重复写入 sample_fifo
//
//   当前 descriptor 格式定义：
//   fifo_din[129:128]: time_quality[1:0]
//   fifo_din[127:64] : packet_timestamp[63:0]
//   fifo_din[63:48]  : packet_sample_count[15:0]
//   fifo_din[47:32]  : channel_id[15:0]
//   fifo_din[31:0]   : seq_id[31:0]
//
//   这样后续 packet_formatter 可以：
//   - 先读一条 descriptor
//   - 再从 sample_fifo 中连续读出对应 payload 样本
//
//   本次修订：
//   - 新增 clear 输入：用于顶层在 cfg_enable=0 时同步清空 FIFO 指针/计数
//   - clear / reset 时仅清空指针、计数和 fifo_dout，不主动清空 mem 数组内容
//     这样更利于综合推断，不会为了“清空存储体”引入额外复位逻辑
//   - 修复 full+read 边界语义：当 FIFO 已满且当前拍真实发生读出时，允许同时接受一笔写入
//     这样可避免上层在“满 + 同拍读”边界丢 descriptor，同时保持 FIFO 水位不变
//
// Dependencies:
//
// Revision:
// Revision 0.03 - Allow write on full when a real read happens in the same cycle
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module packet_info_fifo (
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

    input  wire [129:0] fifo_din;
    input  wire         fifo_wr_en;

    input  wire         fifo_rd_en;
    output  reg [129:0] fifo_dout;

    output wire         fifo_full;
    output wire         fifo_empty;
    output  reg [4:0]   fifo_data_count;

    (* ram_style = "distributed" *) reg [129:0] mem [0:15];
    reg [3:0]   wr_ptr;
    reg [3:0]   rd_ptr;

    wire do_write;
    wire do_read;
    wire [129:0] rd_data_old_w;

    assign fifo_empty = (fifo_data_count == 5'd0);
    assign fifo_full  = (fifo_data_count == 5'd16);

    // 显式规定同址读写语义：当 full + do_read + do_write 导致 rd_ptr==wr_ptr 时，
    // 本次读出必须返回“旧 descriptor”，新写入 descriptor 进入新的队尾位置。
    // 这里使用异步旧值抽头 + 时钟沿寄存，避免依赖工具默认 RAM collision 模式。
    assign rd_data_old_w = mem[rd_ptr];

    // 只有在当前拍真正读走一项时，才允许“满 + 读写同拍”继续接收一笔写入。
    // 这里使用 do_read 而不是 fifo_rd_en，避免在空 FIFO 上误判释放了空间。
    assign do_read  = fifo_rd_en && !fifo_empty;
    assign do_write = fifo_wr_en && (!fifo_full || do_read);

    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            wr_ptr          <= 4'd0;
            rd_ptr          <= 4'd0;
            fifo_data_count <= 5'd0;
            fifo_dout       <= 130'd0;
        end
        else if (clear) begin
            wr_ptr          <= 4'd0;
            rd_ptr          <= 4'd0;
            fifo_data_count <= 5'd0;
            fifo_dout       <= 130'd0;
        end
        else begin
            // 写：在一包完成后写入一条 descriptor
            if (do_write) begin
                mem[wr_ptr] <= fifo_din;
                wr_ptr      <= wr_ptr + 4'd1;
            end

            // 读：packet_formatter 先读取 descriptor，再读取 payload
            if (do_read) begin
                fifo_dout <= rd_data_old_w;
                rd_ptr    <= rd_ptr + 4'd1;
            end

            // FIFO 水位统计
            case ({do_write, do_read})
                2'b10: fifo_data_count <= fifo_data_count + 5'd1;
                2'b01: fifo_data_count <= fifo_data_count - 5'd1;
                default: fifo_data_count <= fifo_data_count;
            endcase
        end
    end

endmodule

`default_nettype wire
