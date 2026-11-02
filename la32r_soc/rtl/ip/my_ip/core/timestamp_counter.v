`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/03/29
// Design Name:
// Module Name: timestamp_counter
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
//   64 位自由运行时间戳计数器：
//   - 复位后清零
//   - 每个时钟周期自增 1
//
//   后续可升级方向：
//   - 支持外部时间基准校准
//   - 支持 enable / clear / load
//   - 支持双节点时间同步校正
//
// Dependencies:
//
// Revision:
// Revision 0.02 - Comments refined
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module timestamp_counter(
        clk_in,
        resetn,
        timestamp_now
    );

    input               clk_in;
    input               resetn;
    output  reg [63:0]  timestamp_now;

    always @(posedge clk_in or negedge resetn)
    if (!resetn)
        timestamp_now <= 64'd0;
    else
        timestamp_now <= timestamp_now + 64'd1;

endmodule
