`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/03/29
// Design Name:
// Module Name: sampler
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
//   采样接纳模块：
//   1) 按 sample_period 控制相邻“被接纳样本”之间的最小时钟间隔
//   2) 按 samples_per_packet 进行分包，并在硬件内限制为 1~256
//   3) 在每包第一个样本处锁存 packet_timestamp
//   4) 向 sample_fifo 输出 sample_out / sample_push
//   5) 向 packet_info_fifo 提供 packet_done 时刻对应的包元信息
//
//   当前语义（本次修订后）：
//   - sample_period = 0 : 只要 ready=1 且 valid=1，就可以连续每拍接纳
//   - sample_period = N : 每接纳 1 个样本后，再等待 N 个时钟周期，才重新允许下一次接纳
//   - 当等待窗口结束后，如果 valid 还没来，则 ready 会一直保持为 1，直到真正接纳到下一个样本
//
//   这样做的好处：
//   - 对 internal source：效果等价于“每隔 N+1 个时钟取一个样本”
//   - 对 external source：sample_ready 可以真正作为 valid/ready 握手的 ready 使用，不会死锁
//
//   说明：
//   - fifo_full / packet_info_full 在顶层可接“保护性 full”信号：
//       已计入本拍 FIFO 读写后，判断下一次接纳/新起一包是否还有安全空位
//   - packet_info_full 只在“准备开始一个新包”时参与阻塞，避免产生无描述符包
//   - packet_active 表示“当前正在组一包但尚未完成”
//   - samples_per_packet 在 sampler 内部再次限幅到 1~256，作为 CSR 之外的防御性保护
//
// Dependencies:
//
// Revision:
// Revision 0.04 - Clamp samples_per_packet to 1..256
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module sampler(
        clk_in,
        resetn,
        enable,

        sample_period,       // 相邻两个“被接纳样本”之间的最小等待周期：0=连续接纳，N=接纳后等待N拍
        samples_per_packet,  // 每包多少个样本：0->1，>256->256

        timestamp_now,       // 来自 timestamp_counter
        sample_data_in,      // 来自 data_gen / 外部输入
        sample_valid_in,
        fifo_full,           // 来自顶层的保护性 full 信号；无安全空位时停止接纳新样本
        packet_info_full,    // 来自顶层的保护性 full 信号；无 descriptor 预留空位时禁止起新包

        sample_ready,        // 当前是否可以真正接纳一个样本（ready 语义）
        sample_out,
        sample_push,         // 写 sample_fifo 脉冲（单周期）
        sample_overrun_pulse,// 单周期：时序窗口打开但 payload FIFO 无安全空位
        packet_start_block_pulse, // 单周期：准备起新包但 descriptor FIFO 无安全空位
        packet_start,        // 一包开始脉冲（单周期）
        packet_done,         // 一包结束脉冲（单周期）
        packet_active,       // 当前是否处于一个未完成的 packet 中
        packet_timestamp,    // 当前包起始时间戳（锁存在包首样本）
        packet_sample_count  // 当前包已收样本数（调试用 / 在 packet_done 时为本包最终样本数）
    );


    input  wire         clk_in;
    input  wire         resetn;
    input  wire         enable;

    input  wire [31:0]  sample_period;
    input  wire [15:0]  samples_per_packet;

    input  wire [63:0]  timestamp_now;
    input  wire [15:0]  sample_data_in;
    input  wire         sample_valid_in;
    input  wire         fifo_full;
    input  wire         packet_info_full;

    output wire         sample_ready;
    output  reg [15:0]  sample_out;
    output  reg         sample_push;
    output  reg         sample_overrun_pulse;
    output  reg         packet_start_block_pulse;
    output  reg         packet_start;
    output  reg         packet_done;
    output wire         packet_active;
    output  reg [63:0]  packet_timestamp;
    output  reg [15:0]  packet_sample_count;

    // ------------------------------------------------------------------------
    // 锁存后的配置寄存器
    // 建议仅在 enable = 0 时修改配置
    // ------------------------------------------------------------------------
    reg         [31:0]  cnt_sample_period;
    reg         [31:0]  sample_period_Max;

    reg         [15:0]  cnt_sample_packet;
    reg         [15:0]  sample_packet_Max;

    // ------------------------------------------------------------------------
    // samples_per_packet safety policy
    // ------------------------------------------------------------------------
    // In the normal SoC path, net_acq_csr already clamps SAMPLES_PER_PACKET to
    // 1..256. The sampler repeats the clamp locally so the module remains safe
    // when it is reused or tested standalone.
    localparam [15:0] C_SAMPLES_PER_PACKET_MIN = 16'd1;
    localparam [15:0] C_SAMPLES_PER_PACKET_MAX = 16'd256;

    function [15:0] f_sanitize_samples_per_packet;
        input [15:0] raw_value;
        begin
            if (raw_value == 16'd0) begin
                f_sanitize_samples_per_packet = C_SAMPLES_PER_PACKET_MIN;
            end else if (raw_value > C_SAMPLES_PER_PACKET_MAX) begin
                f_sanitize_samples_per_packet = C_SAMPLES_PER_PACKET_MAX;
            end else begin
                f_sanitize_samples_per_packet = raw_value;
            end
        end
    endfunction

    // ------------------------------------------------------------------------
    // 当前是否正在一包内部
    // cnt_sample_packet == 0 时表示：当前尚未接纳本包的任何样本
    // ------------------------------------------------------------------------
    wire                packet_idle;
    assign              packet_idle   = (cnt_sample_packet == 16'd0);
    assign              packet_active = !packet_idle;

    // ------------------------------------------------------------------------
    // 若 packet_info_fifo 已满，则禁止开始一个新包
    // 这样可以保证：不会出现 sample_fifo 中已经写入数据，但 packet_info_fifo 中没有对应描述符
    //
    // 这里的设计相当于：
    //   “开始一包之前，先确认描述符 FIFO 至少还有一个空位。”
    // ------------------------------------------------------------------------
    wire                packet_start_block;
    assign              packet_start_block = packet_idle && packet_info_full;

    // ------------------------------------------------------------------------
    // ready 窗口：
    //   cnt_sample_period == 0 表示当前已经满足采样间隔要求，允许接纳新样本
    // ------------------------------------------------------------------------
    wire                sample_accept_window;
    wire                sample_timing_eligible_w;
    wire                sample_overrun_cond_w;
    wire                packet_start_block_cond_w;
    assign              sample_accept_window = (cnt_sample_period == 32'd0);

    // ------------------------------------------------------------------------
    // sample_ready：当前是否真的可以接纳一个新样本
    //
    // 条件：
    //   1) 模块使能
    //   2) 当前已经满足采样间隔要求
    //   3) sample_fifo 未满
    //   4) 若准备开始新包，则 packet_info_fifo 也必须可写
    // ------------------------------------------------------------------------
    assign              sample_ready = enable &&
                                       sample_accept_window &&
                                       !fifo_full &&
                                       !packet_start_block;

    // 只有在“采样时序窗口真正打开”的条件下，才把 FIFO / descriptor 阻塞
    // 视为一次有效的结构性阻塞事件；sample_period 等待期不计入此类事件。
    assign              sample_timing_eligible_w = enable && sample_accept_window && sample_valid_in;
    assign              sample_overrun_cond_w    = sample_timing_eligible_w && fifo_full && !packet_start_block;
    assign              packet_start_block_cond_w= sample_timing_eligible_w && packet_start_block;

    // ------------------------------------------------------------------------
    // sample_accept：当前这个输入样本是否被真正接纳
    // ------------------------------------------------------------------------
    wire                sample_accept;
    assign              sample_accept = sample_ready && sample_valid_in;

    // ------------------------------------------------------------------------
    // 当前接纳样本是否为一包中的第一个样本
    // ------------------------------------------------------------------------
    wire                sample_packet_valid;
    assign              sample_packet_valid = sample_accept && packet_idle;

    // ------------------------------------------------------------------------
    // 当前接纳样本是否为一包中的最后一个样本
    // ------------------------------------------------------------------------
    wire                sample_packet_last;
    assign              sample_packet_last = sample_accept &&
                                             (cnt_sample_packet == (sample_packet_Max - 16'd1));

    // ------------------------------------------------------------------------
    // 配置锁存
    // sample_period_Max:
    //   0 -> 每拍都可接纳
    //   N -> 每接纳1个样本后，至少等待N拍再允许下一次接纳
    //
    // sample_packet_Max:
    //   每包样本数会被限制在 1~256，避免非法包长导致 sample_fifo 被填满后
    //   仍无法生成 descriptor 的结构性阻塞。
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn)
    if (!resetn) begin
        sample_period_Max <= 32'd0;
        sample_packet_Max <= C_SAMPLES_PER_PACKET_MIN;
    end else if (!enable) begin
        sample_period_Max <= sample_period;
        sample_packet_Max <= f_sanitize_samples_per_packet(samples_per_packet);
    end

    // ------------------------------------------------------------------------
    // 抽样间隔计数器
    //
    // 规则：
    //   - 一旦 sample_accept 成功，则 cnt_sample_period 装载为 sample_period_Max
    //   - 之后每个时钟减1，直到减到0
    //   - cnt_sample_period == 0 时，sample_ready 才可能为1
    //   - 若 ready 已经打开但暂时没有 valid，则保持在0，不会自动错过这次接纳窗口
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn)
    if (!resetn)
        cnt_sample_period <= 32'd0;
    else if (!enable)
        cnt_sample_period <= 32'd0;
    else if (sample_accept)
        cnt_sample_period <= sample_period_Max;
    else if (cnt_sample_period != 32'd0)
        cnt_sample_period <= cnt_sample_period - 32'd1;

    // ------------------------------------------------------------------------
    // 包内样本计数器
    // 只在 sample_accept 时推进。
    //
    // cnt_sample_packet 表示：
    //   当前这一包里，已经接纳了多少个样本（从0开始计）
    //
    // 若当前接纳的是最后一个样本，则本拍 packet_done 拉高，并清零计数
    // packet_sample_count 在 packet_done 时保持为本包最终样本数
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn)
    if (!resetn) begin
        cnt_sample_packet   <= 16'd0;
        packet_sample_count <= 16'd0;
    end else if (!enable) begin
        cnt_sample_packet   <= 16'd0;
        packet_sample_count <= 16'd0;
    end else if (sample_accept) begin
        if (sample_packet_last) begin
            cnt_sample_packet   <= 16'd0;
            packet_sample_count <= sample_packet_Max;
        end else begin
            cnt_sample_packet   <= cnt_sample_packet + 16'd1;
            packet_sample_count <= cnt_sample_packet + 16'd1;
        end
    end

    // ------------------------------------------------------------------------
    // 锁存包起始时间戳
    // 在每包的第一个样本真正被接纳时锁存
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn)
    if (!resetn)
        packet_timestamp <= 64'd0;
    else if (!enable)
        packet_timestamp <= 64'd0;
    else if (sample_packet_valid)
        packet_timestamp <= timestamp_now;

    // ------------------------------------------------------------------------
    // 输出控制
    //
    // sample_push  : 写 sample_fifo 脉冲，表示当前样本被真正接纳
    // packet_start : 一包第一个样本到来时拉高1拍
    // packet_done  : 一包最后一个样本到来时拉高1拍
    // sample_out   : 被接纳的样本值
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn)
    if (!resetn) begin
        sample_out               <= 16'd0;
        sample_push              <= 1'b0;
        sample_overrun_pulse     <= 1'b0;
        packet_start_block_pulse <= 1'b0;
        packet_start             <= 1'b0;
        packet_done              <= 1'b0;
    end else if (!enable) begin
        sample_out               <= 16'd0;
        sample_push              <= 1'b0;
        sample_overrun_pulse     <= 1'b0;
        packet_start_block_pulse <= 1'b0;
        packet_start             <= 1'b0;
        packet_done              <= 1'b0;
    end else begin
        // 默认拉低，形成单周期脉冲
        sample_push              <= 1'b0;
        sample_overrun_pulse     <= 1'b0;
        packet_start_block_pulse <= 1'b0;
        packet_start             <= 1'b0;
        packet_done              <= 1'b0;

        if (sample_overrun_cond_w)
            sample_overrun_pulse <= 1'b1;

        if (packet_start_block_cond_w)
            packet_start_block_pulse <= 1'b1;

        if (sample_accept) begin
            sample_out  <= sample_data_in;
            sample_push <= 1'b1;

            if (sample_packet_valid)
                packet_start <= 1'b1;

            if (sample_packet_last)
                packet_done <= 1'b1;
        end
    end

endmodule

`default_nettype wire
