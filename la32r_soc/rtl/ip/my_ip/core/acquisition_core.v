`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/03/29
// Design Name:
// Module Name: acquisition_core
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
//   采集核心模块（当前推荐的统一架构）：
//
//   source_select
//     ├── internal data_gen
//     └── external sample input
//              |
//              v
//   disciplined sys-domain ToD (preferred when valid)
//              |
//              v
//        timestamp source mux
//          |            ^
//          |            |
//          |      timestamp_counter (fallback)
//          +------------+
//              |
//              v
//            sampler  ------------------------------+
//              |                                    |
//              v                                    v
//         sample_fifo                         packet_info_fifo
//              |                                    |
//              +---------------+--------------------+
//                              |
//                              v
//                        packet_formatter
//                              |
//                              v
//                   pkt_data / pkt_valid / pkt_last / pkt_ready
//
//   设计要点：
//   - 样本 payload 与包元信息分开缓存
//   - 时间戳不再重复写入 sample_fifo，而是写入 packet_info_fifo
//   - 预留 cfg_mode / cfg_channel_id / cfg_src_sel，便于后续升级
//   - 输出统一 packet stream，便于未来对接 UART / MAC / UDP 后端
//   - 当前新增 external disciplined timestamp 输入：
//       * ext_timestamp_valid=1 时，优先使用外部 sys 域 ToD 作为包级时间戳
//       * R1: 首次选用外部时间后禁止回退；运行期间无效会锁存故障，要求停机清空。
//     这样可以在不改 0x00~0x44 前端寄存器语义的前提下，把 acquisition packet
//     的时间戳平滑升级到 PTP 校准后的统一时间基。
//
//   本次修订重点：
//   1) 明确 cfg_enable=0 的语义：停止采集并同步清空内部 FIFO / 统计状态
//   2) 新增 cfg_clear_status：仅清 packet_count / last_timestamp / error 等状态，不打断数据通路
//   3) sample_ready_out 不再只是“FIFO 没满”，而是仅在 external source 模式下输出
//      sampler 当前真正可接纳样本的 ready 语义
//   4) stat_busy 不再简单等于 cfg_enable，而是表示“当前包处理链路确实处于忙状态”
//   5) 修复 sampler 与 FIFO 之间 1 拍写入延迟带来的边界时序问题
//   6) 新增外部受校准时间源输入，支持把 acquisition packet 时间戳
//      从本地自由运行 counter 切换到 sys 域 PTP ToD，同时保留旧路径回退
//
// Dependencies:
//
// Revision:
// Revision 0.04 - sample_fifo depth widened from 128 to 256
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module acquisition_core(
        clk_in,
        resetn,

        cfg_enable,
        cfg_clear_status,
        cfg_sample_period,
        cfg_samples_per_packet,
        cfg_mode,
        cfg_channel_id,
        cfg_src_sel,
        cfg_time_quality,

        // 外部受校准时间源（通常由 wrapper 中的 sys 域 PTP ToD 提供）
        // 当 ext_timestamp_valid=1 时，本模块优先使用该时间源；
        // R1: 外部时间首次选中后无效时禁止回退，停止接纳并置故障，要求 cfg_enable=0 清空。
        ext_timestamp_ns,
        ext_timestamp_valid,

        sample_data_in,
        sample_valid_in,
        sample_ready_out,

        pkt_data,
        pkt_valid,
        pkt_last,
        pkt_ready,

        stat_busy,
        stat_error,
        stat_fifo_level,
        stat_packet_count,
        stat_last_timestamp
    );

    input  wire         clk_in;
    input  wire         resetn;

    input  wire         cfg_enable;
    input  wire         cfg_clear_status;     // 1拍脉冲：仅清状态统计/粘滞错误，不清 FIFO / 不打断当前链路
    input  wire [31:0]  cfg_sample_period;
    input  wire [15:0]  cfg_samples_per_packet;
    input  wire [3:0]   cfg_mode;
    input  wire [15:0]  cfg_channel_id;
    input  wire         cfg_src_sel;          // 0: internal data_gen, 1: external sample input
    input  wire [1:0]   cfg_time_quality;     // 0: INVALID, 1: REACQUIRE, 2: LOCKED, 3: HOLDOVER

    // 外部时间源输入：用于把 acquisition packet 的时间戳切到
    // “已经被 PTP 校准过的 sys 域 ToD”。
    //
    // 兼容策略：
    //   - ext_timestamp_valid = 1 -> 使用外部时间源
    //   - ext_timestamp_valid = 0 -> 未曾选择外部时间时兼容 legacy；选择后失效须停机。
    //
    // 这样在未开启 PTP 时，旧主线行为保持不变；
    // 一旦 wrapper 把 sys 域 ToD 标记为有效，采集包头时间戳就自动升级为受校准时间。
    input  wire [63:0]  ext_timestamp_ns;
    input  wire         ext_timestamp_valid;

    input  wire [15:0]  sample_data_in;
    input  wire         sample_valid_in;
    output wire         sample_ready_out;

    output wire [15:0]  pkt_data;
    output wire         pkt_valid;
    output wire         pkt_last;
    input  wire         pkt_ready;

    output wire         stat_busy;
    output wire         stat_error;
    output wire [31:0]  stat_fifo_level;
    output  reg [31:0]  stat_packet_count;
    output  reg [63:0]  stat_last_timestamp;

    // ------------------------------------------------------------------------
    // 内部“停机并清空”控制
    //
    // 语义约定：
    //   cfg_enable       = 0 -> 停止采集，并同步清空内部 FIFO / 核心状态
    //   cfg_clear_status = 1 -> 仅清统计状态 / sticky error，不清 FIFO / 不改 packet_seq_id
    //
    // 这样可以同时满足：
    //   - 关停后 FIFO 不残留旧包
    //   - 软件可在不停机时独立清 packet_count / error 等状态
    // ------------------------------------------------------------------------
    wire core_clear;
    assign core_clear = !cfg_enable;

    // ------------------------------------------------------------------------
    // 运行期配置锁存
    // 建议：enable=0 时修改 mode / channel_id / src_sel
    // ------------------------------------------------------------------------
    reg [3:0]   cfg_mode_r;
    reg [15:0]  cfg_channel_id_r;
    reg         cfg_src_sel_r;

    always @(posedge clk_in or negedge resetn)
    if (!resetn) begin
        cfg_mode_r       <= 4'd0;
        cfg_channel_id_r <= 16'd0;
        cfg_src_sel_r    <= 1'b0;
    end else if (!cfg_enable) begin
        cfg_mode_r       <= cfg_mode;
        cfg_channel_id_r <= cfg_channel_id;
        cfg_src_sel_r    <= cfg_src_sel;
    end

    // ------------------------------------------------------------------------
    // 时间戳 / 数据源 / source select
    // ------------------------------------------------------------------------
    // timestamp_now            : 真正送给 sampler 的包级时间戳时间基
    // timestamp_now_legacy     : 历史自由运行时间戳（保持旧主线兼容）
    // ext_timestamp_valid=1 时 : 使用外部受校准 sys 域 ToD
    // ext_timestamp_valid=0 时 : 外部时间已选中则禁止回退并停止接纳
    wire [63:0] timestamp_now;
    wire [63:0] timestamp_now_legacy;

    wire [15:0] internal_sample_data;
    wire        internal_sample_valid;

    wire [15:0] source_sample_data;
    wire        source_sample_valid;

    assign source_sample_data  = cfg_src_sel_r ? sample_data_in  : internal_sample_data;
    assign source_sample_valid = cfg_src_sel_r ? sample_valid_in : internal_sample_valid;

    // ------------------------------------------------------------------------
    // sampler 输出
    // ------------------------------------------------------------------------
    wire        sampler_sample_ready;
    wire [15:0] sampler_sample_out;
    wire        sampler_sample_push;
    wire        sampler_sample_overrun_pulse;
    wire        sampler_packet_start_block_pulse;
    wire        sampler_packet_start;
    wire        sampler_packet_done;
    wire        sampler_packet_active;
    wire [63:0] sampler_packet_timestamp;
    wire [15:0] sampler_packet_sample_count;

    // ------------------------------------------------------------------------
    // sample_fifo 信号
    // ------------------------------------------------------------------------
    wire [15:0] sample_fifo_dout;
    wire        sample_fifo_full;
    wire        sample_fifo_empty;
    wire [8:0]  sample_fifo_data_count;
    wire        sample_fifo_rd_en;

    // ------------------------------------------------------------------------
    // packet_info_fifo 信号
    // ------------------------------------------------------------------------
    reg  [31:0]  packet_seq_id;
    reg  [1:0]   cfg_time_quality_d;
    reg  [1:0]   packet_time_quality_r;
    wire [1:0]   packet_time_quality_w;
    wire [129:0] packet_info_fifo_din;
    wire         packet_info_fifo_wr_en;
    wire [129:0] packet_info_fifo_dout;
    wire         packet_info_fifo_full;
    wire         packet_info_fifo_empty;
    wire [4:0]   packet_info_fifo_data_count;
    wire         packet_info_fifo_rd_en;

    // ------------------------------------------------------------------------
    // FIFO 边界保护信号
    //
    // 说明：
    //   sampler 的 sample_push / packet_done 是寄存后输出的，真正写入 FIFO 发生在下一拍。
    //   因此 sampler 在本拍决定是否还能继续接纳时，不能只看“当前 fifo_full”，
    //   还必须把“本拍即将发生的 FIFO 读写”一起考虑进去。
    //
    //   这里计算的是“当前这个时钟沿执行完 FIFO 读写后的占用水位”，
    //   并把它折算成给 sampler 使用的保护性 full 信号：
    //     - sample_fifo_guard_full      : 当前沿完成后，sample_fifo 已无空位可为下一次 sample_accept 预留
    //     - packet_info_fifo_guard_full : 当前沿完成后，descriptor FIFO 已无空位可为“新起一包”预留
    //
    //   这样可以最小修改地修复：
    //     - sample_fifo 快满时多接纳 1 个样本
    //     - packet_info_fifo 快满时多启动 1 个包
    // ------------------------------------------------------------------------
    wire        sample_fifo_do_write_curr;
    wire        sample_fifo_do_read_curr;
    wire [9:0]  sample_fifo_level_after_curr;
    wire        sample_fifo_guard_full;

    wire        packet_info_fifo_do_write_curr;
    wire        packet_info_fifo_do_read_curr;
    wire [5:0]  packet_info_fifo_level_after_curr;
    wire        packet_info_fifo_guard_full;

    // ------------------------------------------------------------------------
    // packet_formatter 信号
    // ------------------------------------------------------------------------
    wire        formatter_busy;
    wire        formatter_error_flag;
    wire        formatter_packet_out_done;
    wire [63:0] formatter_packet_out_timestamp;

    // ------------------------------------------------------------------------
    // 状态统计 / 错误标志
    // ------------------------------------------------------------------------
    reg         stat_error_r;
    wire        source_sample_overrun_err;
    wire        new_packet_desc_block_err;
    wire        packet_info_write_fail_err;

    wire timestamp_accept_valid_w;
    wire timestamp_fault_w;
    wire timestamp_external_selected_w;
    assign stat_error = stat_error_r | timestamp_fault_w;

    // stat_busy 的语义定义为：
    //   - sampler 正处于一包中间，或
    //   - formatter 正在吐包，或
    //   - sample_fifo / packet_info_fifo 里仍有待处理数据
    //
    // 这样 stat_busy 就不再仅仅等价于“已使能”，更利于软件读取时判断模块是否仍有在途数据。
    assign stat_busy  = sampler_packet_active |
                        formatter_busy |
                        (sample_fifo_data_count != 9'd0) |
                        (packet_info_fifo_data_count != 5'd0);

    // stat_fifo_level 打包：
    // [31:16] packet_info_fifo level（16bit显示，高11位补0，低5位有效）
    // [15:0]  sample_fifo level（16bit显示，高7位补0，低9位有效）
    //
    // 说明：
    //   sample_fifo 已从 128 拓宽到 256，因此 sample_fifo level 需要 9bit 表示。
    //   packet_info_fifo 当前深度为 16，因此其 level 当前只需 5bit 表示。
    assign stat_fifo_level = {11'd0, packet_info_fifo_data_count, 7'd0, sample_fifo_data_count};

    // sample_ready_out 仅在 external source 模式下有意义：
    //   1) cfg_enable=1
    //   2) 当前选择 external source
    //   3) sampler 当前确实能够接纳一个样本（真实 ready 语义）
    //
    // internal source 模式下该输出拉低，避免软件误把它当成内部数据源状态。
    assign sample_ready_out = cfg_enable && cfg_src_sel_r && sampler_sample_ready && timestamp_accept_valid_w;

    // 包描述符写入：
    // 当前在 sampler 产生 packet_done 后写入一条 descriptor
    // descriptor 格式：
    //   [129:128] time_quality (captured at packet_start)
    //   [127:64] timestamp
    //   [63:48]  sample_count
    //   [47:32]  channel_id
    //   [31:0]   seq_id
    // sampler_packet_start is a registered indication of the first accepted
    // sample, so retain the quality from the preceding (acceptance) cycle.
    // packet_start and packet_done may coincide for a single-sample packet, so
    // the descriptor write data also bypasses the packet register on that cycle.
    always @(posedge clk_in or negedge resetn)
    if (!resetn) begin
        cfg_time_quality_d     <= 2'd0;
        packet_time_quality_r <= 2'd0;
    end else if (core_clear) begin
        cfg_time_quality_d     <= cfg_time_quality;
        packet_time_quality_r <= 2'd0;
    end else begin
        cfg_time_quality_d <= cfg_time_quality;
        if (sampler_packet_start)
            packet_time_quality_r <= cfg_time_quality_d;
    end

    assign packet_time_quality_w = sampler_packet_start ? cfg_time_quality_d
                                                        : packet_time_quality_r;
    assign packet_info_fifo_wr_en = sampler_packet_done;
    assign packet_info_fifo_din   = {packet_time_quality_w,
                                     sampler_packet_timestamp,
                                     sampler_packet_sample_count,
                                     cfg_channel_id_r,
                                     packet_seq_id};

    assign sample_fifo_do_read_curr         = sample_fifo_rd_en      && !sample_fifo_empty;
    // Keep the guard model consistent with sample_fifo: full + real read + write is allowed and keeps level full.
    assign sample_fifo_do_write_curr        = sampler_sample_push    && (!sample_fifo_full || sample_fifo_do_read_curr);
    assign sample_fifo_level_after_curr     = {1'b0, sample_fifo_data_count} +
                                              (sample_fifo_do_write_curr ? 10'd1 : 10'd0) -
                                              (sample_fifo_do_read_curr  ? 10'd1 : 10'd0);
    assign sample_fifo_guard_full           = (sample_fifo_level_after_curr >= 10'd256);

    assign packet_info_fifo_do_read_curr    = packet_info_fifo_rd_en && !packet_info_fifo_empty;
    // Keep the guard/error model consistent with packet_info_fifo full+read+write semantics.
    assign packet_info_fifo_do_write_curr   = packet_info_fifo_wr_en && (!packet_info_fifo_full || packet_info_fifo_do_read_curr);
    assign packet_info_fifo_level_after_curr= {1'b0, packet_info_fifo_data_count} +
                                              (packet_info_fifo_do_write_curr ? 6'd1 : 6'd0) -
                                              (packet_info_fifo_do_read_curr  ? 6'd1 : 6'd0);
    assign packet_info_fifo_guard_full      = (packet_info_fifo_level_after_curr >= 6'd16);

    // ------------------------------------------------------------------------
    // sticky error 分类
    //
    // 说明：
    //   1) 内部源模式下没有 ready/valid 回压能力；sampler 只在“采样时序窗口真正打开”且
    //      payload FIFO 无安全空位时，才输出 sample_overrun_pulse。
    //   2) 内部源模式下，sampler 只在“准备新起一包”且 descriptor FIFO 无安全空位时，
    //      才输出 packet_start_block_pulse。
    //   3) descriptor 写入失败属于结构保护失配，理论上不应发生；一旦发生必须置错。
    //
    //   注意：external source 模式下，上游在 ready=0 时保持 valid=1 是正常 ready/valid 语义，
    //   因此这里不再把该场景直接判成错误。
    // ------------------------------------------------------------------------
    assign source_sample_overrun_err  = (!cfg_src_sel_r) && sampler_sample_overrun_pulse;
    assign new_packet_desc_block_err  = (!cfg_src_sel_r) && sampler_packet_start_block_pulse;
    assign packet_info_write_fail_err = packet_info_fifo_wr_en && !packet_info_fifo_do_write_curr;

    // ------------------------------------------------------------------------
    // R1 时间源保护：外部 PTP 时间失效时不再切回不同单位的 legacy ticks。
    // 新接纳被阻止并置 sticky fault；软件必须 cfg_enable=0 停机清空后重启。
    // 在途数据仍可排出，故障后的部分包不得当作连续、有效采样数据。
    // ------------------------------------------------------------------------
    timestamp_source_guard u_timestamp_source_guard (
        .clk(clk_in), .resetn(resetn), .cfg_enable(cfg_enable),
        .legacy_time(timestamp_now_legacy), .ext_time_ns(ext_timestamp_ns),
        .ext_time_valid(ext_timestamp_valid), .selected_time(timestamp_now),
        .accept_time_valid(timestamp_accept_valid_w),
        .time_fault(timestamp_fault_w), .external_selected(timestamp_external_selected_w)
    );

    // ------------------------------------------------------------------------
    // 例化：timestamp_counter（legacy fallback time base）
    // ------------------------------------------------------------------------
    timestamp_counter u_timestamp_counter (
        .clk_in        (clk_in),
        .resetn        (resetn),
        .timestamp_now (timestamp_now_legacy)
    );

    // ------------------------------------------------------------------------
    // 例化：内部数据源 data_gen
    // ------------------------------------------------------------------------
    data_gen u_data_gen (
        .clk_in     (clk_in),
        .resetn     (resetn),
        .mode       (cfg_mode_r),
        .enable     (cfg_enable && !cfg_src_sel_r),
        .wave_out   (internal_sample_data),
        .wave_valid (internal_sample_valid)
    );

    // ------------------------------------------------------------------------
    // 例化：sampler
    // ------------------------------------------------------------------------
    sampler u_sampler (
        .clk_in              (clk_in),
        .resetn              (resetn),
        .enable              (cfg_enable),

        .sample_period       (cfg_sample_period),
        .samples_per_packet  (cfg_samples_per_packet),

        .timestamp_now       (timestamp_now),
        .sample_data_in      (source_sample_data),
        .sample_valid_in     (source_sample_valid && timestamp_accept_valid_w),
        .fifo_full           (sample_fifo_guard_full),      // 保护性 full：已计入本拍 FIFO 读写后的剩余空间
        .packet_info_full    (packet_info_fifo_guard_full), // 保护性 full：保证“新起一包”始终预留 1 个 descriptor 槽位

        .sample_ready        (sampler_sample_ready),
        .sample_out          (sampler_sample_out),
        .sample_push         (sampler_sample_push),
        .sample_overrun_pulse(sampler_sample_overrun_pulse),
        .packet_start_block_pulse(sampler_packet_start_block_pulse),
        .packet_start        (sampler_packet_start),
        .packet_done         (sampler_packet_done),
        .packet_active       (sampler_packet_active),
        .packet_timestamp    (sampler_packet_timestamp),
        .packet_sample_count (sampler_packet_sample_count)
    );

    // ------------------------------------------------------------------------
    // 例化：sample_fifo
    // ------------------------------------------------------------------------
    sample_fifo u_sample_fifo (
        .clk_in          (clk_in),
        .resetn          (resetn),
        .clear           (core_clear),
        .fifo_din        (sampler_sample_out),
        .fifo_wr_en      (sampler_sample_push),
        .fifo_rd_en      (sample_fifo_rd_en),
        .fifo_dout       (sample_fifo_dout),
        .fifo_full       (sample_fifo_full),
        .fifo_empty      (sample_fifo_empty),
        .fifo_data_count (sample_fifo_data_count)
    );

    // ------------------------------------------------------------------------
    // 例化：packet_info_fifo
    // ------------------------------------------------------------------------
    packet_info_fifo u_packet_info_fifo (
        .clk_in          (clk_in),
        .resetn          (resetn),
        .clear           (core_clear),
        .fifo_din        (packet_info_fifo_din),
        .fifo_wr_en      (packet_info_fifo_wr_en),
        .fifo_rd_en      (packet_info_fifo_rd_en),
        .fifo_dout       (packet_info_fifo_dout),
        .fifo_full       (packet_info_fifo_full),
        .fifo_empty      (packet_info_fifo_empty),
        .fifo_data_count (packet_info_fifo_data_count)
    );

    // ------------------------------------------------------------------------
    // 例化：packet_formatter
    // ------------------------------------------------------------------------
    packet_formatter u_packet_formatter (
        .clk_in               (clk_in),
        .resetn               (resetn),
        .enable               (cfg_enable),
        .clear_status         (cfg_clear_status),

        .sample_fifo_rd_en    (sample_fifo_rd_en),
        .sample_fifo_rd_data  (sample_fifo_dout),
        .sample_fifo_empty    (sample_fifo_empty),

        .packet_info_rd_en    (packet_info_fifo_rd_en),
        .packet_info_rd_data  (packet_info_fifo_dout),
        .packet_info_empty    (packet_info_fifo_empty),

        .pkt_data             (pkt_data),
        .pkt_valid            (pkt_valid),
        .pkt_last             (pkt_last),
        .pkt_ready            (pkt_ready),

        .busy                 (formatter_busy),
        .error_flag           (formatter_error_flag),
        .packet_out_done      (formatter_packet_out_done),
        .packet_out_timestamp (formatter_packet_out_timestamp)
    );

    // ------------------------------------------------------------------------
    // 包序号 / 状态统计
    //
    // 说明：
    //   - cfg_enable=0 时，视为“停机并清空内部状态”
    //   - 这样与 FIFO 的 clear 语义保持一致
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn)
    if (!resetn) begin
        packet_seq_id        <= 32'd0;
        stat_packet_count    <= 32'd0;
        stat_last_timestamp  <= 64'd0;
        stat_error_r         <= 1'b0;
    end else if (!cfg_enable) begin
        packet_seq_id        <= 32'd0;
        stat_packet_count    <= 32'd0;
        stat_last_timestamp  <= 64'd0;
        stat_error_r         <= 1'b0;
    end else begin
        // packet_seq_id 属于数据通路上下文，不受 clear_status 影响。
        // 只有 descriptor 真正写入 FIFO 成功时才递增，避免极端情况下出现序号空洞。
        if (packet_info_fifo_do_write_curr)
            packet_seq_id <= packet_seq_id + 32'd1;

        // cfg_clear_status 仅清状态统计/粘滞错误，不打断当前数据通路。
        if (cfg_clear_status) begin
            stat_packet_count   <= 32'd0;
            stat_last_timestamp <= 64'd0;
            stat_error_r        <= 1'b0;
        end else begin
            // 每完成输出一个 packet，统计一次 packet_count / last_timestamp
            if (formatter_packet_out_done) begin
                stat_packet_count   <= stat_packet_count + 32'd1;
                stat_last_timestamp <= formatter_packet_out_timestamp;
            end

            // 错误标志：
            // 1) 内部源模式下 payload FIFO 满导致样本 overrun
            // 2) 内部源模式下 descriptor FIFO 满导致新包无法建立
            // 3) 理论上不应发生的 descriptor 写入失败（保护性检查）
            // 4) packet_formatter 发现 descriptor / payload 不一致
            if (source_sample_overrun_err ||
                new_packet_desc_block_err ||
                packet_info_write_fail_err ||
                formatter_error_flag)
                stat_error_r <= 1'b1;
        end
    end

endmodule

`default_nettype wire
