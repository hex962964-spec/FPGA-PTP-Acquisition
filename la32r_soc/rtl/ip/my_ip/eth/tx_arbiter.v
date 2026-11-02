`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: tx_arbiter
// -----------------------------------------------------------------------------
// 作用：
//   在“普通采集数据发送链”和“PTP 发送链”之间做按帧仲裁，最终输出到 top_phy。
//
// 设计原则：
//   1) 只在帧边界做仲裁，不做帧内抢占；
//   2) 一旦某个输入拿到 grant，必须持续到该帧 tlast 握手完成；
//   3) 优先级策略拆分为两类：
//        - cfg_ptp_strict_priority = 1：所有 PTP 帧都严格优先于普通数据帧；
//        - cfg_ptp_event_priority_enable = 1：仅 PTP event 帧在竞争时提升优先级；
//          PTP general 帧仍按 round-robin 与普通数据帧竞争。
//   4) tuser 原样透传，便于后级 top_phy 继续使用 SOF / need_timestamp / frame_type。
//////////////////////////////////////////////////////////////////////////////////
module tx_arbiter #(
    parameter integer P_DATA_W  = 8,
    parameter integer P_TUSER_W = 4
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ------------------------------------------------------------------------
    // 全局控制
    // ------------------------------------------------------------------------
    input  wire                     cfg_enable,
    input  wire                     cfg_soft_reset,
    input  wire                     cfg_clear_status,
    input  wire                     cfg_ptp_strict_priority,
    input  wire                     cfg_ptp_event_priority_enable,

    // ------------------------------------------------------------------------
    // 输入 0：普通数据帧（data UDP path）
    // ------------------------------------------------------------------------
    input  wire [P_DATA_W-1:0]      s_data_tdata,
    input  wire                     s_data_tvalid,
    input  wire                     s_data_tlast,
    output wire                     s_data_tready,
    input  wire [P_TUSER_W-1:0]     s_data_tuser,

    // ------------------------------------------------------------------------
    // 输入 1：PTP 帧（ptp UDP path）
    // ------------------------------------------------------------------------
    input  wire [P_DATA_W-1:0]      s_ptp_tdata,
    input  wire                     s_ptp_tvalid,
    input  wire                     s_ptp_tlast,
    output wire                     s_ptp_tready,
    input  wire [P_TUSER_W-1:0]     s_ptp_tuser,

    // ------------------------------------------------------------------------
    // 仲裁后输出
    // ------------------------------------------------------------------------
    output wire [P_DATA_W-1:0]      m_tdata,
    output wire                     m_tvalid,
    output wire                     m_tlast,
    input  wire                     m_tready,
    output wire [P_TUSER_W-1:0]     m_tuser,

    // ------------------------------------------------------------------------
    // 状态 / 统计
    // ------------------------------------------------------------------------
    output wire                     busy,
    output reg  [31:0]              stat_data_frame_count,
    output reg  [31:0]              stat_ptp_frame_count,
    output reg  [31:0]              stat_stall_cycles,
    output reg  [15:0]              stat_last_error
);

    localparam [1:0] GRANT_NONE = 2'd0;
    localparam [1:0] GRANT_DATA = 2'd1;
    localparam [1:0] GRANT_PTP  = 2'd2;

    localparam [15:0] ERR_NONE          = 16'h0000;
    localparam [15:0] ERR_SOURCE_BUBBLE = 16'h0001;

    localparam [1:0] FRAME_TYPE_DATA        = 2'b00;
    localparam [1:0] FRAME_TYPE_PTP_EVENT   = 2'b01;
    localparam [1:0] FRAME_TYPE_PTP_GENERAL = 2'b10;

    reg [1:0] grant_r;
    reg       last_grant_was_ptp_r;

    reg [1:0] arb_choice_c;
    reg [1:0] active_grant_c;
    wire      m_fire;
    wire [1:0] ptp_frame_type_w;
    wire      ptp_is_event_w;
    wire      ptp_event_priority_hit_w;

    assign ptp_frame_type_w          = s_ptp_tuser[3:2];
    assign ptp_is_event_w            = (ptp_frame_type_w == FRAME_TYPE_PTP_EVENT);
    assign ptp_event_priority_hit_w  = cfg_ptp_event_priority_enable &&
                                       s_ptp_tvalid &&
                                       ptp_is_event_w;

    always @(*) begin
        if (!cfg_enable) begin
            arb_choice_c = GRANT_NONE;
        end else if (s_data_tvalid && !s_ptp_tvalid) begin
            arb_choice_c = GRANT_DATA;
        end else if (!s_data_tvalid && s_ptp_tvalid) begin
            arb_choice_c = GRANT_PTP;
        end else if (s_data_tvalid && s_ptp_tvalid) begin
            if (cfg_ptp_strict_priority) begin
                arb_choice_c = GRANT_PTP;
            end else if (ptp_event_priority_hit_w) begin
                arb_choice_c = GRANT_PTP;
            end else begin
                arb_choice_c = last_grant_was_ptp_r ? GRANT_DATA : GRANT_PTP;
            end
        end else begin
            arb_choice_c = GRANT_NONE;
        end
    end

    always @(*) begin
        if (!cfg_enable) begin
            active_grant_c = GRANT_NONE;
        end else if (grant_r != GRANT_NONE) begin
            active_grant_c = grant_r;
        end else begin
            active_grant_c = arb_choice_c;
        end
    end

    assign busy = (active_grant_c != GRANT_NONE);

    assign m_tvalid = (active_grant_c == GRANT_DATA) ? (cfg_enable ? s_data_tvalid : 1'b0) :
                      (active_grant_c == GRANT_PTP ) ? (cfg_enable ? s_ptp_tvalid  : 1'b0) :
                                                        1'b0;

    assign m_tdata  = (active_grant_c == GRANT_DATA) ? s_data_tdata :
                      (active_grant_c == GRANT_PTP ) ? s_ptp_tdata  :
                                                        {P_DATA_W{1'b0}};

    assign m_tlast  = (active_grant_c == GRANT_DATA) ? s_data_tlast :
                      (active_grant_c == GRANT_PTP ) ? s_ptp_tlast  :
                                                        1'b0;

    assign m_tuser  = (active_grant_c == GRANT_DATA) ? s_data_tuser :
                      (active_grant_c == GRANT_PTP ) ? s_ptp_tuser  :
                                                        {P_TUSER_W{1'b0}};

    assign s_data_tready = (active_grant_c == GRANT_DATA) ? (cfg_enable && m_tready) : 1'b0;
    assign s_ptp_tready  = (active_grant_c == GRANT_PTP ) ? (cfg_enable && m_tready) : 1'b0;

    assign m_fire = m_tvalid && m_tready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            grant_r               <= GRANT_NONE;
            last_grant_was_ptp_r  <= 1'b0;
            stat_data_frame_count <= 32'd0;
            stat_ptp_frame_count  <= 32'd0;
            stat_stall_cycles     <= 32'd0;
            stat_last_error       <= ERR_NONE;
        end else begin
            if (cfg_soft_reset || !cfg_enable) begin
                grant_r              <= GRANT_NONE;
                last_grant_was_ptp_r <= 1'b0;
            end else begin
                if (cfg_clear_status) begin
                    stat_data_frame_count <= 32'd0;
                    stat_ptp_frame_count  <= 32'd0;
                    stat_stall_cycles     <= 32'd0;
                    stat_last_error       <= ERR_NONE;
                end

                if (m_tvalid && !m_tready)
                    stat_stall_cycles <= stat_stall_cycles + 32'd1;

                // 已经锁定某一路，但源中途断 valid，记一次错误，方便 bring-up 定位
                if ((grant_r == GRANT_DATA) && cfg_enable && !s_data_tvalid)
                    stat_last_error <= ERR_SOURCE_BUBBLE;
                else if ((grant_r == GRANT_PTP) && cfg_enable && !s_ptp_tvalid)
                    stat_last_error <= ERR_SOURCE_BUBBLE;

                if (grant_r == GRANT_NONE) begin
                    if (m_fire) begin
                        if (m_tlast) begin
                            if (active_grant_c == GRANT_DATA) begin
                                stat_data_frame_count <= stat_data_frame_count + 32'd1;
                                last_grant_was_ptp_r  <= 1'b0;
                            end else if (active_grant_c == GRANT_PTP) begin
                                stat_ptp_frame_count <= stat_ptp_frame_count + 32'd1;
                                last_grant_was_ptp_r <= 1'b1;
                            end
                        end else begin
                            grant_r <= active_grant_c;
                        end
                    end
                end else begin
                    if (m_fire && m_tlast) begin
                        if (grant_r == GRANT_DATA) begin
                            stat_data_frame_count <= stat_data_frame_count + 32'd1;
                            last_grant_was_ptp_r  <= 1'b0;
                        end else if (grant_r == GRANT_PTP) begin
                            stat_ptp_frame_count <= stat_ptp_frame_count + 32'd1;
                            last_grant_was_ptp_r <= 1'b1;
                        end
                        grant_r <= GRANT_NONE;
                    end
                end
            end
        end
    end

endmodule
