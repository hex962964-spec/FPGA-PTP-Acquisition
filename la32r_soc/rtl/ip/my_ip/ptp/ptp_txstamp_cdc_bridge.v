`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: ptp_txstamp_cdc_bridge
// -----------------------------------------------------------------------------
// 作用：
//   把 eth/mac 发送域产生的“单次 TX 时间戳事件”可靠搬到 sys/CSR 域。
//
// 设计目标：
//   1) 支持“单 outstanding event”模型；
//   2) 多位时间戳总线跨域不撕裂；
//   3) 目标域在软件/CSR 真正取走前，valid 一直保持；
//   4) 为以后扩展 tag（例如 sequenceId / event type）留接口。
//
// 使用方式：
//   - 源域：当 src_ts_valid && src_ts_ready 时，提交一笔时间戳事件。
//   - 目标域：看到 dst_ts_valid=1 后，可通过 dst_ts_pop 拉高 1 拍完成“取走”。
//   - 在 dst_ts_pop 之前，src_ts_ready 会保持为 0，从而天然限制为单 outstanding。
//
// CDC / timing 收口说明：
//   - req/ack toggle 的同步链均显式拆成 sync1/sync2；
//   - 为同步链添加 ASYNC_REG / SHREG_EXTRACT 属性；
//   - 保持原有 single-outstanding 与 destructive-pop 语义不变。
//////////////////////////////////////////////////////////////////////////////////
module ptp_txstamp_cdc_bridge #(
    parameter integer P_TS_W  = 64,
    parameter integer P_TAG_W = 16
)(
    // ------------------------------------------------------------------------
    // 源域（通常是 eth_tx_clk / top_phy 域）
    // ------------------------------------------------------------------------
    input  wire                 src_clk,
    input  wire                 src_resetn,
    input  wire                 src_clear_status,

    input  wire                 src_ts_valid,
    input  wire [P_TS_W-1:0]    src_ts_ns,
    input  wire [P_TAG_W-1:0]   src_ts_tag,
    output wire                 src_ts_ready,
    output reg                  src_overflow_sticky,

    // ------------------------------------------------------------------------
    // 目标域（通常是 sys_clk / CSR 域）
    // ------------------------------------------------------------------------
    input  wire                 dst_clk,
    input  wire                 dst_resetn,
    input  wire                 dst_clear_status,

    output reg                  dst_ts_valid,
    output reg  [P_TS_W-1:0]    dst_ts_ns,
    output reg  [P_TAG_W-1:0]   dst_ts_tag,
    output reg                  dst_ts_pulse,
    input  wire                 dst_ts_pop,
    output reg                  dst_overflow_sticky
);

    // ------------------------------------------------------------------------
    // 源域 mailbox：保持稳定直到目标域 pop 并 ACK
    // ------------------------------------------------------------------------
    reg [P_TS_W-1:0]  src_ts_ns_hold_r;
    reg [P_TAG_W-1:0] src_ts_tag_hold_r;
    reg               src_req_toggle_r;
    reg               src_pending_r;

    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg dst_ack_toggle_sync1_src_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg dst_ack_toggle_sync2_src_r;
    reg                                                      dst_ack_toggle_seen_src_r;

    // ------------------------------------------------------------------------
    // 目标域：检测 req toggle，锁存多位总线，再由 pop 触发 ACK
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg src_req_toggle_sync1_dst_r;
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg src_req_toggle_sync2_dst_r;
    reg                                                      src_req_toggle_seen_dst_r;
    reg                                                      dst_ack_toggle_r;

    assign src_ts_ready = !src_pending_r;

    // ------------------------------------------------------------------------
    // 源域逻辑
    // ------------------------------------------------------------------------
    always @(posedge src_clk or negedge src_resetn) begin
        if (!src_resetn) begin
            src_ts_ns_hold_r          <= {P_TS_W{1'b0}};
            src_ts_tag_hold_r         <= {P_TAG_W{1'b0}};
            src_req_toggle_r          <= 1'b0;
            src_pending_r             <= 1'b0;
            dst_ack_toggle_sync1_src_r<= 1'b0;
            dst_ack_toggle_sync2_src_r<= 1'b0;
            dst_ack_toggle_seen_src_r <= 1'b0;
            src_overflow_sticky       <= 1'b0;
        end else begin
            dst_ack_toggle_sync1_src_r <= dst_ack_toggle_r;
            dst_ack_toggle_sync2_src_r <= dst_ack_toggle_sync1_src_r;

            if (src_clear_status)
                src_overflow_sticky <= 1'b0;

            // 先接收目标域 ACK
            if (dst_ack_toggle_sync2_src_r != dst_ack_toggle_seen_src_r) begin
                dst_ack_toggle_seen_src_r <= dst_ack_toggle_sync2_src_r;
                src_pending_r             <= 1'b0;
            end

            // 接收新时间戳事件
            if (src_ts_valid) begin
                if (!src_pending_r) begin
                    src_ts_ns_hold_r <= src_ts_ns;
                    src_ts_tag_hold_r<= src_ts_tag;
                    src_req_toggle_r <= ~src_req_toggle_r;
                    src_pending_r    <= 1'b1;
                end else begin
                    // 当前设计是 single outstanding，新的时间戳先记 overflow。
                    src_overflow_sticky <= 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // 目标域逻辑
    // ------------------------------------------------------------------------
    always @(posedge dst_clk or negedge dst_resetn) begin
        if (!dst_resetn) begin
            src_req_toggle_sync1_dst_r <= 1'b0;
            src_req_toggle_sync2_dst_r <= 1'b0;
            src_req_toggle_seen_dst_r  <= 1'b0;
            dst_ack_toggle_r           <= 1'b0;
            dst_ts_valid               <= 1'b0;
            dst_ts_ns                  <= {P_TS_W{1'b0}};
            dst_ts_tag                 <= {P_TAG_W{1'b0}};
            dst_ts_pulse               <= 1'b0;
            dst_overflow_sticky        <= 1'b0;
        end else begin
            src_req_toggle_sync1_dst_r <= src_req_toggle_r;
            src_req_toggle_sync2_dst_r <= src_req_toggle_sync1_dst_r;
            dst_ts_pulse               <= 1'b0;

            if (dst_clear_status)
                dst_overflow_sticky <= 1'b0;

            // 新时间戳到达
            if (src_req_toggle_sync2_dst_r != src_req_toggle_seen_dst_r) begin
                src_req_toggle_seen_dst_r <= src_req_toggle_sync2_dst_r;

                if (dst_ts_valid)
                    dst_overflow_sticky <= 1'b1;

                // 源域在 ACK 前保持 hold 总线稳定，因此这里可以一次性采样。
                dst_ts_ns    <= src_ts_ns_hold_r;
                dst_ts_tag   <= src_ts_tag_hold_r;
                dst_ts_valid <= 1'b1;
                dst_ts_pulse <= 1'b1;
            end

            // 软件/CSR 取走当前时间戳后，回 ACK 释放源域 mailbox。
            if (dst_ts_pop && dst_ts_valid) begin
                dst_ts_valid     <= 1'b0;
                dst_ack_toggle_r <= ~dst_ack_toggle_r;
            end
        end
    end

endmodule
