`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: ptp_rx_path
// Description:
//   PTP 最小接收路径。
//
//   该模块把原 net_acq_ptp_subsystem 中的 RX 相关结构收成一层：
//     MAC-side RX stream
//         -> ptp_rx_filter      (eth_rx_clk)
//         -> ptp_rx_async_fifo  (eth_rx_clk -> sys_clk)
//         -> ptp_rx_mailbox     (sys_clk)
//
//   这样做的目标是把“接收解析 / 跨域缓存 / CPU 邮箱”三段明确固定在 RX 子系统里，
//   后续不管是扩展统计还是增强匹配规则，都不需要再碰 PTP 顶层壳。
//////////////////////////////////////////////////////////////////////////////////
module ptp_rx_path(
    input  wire         clk_in,
    input  wire         resetn,
    input  wire         eth_rx_clk,
    input  wire         eth_rx_resetn,

    // ------------------------------------------------------------------------
    // RX match configuration already committed into eth_rx domain
    // ------------------------------------------------------------------------
    input  wire         rx_match_ptp_enable_eth,
    input  wire [7:0]   rx_match_ptp_domain_eth,
    input  wire [31:0]  rx_match_host_ip_eth,
    input  wire [31:0]  rx_match_local_ip_eth,
    input  wire [47:0]  rx_match_host_mac_eth,
    input  wire [47:0]  rx_match_local_mac_eth,

    // ------------------------------------------------------------------------
    // Reset / clear pulses in eth_rx and sys domains
    // ------------------------------------------------------------------------
    input  wire         eth_rx_soft_reset_pulse_eth,
    input  wire         eth_rx_clear_status_pulse_eth,
    input  wire         cfg_soft_reset_pulse,
    input  wire         cfg_clear_status_pulse,

    // ------------------------------------------------------------------------
    // MAC-side RX stream + RX timestamp sideband
    // ------------------------------------------------------------------------
    input  wire [7:0]   s_axis_eth_rx_tdata,
    input  wire         s_axis_eth_rx_tvalid,
    input  wire         s_axis_eth_rx_tlast,
    input  wire [63:0]  ptp_rxstamp_ns,
    input  wire         ptp_rxstamp_valid,

    // ------------------------------------------------------------------------
    // sys-domain mailbox visible to CSR / CPU
    // ------------------------------------------------------------------------
    input  wire         ptp_rx_mbox_pop_pulse,
    output wire         ptp_rx_mbox_valid,
    output wire [3:0]   ptp_rx_mbox_msg_type,
    output wire [15:0]  ptp_rx_mbox_sequence_id,
    output wire [63:0]  ptp_rx_mbox_rx_timestamp_ns,
    output wire [79:0]  ptp_rx_mbox_msg_timestamp_80,
    output wire [79:0]  ptp_rx_mbox_requesting_port_identity,
    output wire         ptp_rx_mbox_overflow_sticky,

    // ------------------------------------------------------------------------
    // sys-domain observability counters for pre-scheduler debug
    // ------------------------------------------------------------------------
    output wire [31:0]  stat_rx_accept_count_sys,
    output wire [31:0]  stat_rx_drop_count_sys,
    output wire [15:0]  stat_rx_last_error_sys,
    output wire         stat_rx_fifo_overflow_sticky_sys);

    localparam integer C_PTP_RX_REC_W = 256;

    wire                    ptp_rx_rec_valid_w;
    wire                    ptp_rx_rec_ready_w;
    wire [C_PTP_RX_REC_W-1:0] ptp_rx_rec_data_w;
    wire                    ptp_rx_fifo_rec_valid_w;
    wire                    ptp_rx_fifo_rec_ready_w;
    wire [C_PTP_RX_REC_W-1:0] ptp_rx_fifo_rec_data_w;
    wire                    ptp_rx_fifo_wr_full_w;
    wire                    ptp_rx_fifo_rd_empty_w;
    wire                    ptp_rx_fifo_overflow_sticky_w;
    // RX-domain status snapshot must use the FIFO write-domain sticky bit.
    // Keep the already synchronized read-domain flag for the sys mailbox.
    wire                    ptp_rx_fifo_wr_overflow_sticky_w;
    wire [31:0]             ptp_rx_filter_accept_count_w;
    wire [31:0]             ptp_rx_filter_drop_count_w;
    wire [15:0]             ptp_rx_filter_last_error_w;
    wire                    ptp_rx_mbox_new_pulse_w;
    wire                    unused_keep_w;

    ptp_rx_filter u_ptp_rx_filter (
        .clk                     (eth_rx_clk),
        .rst_n                   (eth_rx_resetn),
        .cfg_enable              (rx_match_ptp_enable_eth),
        .cfg_soft_reset          (eth_rx_soft_reset_pulse_eth),
        .cfg_clear_status        (eth_rx_clear_status_pulse_eth),
        .cfg_local_mac           (rx_match_local_mac_eth),
        .cfg_host_mac            (rx_match_host_mac_eth),
        .cfg_local_ip            (rx_match_local_ip_eth),
        .cfg_host_ip             (rx_match_host_ip_eth),
        .cfg_ptp_domain          (rx_match_ptp_domain_eth),
        .s_axis_tdata            (s_axis_eth_rx_tdata),
        .s_axis_tvalid           (s_axis_eth_rx_tvalid),
        .s_axis_tlast            (s_axis_eth_rx_tlast),
        .ptp_frame_rxstamp_ns    (ptp_rxstamp_ns),
        .ptp_frame_rxstamp_valid (ptp_rxstamp_valid),
        .rec_valid               (ptp_rx_rec_valid_w),
        .rec_ready               (ptp_rx_rec_ready_w),
        .rec_data                (ptp_rx_rec_data_w),
        .stat_accept_count       (ptp_rx_filter_accept_count_w),
        .stat_drop_count         (ptp_rx_filter_drop_count_w),
        .stat_last_error         (ptp_rx_filter_last_error_w)
    );

    ptp_rx_async_fifo #(
        .P_DATA_W (C_PTP_RX_REC_W),
        .P_DEPTH  (8)
    ) u_ptp_rx_async_fifo (
        .wr_clk             (eth_rx_clk),
        .wr_resetn          (eth_rx_resetn),
        .wr_clear           (eth_rx_clear_status_pulse_eth | eth_rx_soft_reset_pulse_eth),
        .s_data             (ptp_rx_rec_data_w),
        .s_valid            (ptp_rx_rec_valid_w),
        .s_ready            (ptp_rx_rec_ready_w),
        .wr_full            (ptp_rx_fifo_wr_full_w),
        .wr_overflow_sticky (ptp_rx_fifo_wr_overflow_sticky_w),
        .rd_clk             (clk_in),
        .rd_resetn          (resetn),
        .rd_clear           (cfg_clear_status_pulse | cfg_soft_reset_pulse),
        .m_data             (ptp_rx_fifo_rec_data_w),
        .m_valid            (ptp_rx_fifo_rec_valid_w),
        .m_ready            (ptp_rx_fifo_rec_ready_w),
        .rd_empty           (ptp_rx_fifo_rd_empty_w),
        .rd_overflow_sticky (ptp_rx_fifo_overflow_sticky_w)
    );

    ptp_rx_mailbox #(
        .P_RECORD_W  (C_PTP_RX_REC_W),
        .P_MSG_TS_W  (80),
        .P_PORT_ID_W (80)
    ) u_ptp_rx_mailbox (
        .clk                           (clk_in),
        .rst_n                         (resetn),
        .clear_status                  (cfg_clear_status_pulse | cfg_soft_reset_pulse),
        .s_rec_valid                   (ptp_rx_fifo_rec_valid_w),
        .s_rec_ready                   (ptp_rx_fifo_rec_ready_w),
        .s_rec_data                    (ptp_rx_fifo_rec_data_w),
        .fifo_overflow_sticky          (ptp_rx_fifo_overflow_sticky_w),
        .mbox_valid                    (ptp_rx_mbox_valid),
        .mbox_msg_type                 (ptp_rx_mbox_msg_type),
        .mbox_sequence_id              (ptp_rx_mbox_sequence_id),
        .mbox_rx_timestamp_ns          (ptp_rx_mbox_rx_timestamp_ns),
        .mbox_msg_timestamp_80         (ptp_rx_mbox_msg_timestamp_80),
        .mbox_requesting_port_identity (ptp_rx_mbox_requesting_port_identity),
        .mbox_new_pulse                (ptp_rx_mbox_new_pulse_w),
        .mbox_pop                      (ptp_rx_mbox_pop_pulse),
        .mbox_overflow_sticky          (ptp_rx_mbox_overflow_sticky)
    );

    ptp_rx_status_cdc u_ptp_rx_status_cdc (
        .eth_clk                  (eth_rx_clk),
        .eth_resetn               (eth_rx_resetn),
        .eth_accept_count         (ptp_rx_filter_accept_count_w),
        .eth_drop_count           (ptp_rx_filter_drop_count_w),
        .eth_last_error           (ptp_rx_filter_last_error_w),
        .eth_fifo_overflow_sticky (ptp_rx_fifo_wr_overflow_sticky_w),
        .sys_clk                  (clk_in),
        .sys_resetn               (resetn),
        .sys_accept_count         (stat_rx_accept_count_sys),
        .sys_drop_count           (stat_rx_drop_count_sys),
        .sys_last_error           (stat_rx_last_error_sys),
        .sys_fifo_overflow_sticky (stat_rx_fifo_overflow_sticky_sys)
    );


    // 保留统计 / future-debug 侧带，避免当前阶段的结构性拆分引入未使用警告。
    assign unused_keep_w = ptp_rx_fifo_wr_full_w |
                           ptp_rx_fifo_rd_empty_w |
                           ptp_rx_mbox_new_pulse_w |
                           ptp_rx_filter_accept_count_w[0] |
                           ptp_rx_filter_drop_count_w[0] |
                           ptp_rx_filter_last_error_w[0];

endmodule
