`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// net_acq_eth_backend_top
// -----------------------------------------------------------------------------
// 说明：
//   这是 net_acq 真实以太网后端的首版顶层，当前分层为：
//
//     packet_formatter(pkt_*)
//         -> net_acq_pkt_ingress_adapter
//         -> udp_tx_framer
//         -> ipv4_tx_framer
//         -> m_tx_* (输出完整 Ethernet II + IPv4 + UDP frame 字节流)
//
//   模块职责：
//     1) net_acq_pkt_ingress_adapter
//        - 接收 packet_formatter 的 16-bit pkt_* 包流
//        - 先收整帧，再吐整帧
//
//     2) udp_tx_framer
//        - 根据 frame_word_count 计算 UDP length
//        - 插入 8-byte UDP header
//        - 把 16-bit payload 转成 8-bit byte stream（高字节先发）
//
//     3) ipv4_tx_framer
//        - 插入 Ethernet Header + IPv4 Header
//        - 输出到更下层 MAC / GMII / RGMII 适配模块
//
//   当前版本范围：
//     - transmit-only
//     - static MAC/IP/UDP config
//     - 不做 ARP / RX / VLAN / PTP 注入
//     - UDP checksum 固定为 0
//     - 默认假设更下层 MAC/PHY 负责 preamble/SFD/FCS
//////////////////////////////////////////////////////////////////////////////////
module net_acq_eth_backend_top #(
    parameter integer P_PKT_W = 16,
    parameter integer P_MAC_W = 8
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // ------------------------------------------------------------------------
    // 0) 全局控制（来自 wrapper / CSR）
    // ------------------------------------------------------------------------
    input  wire                     cfg_enable,
    input  wire                     cfg_soft_reset,
    input  wire                     cfg_clear_status,

    // ------------------------------------------------------------------------
    // 1) 上游 net_acq 包流输入（来自 packet_formatter）
    // ------------------------------------------------------------------------
    input  wire [P_PKT_W-1:0]       s_pkt_data,
    input  wire                     s_pkt_valid,
    input  wire                     s_pkt_last,
    output wire                     s_pkt_ready,

    // ------------------------------------------------------------------------
    // 2) 可选 metadata sideband（当前首版后端不使用，先保留接口）
    // ------------------------------------------------------------------------
    input  wire                     s_meta_valid,
    output wire                     s_meta_ready,
    input  wire [31:0]              s_meta_seq_id,
    input  wire [15:0]              s_meta_channel_id,
    input  wire [15:0]              s_meta_sample_count,
    input  wire [63:0]              s_meta_timestamp,

    // ------------------------------------------------------------------------
    // 3) Ethernet / IPv4 / UDP 静态配置（来自新增 CSR）
    // ------------------------------------------------------------------------
    input  wire [47:0]              cfg_dst_mac,
    input  wire [47:0]              cfg_src_mac,
    input  wire [31:0]              cfg_src_ip,
    input  wire [31:0]              cfg_dst_ip,
    input  wire [15:0]              cfg_udp_src_port,
    input  wire [15:0]              cfg_udp_dst_port,
    input  wire [7:0]               cfg_ip_ttl,
    input  wire [7:0]               cfg_ip_dscp_ecn,
    input  wire [15:0]              cfg_ip_ident_seed,

    // ------------------------------------------------------------------------
    // 4) PTP 预留配置（当前版本只留口，不实现）
    // ------------------------------------------------------------------------
    input  wire                     cfg_ptp_enable,
    input  wire                     cfg_ptp_two_step,
    input  wire [15:0]              cfg_ptp_udp_dst_port,
    input  wire [7:0]               cfg_ptp_domain,
    input  wire [7:0]               cfg_ptp_message_type,
    input  wire                     ptp_tx_ts_valid,
    input  wire [79:0]              ptp_tx_ts_data,
    output wire                     ptp_tx_ts_ready,

    // ------------------------------------------------------------------------
    // 5) 输出到 MAC 侧的完整 Ethernet frame 字节流
    // ------------------------------------------------------------------------
    output wire [P_MAC_W-1:0]       m_tx_data,
    output wire                     m_tx_valid,
    output wire                     m_tx_last,
    input  wire                     m_tx_ready,
    output wire                     m_tx_start,

    // ------------------------------------------------------------------------
    // 6) 状态 / 统计输出（回到 wrapper，再映射到新增 CSR）
    // ------------------------------------------------------------------------
    output wire                     stat_busy,
    output wire                     stat_ingress_busy,
    output wire                     stat_tx_busy,
    output wire [31:0]              stat_tx_frame_count,
    output wire [31:0]              stat_tx_byte_count,
    output wire [31:0]              stat_drop_count,
    output wire [31:0]              stat_backpressure_cycles,
    output wire [15:0]              stat_last_ip_total_length,
    output wire [15:0]              stat_last_udp_length,
    output wire [15:0]              stat_last_error_code
);

    // ------------------------------------------------------------------------
    // 当前首版 transmit-only UDP 后端暂不使用这些预留口：
    //   - metadata sideband
    //   - PTP 配置与发送时间戳返回
    // 先固定 ready，后续扩展时无需改顶层端口表。
    // ------------------------------------------------------------------------
    wire _unused_keep;
    assign _unused_keep = s_meta_valid |
                          s_meta_seq_id[0] |
                          s_meta_channel_id[0] |
                          s_meta_sample_count[0] |
                          s_meta_timestamp[0] |
                          cfg_ptp_enable |
                          cfg_ptp_two_step |
                          cfg_ptp_udp_dst_port[0] |
                          cfg_ptp_domain[0] |
                          cfg_ptp_message_type[0] |
                          ptp_tx_ts_valid |
                          ptp_tx_ts_data[0];

    assign s_meta_ready    = 1'b0;
    assign ptp_tx_ts_ready = 1'b0;

    // ------------------------------------------------------------------------
    // ingress_adapter <-> udp_tx_framer 内部连线
    // ------------------------------------------------------------------------
    wire [P_PKT_W-1:0] ingress_word_data;
    wire               ingress_word_valid;
    wire               ingress_word_last;
    wire               ingress_word_ready;
    wire               ingress_frame_ready;
    wire [15:0]        ingress_frame_word_count;
    wire               ingress_frame_pop;
    wire               ingress_busy;
    wire               ingress_drop_mode;
    wire [31:0]        ingress_stat_frame_accept_count;
    wire [31:0]        ingress_stat_frame_drop_count;
    wire [15:0]        ingress_stat_last_error;

    // ------------------------------------------------------------------------
    // udp_tx_framer <-> ipv4_tx_framer 内部连线
    // ------------------------------------------------------------------------
    wire               udp_frame_ready;
    wire [15:0]        udp_udp_length;
    wire               udp_frame_pop;
    wire [7:0]         udp_byte_data;
    wire               udp_byte_valid;
    wire               udp_byte_last;
    wire               udp_byte_ready;
    wire               udp_busy;
    wire [31:0]        udp_stat_datagram_count;
    wire [15:0]        udp_stat_last_length;
    wire [15:0]        udp_stat_last_error;

    // ------------------------------------------------------------------------
    // ipv4_tx_framer 最终输出与统计
    // ------------------------------------------------------------------------
    wire               ipv4_busy;
    wire [31:0]        ipv4_stat_tx_frame_count;
    wire [31:0]        ipv4_stat_tx_byte_count;
    wire [31:0]        ipv4_stat_backpressure_cycles;
    wire [15:0]        ipv4_stat_last_ip_total_length;
    wire [15:0]        ipv4_stat_last_udp_length;
    wire [15:0]        ipv4_stat_last_error;

    // ------------------------------------------------------------------------
    // 模块 1：入口缓存（先收整帧，再吐整帧）
    // ------------------------------------------------------------------------
    net_acq_pkt_ingress_adapter #(
        .P_WORD_W    (P_PKT_W),
        .P_MAX_WORDS (272)
    ) u_pkt_ingress_adapter (
        .clk                     (clk),
        .rst_n                   (rst_n),
        .cfg_enable              (cfg_enable),
        .cfg_soft_reset          (cfg_soft_reset),
        .cfg_clear_status        (cfg_clear_status),
        .s_pkt_data              (s_pkt_data),
        .s_pkt_valid             (s_pkt_valid),
        .s_pkt_last              (s_pkt_last),
        .s_pkt_ready             (s_pkt_ready),
        .m_word_data             (ingress_word_data),
        .m_word_valid            (ingress_word_valid),
        .m_word_last             (ingress_word_last),
        .m_word_ready            (ingress_word_ready),
        .frame_ready             (ingress_frame_ready),
        .frame_word_count        (ingress_frame_word_count),
        .frame_pop               (ingress_frame_pop),
        .busy                    (ingress_busy),
        .drop_mode               (ingress_drop_mode),
        .stat_frame_accept_count (ingress_stat_frame_accept_count),
        .stat_frame_drop_count   (ingress_stat_frame_drop_count),
        .stat_last_error         (ingress_stat_last_error)
    );

    // ------------------------------------------------------------------------
    // 模块 2：插入 UDP Header，输出 UDP datagram 字节流
    // ------------------------------------------------------------------------
    udp_tx_framer #(
        .P_WORD_W (P_PKT_W)
    ) u_udp_tx_framer (
        .clk                  (clk),
        .rst_n                (rst_n),
        .cfg_enable           (cfg_enable),
        .cfg_soft_reset       (cfg_soft_reset),
        .cfg_clear_status     (cfg_clear_status),
        .cfg_udp_src_port     (cfg_udp_src_port),
        .cfg_udp_dst_port     (cfg_udp_dst_port),
        .s_word_data          (ingress_word_data),
        .s_word_valid         (ingress_word_valid),
        .s_word_last          (ingress_word_last),
        .s_word_ready         (ingress_word_ready),
        .s_frame_ready        (ingress_frame_ready),
        .s_frame_word_count   (ingress_frame_word_count),
        .s_frame_pop          (ingress_frame_pop),
        .m_frame_ready        (udp_frame_ready),
        .m_udp_length         (udp_udp_length),
        .m_frame_pop          (udp_frame_pop),
        .m_byte_data          (udp_byte_data),
        .m_byte_valid         (udp_byte_valid),
        .m_byte_last          (udp_byte_last),
        .m_byte_ready         (udp_byte_ready),
        .busy                 (udp_busy),
        .stat_datagram_count  (udp_stat_datagram_count),
        .stat_last_udp_length (udp_stat_last_length),
        .stat_last_error      (udp_stat_last_error)
    );

    // ------------------------------------------------------------------------
    // 模块 3：插入 Ethernet II + IPv4 Header，输出完整 Ethernet frame 字节流
    // ------------------------------------------------------------------------
    ipv4_tx_framer #(
        .P_MAC_W (P_MAC_W)
    ) u_ipv4_tx_framer (
        .clk                     (clk),
        .rst_n                   (rst_n),
        .cfg_enable              (cfg_enable),
        .cfg_soft_reset          (cfg_soft_reset),
        .cfg_clear_status        (cfg_clear_status),
        .cfg_dst_mac             (cfg_dst_mac),
        .cfg_src_mac             (cfg_src_mac),
        .cfg_src_ip              (cfg_src_ip),
        .cfg_dst_ip              (cfg_dst_ip),
        .cfg_ip_ttl              (cfg_ip_ttl),
        .cfg_ip_dscp_ecn         (cfg_ip_dscp_ecn),
        .cfg_ip_ident_seed       (cfg_ip_ident_seed),
        .s_frame_ready           (udp_frame_ready),
        .s_udp_length            (udp_udp_length),
        .s_frame_pop             (udp_frame_pop),
        .s_byte_data             (udp_byte_data),
        .s_byte_valid            (udp_byte_valid),
        .s_byte_last             (udp_byte_last),
        .s_byte_ready            (udp_byte_ready),
        .m_tx_data               (m_tx_data),
        .m_tx_valid              (m_tx_valid),
        .m_tx_last               (m_tx_last),
        .m_tx_ready              (m_tx_ready),
        .m_tx_start              (m_tx_start),
        .busy                    (ipv4_busy),
        .stat_tx_frame_count     (ipv4_stat_tx_frame_count),
        .stat_tx_byte_count      (ipv4_stat_tx_byte_count),
        .stat_backpressure_cycles(ipv4_stat_backpressure_cycles),
        .stat_last_ip_total_length(ipv4_stat_last_ip_total_length),
        .stat_last_udp_length    (ipv4_stat_last_udp_length),
        .stat_last_error         (ipv4_stat_last_error)
    );

    // ------------------------------------------------------------------------
    // 状态 / 统计汇总
    // ------------------------------------------------------------------------
    assign stat_ingress_busy         = ingress_busy | ingress_drop_mode;
    assign stat_tx_busy              = udp_busy | ipv4_busy;
    assign stat_busy                 = ingress_busy | udp_busy | ipv4_busy;

    assign stat_tx_frame_count       = ipv4_stat_tx_frame_count;
    assign stat_tx_byte_count        = ipv4_stat_tx_byte_count;
    assign stat_drop_count           = ingress_stat_frame_drop_count;
    assign stat_backpressure_cycles  = ipv4_stat_backpressure_cycles;
    assign stat_last_ip_total_length = ipv4_stat_last_ip_total_length;
    assign stat_last_udp_length      = udp_stat_last_length;

    // 最近错误码优先级：先看入口接收，再看 UDP，再看 IPv4 发送
    assign stat_last_error_code = (ingress_stat_last_error != 16'd0) ? ingress_stat_last_error :
                                  (udp_stat_last_error     != 16'd0) ? udp_stat_last_error     :
                                                                        ipv4_stat_last_error;

endmodule
