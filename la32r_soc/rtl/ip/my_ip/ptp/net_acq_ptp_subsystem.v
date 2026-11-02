`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_ptp_subsystem
// Description:
//   Compact PTP 子系统兼容外壳。
//
//   这次重构的目标不是改外部合同，而是把原先已经逐渐变厚的
//   net_acq_ptp_subsystem 再拆成边界清晰的 4 个子块：
//
//     1) ptp_cfg_cmd_manager
//        - sys-domain 配置 / 命令管理
//        - config CDC / tx-cmd CDC / txstamp CDC
//        - busy / pending / outstanding 状态汇总
//
//     2) ptp_tod_fabric
//        - sys / eth_tx / eth_rx 三域 ToD 计数器
//
//     3) ptp_rx_path
//        - ptp_rx_filter + async FIFO + sys mailbox
//
//     4) ptp_tx_path
//        - Delay_Req TX pipeline -> udp_tx_framer -> ipv4_tx_framer
//
//   兼容性要求：
//     - net_acq_wrapper 对 SoC / CSR 的软件可见合同不变；
//     - 软件可见的 TXSTAMP low->high 消费语义不变；
//     - 新增 dedicated PTP peer addr 仅影响内部 wrapper->PTP TX 路由；
//     - RX mailbox 的 single-outstanding 模型不变；
//     - 主动 TX 仍固定收口到 Delay_Req + need_timestamp=1。
//////////////////////////////////////////////////////////////////////////////////
module net_acq_ptp_subsystem(
    input  wire         clk_in,
    input  wire         resetn,
    input  wire         eth_tx_clk,
    input  wire         eth_tx_resetn,
    input  wire         eth_rx_clk,
    input  wire         eth_rx_resetn,

    // ------------------------------------------------------------------------
    // sys-domain pulses shared with the rest of the wrapper
    // ------------------------------------------------------------------------
    input  wire         cfg_soft_reset_pulse,
    input  wire         cfg_clear_status_pulse,

    // ------------------------------------------------------------------------
    // PTP shadow configuration / control from CSR (sys domain)
    // ------------------------------------------------------------------------
    input  wire         cfg_ptp_enable,
    input  wire         cfg_ptp_one_step,
    input  wire         cfg_ptp_two_step,
    input  wire         cfg_ptp_event_priority_enable,
    input  wire         cfg_ptp_hold,
    input  wire         cfg_ptp_strict_priority,
    input  wire [7:0]   cfg_ptp_domain,
    input  wire [63:0]  cfg_ptp_clock_id,
    input  wire [15:0]  cfg_ptp_port_num,
    input  wire         cfg_ptp_update_pulse,
    input  wire         cfg_ptp_set_time_pulse,
    input  wire         cfg_ptp_step_pulse,
    input  wire         cfg_ptp_tx_abort_pulse,
    input  wire         cfg_ptp_tx_trigger_pulse,
    input  wire [63:0]  cfg_ptp_set_time_ns,
    input  wire [31:0]  cfg_ptp_step_ns,
    input  wire [31:0]  cfg_ptp_freq_adj_ppb,
    input  wire [3:0]   cfg_ptp_tx_message_type,
    input  wire [3:0]   cfg_ptp_tx_transport_specific,
    input  wire [15:0]  cfg_ptp_tx_sequence_id,
    input  wire [15:0]  cfg_ptp_tx_flags,
    input  wire [7:0]   cfg_ptp_tx_log_message_interval,
    input  wire         cfg_ptp_tx_need_timestamp,

    // ------------------------------------------------------------------------
    // Active ETH config delivered from net_acq_eth_cfg_manager (eth_tx domain)
    // ------------------------------------------------------------------------
    input  wire [47:0]  eth_cfg_local_mac_eth,
    input  wire [31:0]  eth_cfg_local_ip_eth,
    // Effective dedicated PTP peer address in eth_tx domain.
    // In legacy mode this still aliases the normal UDP data destination.
    input  wire [47:0]  eth_cfg_ptp_peer_mac_eth,
    input  wire [31:0]  eth_cfg_ptp_peer_ip_eth,
    input  wire [7:0]   eth_cfg_ttl_eth,
    input  wire [7:0]   eth_cfg_dscp_ecn_eth,
    input  wire [15:0]  eth_cfg_ip_ident_seed_eth,

    // ------------------------------------------------------------------------
    // Active PTP RX match config (eth_rx domain)
    // ------------------------------------------------------------------------
    input  wire         rx_match_ptp_enable_eth,
    input  wire [7:0]   rx_match_ptp_domain_eth,
    input  wire [31:0]  rx_match_host_ip_eth,
    input  wire [31:0]  rx_match_local_ip_eth,
    input  wire [47:0]  rx_match_host_mac_eth,
    input  wire [47:0]  rx_match_local_mac_eth,

    // ------------------------------------------------------------------------
    // Cross-domain pulses from the ETH config manager
    // ------------------------------------------------------------------------
    input  wire         eth_soft_reset_pulse_eth,
    input  wire         eth_clear_status_pulse_eth,
    input  wire         eth_ptp_tx_abort_pulse_eth,
    input  wire         eth_rx_soft_reset_pulse_eth,
    input  wire         eth_rx_clear_status_pulse_eth,

    // ------------------------------------------------------------------------
    // Board-independent MAC-side RX stream / RX timestamp sideband
    // ------------------------------------------------------------------------
    input  wire [7:0]   s_axis_eth_rx_tdata,
    input  wire         s_axis_eth_rx_tvalid,
    input  wire         s_axis_eth_rx_tlast,
    input  wire [63:0]  ptp_rxstamp_ns,
    input  wire         ptp_rxstamp_valid,

    // ------------------------------------------------------------------------
    // Reserved board-independent MAC-side TX timestamp sideband
    // ------------------------------------------------------------------------
    input  wire [63:0]  ptp_txstamp_ns,
    input  wire         ptp_txstamp_valid,

    // ------------------------------------------------------------------------
    // sys-domain ToD / status exposed to frontend + CSR
    // ------------------------------------------------------------------------
    output wire [63:0]  ptp_sys_tod_time_ns,
    output wire         ptp_sys_tod_time_valid,
    output wire [63:0]  ptp_tod_time_ns_eth,
    output wire         ptp_tod_time_valid_eth,
    output wire [63:0]  ptp_tod_time_ns_rx,
    output wire         ptp_tod_time_valid_rx,
    output wire [63:0]  ptp_txstamp_ns_latched,
    output wire         ptp_txstamp_valid_latched,
    output wire         ptp_txstamp_overflow_sticky,
    input  wire         ptp_txstamp_consume_pulse_sys,
    output wire         ptp_tx_busy_status_sys,
    output wire         ptp_tx_cmd_pending_status_sys,

    // ------------------------------------------------------------------------
    // PTP RX mailbox visible to CSR / CPU (sys domain)
    // ------------------------------------------------------------------------
    input  wire         ptp_rx_mbox_pop_pulse,
    output wire         ptp_rx_mbox_valid,
    output wire [3:0]   ptp_rx_mbox_msg_type,
    output wire [15:0]  ptp_rx_mbox_sequence_id,
    output wire [63:0]  ptp_rx_mbox_rx_timestamp_ns,
    output wire [79:0]  ptp_rx_mbox_msg_timestamp_80,
    output wire [79:0]  ptp_rx_mbox_requesting_port_identity,
    output wire         ptp_rx_mbox_overflow_sticky,
    output wire [31:0]  ptp_rx_accept_count_sys,
    output wire [31:0]  ptp_rx_drop_count_sys,
    output wire [15:0]  ptp_rx_last_error_sys,
    output wire         ptp_rx_fifo_overflow_sticky_sys,

    // ------------------------------------------------------------------------
    // PTP byte-stream output to the egress arbiter (eth_tx domain)
    // ------------------------------------------------------------------------
    output wire [7:0]   ptp_tx_tdata,
    output wire         ptp_tx_tvalid,
    output wire         ptp_tx_tlast,
    input  wire         ptp_tx_tready,
    output wire [3:0]   ptp_tx_tuser,

    // ------------------------------------------------------------------------
    // PTP arbitration control / busy indication for sibling subsystems
    // ------------------------------------------------------------------------
    output wire         ptp_cfg_enable_eth,
    output wire         ptp_cfg_strict_priority_eth,
    output wire         ptp_cfg_event_priority_enable_eth,
    output wire         ptp_path_busy_eth,

    // ------------------------------------------------------------------------
    // Pre-scheduler observability extension (eth_tx domain)
    // ------------------------------------------------------------------------
    output wire [31:0]  ptp_tx_trigger_req_count_eth,
    output wire [31:0]  ptp_tx_trigger_accept_count_eth,
    output wire [31:0]  ptp_tx_trigger_while_busy_count_eth,
    output wire [15:0]  ptp_tx_last_error_eth
);

    // Local reset release trees: asynchronous assertion, two local clock edges
    // to release. Keep each bank distinct so implementation can place it near
    // its consumers; FIFO port resets and soft-clear protocols are unchanged.
    wire cfg_sys_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_cfg_sys_resetn (
        .clk(clk_in), .rst_n_in(resetn), .rst_n_out(cfg_sys_resetn)
    );
    wire cfg_tx_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_cfg_tx_resetn (
        .clk(eth_tx_clk), .rst_n_in(eth_tx_resetn), .rst_n_out(cfg_tx_resetn)
    );
    wire cfg_rx_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_cfg_rx_resetn (
        .clk(eth_rx_clk), .rst_n_in(eth_rx_resetn), .rst_n_out(cfg_rx_resetn)
    );
    wire tod_sys_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_tod_sys_resetn (
        .clk(clk_in), .rst_n_in(resetn), .rst_n_out(tod_sys_resetn)
    );
    wire tod_tx_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_tod_tx_resetn (
        .clk(eth_tx_clk), .rst_n_in(eth_tx_resetn), .rst_n_out(tod_tx_resetn)
    );
    wire tod_rx_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_tod_rx_resetn (
        .clk(eth_rx_clk), .rst_n_in(eth_rx_resetn), .rst_n_out(tod_rx_resetn)
    );
    wire rx_sys_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_rx_sys_resetn (
        .clk(clk_in), .rst_n_in(resetn), .rst_n_out(rx_sys_resetn)
    );
    wire rx_path_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_rx_path_resetn (
        .clk(eth_rx_clk), .rst_n_in(eth_rx_resetn), .rst_n_out(rx_path_resetn)
    );
    wire tx_path_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_tx_path_resetn (
        .clk(eth_tx_clk), .rst_n_in(eth_tx_resetn), .rst_n_out(tx_path_resetn)
    );

    // ------------------------------------------------------------------------
    // Internal inter-subsystem wiring
    // ------------------------------------------------------------------------
    wire         tx_cfg_enable_eth_w;
    wire         tx_cfg_hold_eth_w;
    wire [7:0]   tx_cfg_domain_eth_w;
    wire [63:0]  tx_cfg_clock_id_eth_w;
    wire [15:0]  tx_cfg_port_num_eth_w;
    wire [63:0]  tx_cfg_set_time_ns_eth_w;
    wire [31:0]  tx_cfg_step_ns_eth_w;
    wire [31:0]  tx_cfg_freq_adj_ppb_eth_w;
    wire         tx_cfg_set_time_pulse_eth_w;
    wire         tx_cfg_step_pulse_eth_w;
    wire         tx_cfg_update_pulse_eth_w;

    wire         rx_tod_enable_eth_w;
    wire         rx_tod_hold_eth_w;
    wire [63:0]  rx_tod_set_time_ns_eth_w;
    wire [31:0]  rx_tod_step_ns_eth_w;
    wire [31:0]  rx_tod_freq_adj_ppb_eth_w;
    wire         rx_tod_set_time_pulse_eth_w;
    wire         rx_tod_step_pulse_eth_w;
    wire         rx_tod_update_pulse_eth_w;

    wire         tx_cmd_update_pulse_eth_w;
    wire         tx_cmd_need_timestamp_eth_w;
    wire [7:0]   tx_cmd_log_message_interval_eth_w;
    wire [15:0]  tx_cmd_flags_eth_w;
    wire [15:0]  tx_cmd_sequence_id_eth_w;
    wire [3:0]   tx_cmd_transport_specific_eth_w;
    wire [3:0]   tx_cmd_message_type_eth_w;
    wire [15:0]  tx_cmd_port_num_eth_w;
    wire [63:0]  tx_cmd_clock_id_eth_w;
    wire [7:0]   tx_cmd_domain_eth_w;

    wire         ptp_txstamp_src_ready_eth_w;
    wire         ptp_tx_pending_any_eth_w;
    wire         ptp_tx_cmd_exec_ack_pulse_eth_w;

    // ------------------------------------------------------------------------
    // 1) Control plane / command plane / TXSTAMP plane
    // ------------------------------------------------------------------------
    ptp_cfg_cmd_manager u_ptp_cfg_cmd_manager (
        .clk_in                         (clk_in),
        .resetn                         (cfg_sys_resetn),
        .eth_tx_clk                     (eth_tx_clk),
        .eth_tx_resetn                  (cfg_tx_resetn),
        .eth_rx_clk                     (eth_rx_clk),
        .eth_rx_resetn                  (cfg_rx_resetn),
        .cfg_soft_reset_pulse           (cfg_soft_reset_pulse),
        .cfg_clear_status_pulse         (cfg_clear_status_pulse),
        .cfg_ptp_enable                 (cfg_ptp_enable),
        .cfg_ptp_one_step               (cfg_ptp_one_step),
        .cfg_ptp_two_step               (cfg_ptp_two_step),
        .cfg_ptp_event_priority_enable  (cfg_ptp_event_priority_enable),
        .cfg_ptp_hold                   (cfg_ptp_hold),
        .cfg_ptp_strict_priority        (cfg_ptp_strict_priority),
        .cfg_ptp_domain                 (cfg_ptp_domain),
        .cfg_ptp_clock_id               (cfg_ptp_clock_id),
        .cfg_ptp_port_num               (cfg_ptp_port_num),
        .cfg_ptp_update_pulse           (cfg_ptp_update_pulse),
        .cfg_ptp_set_time_pulse         (cfg_ptp_set_time_pulse),
        .cfg_ptp_step_pulse             (cfg_ptp_step_pulse),
        .cfg_ptp_tx_abort_pulse         (cfg_ptp_tx_abort_pulse),
        .cfg_ptp_tx_trigger_pulse       (cfg_ptp_tx_trigger_pulse),
        .cfg_ptp_set_time_ns            (cfg_ptp_set_time_ns),
        .cfg_ptp_step_ns                (cfg_ptp_step_ns),
        .cfg_ptp_freq_adj_ppb           (cfg_ptp_freq_adj_ppb),
        .cfg_ptp_tx_message_type        (cfg_ptp_tx_message_type),
        .cfg_ptp_tx_transport_specific  (cfg_ptp_tx_transport_specific),
        .cfg_ptp_tx_sequence_id         (cfg_ptp_tx_sequence_id),
        .cfg_ptp_tx_flags               (cfg_ptp_tx_flags),
        .cfg_ptp_tx_log_message_interval(cfg_ptp_tx_log_message_interval),
        .cfg_ptp_tx_need_timestamp      (cfg_ptp_tx_need_timestamp),
        .ptp_tx_cmd_exec_ack_pulse_eth  (ptp_tx_cmd_exec_ack_pulse_eth_w),
        .ptp_path_busy_eth              (ptp_path_busy_eth),
        .ptp_tx_pending_any_eth         (ptp_tx_pending_any_eth_w),
        .ptp_txstamp_ns                 (ptp_txstamp_ns),
        .ptp_txstamp_valid              (ptp_txstamp_valid),
        .ptp_txstamp_ns_latched         (ptp_txstamp_ns_latched),
        .ptp_txstamp_valid_latched      (ptp_txstamp_valid_latched),
        .ptp_txstamp_overflow_sticky    (ptp_txstamp_overflow_sticky),
        .ptp_txstamp_consume_pulse_sys  (ptp_txstamp_consume_pulse_sys),
        .ptp_tx_busy_status_sys         (ptp_tx_busy_status_sys),
        .ptp_tx_cmd_pending_status_sys  (ptp_tx_cmd_pending_status_sys),
        .tx_cfg_enable_eth              (tx_cfg_enable_eth_w),
        .tx_cfg_hold_eth                (tx_cfg_hold_eth_w),
        .tx_cfg_strict_priority_eth     (),
        .tx_cfg_event_priority_enable_eth(),
        .tx_cfg_domain_eth              (tx_cfg_domain_eth_w),
        .tx_cfg_clock_id_eth            (tx_cfg_clock_id_eth_w),
        .tx_cfg_port_num_eth            (tx_cfg_port_num_eth_w),
        .tx_cfg_set_time_ns_eth         (tx_cfg_set_time_ns_eth_w),
        .tx_cfg_step_ns_eth             (tx_cfg_step_ns_eth_w),
        .tx_cfg_freq_adj_ppb_eth        (tx_cfg_freq_adj_ppb_eth_w),
        .tx_cfg_set_time_pulse_eth      (tx_cfg_set_time_pulse_eth_w),
        .tx_cfg_step_pulse_eth          (tx_cfg_step_pulse_eth_w),
        .tx_cfg_update_pulse_eth        (tx_cfg_update_pulse_eth_w),
        .rx_tod_enable_eth              (rx_tod_enable_eth_w),
        .rx_tod_hold_eth                (rx_tod_hold_eth_w),
        .rx_tod_set_time_ns_eth         (rx_tod_set_time_ns_eth_w),
        .rx_tod_step_ns_eth             (rx_tod_step_ns_eth_w),
        .rx_tod_freq_adj_ppb_eth        (rx_tod_freq_adj_ppb_eth_w),
        .rx_tod_set_time_pulse_eth      (rx_tod_set_time_pulse_eth_w),
        .rx_tod_step_pulse_eth          (rx_tod_step_pulse_eth_w),
        .rx_tod_update_pulse_eth        (rx_tod_update_pulse_eth_w),
        .tx_cmd_update_pulse_eth        (tx_cmd_update_pulse_eth_w),
        .tx_cmd_need_timestamp_eth      (tx_cmd_need_timestamp_eth_w),
        .tx_cmd_log_message_interval_eth(tx_cmd_log_message_interval_eth_w),
        .tx_cmd_flags_eth               (tx_cmd_flags_eth_w),
        .tx_cmd_sequence_id_eth         (tx_cmd_sequence_id_eth_w),
        .tx_cmd_transport_specific_eth  (tx_cmd_transport_specific_eth_w),
        .tx_cmd_message_type_eth        (tx_cmd_message_type_eth_w),
        .tx_cmd_port_num_eth            (tx_cmd_port_num_eth_w),
        .tx_cmd_clock_id_eth            (tx_cmd_clock_id_eth_w),
        .tx_cmd_domain_eth              (tx_cmd_domain_eth_w),
        .ptp_cfg_enable_eth             (ptp_cfg_enable_eth),
        .ptp_cfg_strict_priority_eth    (ptp_cfg_strict_priority_eth),
        .ptp_cfg_event_priority_enable_eth(ptp_cfg_event_priority_enable_eth),
        .ptp_txstamp_src_ready_eth      (ptp_txstamp_src_ready_eth_w)
    );

    // ------------------------------------------------------------------------
    // 2) Three-domain ToD counters
    // ------------------------------------------------------------------------
    ptp_tod_fabric u_ptp_tod_fabric (
        .clk_in                   (clk_in),
        .resetn                   (tod_sys_resetn),
        .eth_tx_clk               (eth_tx_clk),
        .eth_tx_resetn            (tod_tx_resetn),
        .eth_rx_clk               (eth_rx_clk),
        .eth_rx_resetn            (tod_rx_resetn),
        .cfg_ptp_enable           (cfg_ptp_enable),
        .cfg_ptp_hold             (cfg_ptp_hold),
        .cfg_ptp_update_pulse     (cfg_ptp_update_pulse),
        .cfg_ptp_set_time_pulse   (cfg_ptp_set_time_pulse),
        .cfg_ptp_step_pulse       (cfg_ptp_step_pulse),
        .cfg_ptp_set_time_ns      (cfg_ptp_set_time_ns),
        .cfg_ptp_step_ns          (cfg_ptp_step_ns),
        .cfg_ptp_freq_adj_ppb     (cfg_ptp_freq_adj_ppb),
        .tx_cfg_enable_eth        (tx_cfg_enable_eth_w),
        .tx_cfg_hold_eth          (tx_cfg_hold_eth_w),
        .tx_cfg_update_pulse_eth  (tx_cfg_update_pulse_eth_w),
        .tx_cfg_set_time_pulse_eth(tx_cfg_set_time_pulse_eth_w),
        .tx_cfg_step_pulse_eth    (tx_cfg_step_pulse_eth_w),
        .tx_cfg_set_time_ns_eth   (tx_cfg_set_time_ns_eth_w),
        .tx_cfg_step_ns_eth       (tx_cfg_step_ns_eth_w),
        .tx_cfg_freq_adj_ppb_eth  (tx_cfg_freq_adj_ppb_eth_w),
        .rx_tod_enable_eth        (rx_tod_enable_eth_w),
        .rx_tod_hold_eth          (rx_tod_hold_eth_w),
        .rx_tod_update_pulse_eth  (rx_tod_update_pulse_eth_w),
        .rx_tod_set_time_pulse_eth(rx_tod_set_time_pulse_eth_w),
        .rx_tod_step_pulse_eth    (rx_tod_step_pulse_eth_w),
        .rx_tod_set_time_ns_eth   (rx_tod_set_time_ns_eth_w),
        .rx_tod_step_ns_eth       (rx_tod_step_ns_eth_w),
        .rx_tod_freq_adj_ppb_eth  (rx_tod_freq_adj_ppb_eth_w),
        .ptp_sys_tod_time_ns      (ptp_sys_tod_time_ns),
        .ptp_sys_tod_time_valid   (ptp_sys_tod_time_valid),
        .ptp_tod_time_ns_eth      (ptp_tod_time_ns_eth),
        .ptp_tod_time_valid_eth   (ptp_tod_time_valid_eth),
        .ptp_tod_time_ns_rx       (ptp_tod_time_ns_rx),
        .ptp_tod_time_valid_rx    (ptp_tod_time_valid_rx)
    );

    // ------------------------------------------------------------------------
    // 3) RX path
    // ------------------------------------------------------------------------
    ptp_rx_path u_ptp_rx_path (
        .clk_in                         (clk_in),
        .resetn                         (rx_sys_resetn),
        .eth_rx_clk                     (eth_rx_clk),
        .eth_rx_resetn                  (rx_path_resetn),
        .rx_match_ptp_enable_eth        (rx_match_ptp_enable_eth),
        .rx_match_ptp_domain_eth        (rx_match_ptp_domain_eth),
        .rx_match_host_ip_eth           (rx_match_host_ip_eth),
        .rx_match_local_ip_eth          (rx_match_local_ip_eth),
        .rx_match_host_mac_eth          (rx_match_host_mac_eth),
        .rx_match_local_mac_eth         (rx_match_local_mac_eth),
        .eth_rx_soft_reset_pulse_eth    (eth_rx_soft_reset_pulse_eth),
        .eth_rx_clear_status_pulse_eth  (eth_rx_clear_status_pulse_eth),
        .cfg_soft_reset_pulse           (cfg_soft_reset_pulse),
        .cfg_clear_status_pulse         (cfg_clear_status_pulse),
        .s_axis_eth_rx_tdata            (s_axis_eth_rx_tdata),
        .s_axis_eth_rx_tvalid           (s_axis_eth_rx_tvalid),
        .s_axis_eth_rx_tlast            (s_axis_eth_rx_tlast),
        .ptp_rxstamp_ns                 (ptp_rxstamp_ns),
        .ptp_rxstamp_valid              (ptp_rxstamp_valid),
        .ptp_rx_mbox_pop_pulse          (ptp_rx_mbox_pop_pulse),
        .ptp_rx_mbox_valid              (ptp_rx_mbox_valid),
        .ptp_rx_mbox_msg_type           (ptp_rx_mbox_msg_type),
        .ptp_rx_mbox_sequence_id        (ptp_rx_mbox_sequence_id),
        .ptp_rx_mbox_rx_timestamp_ns    (ptp_rx_mbox_rx_timestamp_ns),
        .ptp_rx_mbox_msg_timestamp_80   (ptp_rx_mbox_msg_timestamp_80),
        .ptp_rx_mbox_requesting_port_identity(ptp_rx_mbox_requesting_port_identity),
        .ptp_rx_mbox_overflow_sticky    (ptp_rx_mbox_overflow_sticky),
        .stat_rx_accept_count_sys       (ptp_rx_accept_count_sys),
        .stat_rx_drop_count_sys         (ptp_rx_drop_count_sys),
        .stat_rx_last_error_sys         (ptp_rx_last_error_sys),
        .stat_rx_fifo_overflow_sticky_sys(ptp_rx_fifo_overflow_sticky_sys)
    );

    // ------------------------------------------------------------------------
    // 4) TX path
    // ------------------------------------------------------------------------
    ptp_tx_path u_ptp_tx_path (
        .eth_tx_clk                     (eth_tx_clk),
        .eth_tx_resetn                  (tx_path_resetn),
        .tx_cfg_enable_eth              (tx_cfg_enable_eth_w),
        .eth_soft_reset_pulse_eth       (eth_soft_reset_pulse_eth),
        .eth_clear_status_pulse_eth     (eth_clear_status_pulse_eth),
        .eth_ptp_tx_abort_pulse_eth     (eth_ptp_tx_abort_pulse_eth),
        .eth_cfg_local_mac_eth          (eth_cfg_local_mac_eth),
        .eth_cfg_local_ip_eth           (eth_cfg_local_ip_eth),
        .eth_cfg_ptp_peer_mac_eth       (eth_cfg_ptp_peer_mac_eth),
        .eth_cfg_ptp_peer_ip_eth        (eth_cfg_ptp_peer_ip_eth),
        .eth_cfg_ttl_eth                (eth_cfg_ttl_eth),
        .eth_cfg_dscp_ecn_eth           (eth_cfg_dscp_ecn_eth),
        .eth_cfg_ip_ident_seed_eth      (eth_cfg_ip_ident_seed_eth),
        .tx_cmd_update_pulse_eth        (tx_cmd_update_pulse_eth_w),
        .tx_cmd_need_timestamp_eth      (tx_cmd_need_timestamp_eth_w),
        .tx_cmd_log_message_interval_eth(tx_cmd_log_message_interval_eth_w),
        .tx_cmd_flags_eth               (tx_cmd_flags_eth_w),
        .tx_cmd_sequence_id_eth         (tx_cmd_sequence_id_eth_w),
        .tx_cmd_transport_specific_eth  (tx_cmd_transport_specific_eth_w),
        .tx_cmd_message_type_eth        (tx_cmd_message_type_eth_w),
        .tx_cmd_port_num_eth            (tx_cmd_port_num_eth_w),
        .tx_cmd_clock_id_eth            (tx_cmd_clock_id_eth_w),
        .tx_cmd_domain_eth              (tx_cmd_domain_eth_w),
        .ptp_txstamp_src_ready_eth      (ptp_txstamp_src_ready_eth_w),
        .ptp_tx_tdata                   (ptp_tx_tdata),
        .ptp_tx_tvalid                  (ptp_tx_tvalid),
        .ptp_tx_tlast                   (ptp_tx_tlast),
        .ptp_tx_tready                  (ptp_tx_tready),
        .ptp_tx_tuser                   (ptp_tx_tuser),
        .ptp_path_busy_eth              (ptp_path_busy_eth),
        .ptp_tx_pending_any_eth         (ptp_tx_pending_any_eth_w),
        .ptp_tx_cmd_exec_ack_pulse_eth  (ptp_tx_cmd_exec_ack_pulse_eth_w),
        .ptp_tx_trigger_req_count_eth   (ptp_tx_trigger_req_count_eth),
        .ptp_tx_trigger_accept_count_eth(ptp_tx_trigger_accept_count_eth),
        .ptp_tx_trigger_while_busy_count_eth(ptp_tx_trigger_while_busy_count_eth),
        .ptp_tx_last_error_eth          (ptp_tx_last_error_eth)
    );

endmodule
