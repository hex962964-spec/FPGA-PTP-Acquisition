`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_eth_cfg_manager
// Description:
//   System/control-plane manager for the Ethernet-related parts of net_acq.
//
//   This module collects the logic that previously made net_acq_wrapper heavy:
//     1) ETH shadow -> active safe-commit FSM (sys domain)
//        - cfg_active is lowered when a commit is requested
//        - cfg_active is raised only after BOTH eth_tx config CDC and
//          eth_rx match-config CDC have ACKed the same commit epoch
//     2) sys->eth_tx active-config CDC
//     3) sys->eth_rx PTP-match-config CDC
//     4) sys->eth pulse CDCs (soft_reset / clear_status / tx_abort / PHY ctrl)
//     5) eth->sys status snapshot CDC and sticky-status synthesis
//     6) optional split PTP peer address selection while preserving one shared
//        DUT local identity for both the compact PTP path and the normal UDP path
//
//   Commit policy is preserved exactly:
//     - front-end must already be disabled
//     - acquisition core must be idle
//     - sys->eth packet CDC FIFO must be empty
//     - eth-domain backend/PTP transmit path must not be busy
//////////////////////////////////////////////////////////////////////////////////
module net_acq_eth_cfg_manager(
    input  wire         clk_in,
    input  wire         resetn,
    input  wire         eth_tx_clk,
    input  wire         eth_tx_resetn,
    input  wire         eth_rx_clk,
    input  wire         eth_rx_resetn,

    // ------------------------------------------------------------------------
    // Front-end / wrapper state needed for safe commit
    // ------------------------------------------------------------------------
    input  wire         cfg_enable,
    input  wire         cfg_soft_reset_pulse,
    input  wire         cfg_clear_status_pulse,
    input  wire         core_stat_busy,
    input  wire         cdc_fifo_empty_sys,

    // ------------------------------------------------------------------------
    // ETH shadow configuration from CSR (sys domain)
    // ------------------------------------------------------------------------
    input  wire         cfg_eth_backend_enable,
    input  wire         cfg_eth_commit_pulse,
    input  wire         cfg_eth_tx_mirror_enable,
    input  wire         cfg_eth_append_fcs,
    input  wire         cfg_eth_udp_checksum_enable,
    input  wire         cfg_eth_stall_drop_enable,
    input  wire [47:0]  cfg_eth_local_mac,
    input  wire [47:0]  cfg_eth_dst_mac,
    input  wire [31:0]  cfg_eth_local_ip,
    input  wire [31:0]  cfg_eth_dst_ip,
    // Optional split PTP peer shadow bank. These fields join the SAME safe ETH
    // commit epoch as cfg_eth_dst_*. When disabled, legacy behavior is preserved
    // and the effective PTP peer still follows cfg_eth_dst_mac/cfg_eth_dst_ip.
    input  wire         cfg_ptp_addr_split_enable,
    input  wire [47:0]  cfg_ptp_peer_mac,
    input  wire [31:0]  cfg_ptp_peer_ip,
    input  wire [15:0]  cfg_eth_src_port,
    input  wire [15:0]  cfg_eth_dst_port,
    input  wire [7:0]   cfg_eth_ttl,
    input  wire [7:0]   cfg_eth_dscp_ecn,
    input  wire [15:0]  cfg_eth_ip_ident_seed,

    // ------------------------------------------------------------------------
    // TSN-lite v1 scheduler shadow configuration (sys domain)
    //   These fields join the same ETH shadow->active safe-commit epoch as the
    //   normal L2/L3 shadow registers.
    // ------------------------------------------------------------------------
    input  wire         cfg_txs_enable,
    input  wire         cfg_txs_block_when_time_invalid,
    input  wire         cfg_txs_strict_window,
    input  wire [63:0]  cfg_txs_base_time_ns,
    input  wire [31:0]  cfg_txs_period_ns,
    input  wire [31:0]  cfg_txs_slot_offset_ns,
    input  wire [31:0]  cfg_txs_slot_width_ns,
    input  wire [31:0]  cfg_txs_advance_ns,
    input  wire [31:0]  cfg_txs_late_tol_ns,

    // ------------------------------------------------------------------------
    // PTP shadow configuration bits that affect ETH TX / RX matching
    // ------------------------------------------------------------------------
    input  wire         cfg_ptp_enable,
    input  wire         cfg_ptp_two_step,
    input  wire [7:0]   cfg_ptp_domain,
    input  wire         cfg_ptp_update_pulse,
    input  wire         cfg_ptp_tx_abort_pulse,

    // ------------------------------------------------------------------------
    // PHY management control pulses (sys domain)
    // ------------------------------------------------------------------------
    input  wire         cfg_phy_soft_reset_pulse,
    input  wire         cfg_phy_restart_an_pulse,

    // ------------------------------------------------------------------------
    // eth-domain statistics / sideband to be brought back to sys domain
    // ------------------------------------------------------------------------
    input  wire         eth_backend_stat_busy_eth,
    input  wire         eth_backend_stat_tx_busy_eth,
    input  wire [31:0]  eth_backend_stat_tx_frame_count_eth,
    input  wire [31:0]  eth_backend_stat_tx_byte_count_eth,
    input  wire [31:0]  eth_backend_stat_drop_count_eth,
    input  wire [15:0]  eth_backend_stat_last_ip_total_length_eth,
    input  wire [15:0]  eth_backend_stat_last_udp_length_eth,
    input  wire [15:0]  eth_backend_stat_last_error_code_eth,
    input  wire         ptp_path_busy_eth,
    input  wire         mac_link_up,
    input  wire [31:0]  phy_status_word,
    input  wire [127:0] phy_reg_snapshot,

    // ------------------------------------------------------------------------
    // Active data-plane configuration in eth_tx domain
    // ------------------------------------------------------------------------
    output wire         eth_cfg_enable_eth,
    output wire [47:0]  eth_cfg_local_mac_eth,
    output wire [47:0]  eth_cfg_dst_mac_eth,
    output wire [31:0]  eth_cfg_local_ip_eth,
    output wire [31:0]  eth_cfg_dst_ip_eth,
    // Effective compact-PTP peer address already resolved in sys domain.
    output wire [47:0]  eth_cfg_ptp_peer_mac_eth,
    output wire [31:0]  eth_cfg_ptp_peer_ip_eth,
    output wire [15:0]  eth_cfg_src_port_eth,
    output wire [15:0]  eth_cfg_dst_port_eth,
    output wire [7:0]   eth_cfg_ttl_eth,
    output wire [7:0]   eth_cfg_dscp_ecn_eth,
    output wire [15:0]  eth_cfg_ip_ident_seed_eth,
    output wire         eth_cfg_ptp_enable_eth,
    output wire         eth_cfg_ptp_two_step_eth,
    output wire [7:0]   eth_cfg_ptp_domain_eth,
    output wire         eth_cfg_update_pulse_eth,

    // ------------------------------------------------------------------------
    // Active TSN-lite v1 scheduler configuration in eth_tx domain
    // ------------------------------------------------------------------------
    output wire         txs_cfg_enable_eth,
    output wire         txs_cfg_block_when_time_invalid_eth,
    output wire         txs_cfg_strict_window_eth,
    output wire [63:0]  txs_cfg_base_time_ns_eth,
    output wire [31:0]  txs_cfg_period_ns_eth,
    output wire [31:0]  txs_cfg_slot_offset_ns_eth,
    output wire [31:0]  txs_cfg_slot_width_ns_eth,
    output wire [31:0]  txs_cfg_advance_ns_eth,
    output wire [31:0]  txs_cfg_late_tol_ns_eth,

    // ------------------------------------------------------------------------
    // Active PTP RX match configuration in eth_rx domain
    // ------------------------------------------------------------------------
    output wire         rx_match_ptp_enable_eth,
    output wire [7:0]   rx_match_ptp_domain_eth,
    output wire [31:0]  rx_match_host_ip_eth,
    output wire [31:0]  rx_match_local_ip_eth,
    output wire [47:0]  rx_match_host_mac_eth,
    output wire [47:0]  rx_match_local_mac_eth,

    // ------------------------------------------------------------------------
    // Cross-domain control pulses
    // ------------------------------------------------------------------------
    output wire         eth_soft_reset_pulse_eth,
    output wire         eth_clear_status_pulse_eth,
    output wire         eth_ptp_tx_abort_pulse_eth,
    output wire         eth_rx_soft_reset_pulse_eth,
    output wire         eth_rx_clear_status_pulse_eth,
    output wire         phy_soft_reset_pulse,
    output wire         phy_restart_an_pulse,

    // ------------------------------------------------------------------------
    // sys-domain active-state view / status exposed to CSR and wrapper
    // ------------------------------------------------------------------------
    output wire         eth_backend_selected,
    output wire         eth_tx_mirror_enable_active,
    output wire         eth_cfg_active,

    // ------------------------------------------------------------------------
    // Active TSN-lite v1 scheduler configuration reflected in sys domain.
    // These signals expose the committed ACTIVE scheduler bank, not the CSR
    // shadow bank, so software can verify which scheduler epoch is actually in
    // force after ETH cfg_active returns high.
    // ------------------------------------------------------------------------
    output wire         txs_active_enable_sys,
    output wire         txs_active_block_when_time_invalid_sys,
    output wire         txs_active_strict_window_sys,
    output wire [63:0]  txs_active_base_time_ns_sys,
    output wire [31:0]  txs_active_period_ns_sys,
    output wire [31:0]  txs_active_slot_offset_ns_sys,
    output wire [31:0]  txs_active_slot_width_ns_sys,
    output wire [31:0]  txs_active_advance_ns_sys,
    output wire [31:0]  txs_active_late_tol_ns_sys,

    output wire         eth_backend_stat_busy_sys,
    output wire         eth_backend_stat_tx_busy_sys,
    output wire [31:0]  eth_backend_stat_tx_frame_count_sys,
    output wire [31:0]  eth_backend_stat_tx_byte_count_sys,
    output wire [31:0]  eth_backend_stat_drop_count_sys,
    output wire [15:0]  eth_backend_stat_last_ip_total_length_sys,
    output wire [15:0]  eth_backend_stat_last_udp_length_sys,
    output wire [15:0]  eth_backend_stat_last_error_code_sys,
    output wire         eth_tx_error_sticky,
    output wire         eth_pkt_buf_overflow_sticky,
    output wire         eth_mac_stall_timeout_sticky,
    output wire         mac_link_up_sys,
    output wire [31:0]  phy_status_word_sys,
    output wire [127:0] phy_reg_snapshot_sys
);

    // ------------------------------------------------------------------------
    // Active configuration registers in sys domain.
    // ------------------------------------------------------------------------
    reg                 eth_backend_enable_active_r;
    reg                 eth_tx_mirror_enable_active_r;
    reg                 eth_append_fcs_active_r;
    reg                 eth_udp_checksum_enable_active_r;
    reg                 eth_stall_drop_enable_active_r;
    reg [47:0]          eth_local_mac_active_r;
    reg [47:0]          eth_dst_mac_active_r;
    reg [31:0]          eth_local_ip_active_r;
    reg [31:0]          eth_dst_ip_active_r;
    reg                 ptp_addr_split_enable_active_r;
    reg [47:0]          ptp_peer_mac_active_r;
    reg [31:0]          ptp_peer_ip_active_r;
    reg [15:0]          eth_src_port_active_r;
    reg [15:0]          eth_dst_port_active_r;
    reg [7:0]           eth_ttl_active_r;
    reg [7:0]           eth_dscp_ecn_active_r;
    reg [15:0]          eth_ip_ident_seed_active_r;
    reg                 eth_ptp_enable_active_r;
    reg                 eth_ptp_two_step_active_r;
    reg [7:0]           eth_ptp_domain_active_r;
    reg                 txs_enable_active_r;
    reg                 txs_block_when_time_invalid_active_r;
    reg                 txs_strict_window_active_r;
    reg [63:0]          txs_base_time_ns_active_r;
    reg [31:0]          txs_period_ns_active_r;
    reg [31:0]          txs_slot_offset_ns_active_r;
    reg [31:0]          txs_slot_width_ns_active_r;
    reg [31:0]          txs_advance_ns_active_r;
    reg [31:0]          txs_late_tol_ns_active_r;

    // eth_commit_pending_r : a software commit request is queued but not yet launched
    // eth_commit_*_inflight_r: the launched commit still waits for the corresponding ACK
    reg                 eth_commit_pending_r;
    reg                 eth_commit_tx_inflight_r;
    reg                 eth_commit_rx_inflight_r;
    reg                 eth_cfg_req_toggle_r;
    reg                 eth_cfg_ack_toggle_seen_r;

    // ------------------------------------------------------------------------
    // sys->eth_tx active configuration bus
    //   Besides the normal UDP data-plane destination, the bus now also carries
    //   the effective PTP peer destination used by Delay_Req TX. In legacy mode
    //   the effective peer simply aliases eth_dst_*; in split mode it uses the
    //   dedicated ptp_peer_* shadow bank committed in the same epoch.
    // ------------------------------------------------------------------------
    localparam integer C_ETH_CFG_W = 542;
    wire [C_ETH_CFG_W-1:0] eth_cfg_bus_sys_w;
    wire [C_ETH_CFG_W-1:0] eth_cfg_bus_eth_w;
    wire                   eth_cfg_ack_toggle_sys_w;
    wire                   eth_cfg_update_pulse_eth_w;
    wire [47:0]            ptp_peer_mac_effective_active_w;
    wire [31:0]            ptp_peer_ip_effective_active_w;
    wire [47:0]            ptp_peer_mac_effective_commit_w;
    wire [31:0]            ptp_peer_ip_effective_commit_w;

    assign ptp_peer_mac_effective_active_w =
        ptp_addr_split_enable_active_r ? ptp_peer_mac_active_r : eth_dst_mac_active_r;
    assign ptp_peer_ip_effective_active_w =
        ptp_addr_split_enable_active_r ? ptp_peer_ip_active_r : eth_dst_ip_active_r;
    assign ptp_peer_mac_effective_commit_w =
        cfg_ptp_addr_split_enable ? cfg_ptp_peer_mac : cfg_eth_dst_mac;
    assign ptp_peer_ip_effective_commit_w =
        cfg_ptp_addr_split_enable ? cfg_ptp_peer_ip : cfg_eth_dst_ip;

    assign eth_cfg_bus_sys_w = {
        txs_late_tol_ns_active_r,
        txs_advance_ns_active_r,
        txs_slot_width_ns_active_r,
        txs_slot_offset_ns_active_r,
        txs_period_ns_active_r,
        txs_base_time_ns_active_r,
        txs_strict_window_active_r,
        txs_block_when_time_invalid_active_r,
        txs_enable_active_r,
        eth_ptp_domain_active_r,
        eth_ptp_two_step_active_r,
        eth_ptp_enable_active_r,
        eth_ip_ident_seed_active_r,
        eth_dscp_ecn_active_r,
        eth_ttl_active_r,
        eth_dst_port_active_r,
        eth_src_port_active_r,
        ptp_peer_ip_effective_active_w,
        eth_dst_ip_active_r,
        eth_local_ip_active_r,
        ptp_peer_mac_effective_active_w,
        eth_dst_mac_active_r,
        eth_local_mac_active_r,
        eth_backend_enable_active_r
    };

    assign {
        txs_cfg_late_tol_ns_eth,
        txs_cfg_advance_ns_eth,
        txs_cfg_slot_width_ns_eth,
        txs_cfg_slot_offset_ns_eth,
        txs_cfg_period_ns_eth,
        txs_cfg_base_time_ns_eth,
        txs_cfg_strict_window_eth,
        txs_cfg_block_when_time_invalid_eth,
        txs_cfg_enable_eth,
        eth_cfg_ptp_domain_eth,
        eth_cfg_ptp_two_step_eth,
        eth_cfg_ptp_enable_eth,
        eth_cfg_ip_ident_seed_eth,
        eth_cfg_dscp_ecn_eth,
        eth_cfg_ttl_eth,
        eth_cfg_dst_port_eth,
        eth_cfg_src_port_eth,
        eth_cfg_ptp_peer_ip_eth,
        eth_cfg_dst_ip_eth,
        eth_cfg_local_ip_eth,
        eth_cfg_ptp_peer_mac_eth,
        eth_cfg_dst_mac_eth,
        eth_cfg_local_mac_eth,
        eth_cfg_enable_eth
    } = eth_cfg_bus_eth_w;

    assign eth_cfg_update_pulse_eth = eth_cfg_update_pulse_eth_w;

    net_acq_cfg_cdc_bridge #(
        .P_CFG_W   (C_ETH_CFG_W),
        .P_INIT_CFG({
            32'd0,
            32'd0,
            32'd0,
            32'd0,
            32'd0,
            64'd0,
            1'b1,
            1'b1,
            1'b0,
            8'd0,
            1'b0,
            1'b0,
            16'd0,
            8'd0,
            8'd64,
            16'd0,
            16'd0,
            32'd0,
            32'd0,
            32'd0,
            48'd0,
            48'd0,
            48'd0,
            1'b0
        })
    ) u_net_acq_cfg_cdc_bridge (
        .sys_clk              (clk_in),
        .sys_resetn           (resetn),
        .sys_cfg_bus          (eth_cfg_bus_sys_w),
        .sys_req_toggle       (eth_cfg_req_toggle_r),
        .sys_ack_toggle       (eth_cfg_ack_toggle_sys_w),
        .eth_clk              (eth_tx_clk),
        .eth_resetn           (eth_tx_resetn),
        .eth_cfg_bus          (eth_cfg_bus_eth_w),
        .eth_cfg_update_pulse (eth_cfg_update_pulse_eth_w)
    );

    // ------------------------------------------------------------------------
    // sys->eth_rx PTP match configuration bus
    //   RX matching always receives the EFFECTIVE PTP peer view. This keeps the
    //   existing parser/mailbox contract unchanged while allowing the normal UDP
    //   data-plane destination to diverge from the PTP observe/peer destination.
    // ------------------------------------------------------------------------
    localparam integer C_RX_MATCH_CFG_W = 169;
    reg  [C_RX_MATCH_CFG_W-1:0] rx_match_cfg_bus_sys_r;
    reg                         rx_match_cfg_req_toggle_r;
    reg                         rx_match_cfg_ack_toggle_seen_r;
    reg                         rx_match_cfg_pending_r;
    wire                        rx_match_cfg_ack_toggle_sys_w;
    wire [C_RX_MATCH_CFG_W-1:0] rx_match_cfg_bus_eth_w;
    wire                        rx_match_cfg_update_pulse_eth_w;

    assign {
        rx_match_ptp_enable_eth,
        rx_match_ptp_domain_eth,
        rx_match_host_ip_eth,
        rx_match_local_ip_eth,
        rx_match_host_mac_eth,
        rx_match_local_mac_eth
    } = rx_match_cfg_bus_eth_w;

    net_acq_cfg_cdc_bridge #(
        .P_CFG_W   (C_RX_MATCH_CFG_W),
        .P_INIT_CFG({
            1'b0,
            8'd0,
            32'd0,
            32'd0,
            48'd0,
            48'd0
        })
    ) u_ptp_rx_match_cfg_cdc_bridge (
        .sys_clk              (clk_in),
        .sys_resetn           (resetn),
        .sys_cfg_bus          (rx_match_cfg_bus_sys_r),
        .sys_req_toggle       (rx_match_cfg_req_toggle_r),
        .sys_ack_toggle       (rx_match_cfg_ack_toggle_sys_w),
        .eth_clk              (eth_rx_clk),
        .eth_resetn           (eth_rx_resetn),
        .eth_cfg_bus          (rx_match_cfg_bus_eth_w),
        .eth_cfg_update_pulse (rx_match_cfg_update_pulse_eth_w)
    );

    // ------------------------------------------------------------------------
    // Pulse CDCs
    // ------------------------------------------------------------------------
    net_acq_pulse_cdc u_net_acq_soft_reset_pulse_cdc (
        .src_clk    (clk_in),
        .src_resetn (resetn),
        .src_pulse  (cfg_soft_reset_pulse),
        .dst_clk    (eth_tx_clk),
        .dst_resetn (eth_tx_resetn),
        .dst_pulse  (eth_soft_reset_pulse_eth)
    );

    net_acq_pulse_cdc u_net_acq_clear_status_pulse_cdc (
        .src_clk    (clk_in),
        .src_resetn (resetn),
        .src_pulse  (cfg_clear_status_pulse),
        .dst_clk    (eth_tx_clk),
        .dst_resetn (eth_tx_resetn),
        .dst_pulse  (eth_clear_status_pulse_eth)
    );

    net_acq_pulse_cdc u_net_acq_ptp_tx_abort_pulse_cdc (
        .src_clk    (clk_in),
        .src_resetn (resetn),
        .src_pulse  (cfg_ptp_tx_abort_pulse),
        .dst_clk    (eth_tx_clk),
        .dst_resetn (eth_tx_resetn),
        .dst_pulse  (eth_ptp_tx_abort_pulse_eth)
    );

    net_acq_pulse_cdc u_net_acq_rx_soft_reset_pulse_cdc (
        .src_clk    (clk_in),
        .src_resetn (resetn),
        .src_pulse  (cfg_soft_reset_pulse),
        .dst_clk    (eth_rx_clk),
        .dst_resetn (eth_rx_resetn),
        .dst_pulse  (eth_rx_soft_reset_pulse_eth)
    );

    net_acq_pulse_cdc u_net_acq_rx_clear_status_pulse_cdc (
        .src_clk    (clk_in),
        .src_resetn (resetn),
        .src_pulse  (cfg_clear_status_pulse),
        .dst_clk    (eth_rx_clk),
        .dst_resetn (eth_rx_resetn),
        .dst_pulse  (eth_rx_clear_status_pulse_eth)
    );

    net_acq_pulse_cdc u_phy_soft_reset_cdc (
        .src_clk    (clk_in),
        .src_resetn (resetn),
        .src_pulse  (cfg_phy_soft_reset_pulse),
        .dst_clk    (eth_tx_clk),
        .dst_resetn (eth_tx_resetn),
        .dst_pulse  (phy_soft_reset_pulse)
    );

    net_acq_pulse_cdc u_phy_restart_an_cdc (
        .src_clk    (clk_in),
        .src_resetn (resetn),
        .src_pulse  (cfg_phy_restart_an_pulse),
        .dst_clk    (eth_tx_clk),
        .dst_resetn (eth_tx_resetn),
        .dst_pulse  (phy_restart_an_pulse)
    );

    // ------------------------------------------------------------------------
    // eth->sys status CDC
    // ------------------------------------------------------------------------
    wire                eth_status_update_pulse_sys_w;
    wire                eth_stat_busy_combined_eth_w;
    wire                eth_stat_tx_busy_combined_eth_w;

    assign eth_stat_busy_combined_eth_w    = eth_backend_stat_busy_eth | ptp_path_busy_eth;
    assign eth_stat_tx_busy_combined_eth_w = eth_backend_stat_tx_busy_eth | ptp_path_busy_eth;

    net_acq_status_cdc_bridge u_net_acq_status_cdc_bridge (
        .eth_clk                       (eth_tx_clk),
        .eth_resetn                    (eth_tx_resetn),
        .eth_stat_busy                 (eth_stat_busy_combined_eth_w),
        .eth_stat_tx_busy              (eth_stat_tx_busy_combined_eth_w),
        .eth_stat_tx_packet_count      (eth_backend_stat_tx_frame_count_eth),
        .eth_stat_tx_byte_count        (eth_backend_stat_tx_byte_count_eth),
        .eth_stat_drop_count           (eth_backend_stat_drop_count_eth),
        .eth_stat_last_ip_total_length (eth_backend_stat_last_ip_total_length_eth),
        .eth_stat_last_udp_length      (eth_backend_stat_last_udp_length_eth),
        .eth_stat_last_error_code      (eth_backend_stat_last_error_code_eth),
        .eth_phy_status_word           (phy_status_word),
        .eth_phy_reg_snapshot          (phy_reg_snapshot),
        .sys_clk                       (clk_in),
        .sys_resetn                    (resetn),
        .sys_stat_busy                 (eth_backend_stat_busy_sys),
        .sys_stat_tx_busy              (eth_backend_stat_tx_busy_sys),
        .sys_stat_tx_packet_count      (eth_backend_stat_tx_frame_count_sys),
        .sys_stat_tx_byte_count        (eth_backend_stat_tx_byte_count_sys),
        .sys_stat_drop_count           (eth_backend_stat_drop_count_sys),
        .sys_stat_last_ip_total_length (eth_backend_stat_last_ip_total_length_sys),
        .sys_stat_last_udp_length      (eth_backend_stat_last_udp_length_sys),
        .sys_stat_last_error_code      (eth_backend_stat_last_error_code_sys),
        .sys_phy_status_word           (phy_status_word_sys),
        .sys_phy_reg_snapshot          (phy_reg_snapshot_sys),
        .sys_status_update_pulse       (eth_status_update_pulse_sys_w)
    );

    // ------------------------------------------------------------------------
    // mac_link_up synchronized to sys domain for CSR visibility.
    // ------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE", SHREG_EXTRACT = "NO" *) reg [1:0] mac_link_up_sync_sys_r;
    assign mac_link_up_sys = mac_link_up_sync_sys_r[1];

    always @(posedge clk_in or negedge resetn) begin
        if (!resetn)
            mac_link_up_sync_sys_r <= 2'b00;
        else
            mac_link_up_sync_sys_r <= {mac_link_up_sync_sys_r[0], mac_link_up};
    end

    // ------------------------------------------------------------------------
    // Sticky error synthesis in sys domain.
    // ------------------------------------------------------------------------
    reg        eth_tx_error_sticky_r;
    reg        eth_pkt_buf_overflow_sticky_r;
    reg        eth_mac_stall_timeout_sticky_r;
    reg [15:0] eth_last_error_code_seen_r;

    assign eth_tx_error_sticky         = eth_tx_error_sticky_r;
    assign eth_pkt_buf_overflow_sticky = eth_pkt_buf_overflow_sticky_r;
    assign eth_mac_stall_timeout_sticky= eth_mac_stall_timeout_sticky_r;

    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            eth_tx_error_sticky_r          <= 1'b0;
            eth_pkt_buf_overflow_sticky_r  <= 1'b0;
            eth_mac_stall_timeout_sticky_r <= 1'b0;
            eth_last_error_code_seen_r     <= 16'd0;
        end else if (cfg_soft_reset_pulse || cfg_clear_status_pulse) begin
            eth_tx_error_sticky_r          <= 1'b0;
            eth_pkt_buf_overflow_sticky_r  <= 1'b0;
            eth_mac_stall_timeout_sticky_r <= 1'b0;
            eth_last_error_code_seen_r     <= eth_backend_stat_last_error_code_sys;
        end else begin
            if (eth_backend_stat_last_error_code_sys != eth_last_error_code_seen_r) begin
                eth_last_error_code_seen_r <= eth_backend_stat_last_error_code_sys;

                if (eth_backend_stat_last_error_code_sys != 16'd0)
                    eth_tx_error_sticky_r <= 1'b1;

                if (eth_backend_stat_last_error_code_sys == 16'd1)
                    eth_pkt_buf_overflow_sticky_r <= 1'b1;

                // stall timeout source is still reserved in the current RTL.
            end
        end
    end

    // ------------------------------------------------------------------------
    // Safe shadow->active commit FSM.
    // ------------------------------------------------------------------------
    wire eth_cfg_ack_seen_match_w;
    wire tx_cfg_ack_pulse_sys_w;
    wire rx_cfg_ack_pulse_sys_w;
    wire eth_tx_cfg_idle_w;
    wire eth_rx_cfg_idle_w;
    wire eth_commit_request_w;
    wire eth_can_commit_w;
    wire eth_commit_launch_w;
    wire unused_cfg_bits_w;

    // tx config CDC is idle only when no unseen ACK is pending and there is no
    // previously launched commit waiting for the tx-domain bridge to acknowledge.
    assign eth_cfg_ack_seen_match_w = (eth_cfg_ack_toggle_sys_w == eth_cfg_ack_toggle_seen_r);
    assign tx_cfg_ack_pulse_sys_w   = (eth_cfg_ack_toggle_sys_w != eth_cfg_ack_toggle_seen_r);
    assign rx_cfg_ack_pulse_sys_w   = (rx_match_cfg_ack_toggle_sys_w != rx_match_cfg_ack_toggle_seen_r);
    assign eth_tx_cfg_idle_w        = eth_cfg_ack_seen_match_w && (!eth_commit_tx_inflight_r);
    assign eth_rx_cfg_idle_w        = (!rx_match_cfg_pending_r) && (!eth_commit_rx_inflight_r);

    // cfg_active now means: there is no queued commit request and the last
    // launched ETH commit has been ACKed by BOTH the eth_tx config bridge and
    // the eth_rx match-config bridge. Standalone PTP-triggered rx-match refresh
    // pulses do not de-assert cfg_active; only ETH commit epochs do.
    assign eth_commit_request_w = cfg_eth_commit_pulse || eth_commit_pending_r;
    assign eth_can_commit_w     = (!cfg_enable) &&
                                  (!core_stat_busy) &&
                                  cdc_fifo_empty_sys &&
                                  (!eth_backend_stat_busy_sys) &&
                                  eth_tx_cfg_idle_w &&
                                  eth_rx_cfg_idle_w;
    assign eth_commit_launch_w  = eth_commit_request_w && eth_can_commit_w;

    assign eth_backend_selected        = eth_backend_enable_active_r;
    assign eth_tx_mirror_enable_active = eth_tx_mirror_enable_active_r;
    assign eth_cfg_active              = (!cfg_eth_commit_pulse) &&
                                         (!eth_commit_pending_r) &&
                                         (!eth_commit_tx_inflight_r) &&
                                         (!eth_commit_rx_inflight_r);

    assign txs_active_enable_sys                  = txs_enable_active_r;
    assign txs_active_block_when_time_invalid_sys = txs_block_when_time_invalid_active_r;
    assign txs_active_strict_window_sys           = txs_strict_window_active_r;
    assign txs_active_base_time_ns_sys            = txs_base_time_ns_active_r;
    assign txs_active_period_ns_sys               = txs_period_ns_active_r;
    assign txs_active_slot_offset_ns_sys          = txs_slot_offset_ns_active_r;
    assign txs_active_slot_width_ns_sys           = txs_slot_width_ns_active_r;
    assign txs_active_advance_ns_sys              = txs_advance_ns_active_r;
    assign txs_active_late_tol_ns_sys             = txs_late_tol_ns_active_r;

    // Keep planning bits visibly preserved in the active register bank even
    // though the current downstream datapath does not consume them yet.
    assign unused_cfg_bits_w = eth_append_fcs_active_r |
                               eth_udp_checksum_enable_active_r |
                               eth_stall_drop_enable_active_r |
                               eth_cfg_update_pulse_eth_w |
                               rx_match_cfg_update_pulse_eth_w |
                               eth_status_update_pulse_sys_w;

    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            eth_backend_enable_active_r      <= 1'b0;
            eth_tx_mirror_enable_active_r    <= 1'b1;
            eth_append_fcs_active_r          <= 1'b0;
            eth_udp_checksum_enable_active_r <= 1'b0;
            eth_stall_drop_enable_active_r   <= 1'b0;
            eth_local_mac_active_r           <= 48'd0;
            eth_dst_mac_active_r             <= 48'd0;
            eth_local_ip_active_r            <= 32'd0;
            eth_dst_ip_active_r              <= 32'd0;
            ptp_addr_split_enable_active_r   <= 1'b0;
            ptp_peer_mac_active_r            <= 48'd0;
            ptp_peer_ip_active_r             <= 32'd0;
            eth_src_port_active_r            <= 16'd0;
            eth_dst_port_active_r            <= 16'd0;
            eth_ttl_active_r                 <= 8'd64;
            eth_dscp_ecn_active_r            <= 8'd0;
            eth_ip_ident_seed_active_r       <= 16'd0;
            eth_ptp_enable_active_r          <= 1'b0;
            eth_ptp_two_step_active_r        <= 1'b0;
            eth_ptp_domain_active_r          <= 8'd0;
            txs_enable_active_r              <= 1'b0;
            txs_block_when_time_invalid_active_r <= 1'b1;
            txs_strict_window_active_r       <= 1'b1;
            txs_base_time_ns_active_r        <= 64'd0;
            txs_period_ns_active_r           <= 32'd0;
            txs_slot_offset_ns_active_r      <= 32'd0;
            txs_slot_width_ns_active_r       <= 32'd0;
            txs_advance_ns_active_r          <= 32'd0;
            txs_late_tol_ns_active_r         <= 32'd0;
            eth_commit_pending_r             <= 1'b0;
            eth_commit_tx_inflight_r         <= 1'b0;
            eth_commit_rx_inflight_r         <= 1'b0;
            eth_cfg_req_toggle_r             <= 1'b0;
            eth_cfg_ack_toggle_seen_r        <= 1'b0;
        end else begin
            // tx-domain config ACK closes the tx side of the current ETH commit.
            if (tx_cfg_ack_pulse_sys_w) begin
                eth_cfg_ack_toggle_seen_r <= eth_cfg_ack_toggle_sys_w;
                eth_commit_tx_inflight_r  <= 1'b0;
            end

            // rx-domain match-config ACK closes the rx side of the current ETH commit.
            if (rx_cfg_ack_pulse_sys_w && eth_commit_rx_inflight_r) begin
                eth_commit_rx_inflight_r <= 1'b0;
            end

            // Queue a software commit request until it can be launched. If the
            // request is launched in the same cycle, the queue is consumed.
            eth_commit_pending_r <= (!eth_commit_launch_w) &&
                                    (eth_commit_pending_r || cfg_eth_commit_pulse);

            if (eth_commit_launch_w) begin
                // Commit the full ETH shadow bank into the sys-domain active bank.
                // cfg_active will stay low until both tx/rx CDC bridges ACK this epoch.
                eth_backend_enable_active_r      <= cfg_eth_backend_enable;
                eth_tx_mirror_enable_active_r    <= cfg_eth_tx_mirror_enable;
                eth_append_fcs_active_r          <= cfg_eth_append_fcs;
                eth_udp_checksum_enable_active_r <= cfg_eth_udp_checksum_enable;
                eth_stall_drop_enable_active_r   <= cfg_eth_stall_drop_enable;
                eth_local_mac_active_r           <= cfg_eth_local_mac;
                eth_dst_mac_active_r             <= cfg_eth_dst_mac;
                eth_local_ip_active_r            <= cfg_eth_local_ip;
                eth_dst_ip_active_r              <= cfg_eth_dst_ip;
                ptp_addr_split_enable_active_r   <= cfg_ptp_addr_split_enable;
                ptp_peer_mac_active_r            <= cfg_ptp_peer_mac;
                ptp_peer_ip_active_r             <= cfg_ptp_peer_ip;
                eth_src_port_active_r            <= cfg_eth_src_port;
                eth_dst_port_active_r            <= cfg_eth_dst_port;
                eth_ttl_active_r                 <= cfg_eth_ttl;
                eth_dscp_ecn_active_r            <= cfg_eth_dscp_ecn;
                eth_ip_ident_seed_active_r       <= cfg_eth_ip_ident_seed;
                eth_ptp_enable_active_r          <= cfg_ptp_enable;
                eth_ptp_two_step_active_r        <= cfg_ptp_two_step;
                eth_ptp_domain_active_r          <= cfg_ptp_domain;
                txs_enable_active_r              <= cfg_txs_enable;
                txs_block_when_time_invalid_active_r <= cfg_txs_block_when_time_invalid;
                txs_strict_window_active_r       <= cfg_txs_strict_window;
                txs_base_time_ns_active_r        <= cfg_txs_base_time_ns;
                txs_period_ns_active_r           <= cfg_txs_period_ns;
                txs_slot_offset_ns_active_r      <= cfg_txs_slot_offset_ns;
                txs_slot_width_ns_active_r       <= cfg_txs_slot_width_ns;
                txs_advance_ns_active_r          <= cfg_txs_advance_ns;
                txs_late_tol_ns_active_r         <= cfg_txs_late_tol_ns;

                eth_cfg_req_toggle_r             <= ~eth_cfg_req_toggle_r;
                eth_commit_tx_inflight_r         <= 1'b1;
                eth_commit_rx_inflight_r         <= 1'b1;
            end
        end
    end

    // ------------------------------------------------------------------------
    // sys->eth_rx match-config updates.
    // For an ETH commit, the RX match update is launched in the SAME epoch as
    // the tx active-config update, and cfg_active is only allowed to return high
    // after this side also ACKs. Standalone PTP refreshes keep the old behavior.
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            rx_match_cfg_bus_sys_r         <= {C_RX_MATCH_CFG_W{1'b0}};
            rx_match_cfg_req_toggle_r      <= 1'b0;
            rx_match_cfg_ack_toggle_seen_r <= 1'b0;
            rx_match_cfg_pending_r         <= 1'b0;
        end else begin
            if (rx_match_cfg_ack_toggle_sys_w != rx_match_cfg_ack_toggle_seen_r) begin
                rx_match_cfg_ack_toggle_seen_r <= rx_match_cfg_ack_toggle_sys_w;
                rx_match_cfg_pending_r         <= 1'b0;
            end

            if (!rx_match_cfg_pending_r) begin
                if (eth_commit_launch_w) begin
                    rx_match_cfg_bus_sys_r <= {
                        cfg_ptp_enable,
                        cfg_ptp_domain,
                        ptp_peer_ip_effective_commit_w,
                        cfg_eth_local_ip,
                        ptp_peer_mac_effective_commit_w,
                        cfg_eth_local_mac
                    };
                    rx_match_cfg_req_toggle_r <= ~rx_match_cfg_req_toggle_r;
                    rx_match_cfg_pending_r    <= 1'b1;
                end else if (cfg_ptp_update_pulse) begin
                    // Standalone PTP refreshes reuse the CURRENT active effective
                    // peer selection. Uncommitted ptp_peer_* shadow values never
                    // cross domains until the normal ETH safe-commit launches.
                    rx_match_cfg_bus_sys_r <= {
                        cfg_ptp_enable,
                        cfg_ptp_domain,
                        ptp_peer_ip_effective_active_w,
                        eth_local_ip_active_r,
                        ptp_peer_mac_effective_active_w,
                        eth_local_mac_active_r
                    };
                    rx_match_cfg_req_toggle_r <= ~rx_match_cfg_req_toggle_r;
                    rx_match_cfg_pending_r    <= 1'b1;
                end
            end
        end
    end

endmodule
