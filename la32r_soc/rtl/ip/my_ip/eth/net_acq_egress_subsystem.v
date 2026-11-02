`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_egress_subsystem
// Description:
//   Consolidates the Ethernet egress data path in eth_tx_clk domain:
//     1) acquisition packet -> UDP/IPv4 frame generation
//     2) TSN-lite v1 slot-based scheduler on the normal data path
//     3) PTP frame arbitration against data traffic
//     4) final MAC-side AXIS-like output adaptation
//
//   Architectural intent:
//     - Keep data UDP framing and frame arbitration in one dedicated egress
//       subsystem so deterministic egress control stays localized.
//     - TSN-lite v1 is inserted only on the normal data path; PTP traffic keeps
//       the existing compact path and arbitration contract.
//////////////////////////////////////////////////////////////////////////////////
module net_acq_egress_subsystem(
    input  wire         eth_tx_clk,
    input  wire         eth_tx_resetn,

    // ------------------------------------------------------------------------
    // Common egress control (eth_tx domain)
    // ------------------------------------------------------------------------
    input  wire         eth_soft_reset_pulse,
    input  wire         eth_clear_status_pulse,

    // ------------------------------------------------------------------------
    // Active Ethernet data-plane configuration (eth_tx domain)
    // ------------------------------------------------------------------------
    input  wire         eth_cfg_enable,
    input  wire [47:0]  eth_cfg_local_mac,
    input  wire [47:0]  eth_cfg_dst_mac,
    input  wire [31:0]  eth_cfg_local_ip,
    input  wire [31:0]  eth_cfg_dst_ip,
    input  wire [15:0]  eth_cfg_src_port,
    input  wire [15:0]  eth_cfg_dst_port,
    input  wire [7:0]   eth_cfg_ttl,
    input  wire [7:0]   eth_cfg_dscp_ecn,
    input  wire [15:0]  eth_cfg_ip_ident_seed,
    input  wire         eth_cfg_ptp_enable,
    input  wire         eth_cfg_ptp_two_step,
    input  wire [7:0]   eth_cfg_ptp_domain,

    // ------------------------------------------------------------------------
    // Active TSN-lite v1 scheduler configuration (eth_tx domain)
    // ------------------------------------------------------------------------
    input  wire         eth_cfg_update_pulse,
    input  wire         txs_cfg_enable,
    input  wire         txs_cfg_block_when_time_invalid,
    input  wire         txs_cfg_strict_window,
    input  wire [63:0]  txs_cfg_base_time_ns,
    input  wire [31:0]  txs_cfg_period_ns,
    input  wire [31:0]  txs_cfg_slot_offset_ns,
    input  wire [31:0]  txs_cfg_slot_width_ns,
    input  wire [31:0]  txs_cfg_advance_ns,
    input  wire [31:0]  txs_cfg_late_tol_ns,
    input  wire [63:0]  txs_tod_time_ns,
    input  wire         txs_tod_time_valid,

    // ------------------------------------------------------------------------
    // PTP arbitration control (eth_tx domain)
    // ------------------------------------------------------------------------
    input  wire         ptp_cfg_enable,
    input  wire         ptp_cfg_strict_priority,
    input  wire         ptp_cfg_event_priority_enable,

    // ------------------------------------------------------------------------
    // Data packet stream from sys->eth CDC FIFO
    // ------------------------------------------------------------------------
    input  wire [15:0]  cdc_pkt_data,
    input  wire         cdc_pkt_valid,
    input  wire         cdc_pkt_last,
    output wire         cdc_pkt_ready,

    // ------------------------------------------------------------------------
    // Pre-built PTP Ethernet-byte stream from PTP subsystem
    // ------------------------------------------------------------------------
    input  wire [7:0]   ptp_tx_tdata,
    input  wire         ptp_tx_tvalid,
    input  wire         ptp_tx_tlast,
    output wire         ptp_tx_tready,
    input  wire [3:0]   ptp_tx_tuser,

    // ------------------------------------------------------------------------
    // Final MAC-side AXIS-like egress output
    // ------------------------------------------------------------------------
    output wire [7:0]   m_axis_eth_tx_tdata,
    output wire         m_axis_eth_tx_tvalid,
    output wire         m_axis_eth_tx_tlast,
    input  wire         m_axis_eth_tx_tready,
    output wire [3:0]   m_axis_eth_tx_tuser,

    // ------------------------------------------------------------------------
    // Backend-only statistics (eth_tx domain)
    // ------------------------------------------------------------------------
    output wire         eth_backend_stat_busy,
    output wire         eth_backend_stat_ingress_busy,
    output wire         eth_backend_stat_tx_busy,
    output wire [31:0]  eth_backend_stat_tx_frame_count,
    output wire [31:0]  eth_backend_stat_tx_byte_count,
    output wire [31:0]  eth_backend_stat_drop_count,
    output wire [31:0]  eth_backend_stat_backpressure_cycles,
    output wire [15:0]  eth_backend_stat_last_ip_total_length,
    output wire [15:0]  eth_backend_stat_last_udp_length,
    output wire [15:0]  eth_backend_stat_last_error_code,

    // ------------------------------------------------------------------------
    // Pre-scheduler observability extension (eth_tx domain)
    // ------------------------------------------------------------------------
    output wire [31:0]  dbg_arb_data_frame_count,
    output wire [31:0]  dbg_arb_ptp_frame_count,
    output wire [31:0]  dbg_arb_stall_cycles,
    output wire [15:0]  dbg_arb_last_error,

    // ------------------------------------------------------------------------
    // TSN-lite v1 scheduler observability (eth_tx domain)
    // ------------------------------------------------------------------------
    output wire [31:0]  txs_stat_status,
    output wire [31:0]  txs_stat_held_count,
    output wire [31:0]  txs_stat_pass_count,
    output wire [31:0]  txs_stat_late_count,
    output wire [31:0]  txs_stat_slot_miss_count,
    output wire [63:0]  txs_stat_last_plan_ns,
    output wire [63:0]  txs_stat_last_actual_ns,
    output wire [31:0]  txs_stat_last_delta_ns
);

    wire [7:0]  data_m_tx_data_w;
    wire        data_m_tx_valid_w;
    wire        data_m_tx_last_w;
    wire        data_m_tx_start_w;
    wire        data_m_tx_ready_w;

    wire [7:0]  sched_m_tx_data_w;
    wire        sched_m_tx_valid_w;
    wire        sched_m_tx_last_w;
    wire        sched_m_tx_ready_w;
    wire [3:0]  sched_m_tx_tuser_w;

    wire [7:0]  mux_m_tx_data_w;
    wire        mux_m_tx_valid_w;
    wire        mux_m_tx_last_w;
    wire [3:0]  mux_m_tx_tuser_w;
    wire        tx_arbiter_busy_w;
    wire [31:0] dbg_arb_data_frame_count_w;
    wire [31:0] dbg_arb_ptp_frame_count_w;
    wire [31:0] dbg_arb_stall_cycles_w;
    wire [15:0] dbg_arb_last_error_w;

    wire [31:0] txs_stat_status_w;
    wire [31:0] txs_stat_held_count_w;
    wire [31:0] txs_stat_pass_count_w;
    wire [31:0] txs_stat_late_count_w;
    wire [31:0] txs_stat_slot_miss_count_w;
    wire [63:0] txs_stat_last_plan_ns_w;
    wire [63:0] txs_stat_last_actual_ns_w;
    wire [31:0] txs_stat_last_delta_ns_w;

    wire        eth_tx_path_enable_w;
    wire        eth_axis_tvalid_w;
    wire        eth_axis_tlast_w;
    wire        eth_axis_fire_w;
    reg         eth_axis_frame_active_r;
    wire        ptp_tx_ts_ready_unused_w;

    assign eth_tx_path_enable_w = eth_cfg_enable | ptp_cfg_enable;
    assign eth_axis_tvalid_w    = eth_tx_path_enable_w ? mux_m_tx_valid_w : 1'b0;
    assign eth_axis_tlast_w     = eth_tx_path_enable_w ? mux_m_tx_last_w  : 1'b0;
    assign eth_axis_fire_w      = eth_axis_tvalid_w && m_axis_eth_tx_tready;

    // ------------------------------------------------------------------------
    // Existing acquisition-data UDP/IPv4 transmit backend.
    // ------------------------------------------------------------------------
    net_acq_eth_backend_top #(
        .P_PKT_W (16),
        .P_MAC_W (8)
    ) u_net_acq_eth_backend_top (
        .clk                      (eth_tx_clk),
        .rst_n                    (eth_tx_resetn),
        .cfg_enable               (eth_cfg_enable),
        .cfg_soft_reset           (eth_soft_reset_pulse),
        .cfg_clear_status         (eth_clear_status_pulse),
        .s_pkt_data               (cdc_pkt_data),
        .s_pkt_valid              (cdc_pkt_valid),
        .s_pkt_last               (cdc_pkt_last),
        .s_pkt_ready              (cdc_pkt_ready),
        .s_meta_valid             (1'b0),
        .s_meta_ready             (),
        .s_meta_seq_id            (32'd0),
        .s_meta_channel_id        (16'd0),
        .s_meta_sample_count      (16'd0),
        .s_meta_timestamp         (64'd0),
        .cfg_dst_mac              (eth_cfg_dst_mac),
        .cfg_src_mac              (eth_cfg_local_mac),
        .cfg_src_ip               (eth_cfg_local_ip),
        .cfg_dst_ip               (eth_cfg_dst_ip),
        .cfg_udp_src_port         (eth_cfg_src_port),
        .cfg_udp_dst_port         (eth_cfg_dst_port),
        .cfg_ip_ttl               (eth_cfg_ttl),
        .cfg_ip_dscp_ecn          (eth_cfg_dscp_ecn),
        .cfg_ip_ident_seed        (eth_cfg_ip_ident_seed),
        .cfg_ptp_enable           (eth_cfg_ptp_enable),
        .cfg_ptp_two_step         (eth_cfg_ptp_two_step),
        .cfg_ptp_udp_dst_port     (16'd0),
        .cfg_ptp_domain           (eth_cfg_ptp_domain),
        .cfg_ptp_message_type     (8'd0),
        .ptp_tx_ts_valid          (1'b0),
        .ptp_tx_ts_data           (80'd0),
        .ptp_tx_ts_ready          (ptp_tx_ts_ready_unused_w),
        .m_tx_data                (data_m_tx_data_w),
        .m_tx_valid               (data_m_tx_valid_w),
        .m_tx_last                (data_m_tx_last_w),
        .m_tx_ready               (data_m_tx_ready_w),
        .m_tx_start               (data_m_tx_start_w),
        .stat_busy                (eth_backend_stat_busy),
        .stat_ingress_busy        (eth_backend_stat_ingress_busy),
        .stat_tx_busy             (eth_backend_stat_tx_busy),
        .stat_tx_frame_count      (eth_backend_stat_tx_frame_count),
        .stat_tx_byte_count       (eth_backend_stat_tx_byte_count),
        .stat_drop_count          (eth_backend_stat_drop_count),
        .stat_backpressure_cycles (eth_backend_stat_backpressure_cycles),
        .stat_last_ip_total_length(eth_backend_stat_last_ip_total_length),
        .stat_last_udp_length     (eth_backend_stat_last_udp_length),
        .stat_last_error_code     (eth_backend_stat_last_error_code)
    );

    // ------------------------------------------------------------------------
    // TSN-lite v1 slot-based scheduler on the normal data path only.
    // ------------------------------------------------------------------------

    net_acq_data_tx_scheduler #(
        .P_DATA_W  (8),
        .P_TUSER_W (4)
    ) u_net_acq_data_tx_scheduler (
        .clk                              (eth_tx_clk),
        .rst_n                            (eth_tx_resetn),
        .cfg_enable                       (eth_cfg_enable),
        .cfg_soft_reset                   (eth_soft_reset_pulse),
        .cfg_clear_status                 (eth_clear_status_pulse),
        .cfg_sched_update_pulse           (eth_cfg_update_pulse),
        .cfg_sched_enable                 (txs_cfg_enable),
        .cfg_sched_block_when_time_invalid(txs_cfg_block_when_time_invalid),
        .cfg_sched_strict_window          (txs_cfg_strict_window),
        .cfg_base_time_ns                 (txs_cfg_base_time_ns),
        .cfg_period_ns                    (txs_cfg_period_ns),
        .cfg_slot_offset_ns               (txs_cfg_slot_offset_ns),
        .cfg_slot_width_ns                (txs_cfg_slot_width_ns),
        .cfg_advance_ns                   (txs_cfg_advance_ns),
        .cfg_late_tol_ns                  (txs_cfg_late_tol_ns),
        .tod_time_ns                      (txs_tod_time_ns),
        .tod_time_valid                   (txs_tod_time_valid),
        .s_tdata                          (data_m_tx_data_w),
        .s_tvalid                         (data_m_tx_valid_w),
        .s_tlast                          (data_m_tx_last_w),
        .s_tready                         (data_m_tx_ready_w),
        .s_tuser                          (4'b0000),
        .m_tdata                          (sched_m_tx_data_w),
        .m_tvalid                         (sched_m_tx_valid_w),
        .m_tlast                          (sched_m_tx_last_w),
        .m_tready                         (sched_m_tx_ready_w),
        .m_tuser                          (sched_m_tx_tuser_w),
        .stat_status                      (txs_stat_status_w),
        .stat_held_count                  (txs_stat_held_count_w),
        .stat_pass_count                  (txs_stat_pass_count_w),
        .stat_late_count                  (txs_stat_late_count_w),
        .stat_slot_miss_count             (txs_stat_slot_miss_count_w),
        .stat_last_plan_ns                (txs_stat_last_plan_ns_w),
        .stat_last_actual_ns              (txs_stat_last_actual_ns_w),
        .stat_last_delta_ns               (txs_stat_last_delta_ns_w)
    );

    // ------------------------------------------------------------------------
    // tx_arbiter remains the stable frame-level merge point for data/PTP.
    // ------------------------------------------------------------------------
    tx_arbiter #(
        .P_DATA_W  (8),
        .P_TUSER_W (4)
    ) u_tx_arbiter (
        .clk                         (eth_tx_clk),
        .rst_n                       (eth_tx_resetn),
        .cfg_enable                  (eth_tx_path_enable_w),
        .cfg_soft_reset              (eth_soft_reset_pulse),
        .cfg_clear_status            (eth_clear_status_pulse),
        .cfg_ptp_strict_priority     (ptp_cfg_strict_priority),
        .cfg_ptp_event_priority_enable(ptp_cfg_event_priority_enable),
        .s_data_tdata                (sched_m_tx_data_w),
        .s_data_tvalid               (sched_m_tx_valid_w),
        .s_data_tlast                (sched_m_tx_last_w),
        .s_data_tready               (sched_m_tx_ready_w),
        .s_data_tuser                (sched_m_tx_tuser_w),
        .s_ptp_tdata                 (ptp_tx_tdata),
        .s_ptp_tvalid                (ptp_tx_tvalid),
        .s_ptp_tlast                 (ptp_tx_tlast),
        .s_ptp_tready                (ptp_tx_tready),
        .s_ptp_tuser                 (ptp_tx_tuser),
        .m_tdata                     (mux_m_tx_data_w),
        .m_tvalid                    (mux_m_tx_valid_w),
        .m_tlast                     (mux_m_tx_last_w),
        .m_tready                    (m_axis_eth_tx_tready),
        .m_tuser                     (mux_m_tx_tuser_w),
        .busy                        (tx_arbiter_busy_w),
        .stat_data_frame_count       (dbg_arb_data_frame_count_w),
        .stat_ptp_frame_count        (dbg_arb_ptp_frame_count_w),
        .stat_stall_cycles           (dbg_arb_stall_cycles_w),
        .stat_last_error             (dbg_arb_last_error_w)
    );

    // ------------------------------------------------------------------------
    // Final MAC-side adaptation.
    // ------------------------------------------------------------------------
    always @(posedge eth_tx_clk or negedge eth_tx_resetn) begin
        if (!eth_tx_resetn) begin
            eth_axis_frame_active_r <= 1'b0;
        end else begin
            if (eth_soft_reset_pulse || !eth_tx_path_enable_w) begin
                eth_axis_frame_active_r <= 1'b0;
            end else if (eth_axis_fire_w) begin
                if (!eth_axis_frame_active_r)
                    eth_axis_frame_active_r <= !eth_axis_tlast_w;
                else if (eth_axis_tlast_w)
                    eth_axis_frame_active_r <= 1'b0;
            end
        end
    end

    assign m_axis_eth_tx_tdata      = eth_axis_tvalid_w ? mux_m_tx_data_w : 8'd0;
    assign m_axis_eth_tx_tvalid     = eth_axis_tvalid_w;
    assign m_axis_eth_tx_tlast      = eth_axis_tlast_w;
    assign m_axis_eth_tx_tuser[0]   = eth_axis_tvalid_w && !eth_axis_frame_active_r;
    assign m_axis_eth_tx_tuser[1]   = eth_axis_tvalid_w ? mux_m_tx_tuser_w[1]   : 1'b0;
    assign m_axis_eth_tx_tuser[3:2] = eth_axis_tvalid_w ? mux_m_tx_tuser_w[3:2] : 2'b00;

    assign dbg_arb_data_frame_count = dbg_arb_data_frame_count_w;
    assign dbg_arb_ptp_frame_count  = dbg_arb_ptp_frame_count_w;
    assign dbg_arb_stall_cycles     = dbg_arb_stall_cycles_w;
    assign dbg_arb_last_error       = dbg_arb_last_error_w;

    assign txs_stat_status          = txs_stat_status_w;
    assign txs_stat_held_count      = txs_stat_held_count_w;
    assign txs_stat_pass_count      = txs_stat_pass_count_w;
    assign txs_stat_late_count      = txs_stat_late_count_w;
    assign txs_stat_slot_miss_count = txs_stat_slot_miss_count_w;
    assign txs_stat_last_plan_ns    = txs_stat_last_plan_ns_w;
    assign txs_stat_last_actual_ns  = txs_stat_last_actual_ns_w;
    assign txs_stat_last_delta_ns   = txs_stat_last_delta_ns_w;

endmodule

`default_nettype wire
