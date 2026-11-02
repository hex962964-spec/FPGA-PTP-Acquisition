`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_wrapper
// Description:
//   Compatibility shell for the net_acq SoC-facing top-level wrapper.
//
//   External behavior is preserved, but the internal structure is regularized
//   into dedicated subsystems so future maintenance and feature upgrades are
//   easier to control:
//     - net_acq_csr               : unified CSR / software contract
//     - net_acq_frontend_subsystem: sys_clk front-end acquisition + debug sink
//     - net_acq_pkt_async_fifo    : sys->eth packet CDC bridge
//     - net_acq_eth_cfg_manager   : control plane / commit / CDC / status plane
//     - net_acq_ptp_subsystem     : compact PTP ToD / TX / RX mailbox logic
//     - net_acq_egress_subsystem  : data UDP backend + arbiter + MAC-side egress
//
//   Important compatibility notes:
//     1) Local CSR offsets, software-visible semantics, and wrapper ports are
//        intentionally kept unchanged.
//     2) The low->high destructive-read contract for PTP_TXSTAMP is preserved.
//     3) The future standard insertion point for deterministic egress timing
//        remains: data backend output -> scheduler -> tx_arbiter.
//////////////////////////////////////////////////////////////////////////////////
module net_acq_wrapper(
    // ------------------------------------------------------------------------
    // Clock / reset
    // ------------------------------------------------------------------------
    input  wire         clk_in,
    input  wire         resetn,
    input  wire         eth_tx_clk,
    input  wire         eth_tx_resetn,
    input  wire         eth_rx_clk,
    input  wire         eth_rx_resetn,

    // ------------------------------------------------------------------------
    // Local CSR-like interface (sys_clk domain)
    // ------------------------------------------------------------------------
    input  wire         csr_en,
    input  wire         csr_we,
    input  wire [11:0]  csr_addr,
    input  wire [31:0]  csr_wdata,
    output wire [31:0]  csr_rdata,
    output wire         csr_ready,

    // ------------------------------------------------------------------------
    // External sample input side (sys_clk domain)
    // ------------------------------------------------------------------------
    input  wire [15:0]  sample_data_in,
    input  wire         sample_valid_in,
    output wire         sample_ready_out,

    // ------------------------------------------------------------------------
    // Legacy packet mirror / bypass output side (sys_clk domain)
    // ------------------------------------------------------------------------
    output wire [15:0]  pkt_data,
    output wire         pkt_valid,
    output wire         pkt_last,
    input  wire         pkt_ready,

    // ------------------------------------------------------------------------
    // Board-independent MAC-side TX AXIS-like stream (eth_tx_clk domain)
    // ------------------------------------------------------------------------
    output wire [7:0]   m_axis_eth_tx_tdata,
    output wire         m_axis_eth_tx_tvalid,
    output wire         m_axis_eth_tx_tlast,
    input  wire         m_axis_eth_tx_tready,
    output wire [3:0]   m_axis_eth_tx_tuser,
    output wire [63:0]  ptp_tod_time_ns_eth,
    output wire         ptp_tod_time_valid_eth,

    // ------------------------------------------------------------------------
    // Board-independent MAC-side RX byte stream (eth_rx_clk domain)
    // ------------------------------------------------------------------------
    input  wire [7:0]   s_axis_eth_rx_tdata,
    input  wire         s_axis_eth_rx_tvalid,
    input  wire         s_axis_eth_rx_tlast,
    input  wire [63:0]  ptp_rxstamp_ns,
    input  wire         ptp_rxstamp_valid,
    output wire [63:0]  ptp_tod_time_ns_rx,
    output wire         ptp_tod_time_valid_rx,

    // ------------------------------------------------------------------------
    // Reserved future PTP sideband inputs
    // ------------------------------------------------------------------------
    input  wire [63:0]  tod_time_ns,
    input  wire         tod_time_valid,
    input  wire [63:0]  ptp_txstamp_ns,
    input  wire         ptp_txstamp_valid,

    // ------------------------------------------------------------------------
    // Optional board/link status input + PHY management sideband
    // ------------------------------------------------------------------------
    input  wire         mac_link_up,
    input  wire [31:0]  phy_status_word,
    input  wire [127:0] phy_reg_snapshot,
    output wire         phy_soft_reset_pulse,
    output wire         phy_restart_an_pulse
);

    // Local reset release trees: asynchronous assertion, two local clock edges
    // to release. Keep each bank distinct so implementation can place it near
    // its consumers; FIFO port resets and soft-clear protocols are unchanged.
    wire frontend_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_frontend_resetn (
        .clk(clk_in), .rst_n_in(resetn), .rst_n_out(frontend_resetn)
    );
    wire csr_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_csr_resetn (
        .clk(clk_in), .rst_n_in(resetn), .rst_n_out(csr_resetn)
    );
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
    wire egress_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_egress_resetn (
        .clk(eth_tx_clk), .rst_n_in(eth_tx_resetn), .rst_n_out(egress_resetn)
    );
    wire observe_sys_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_observe_sys_resetn (
        .clk(clk_in), .rst_n_in(resetn), .rst_n_out(observe_sys_resetn)
    );
    wire observe_tx_resetn;
    (* DONT_TOUCH = "TRUE" *) rst_sync u_rst_observe_tx_resetn (
        .clk(eth_tx_clk), .rst_n_in(eth_tx_resetn), .rst_n_out(observe_tx_resetn)
    );

    // ------------------------------------------------------------------------
    // CSR -> acquisition-core/front-end config (sys domain)
    // ------------------------------------------------------------------------
    wire                cfg_enable;
    wire                cfg_soft_reset_pulse;
    wire                cfg_clear_status_pulse;
    wire [31:0]         cfg_sample_period;
    wire [15:0]         cfg_samples_per_packet;
    wire [3:0]          cfg_mode;
    wire [15:0]         cfg_channel_id;
    wire                cfg_src_sel;

    // ------------------------------------------------------------------------
    // CSR -> Ethernet backend shadow config (sys domain)
    // ------------------------------------------------------------------------
    wire                cfg_eth_backend_enable;
    wire                cfg_eth_commit_pulse;
    wire                cfg_eth_tx_mirror_enable;
    wire                cfg_eth_append_fcs;
    wire                cfg_eth_udp_checksum_enable;
    wire                cfg_eth_stall_drop_enable;
    wire [47:0]         cfg_eth_local_mac;
    wire [47:0]         cfg_eth_dst_mac;
    wire [31:0]         cfg_eth_local_ip;
    wire [31:0]         cfg_eth_dst_ip;
    wire [15:0]         cfg_eth_src_port;
    wire [15:0]         cfg_eth_dst_port;
    wire [7:0]          cfg_eth_ttl;
    wire [7:0]          cfg_eth_dscp_ecn;
    wire [15:0]         cfg_eth_ip_ident_seed;
    // Dedicated PTP peer shadow bank. local identity remains shared.
    wire                cfg_ptp_addr_split_enable;
    wire [47:0]         cfg_ptp_peer_mac;
    wire [31:0]         cfg_ptp_peer_ip;
    wire                cfg_txs_enable;
    wire                cfg_txs_block_when_time_invalid;
    wire                cfg_txs_strict_window;
    wire [63:0]         cfg_txs_base_time_ns;
    wire [31:0]         cfg_txs_period_ns;
    wire [31:0]         cfg_txs_slot_offset_ns;
    wire [31:0]         cfg_txs_slot_width_ns;
    wire [31:0]         cfg_txs_advance_ns;
    wire [31:0]         cfg_txs_late_tol_ns;

    // ------------------------------------------------------------------------
    // CSR -> PTP shadow config (sys domain)
    // ------------------------------------------------------------------------
    wire                cfg_ptp_enable;
    wire                cfg_ptp_one_step;
    wire                cfg_ptp_two_step;
    wire                cfg_ptp_event_priority_enable;
    wire                cfg_ptp_hold;
    wire                cfg_ptp_strict_priority;
    wire [7:0]          cfg_ptp_domain;
    wire [63:0]         cfg_ptp_clock_id;
    wire [15:0]         cfg_ptp_port_num;
    wire                cfg_ptp_update_pulse;
    wire                cfg_ptp_set_time_pulse;
    wire                cfg_ptp_step_pulse;
    wire                cfg_ptp_tx_abort_pulse;
    wire                cfg_ptp_tx_trigger_pulse;
    wire [63:0]         cfg_ptp_set_time_ns;
    wire [31:0]         cfg_ptp_step_ns;
    wire [31:0]         cfg_ptp_freq_adj_ppb;
    wire [3:0]          cfg_ptp_tx_message_type;
    wire [3:0]          cfg_ptp_tx_transport_specific;
    wire [15:0]         cfg_ptp_tx_sequence_id;
    wire [15:0]         cfg_ptp_tx_flags;
    wire [7:0]          cfg_ptp_tx_log_message_interval;
    wire                cfg_ptp_tx_need_timestamp;
    wire [1:0]          cfg_time_quality;

    // ------------------------------------------------------------------------
    // PHY management control pulses from CSR (sys domain)
    // ------------------------------------------------------------------------
    wire                cfg_phy_soft_reset_pulse;
    wire                cfg_phy_restart_an_pulse;

    // ------------------------------------------------------------------------
    // Front-end subsystem outputs (sys domain)
    // ------------------------------------------------------------------------
    wire [15:0]         front_pkt_data_w;
    wire                front_pkt_valid_w;
    wire                front_pkt_last_w;
    wire                front_pkt_ready_w;
    wire                core_stat_busy;
    wire                core_stat_error;
    wire [31:0]         core_stat_fifo_level;
    wire [31:0]         core_stat_packet_count;
    wire [63:0]         core_stat_last_timestamp;
    wire                dbg_error_sticky;
    wire [31:0]         dbg_good_packet_count;
    wire [31:0]         dbg_bad_packet_count;
    wire [31:0]         dbg_last_seq_id;
    wire [63:0]         dbg_last_timestamp;
    wire [15:0]         dbg_last_sample_count;
    wire [15:0]         dbg_last_payload_count;
    wire [15:0]         dbg_last_error_code;
    wire [15:0]         dbg_last_error_word_index;
    wire [7:0]          dbg_last_protocol_version;
    wire [1:0]          dbg_last_quality;

    // ------------------------------------------------------------------------
    // Front-end status aggregated for legacy CSR map
    // ------------------------------------------------------------------------
    wire                stat_busy;
    wire                stat_error;
    wire [31:0]         stat_fifo_level;
    wire [31:0]         stat_packet_count;
    wire [63:0]         stat_last_timestamp;

    assign stat_busy           = core_stat_busy;
    assign stat_error          = core_stat_error | dbg_error_sticky;
    assign stat_fifo_level     = core_stat_fifo_level;
    assign stat_packet_count   = core_stat_packet_count;
    assign stat_last_timestamp = core_stat_last_timestamp;

    // ------------------------------------------------------------------------
    // sys -> eth packet CDC FIFO
    // ------------------------------------------------------------------------
    wire [15:0]         cdc_pkt_data_eth_w;
    wire                cdc_pkt_valid_eth_w;
    wire                cdc_pkt_last_eth_w;
    wire                cdc_s_pkt_ready_sys_w;
    wire                cdc_m_pkt_ready_eth_w;
    wire                cdc_fifo_empty_sys_w;
    wire                cdc_fifo_full_sys_w;
    wire                cdc_fifo_empty_eth_w;

    // ------------------------------------------------------------------------
    // ETH config / status manager outputs
    // ------------------------------------------------------------------------
    wire                eth_cfg_enable_eth_w;
    wire [47:0]         eth_cfg_local_mac_eth_w;
    wire [47:0]         eth_cfg_dst_mac_eth_w;
    wire [31:0]         eth_cfg_local_ip_eth_w;
    wire [31:0]         eth_cfg_dst_ip_eth_w;
    wire [47:0]         eth_cfg_ptp_peer_mac_eth_w;
    wire [31:0]         eth_cfg_ptp_peer_ip_eth_w;
    wire [15:0]         eth_cfg_src_port_eth_w;
    wire [15:0]         eth_cfg_dst_port_eth_w;
    wire [7:0]          eth_cfg_ttl_eth_w;
    wire [7:0]          eth_cfg_dscp_ecn_eth_w;
    wire [15:0]         eth_cfg_ip_ident_seed_eth_w;
    wire                eth_cfg_ptp_enable_eth_w;
    wire                eth_cfg_ptp_two_step_eth_w;
    wire [7:0]          eth_cfg_ptp_domain_eth_w;
    wire                eth_cfg_update_pulse_eth_w;
    wire                txs_cfg_enable_eth_w;
    wire                txs_cfg_block_when_time_invalid_eth_w;
    wire                txs_cfg_strict_window_eth_w;
    wire [63:0]         txs_cfg_base_time_ns_eth_w;
    wire [31:0]         txs_cfg_period_ns_eth_w;
    wire [31:0]         txs_cfg_slot_offset_ns_eth_w;
    wire [31:0]         txs_cfg_slot_width_ns_eth_w;
    wire [31:0]         txs_cfg_advance_ns_eth_w;
    wire [31:0]         txs_cfg_late_tol_ns_eth_w;

    wire                rx_match_ptp_enable_eth_w;
    wire [7:0]          rx_match_ptp_domain_eth_w;
    wire [31:0]         rx_match_host_ip_eth_w;
    wire [31:0]         rx_match_local_ip_eth_w;
    wire [47:0]         rx_match_host_mac_eth_w;
    wire [47:0]         rx_match_local_mac_eth_w;

    wire                eth_soft_reset_pulse_w;
    wire                eth_clear_status_pulse_w;
    wire                eth_ptp_tx_abort_pulse_w;
    wire                eth_rx_soft_reset_pulse_w;
    wire                eth_rx_clear_status_pulse_w;

    wire                eth_backend_selected;
    wire                eth_tx_mirror_enable_active_r;
    wire                eth_cfg_active_r;
    wire                txs_active_enable_sys_w;
    wire                txs_active_block_when_time_invalid_sys_w;
    wire                txs_active_strict_window_sys_w;
    wire [63:0]         txs_active_base_time_ns_sys_w;
    wire [31:0]         txs_active_period_ns_sys_w;
    wire [31:0]         txs_active_slot_offset_ns_sys_w;
    wire [31:0]         txs_active_slot_width_ns_sys_w;
    wire [31:0]         txs_active_advance_ns_sys_w;
    wire [31:0]         txs_active_late_tol_ns_sys_w;
    wire                eth_backend_stat_busy_sys_w;
    wire                eth_backend_stat_tx_busy_sys_w;
    wire [31:0]         eth_backend_stat_tx_frame_count_sys_w;
    wire [31:0]         eth_backend_stat_tx_byte_count_sys_w;
    wire [31:0]         eth_backend_stat_drop_count_sys_w;
    wire [15:0]         eth_backend_stat_last_ip_total_length_sys_w;
    wire [15:0]         eth_backend_stat_last_udp_length_sys_w;
    wire [15:0]         eth_backend_stat_last_error_code_sys_w;
    wire                eth_tx_error_sticky_r;
    wire                eth_pkt_buf_overflow_sticky_r;
    wire                eth_mac_stall_timeout_sticky_r;
    wire                mac_link_up_sys_w;
    wire [31:0]         phy_status_word_sys_w;
    wire [127:0]        phy_reg_snapshot_sys_w;

    // ------------------------------------------------------------------------
    // PTP subsystem outputs
    // ------------------------------------------------------------------------
    wire [63:0]         ptp_sys_tod_time_ns_w;
    wire                ptp_sys_tod_time_valid_w;
    wire [63:0]         ptp_txstamp_ns_latched_r;
    wire                ptp_txstamp_valid_latched_r;
    wire                ptp_txstamp_overflow_sticky_r;
    wire                ptp_tx_busy_status_sys_w;
    wire                ptp_tx_cmd_pending_status_sys_w;
    wire                ptp_rx_mbox_pop_pulse;
    wire                ptp_rx_mbox_valid_w;
    wire [3:0]          ptp_rx_mbox_msg_type_w;
    wire [15:0]         ptp_rx_mbox_sequence_id_w;
    wire [63:0]         ptp_rx_mbox_rx_timestamp_ns_w;
    wire [79:0]         ptp_rx_mbox_msg_timestamp_80_w;
    wire [79:0]         ptp_rx_mbox_requesting_port_identity_w;
    wire                ptp_rx_mbox_overflow_sticky_w;
    wire [31:0]         ptp_rx_accept_count_sys_w;
    wire [31:0]         ptp_rx_drop_count_sys_w;
    wire [15:0]         ptp_rx_last_error_sys_w;
    wire                ptp_rx_fifo_overflow_sticky_sys_w;
    wire [7:0]          ptp_tx_tdata_w;
    wire                ptp_tx_tvalid_w;
    wire                ptp_tx_tlast_w;
    wire                ptp_tx_tready_w;
    wire [3:0]          ptp_tx_tuser_w;
    wire                ptp_cfg_enable_eth_w;
    wire                ptp_cfg_strict_priority_eth_w;
    wire                ptp_cfg_event_priority_enable_eth_w;
    wire                ptp_path_busy_eth_w;
    wire [31:0]         ptp_tx_trigger_req_count_eth_w;
    wire [31:0]         ptp_tx_trigger_accept_count_eth_w;
    wire [31:0]         ptp_tx_trigger_while_busy_count_eth_w;
    wire [15:0]         ptp_tx_last_error_eth_w;

    // ------------------------------------------------------------------------
    // Egress subsystem eth-domain statistics
    // ------------------------------------------------------------------------
    wire                eth_backend_stat_busy_eth_w;
    wire                eth_backend_stat_ingress_busy_eth_w;
    wire                eth_backend_stat_tx_busy_eth_w;
    wire [31:0]         eth_backend_stat_tx_frame_count_eth_w;
    wire [31:0]         eth_backend_stat_tx_byte_count_eth_w;
    wire [31:0]         eth_backend_stat_drop_count_eth_w;
    wire [31:0]         eth_backend_stat_backpressure_cycles_eth_w;
    wire [15:0]         eth_backend_stat_last_ip_total_length_eth_w;
    wire [15:0]         eth_backend_stat_last_udp_length_eth_w;
    wire [15:0]         eth_backend_stat_last_error_code_eth_w;
    wire [31:0]         dbgext_arb_data_frame_count_eth_w;
    wire [31:0]         dbgext_arb_ptp_frame_count_eth_w;
    wire [31:0]         dbgext_arb_stall_cycles_eth_w;
    wire [15:0]         dbgext_arb_last_error_eth_w;
    wire [31:0]         txs_stat_status_eth_w;
    wire [31:0]         txs_stat_held_count_eth_w;
    wire [31:0]         txs_stat_pass_count_eth_w;
    wire [31:0]         txs_stat_late_count_eth_w;
    wire [31:0]         txs_stat_slot_miss_count_eth_w;
    wire [63:0]         txs_stat_last_plan_ns_eth_w;
    wire [63:0]         txs_stat_last_actual_ns_eth_w;
    wire [31:0]         txs_stat_last_delta_ns_eth_w;

    // ------------------------------------------------------------------------
    // Debug/scheduler-extension stats observed in sys_clk domain
    // ------------------------------------------------------------------------
    wire [31:0]         dbgext_arb_data_frame_count_sys_w;
    wire [31:0]         dbgext_arb_ptp_frame_count_sys_w;
    wire [31:0]         dbgext_arb_stall_cycles_sys_w;
    wire [15:0]         dbgext_arb_last_error_sys_w;
    wire [31:0]         dbgext_backend_backpressure_cycles_sys_w;
    wire [31:0]         dbgext_ptp_tx_trigger_req_count_sys_w;
    wire [31:0]         dbgext_ptp_tx_trigger_accept_count_sys_w;
    wire [31:0]         dbgext_ptp_tx_trigger_while_busy_count_sys_w;
    wire [15:0]         dbgext_ptp_tx_last_error_sys_w;
    wire                dbgext_status_update_pulse_sys_w;
    wire [31:0]         txs_stat_status_sys_w;
    wire [31:0]         txs_stat_held_count_sys_w;
    wire [31:0]         txs_stat_pass_count_sys_w;
    wire [31:0]         txs_stat_late_count_sys_w;
    wire [31:0]         txs_stat_slot_miss_count_sys_w;
    wire [63:0]         txs_stat_last_plan_ns_sys_w;
    wire [63:0]         txs_stat_last_actual_ns_sys_w;
    wire [31:0]         txs_stat_last_delta_ns_sys_w;
    wire                txs_status_update_pulse_sys_w;

    wire                ptp_sys_tod_time_valid_qualified_w;
    wire                txs_tod_time_valid_qualified_eth_w;

    // ------------------------------------------------------------------------
    // PTP TXSTAMP destructive-read compatibility handling remains at wrapper
    // boundary because it depends on the local CSR address phase.
    // ------------------------------------------------------------------------
    localparam [11:0]  C_ADDR_PTP_TXSTAMP_L = 12'h0D0;
    localparam [11:0]  C_ADDR_PTP_TXSTAMP_H = 12'h0D4;
    reg                ptp_txstamp_read_lo_seen_r;
    reg                ptp_txstamp_consume_rsp_pending_r;
    wire               ptp_txstamp_read_lo_req_sys_w;
    wire               ptp_txstamp_read_hi_req_sys_w;
    wire               ptp_txstamp_consume_pulse_sys_w;
    wire               unused_keep_w;

    assign ptp_sys_tod_time_valid_qualified_w = cfg_ptp_enable && ptp_sys_tod_time_valid_w;
    assign txs_tod_time_valid_qualified_eth_w = ptp_cfg_enable_eth_w && ptp_tod_time_valid_eth;

    assign ptp_txstamp_read_lo_req_sys_w = csr_en && !csr_we && (csr_addr == C_ADDR_PTP_TXSTAMP_L);
    assign ptp_txstamp_read_hi_req_sys_w = csr_en && !csr_we && (csr_addr == C_ADDR_PTP_TXSTAMP_H);
    assign ptp_txstamp_consume_pulse_sys_w = ptp_txstamp_consume_rsp_pending_r && csr_ready;

    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            ptp_txstamp_read_lo_seen_r        <= 1'b0;
            ptp_txstamp_consume_rsp_pending_r <= 1'b0;
        end else begin
            if (cfg_soft_reset_pulse || cfg_clear_status_pulse || cfg_ptp_tx_abort_pulse) begin
                ptp_txstamp_read_lo_seen_r        <= 1'b0;
                ptp_txstamp_consume_rsp_pending_r <= 1'b0;
            end else begin
                if (ptp_txstamp_consume_pulse_sys_w)
                    ptp_txstamp_consume_rsp_pending_r <= 1'b0;

                if (ptp_txstamp_read_lo_req_sys_w)
                    ptp_txstamp_read_lo_seen_r <= 1'b1;

                if (ptp_txstamp_read_hi_req_sys_w) begin
                    if (ptp_txstamp_read_lo_seen_r && ptp_txstamp_valid_latched_r)
                        ptp_txstamp_consume_rsp_pending_r <= 1'b1;
                    ptp_txstamp_read_lo_seen_r <= 1'b0;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // Front-end subsystem
    // ------------------------------------------------------------------------
    net_acq_frontend_subsystem u_net_acq_frontend_subsystem (
        .clk_in                    (clk_in),
        .resetn                    (frontend_resetn),
        .cfg_enable                (cfg_enable),
        .cfg_soft_reset_pulse      (cfg_soft_reset_pulse),
        .cfg_clear_status_pulse    (cfg_clear_status_pulse),
        .cfg_sample_period         (cfg_sample_period),
        .cfg_samples_per_packet    (cfg_samples_per_packet),
        .cfg_mode                  (cfg_mode),
        .cfg_channel_id            (cfg_channel_id),
        .cfg_src_sel               (cfg_src_sel),
        .cfg_time_quality          (cfg_time_quality),
        .cfg_ptp_enable            (cfg_ptp_enable),
        .cfg_ptp_set_time_pulse    (cfg_ptp_set_time_pulse),
        .ptp_sys_tod_time_ns       (ptp_sys_tod_time_ns_w),
        .ptp_sys_tod_time_valid    (ptp_sys_tod_time_valid_w),
        .sample_data_in            (sample_data_in),
        .sample_valid_in           (sample_valid_in),
        .sample_ready_out          (sample_ready_out),
        .pkt_data                  (front_pkt_data_w),
        .pkt_valid                 (front_pkt_valid_w),
        .pkt_last                  (front_pkt_last_w),
        .pkt_ready                 (front_pkt_ready_w),
        .core_stat_busy            (core_stat_busy),
        .core_stat_error           (core_stat_error),
        .core_stat_fifo_level      (core_stat_fifo_level),
        .core_stat_packet_count    (core_stat_packet_count),
        .core_stat_last_timestamp  (core_stat_last_timestamp),
        .dbg_error_sticky          (dbg_error_sticky),
        .dbg_good_packet_count     (dbg_good_packet_count),
        .dbg_bad_packet_count      (dbg_bad_packet_count),
        .dbg_last_seq_id           (dbg_last_seq_id),
        .dbg_last_timestamp        (dbg_last_timestamp),
        .dbg_last_sample_count     (dbg_last_sample_count),
        .dbg_last_payload_count    (dbg_last_payload_count),
        .dbg_last_error_code       (dbg_last_error_code),
        .dbg_last_error_word_index (dbg_last_error_word_index),
        .dbg_last_protocol_version (dbg_last_protocol_version),
        .dbg_last_quality          (dbg_last_quality)
    );

    // ------------------------------------------------------------------------
    // sys->eth packet CDC FIFO
    // ------------------------------------------------------------------------
    net_acq_pkt_async_fifo #(
        .P_WORD_W (16),
        .P_DEPTH  (1024)
    ) u_net_acq_pkt_async_fifo (
        .wr_clk      (clk_in),
        .wr_resetn   (resetn),
        .wr_clear    (cfg_soft_reset_pulse),
        .s_pkt_data  (front_pkt_data_w),
        .s_pkt_valid (front_pkt_valid_w && eth_backend_selected),
        .s_pkt_last  (front_pkt_last_w  && eth_backend_selected),
        .s_pkt_ready (cdc_s_pkt_ready_sys_w),
        .wr_empty    (cdc_fifo_empty_sys_w),
        .wr_full     (cdc_fifo_full_sys_w),
        .rd_clk      (eth_tx_clk),
        .rd_resetn   (eth_tx_resetn),
        .rd_clear    (eth_soft_reset_pulse_w),
        .m_pkt_data  (cdc_pkt_data_eth_w),
        .m_pkt_valid (cdc_pkt_valid_eth_w),
        .m_pkt_last  (cdc_pkt_last_eth_w),
        .m_pkt_ready (cdc_m_pkt_ready_eth_w),
        .rd_empty    (cdc_fifo_empty_eth_w)
    );

    // ------------------------------------------------------------------------
    // Control-plane / config / status manager
    // ------------------------------------------------------------------------
    net_acq_eth_cfg_manager u_net_acq_eth_cfg_manager (
        .clk_in                              (clk_in),
        .resetn                              (cfg_sys_resetn),
        .eth_tx_clk                          (eth_tx_clk),
        .eth_tx_resetn                       (cfg_tx_resetn),
        .eth_rx_clk                          (eth_rx_clk),
        .eth_rx_resetn                       (cfg_rx_resetn),
        .cfg_enable                          (cfg_enable),
        .cfg_soft_reset_pulse                (cfg_soft_reset_pulse),
        .cfg_clear_status_pulse              (cfg_clear_status_pulse),
        .core_stat_busy                      (core_stat_busy),
        .cdc_fifo_empty_sys                  (cdc_fifo_empty_sys_w),
        .cfg_eth_backend_enable              (cfg_eth_backend_enable),
        .cfg_eth_commit_pulse                (cfg_eth_commit_pulse),
        .cfg_eth_tx_mirror_enable            (cfg_eth_tx_mirror_enable),
        .cfg_eth_append_fcs                  (cfg_eth_append_fcs),
        .cfg_eth_udp_checksum_enable         (cfg_eth_udp_checksum_enable),
        .cfg_eth_stall_drop_enable           (cfg_eth_stall_drop_enable),
        .cfg_eth_local_mac                   (cfg_eth_local_mac),
        .cfg_eth_dst_mac                     (cfg_eth_dst_mac),
        .cfg_eth_local_ip                    (cfg_eth_local_ip),
        .cfg_eth_dst_ip                      (cfg_eth_dst_ip),
        .cfg_eth_src_port                    (cfg_eth_src_port),
        .cfg_eth_dst_port                    (cfg_eth_dst_port),
        .cfg_eth_ttl                         (cfg_eth_ttl),
        .cfg_eth_dscp_ecn                    (cfg_eth_dscp_ecn),
        .cfg_eth_ip_ident_seed               (cfg_eth_ip_ident_seed),
        .cfg_ptp_addr_split_enable           (cfg_ptp_addr_split_enable),
        .cfg_ptp_peer_mac                    (cfg_ptp_peer_mac),
        .cfg_ptp_peer_ip                     (cfg_ptp_peer_ip),
        .cfg_txs_enable                      (cfg_txs_enable),
        .cfg_txs_block_when_time_invalid     (cfg_txs_block_when_time_invalid),
        .cfg_txs_strict_window               (cfg_txs_strict_window),
        .cfg_txs_base_time_ns                (cfg_txs_base_time_ns),
        .cfg_txs_period_ns                   (cfg_txs_period_ns),
        .cfg_txs_slot_offset_ns              (cfg_txs_slot_offset_ns),
        .cfg_txs_slot_width_ns               (cfg_txs_slot_width_ns),
        .cfg_txs_advance_ns                  (cfg_txs_advance_ns),
        .cfg_txs_late_tol_ns                 (cfg_txs_late_tol_ns),
        .cfg_ptp_enable                      (cfg_ptp_enable),
        .cfg_ptp_two_step                    (cfg_ptp_two_step),
        .cfg_ptp_domain                      (cfg_ptp_domain),
        .cfg_ptp_update_pulse                (cfg_ptp_update_pulse),
        .cfg_ptp_tx_abort_pulse              (cfg_ptp_tx_abort_pulse),
        .cfg_phy_soft_reset_pulse            (cfg_phy_soft_reset_pulse),
        .cfg_phy_restart_an_pulse            (cfg_phy_restart_an_pulse),
        .eth_backend_stat_busy_eth           (eth_backend_stat_busy_eth_w),
        .eth_backend_stat_tx_busy_eth        (eth_backend_stat_tx_busy_eth_w),
        .eth_backend_stat_tx_frame_count_eth (eth_backend_stat_tx_frame_count_eth_w),
        .eth_backend_stat_tx_byte_count_eth  (eth_backend_stat_tx_byte_count_eth_w),
        .eth_backend_stat_drop_count_eth     (eth_backend_stat_drop_count_eth_w),
        .eth_backend_stat_last_ip_total_length_eth(eth_backend_stat_last_ip_total_length_eth_w),
        .eth_backend_stat_last_udp_length_eth(eth_backend_stat_last_udp_length_eth_w),
        .eth_backend_stat_last_error_code_eth(eth_backend_stat_last_error_code_eth_w),
        .ptp_path_busy_eth                   (ptp_path_busy_eth_w),
        .mac_link_up                         (mac_link_up),
        .phy_status_word                     (phy_status_word),
        .phy_reg_snapshot                    (phy_reg_snapshot),
        .eth_cfg_enable_eth                  (eth_cfg_enable_eth_w),
        .eth_cfg_local_mac_eth               (eth_cfg_local_mac_eth_w),
        .eth_cfg_dst_mac_eth                 (eth_cfg_dst_mac_eth_w),
        .eth_cfg_local_ip_eth                (eth_cfg_local_ip_eth_w),
        .eth_cfg_dst_ip_eth                  (eth_cfg_dst_ip_eth_w),
        .eth_cfg_ptp_peer_mac_eth            (eth_cfg_ptp_peer_mac_eth_w),
        .eth_cfg_ptp_peer_ip_eth             (eth_cfg_ptp_peer_ip_eth_w),
        .eth_cfg_src_port_eth                (eth_cfg_src_port_eth_w),
        .eth_cfg_dst_port_eth                (eth_cfg_dst_port_eth_w),
        .eth_cfg_ttl_eth                     (eth_cfg_ttl_eth_w),
        .eth_cfg_dscp_ecn_eth                (eth_cfg_dscp_ecn_eth_w),
        .eth_cfg_ip_ident_seed_eth           (eth_cfg_ip_ident_seed_eth_w),
        .eth_cfg_ptp_enable_eth              (eth_cfg_ptp_enable_eth_w),
        .eth_cfg_ptp_two_step_eth            (eth_cfg_ptp_two_step_eth_w),
        .eth_cfg_ptp_domain_eth              (eth_cfg_ptp_domain_eth_w),
        .eth_cfg_update_pulse_eth            (eth_cfg_update_pulse_eth_w),
        .txs_cfg_enable_eth                  (txs_cfg_enable_eth_w),
        .txs_cfg_block_when_time_invalid_eth (txs_cfg_block_when_time_invalid_eth_w),
        .txs_cfg_strict_window_eth           (txs_cfg_strict_window_eth_w),
        .txs_cfg_base_time_ns_eth            (txs_cfg_base_time_ns_eth_w),
        .txs_cfg_period_ns_eth               (txs_cfg_period_ns_eth_w),
        .txs_cfg_slot_offset_ns_eth          (txs_cfg_slot_offset_ns_eth_w),
        .txs_cfg_slot_width_ns_eth           (txs_cfg_slot_width_ns_eth_w),
        .txs_cfg_advance_ns_eth              (txs_cfg_advance_ns_eth_w),
        .txs_cfg_late_tol_ns_eth             (txs_cfg_late_tol_ns_eth_w),
        .rx_match_ptp_enable_eth             (rx_match_ptp_enable_eth_w),
        .rx_match_ptp_domain_eth             (rx_match_ptp_domain_eth_w),
        .rx_match_host_ip_eth                (rx_match_host_ip_eth_w),
        .rx_match_local_ip_eth               (rx_match_local_ip_eth_w),
        .rx_match_host_mac_eth               (rx_match_host_mac_eth_w),
        .rx_match_local_mac_eth              (rx_match_local_mac_eth_w),
        .eth_soft_reset_pulse_eth            (eth_soft_reset_pulse_w),
        .eth_clear_status_pulse_eth          (eth_clear_status_pulse_w),
        .eth_ptp_tx_abort_pulse_eth          (eth_ptp_tx_abort_pulse_w),
        .eth_rx_soft_reset_pulse_eth         (eth_rx_soft_reset_pulse_w),
        .eth_rx_clear_status_pulse_eth       (eth_rx_clear_status_pulse_w),
        .phy_soft_reset_pulse                (phy_soft_reset_pulse),
        .phy_restart_an_pulse                (phy_restart_an_pulse),
        .eth_backend_selected                (eth_backend_selected),
        .eth_tx_mirror_enable_active         (eth_tx_mirror_enable_active_r),
        .eth_cfg_active                      (eth_cfg_active_r),
        .txs_active_enable_sys               (txs_active_enable_sys_w),
        .txs_active_block_when_time_invalid_sys(txs_active_block_when_time_invalid_sys_w),
        .txs_active_strict_window_sys        (txs_active_strict_window_sys_w),
        .txs_active_base_time_ns_sys         (txs_active_base_time_ns_sys_w),
        .txs_active_period_ns_sys            (txs_active_period_ns_sys_w),
        .txs_active_slot_offset_ns_sys       (txs_active_slot_offset_ns_sys_w),
        .txs_active_slot_width_ns_sys        (txs_active_slot_width_ns_sys_w),
        .txs_active_advance_ns_sys           (txs_active_advance_ns_sys_w),
        .txs_active_late_tol_ns_sys          (txs_active_late_tol_ns_sys_w),
        .eth_backend_stat_busy_sys           (eth_backend_stat_busy_sys_w),
        .eth_backend_stat_tx_busy_sys        (eth_backend_stat_tx_busy_sys_w),
        .eth_backend_stat_tx_frame_count_sys (eth_backend_stat_tx_frame_count_sys_w),
        .eth_backend_stat_tx_byte_count_sys  (eth_backend_stat_tx_byte_count_sys_w),
        .eth_backend_stat_drop_count_sys     (eth_backend_stat_drop_count_sys_w),
        .eth_backend_stat_last_ip_total_length_sys(eth_backend_stat_last_ip_total_length_sys_w),
        .eth_backend_stat_last_udp_length_sys(eth_backend_stat_last_udp_length_sys_w),
        .eth_backend_stat_last_error_code_sys(eth_backend_stat_last_error_code_sys_w),
        .eth_tx_error_sticky                 (eth_tx_error_sticky_r),
        .eth_pkt_buf_overflow_sticky         (eth_pkt_buf_overflow_sticky_r),
        .eth_mac_stall_timeout_sticky        (eth_mac_stall_timeout_sticky_r),
        .mac_link_up_sys                     (mac_link_up_sys_w),
        .phy_status_word_sys                 (phy_status_word_sys_w),
        .phy_reg_snapshot_sys                (phy_reg_snapshot_sys_w)
    );

    // ------------------------------------------------------------------------
    // PTP subsystem
    // ------------------------------------------------------------------------
    net_acq_ptp_subsystem u_net_acq_ptp_subsystem (
        .clk_in                              (clk_in),
        .resetn                              (resetn),
        .eth_tx_clk                          (eth_tx_clk),
        .eth_tx_resetn                       (eth_tx_resetn),
        .eth_rx_clk                          (eth_rx_clk),
        .eth_rx_resetn                       (eth_rx_resetn),
        .cfg_soft_reset_pulse                (cfg_soft_reset_pulse),
        .cfg_clear_status_pulse              (cfg_clear_status_pulse),
        .cfg_ptp_enable                      (cfg_ptp_enable),
        .cfg_ptp_one_step                    (cfg_ptp_one_step),
        .cfg_ptp_two_step                    (cfg_ptp_two_step),
        .cfg_ptp_event_priority_enable       (cfg_ptp_event_priority_enable),
        .cfg_ptp_hold                        (cfg_ptp_hold),
        .cfg_ptp_strict_priority             (cfg_ptp_strict_priority),
        .cfg_ptp_domain                      (cfg_ptp_domain),
        .cfg_ptp_clock_id                    (cfg_ptp_clock_id),
        .cfg_ptp_port_num                    (cfg_ptp_port_num),
        .cfg_ptp_update_pulse                (cfg_ptp_update_pulse),
        .cfg_ptp_set_time_pulse              (cfg_ptp_set_time_pulse),
        .cfg_ptp_step_pulse                  (cfg_ptp_step_pulse),
        .cfg_ptp_tx_abort_pulse              (cfg_ptp_tx_abort_pulse),
        .cfg_ptp_tx_trigger_pulse            (cfg_ptp_tx_trigger_pulse),
        .cfg_ptp_set_time_ns                 (cfg_ptp_set_time_ns),
        .cfg_ptp_step_ns                     (cfg_ptp_step_ns),
        .cfg_ptp_freq_adj_ppb                (cfg_ptp_freq_adj_ppb),
        .cfg_ptp_tx_message_type             (cfg_ptp_tx_message_type),
        .cfg_ptp_tx_transport_specific       (cfg_ptp_tx_transport_specific),
        .cfg_ptp_tx_sequence_id              (cfg_ptp_tx_sequence_id),
        .cfg_ptp_tx_flags                    (cfg_ptp_tx_flags),
        .cfg_ptp_tx_log_message_interval     (cfg_ptp_tx_log_message_interval),
        .cfg_ptp_tx_need_timestamp           (cfg_ptp_tx_need_timestamp),
        .eth_cfg_local_mac_eth               (eth_cfg_local_mac_eth_w),
        .eth_cfg_local_ip_eth                (eth_cfg_local_ip_eth_w),
        .eth_cfg_ptp_peer_mac_eth            (eth_cfg_ptp_peer_mac_eth_w),
        .eth_cfg_ptp_peer_ip_eth             (eth_cfg_ptp_peer_ip_eth_w),
        .eth_cfg_ttl_eth                     (eth_cfg_ttl_eth_w),
        .eth_cfg_dscp_ecn_eth                (eth_cfg_dscp_ecn_eth_w),
        .eth_cfg_ip_ident_seed_eth           (eth_cfg_ip_ident_seed_eth_w),
        .rx_match_ptp_enable_eth             (rx_match_ptp_enable_eth_w),
        .rx_match_ptp_domain_eth             (rx_match_ptp_domain_eth_w),
        .rx_match_host_ip_eth                (rx_match_host_ip_eth_w),
        .rx_match_local_ip_eth               (rx_match_local_ip_eth_w),
        .rx_match_host_mac_eth               (rx_match_host_mac_eth_w),
        .rx_match_local_mac_eth              (rx_match_local_mac_eth_w),
        .eth_soft_reset_pulse_eth            (eth_soft_reset_pulse_w),
        .eth_clear_status_pulse_eth          (eth_clear_status_pulse_w),
        .eth_ptp_tx_abort_pulse_eth          (eth_ptp_tx_abort_pulse_w),
        .eth_rx_soft_reset_pulse_eth         (eth_rx_soft_reset_pulse_w),
        .eth_rx_clear_status_pulse_eth       (eth_rx_clear_status_pulse_w),
        .s_axis_eth_rx_tdata                 (s_axis_eth_rx_tdata),
        .s_axis_eth_rx_tvalid                (s_axis_eth_rx_tvalid),
        .s_axis_eth_rx_tlast                 (s_axis_eth_rx_tlast),
        .ptp_rxstamp_ns                      (ptp_rxstamp_ns),
        .ptp_rxstamp_valid                   (ptp_rxstamp_valid),
        .ptp_txstamp_ns                      (ptp_txstamp_ns),
        .ptp_txstamp_valid                   (ptp_txstamp_valid),
        .ptp_sys_tod_time_ns                 (ptp_sys_tod_time_ns_w),
        .ptp_sys_tod_time_valid              (ptp_sys_tod_time_valid_w),
        .ptp_tod_time_ns_eth                 (ptp_tod_time_ns_eth),
        .ptp_tod_time_valid_eth              (ptp_tod_time_valid_eth),
        .ptp_tod_time_ns_rx                  (ptp_tod_time_ns_rx),
        .ptp_tod_time_valid_rx               (ptp_tod_time_valid_rx),
        .ptp_txstamp_ns_latched              (ptp_txstamp_ns_latched_r),
        .ptp_txstamp_valid_latched           (ptp_txstamp_valid_latched_r),
        .ptp_txstamp_overflow_sticky         (ptp_txstamp_overflow_sticky_r),
        .ptp_txstamp_consume_pulse_sys       (ptp_txstamp_consume_pulse_sys_w),
        .ptp_tx_busy_status_sys              (ptp_tx_busy_status_sys_w),
        .ptp_tx_cmd_pending_status_sys       (ptp_tx_cmd_pending_status_sys_w),
        .ptp_rx_mbox_pop_pulse               (ptp_rx_mbox_pop_pulse),
        .ptp_rx_mbox_valid                   (ptp_rx_mbox_valid_w),
        .ptp_rx_mbox_msg_type                (ptp_rx_mbox_msg_type_w),
        .ptp_rx_mbox_sequence_id             (ptp_rx_mbox_sequence_id_w),
        .ptp_rx_mbox_rx_timestamp_ns         (ptp_rx_mbox_rx_timestamp_ns_w),
        .ptp_rx_mbox_msg_timestamp_80        (ptp_rx_mbox_msg_timestamp_80_w),
        .ptp_rx_mbox_requesting_port_identity(ptp_rx_mbox_requesting_port_identity_w),
        .ptp_rx_mbox_overflow_sticky         (ptp_rx_mbox_overflow_sticky_w),
        .ptp_rx_accept_count_sys             (ptp_rx_accept_count_sys_w),
        .ptp_rx_drop_count_sys               (ptp_rx_drop_count_sys_w),
        .ptp_rx_last_error_sys               (ptp_rx_last_error_sys_w),
        .ptp_rx_fifo_overflow_sticky_sys     (ptp_rx_fifo_overflow_sticky_sys_w),
        .ptp_tx_tdata                        (ptp_tx_tdata_w),
        .ptp_tx_tvalid                       (ptp_tx_tvalid_w),
        .ptp_tx_tlast                        (ptp_tx_tlast_w),
        .ptp_tx_tready                       (ptp_tx_tready_w),
        .ptp_tx_tuser                        (ptp_tx_tuser_w),
        .ptp_cfg_enable_eth                  (ptp_cfg_enable_eth_w),
        .ptp_cfg_strict_priority_eth         (ptp_cfg_strict_priority_eth_w),
        .ptp_cfg_event_priority_enable_eth   (ptp_cfg_event_priority_enable_eth_w),
        .ptp_path_busy_eth                   (ptp_path_busy_eth_w),
        .ptp_tx_trigger_req_count_eth        (ptp_tx_trigger_req_count_eth_w),
        .ptp_tx_trigger_accept_count_eth     (ptp_tx_trigger_accept_count_eth_w),
        .ptp_tx_trigger_while_busy_count_eth (ptp_tx_trigger_while_busy_count_eth_w),
        .ptp_tx_last_error_eth               (ptp_tx_last_error_eth_w)
    );

    // ------------------------------------------------------------------------
    // Egress subsystem
    // ------------------------------------------------------------------------
    net_acq_egress_subsystem u_net_acq_egress_subsystem (
        .eth_tx_clk                          (eth_tx_clk),
        .eth_tx_resetn                       (egress_resetn),
        .eth_soft_reset_pulse                (eth_soft_reset_pulse_w),
        .eth_clear_status_pulse              (eth_clear_status_pulse_w),
        .eth_cfg_enable                      (eth_cfg_enable_eth_w),
        .eth_cfg_local_mac                   (eth_cfg_local_mac_eth_w),
        .eth_cfg_dst_mac                     (eth_cfg_dst_mac_eth_w),
        .eth_cfg_local_ip                    (eth_cfg_local_ip_eth_w),
        .eth_cfg_dst_ip                      (eth_cfg_dst_ip_eth_w),
        .eth_cfg_src_port                    (eth_cfg_src_port_eth_w),
        .eth_cfg_dst_port                    (eth_cfg_dst_port_eth_w),
        .eth_cfg_ttl                         (eth_cfg_ttl_eth_w),
        .eth_cfg_dscp_ecn                    (eth_cfg_dscp_ecn_eth_w),
        .eth_cfg_ip_ident_seed               (eth_cfg_ip_ident_seed_eth_w),
        .eth_cfg_ptp_enable                  (eth_cfg_ptp_enable_eth_w),
        .eth_cfg_ptp_two_step                (eth_cfg_ptp_two_step_eth_w),
        .eth_cfg_ptp_domain                  (eth_cfg_ptp_domain_eth_w),
        .eth_cfg_update_pulse                (eth_cfg_update_pulse_eth_w),
        .txs_cfg_enable                      (txs_cfg_enable_eth_w),
        .txs_cfg_block_when_time_invalid     (txs_cfg_block_when_time_invalid_eth_w),
        .txs_cfg_strict_window               (txs_cfg_strict_window_eth_w),
        .txs_cfg_base_time_ns                (txs_cfg_base_time_ns_eth_w),
        .txs_cfg_period_ns                   (txs_cfg_period_ns_eth_w),
        .txs_cfg_slot_offset_ns              (txs_cfg_slot_offset_ns_eth_w),
        .txs_cfg_slot_width_ns               (txs_cfg_slot_width_ns_eth_w),
        .txs_cfg_advance_ns                  (txs_cfg_advance_ns_eth_w),
        .txs_cfg_late_tol_ns                 (txs_cfg_late_tol_ns_eth_w),
        .txs_tod_time_ns                     (ptp_tod_time_ns_eth),
        .txs_tod_time_valid                  (txs_tod_time_valid_qualified_eth_w),
        .ptp_cfg_enable                      (ptp_cfg_enable_eth_w),
        .ptp_cfg_strict_priority             (ptp_cfg_strict_priority_eth_w),
        .ptp_cfg_event_priority_enable       (ptp_cfg_event_priority_enable_eth_w),
        .cdc_pkt_data                        (cdc_pkt_data_eth_w),
        .cdc_pkt_valid                       (cdc_pkt_valid_eth_w),
        .cdc_pkt_last                        (cdc_pkt_last_eth_w),
        .cdc_pkt_ready                       (cdc_m_pkt_ready_eth_w),
        .ptp_tx_tdata                        (ptp_tx_tdata_w),
        .ptp_tx_tvalid                       (ptp_tx_tvalid_w),
        .ptp_tx_tlast                        (ptp_tx_tlast_w),
        .ptp_tx_tready                       (ptp_tx_tready_w),
        .ptp_tx_tuser                        (ptp_tx_tuser_w),
        .m_axis_eth_tx_tdata                 (m_axis_eth_tx_tdata),
        .m_axis_eth_tx_tvalid                (m_axis_eth_tx_tvalid),
        .m_axis_eth_tx_tlast                 (m_axis_eth_tx_tlast),
        .m_axis_eth_tx_tready                (m_axis_eth_tx_tready),
        .m_axis_eth_tx_tuser                 (m_axis_eth_tx_tuser),
        .eth_backend_stat_busy               (eth_backend_stat_busy_eth_w),
        .eth_backend_stat_ingress_busy       (eth_backend_stat_ingress_busy_eth_w),
        .eth_backend_stat_tx_busy            (eth_backend_stat_tx_busy_eth_w),
        .eth_backend_stat_tx_frame_count     (eth_backend_stat_tx_frame_count_eth_w),
        .eth_backend_stat_tx_byte_count      (eth_backend_stat_tx_byte_count_eth_w),
        .eth_backend_stat_drop_count         (eth_backend_stat_drop_count_eth_w),
        .eth_backend_stat_backpressure_cycles(eth_backend_stat_backpressure_cycles_eth_w),
        .eth_backend_stat_last_ip_total_length(eth_backend_stat_last_ip_total_length_eth_w),
        .eth_backend_stat_last_udp_length    (eth_backend_stat_last_udp_length_eth_w),
        .eth_backend_stat_last_error_code    (eth_backend_stat_last_error_code_eth_w),
        .dbg_arb_data_frame_count            (dbgext_arb_data_frame_count_eth_w),
        .dbg_arb_ptp_frame_count             (dbgext_arb_ptp_frame_count_eth_w),
        .dbg_arb_stall_cycles                (dbgext_arb_stall_cycles_eth_w),
        .dbg_arb_last_error                  (dbgext_arb_last_error_eth_w),
        .txs_stat_status                     (txs_stat_status_eth_w),
        .txs_stat_held_count                 (txs_stat_held_count_eth_w),
        .txs_stat_pass_count                 (txs_stat_pass_count_eth_w),
        .txs_stat_late_count                 (txs_stat_late_count_eth_w),
        .txs_stat_slot_miss_count            (txs_stat_slot_miss_count_eth_w),
        .txs_stat_last_plan_ns               (txs_stat_last_plan_ns_eth_w),
        .txs_stat_last_actual_ns             (txs_stat_last_actual_ns_eth_w),
        .txs_stat_last_delta_ns              (txs_stat_last_delta_ns_eth_w)
    );

    // ------------------------------------------------------------------------
    // eth_tx -> sys debug/scheduler-extension observability bridge
    //   Keep this path independent from the stable ETH_STATUS legacy group so
    //   future data_tx_scheduler counters can be added without perturbing the
    //   existing software contract.
    // ------------------------------------------------------------------------
    net_acq_dbgext_cdc_bridge u_net_acq_dbgext_cdc_bridge (
        .eth_clk                             (eth_tx_clk),
        .eth_resetn                          (observe_tx_resetn),
        .eth_arb_data_frame_count            (dbgext_arb_data_frame_count_eth_w),
        .eth_arb_ptp_frame_count             (dbgext_arb_ptp_frame_count_eth_w),
        .eth_arb_stall_cycles                (dbgext_arb_stall_cycles_eth_w),
        .eth_arb_last_error                  (dbgext_arb_last_error_eth_w),
        .eth_backend_backpressure_cycles     (eth_backend_stat_backpressure_cycles_eth_w),
        .eth_ptp_tx_trigger_req_count        (ptp_tx_trigger_req_count_eth_w),
        .eth_ptp_tx_trigger_accept_count     (ptp_tx_trigger_accept_count_eth_w),
        .eth_ptp_tx_trigger_while_busy_count (ptp_tx_trigger_while_busy_count_eth_w),
        .eth_ptp_tx_last_error               (ptp_tx_last_error_eth_w),
        .sys_clk                             (clk_in),
        .sys_resetn                          (observe_sys_resetn),
        .sys_arb_data_frame_count            (dbgext_arb_data_frame_count_sys_w),
        .sys_arb_ptp_frame_count             (dbgext_arb_ptp_frame_count_sys_w),
        .sys_arb_stall_cycles                (dbgext_arb_stall_cycles_sys_w),
        .sys_arb_last_error                  (dbgext_arb_last_error_sys_w),
        .sys_backend_backpressure_cycles     (dbgext_backend_backpressure_cycles_sys_w),
        .sys_ptp_tx_trigger_req_count        (dbgext_ptp_tx_trigger_req_count_sys_w),
        .sys_ptp_tx_trigger_accept_count     (dbgext_ptp_tx_trigger_accept_count_sys_w),
        .sys_ptp_tx_trigger_while_busy_count (dbgext_ptp_tx_trigger_while_busy_count_sys_w),
        .sys_ptp_tx_last_error               (dbgext_ptp_tx_last_error_sys_w),
        .sys_dbgext_update_pulse             (dbgext_status_update_pulse_sys_w)
    );

    // ------------------------------------------------------------------------
    // Dedicated scheduler-status CDC back to sys_clk.
    //   Keep scheduler status independent from dbgext so the existing 0x140+
    //   pre-scheduler observability remains stable while 0x180+ can evolve.
    // ------------------------------------------------------------------------
    net_acq_txsched_status_cdc_bridge u_net_acq_txsched_status_cdc_bridge (
        .eth_clk               (eth_tx_clk),
        .eth_resetn            (observe_tx_resetn),
        .eth_txs_status        (txs_stat_status_eth_w),
        .eth_txs_held_count    (txs_stat_held_count_eth_w),
        .eth_txs_pass_count    (txs_stat_pass_count_eth_w),
        .eth_txs_late_count    (txs_stat_late_count_eth_w),
        .eth_txs_slot_miss_count(txs_stat_slot_miss_count_eth_w),
        .eth_txs_last_plan_ns  (txs_stat_last_plan_ns_eth_w),
        .eth_txs_last_actual_ns(txs_stat_last_actual_ns_eth_w),
        .eth_txs_last_delta_ns (txs_stat_last_delta_ns_eth_w),
        .sys_clk               (clk_in),
        .sys_resetn            (observe_sys_resetn),
        .sys_txs_status        (txs_stat_status_sys_w),
        .sys_txs_held_count    (txs_stat_held_count_sys_w),
        .sys_txs_pass_count    (txs_stat_pass_count_sys_w),
        .sys_txs_late_count    (txs_stat_late_count_sys_w),
        .sys_txs_slot_miss_count(txs_stat_slot_miss_count_sys_w),
        .sys_txs_last_plan_ns  (txs_stat_last_plan_ns_sys_w),
        .sys_txs_last_actual_ns(txs_stat_last_actual_ns_sys_w),
        .sys_txs_last_delta_ns (txs_stat_last_delta_ns_sys_w),
        .sys_txs_update_pulse  (txs_status_update_pulse_sys_w)
    );

    // ------------------------------------------------------------------------
    // Legacy packet mirror / bypass behavior
    // ------------------------------------------------------------------------
    assign front_pkt_ready_w = eth_backend_selected ? cdc_s_pkt_ready_sys_w : pkt_ready;
    assign pkt_data          = (eth_backend_selected && !eth_tx_mirror_enable_active_r) ? 16'd0 : front_pkt_data_w;
    assign pkt_valid         = (eth_backend_selected && !eth_tx_mirror_enable_active_r) ? 1'b0  : front_pkt_valid_w;
    assign pkt_last          = (eth_backend_selected && !eth_tx_mirror_enable_active_r) ? 1'b0  : front_pkt_last_w;

    // ------------------------------------------------------------------------
    // CSR block (software-visible contract remains unchanged)
    // ------------------------------------------------------------------------
    net_acq_csr u_net_acq_csr (
        .clk_in                      (clk_in),
        .resetn                      (csr_resetn),
        .csr_en                      (csr_en),
        .csr_we                      (csr_we),
        .csr_addr                    (csr_addr),
        .csr_wdata                   (csr_wdata),
        .csr_rdata                   (csr_rdata),
        .csr_ready                   (csr_ready),
        .cfg_enable                  (cfg_enable),
        .cfg_soft_reset_pulse        (cfg_soft_reset_pulse),
        .cfg_clear_status_pulse      (cfg_clear_status_pulse),
        .cfg_sample_period           (cfg_sample_period),
        .cfg_samples_per_packet      (cfg_samples_per_packet),
        .cfg_mode                    (cfg_mode),
        .cfg_channel_id              (cfg_channel_id),
        .cfg_src_sel                 (cfg_src_sel),
        .cfg_time_quality            (cfg_time_quality),
        .cfg_eth_backend_enable      (cfg_eth_backend_enable),
        .cfg_eth_commit_pulse        (cfg_eth_commit_pulse),
        .cfg_eth_tx_mirror_enable    (cfg_eth_tx_mirror_enable),
        .cfg_eth_append_fcs          (cfg_eth_append_fcs),
        .cfg_eth_udp_checksum_enable (cfg_eth_udp_checksum_enable),
        .cfg_eth_stall_drop_enable   (cfg_eth_stall_drop_enable),
        .cfg_eth_local_mac           (cfg_eth_local_mac),
        .cfg_eth_dst_mac             (cfg_eth_dst_mac),
        .cfg_eth_local_ip            (cfg_eth_local_ip),
        .cfg_eth_dst_ip              (cfg_eth_dst_ip),
        .cfg_eth_src_port            (cfg_eth_src_port),
        .cfg_eth_dst_port            (cfg_eth_dst_port),
        .cfg_eth_ttl                 (cfg_eth_ttl),
        .cfg_eth_dscp_ecn            (cfg_eth_dscp_ecn),
        .cfg_eth_ip_ident_seed       (cfg_eth_ip_ident_seed),
        .cfg_ptp_addr_split_enable   (cfg_ptp_addr_split_enable),
        .cfg_ptp_peer_mac            (cfg_ptp_peer_mac),
        .cfg_ptp_peer_ip             (cfg_ptp_peer_ip),
        .cfg_txs_enable              (cfg_txs_enable),
        .cfg_txs_block_when_time_invalid(cfg_txs_block_when_time_invalid),
        .cfg_txs_strict_window       (cfg_txs_strict_window),
        .cfg_txs_base_time_ns        (cfg_txs_base_time_ns),
        .cfg_txs_period_ns           (cfg_txs_period_ns),
        .cfg_txs_slot_offset_ns      (cfg_txs_slot_offset_ns),
        .cfg_txs_slot_width_ns       (cfg_txs_slot_width_ns),
        .cfg_txs_advance_ns          (cfg_txs_advance_ns),
        .cfg_txs_late_tol_ns         (cfg_txs_late_tol_ns),
        .cfg_ptp_enable              (cfg_ptp_enable),
        .cfg_ptp_one_step            (cfg_ptp_one_step),
        .cfg_ptp_two_step            (cfg_ptp_two_step),
        .cfg_ptp_event_priority_enable(cfg_ptp_event_priority_enable),
        .cfg_ptp_hold                (cfg_ptp_hold),
        .cfg_ptp_strict_priority     (cfg_ptp_strict_priority),
        .cfg_ptp_domain              (cfg_ptp_domain),
        .cfg_ptp_clock_id            (cfg_ptp_clock_id),
        .cfg_ptp_port_num            (cfg_ptp_port_num),
        .cfg_ptp_update_pulse        (cfg_ptp_update_pulse),
        .cfg_ptp_set_time_pulse      (cfg_ptp_set_time_pulse),
        .cfg_ptp_step_pulse          (cfg_ptp_step_pulse),
        .cfg_ptp_tx_abort_pulse      (cfg_ptp_tx_abort_pulse),
        .cfg_ptp_tx_trigger_pulse    (cfg_ptp_tx_trigger_pulse),
        .cfg_ptp_set_time_ns         (cfg_ptp_set_time_ns),
        .cfg_ptp_step_ns             (cfg_ptp_step_ns),
        .cfg_ptp_freq_adj_ppb        (cfg_ptp_freq_adj_ppb),
        .cfg_ptp_tx_message_type     (cfg_ptp_tx_message_type),
        .cfg_ptp_tx_transport_specific(cfg_ptp_tx_transport_specific),
        .cfg_ptp_tx_sequence_id      (cfg_ptp_tx_sequence_id),
        .cfg_ptp_tx_flags            (cfg_ptp_tx_flags),
        .cfg_ptp_tx_log_message_interval(cfg_ptp_tx_log_message_interval),
        .cfg_ptp_tx_need_timestamp   (cfg_ptp_tx_need_timestamp),
        .stat_busy                   (stat_busy),
        .stat_error                  (stat_error),
        .stat_fifo_level             (stat_fifo_level),
        .stat_packet_count           (stat_packet_count),
        .stat_last_timestamp         (stat_last_timestamp),
        .dbg_core_error              (core_stat_error),
        .dbg_error_sticky            (dbg_error_sticky),
        .dbg_good_packet_count       (dbg_good_packet_count),
        .dbg_bad_packet_count        (dbg_bad_packet_count),
        .dbg_last_seq_id             (dbg_last_seq_id),
        .dbg_last_timestamp          (dbg_last_timestamp),
        .dbg_last_sample_count       (dbg_last_sample_count),
        .dbg_last_payload_count      (dbg_last_payload_count),
        .dbg_last_error_code         (dbg_last_error_code),
        .dbg_last_error_word_index   (dbg_last_error_word_index),
        .dbg_last_protocol_version   (dbg_last_protocol_version),
        .dbg_last_quality            (dbg_last_quality),
        .eth_stat_tx_busy            (eth_backend_stat_tx_busy_sys_w),
        .eth_stat_tx_error_sticky    (eth_tx_error_sticky_r),
        .eth_stat_pkt_buf_overflow   (eth_pkt_buf_overflow_sticky_r),
        .eth_stat_mac_stall_timeout  (eth_mac_stall_timeout_sticky_r),
        .eth_stat_cfg_active         (eth_cfg_active_r),
        .eth_stat_backend_selected   (eth_backend_selected),
        .eth_stat_link_up            (mac_link_up_sys_w),
        .eth_stat_tx_packet_count    (eth_backend_stat_tx_frame_count_sys_w),
        .eth_stat_tx_byte_count      (eth_backend_stat_tx_byte_count_sys_w),
        .eth_stat_drop_count         (eth_backend_stat_drop_count_sys_w),
        .eth_stat_last_ip_total_length(eth_backend_stat_last_ip_total_length_sys_w),
        .eth_stat_last_udp_length    (eth_backend_stat_last_udp_length_sys_w),
        .eth_stat_last_error_code    (eth_backend_stat_last_error_code_sys_w),
        .ptp_txstamp_ns_latched      (ptp_txstamp_ns_latched_r),
        .ptp_txstamp_valid_latched   (ptp_txstamp_valid_latched_r),
        .ptp_cur_time_ns             (ptp_sys_tod_time_ns_w),
        .ptp_cur_time_valid          (ptp_sys_tod_time_valid_qualified_w),
        .ptp_txstamp_overflow_sticky (ptp_txstamp_overflow_sticky_r),
        .ptp_tx_busy                 (ptp_tx_busy_status_sys_w),
        .ptp_tx_cmd_pending          (ptp_tx_cmd_pending_status_sys_w),
        .ptp_rx_mbox_pop_pulse       (ptp_rx_mbox_pop_pulse),
        .ptp_rx_mbox_valid           (ptp_rx_mbox_valid_w),
        .ptp_rx_mbox_msg_type        (ptp_rx_mbox_msg_type_w),
        .ptp_rx_mbox_sequence_id     (ptp_rx_mbox_sequence_id_w),
        .ptp_rx_mbox_rx_timestamp_ns (ptp_rx_mbox_rx_timestamp_ns_w),
        .ptp_rx_mbox_msg_timestamp_80(ptp_rx_mbox_msg_timestamp_80_w),
        .ptp_rx_mbox_requesting_port_identity(ptp_rx_mbox_requesting_port_identity_w),
        .ptp_rx_mbox_overflow_sticky (ptp_rx_mbox_overflow_sticky_w),
        .dbgext_arb_data_frame_count (dbgext_arb_data_frame_count_sys_w),
        .dbgext_arb_ptp_frame_count  (dbgext_arb_ptp_frame_count_sys_w),
        .dbgext_arb_stall_cycles     (dbgext_arb_stall_cycles_sys_w),
        .dbgext_arb_last_error       (dbgext_arb_last_error_sys_w),
        .dbgext_backend_backpressure_cycles(dbgext_backend_backpressure_cycles_sys_w),
        .dbgext_ptp_tx_trigger_req_count(dbgext_ptp_tx_trigger_req_count_sys_w),
        .dbgext_ptp_tx_trigger_accept_count(dbgext_ptp_tx_trigger_accept_count_sys_w),
        .dbgext_ptp_tx_trigger_while_busy_count(dbgext_ptp_tx_trigger_while_busy_count_sys_w),
        .dbgext_ptp_tx_last_error    (dbgext_ptp_tx_last_error_sys_w),
        .dbgext_ptp_rx_accept_count  (ptp_rx_accept_count_sys_w),
        .dbgext_ptp_rx_drop_count    (ptp_rx_drop_count_sys_w),
        .dbgext_ptp_rx_last_error    (ptp_rx_last_error_sys_w),
        .dbgext_ptp_rx_status        ({31'd0, ptp_rx_fifo_overflow_sticky_sys_w}),
        .txs_stat_status             (txs_stat_status_sys_w),
        .txs_stat_held_count         (txs_stat_held_count_sys_w),
        .txs_stat_pass_count         (txs_stat_pass_count_sys_w),
        .txs_stat_late_count         (txs_stat_late_count_sys_w),
        .txs_stat_slot_miss_count    (txs_stat_slot_miss_count_sys_w),
        .txs_stat_last_plan_ns       (txs_stat_last_plan_ns_sys_w),
        .txs_stat_last_actual_ns     (txs_stat_last_actual_ns_sys_w),
        .txs_stat_last_delta_ns      (txs_stat_last_delta_ns_sys_w),
        .txs_active_enable           (txs_active_enable_sys_w),
        .txs_active_block_when_time_invalid(txs_active_block_when_time_invalid_sys_w),
        .txs_active_strict_window    (txs_active_strict_window_sys_w),
        .txs_active_base_time_ns     (txs_active_base_time_ns_sys_w),
        .txs_active_period_ns        (txs_active_period_ns_sys_w),
        .txs_active_slot_offset_ns   (txs_active_slot_offset_ns_sys_w),
        .txs_active_slot_width_ns    (txs_active_slot_width_ns_sys_w),
        .txs_active_advance_ns       (txs_active_advance_ns_sys_w),
        .txs_active_late_tol_ns      (txs_active_late_tol_ns_sys_w),
        .cfg_phy_soft_reset_pulse    (cfg_phy_soft_reset_pulse),
        .cfg_phy_restart_an_pulse    (cfg_phy_restart_an_pulse),
        .phy_status_word             (phy_status_word_sys_w),
        .phy_reg_snapshot            (phy_reg_snapshot_sys_w)
    );

    // ------------------------------------------------------------------------
    // Consolidated unused keepers for reserved inputs / debug-only sidebands.
    // ------------------------------------------------------------------------
    assign unused_keep_w = tod_time_ns[0] |
                           tod_time_valid |
                           cdc_fifo_full_sys_w |
                           cdc_fifo_empty_eth_w |
                           eth_backend_stat_ingress_busy_eth_w |
                           dbgext_status_update_pulse_sys_w |
                           txs_status_update_pulse_sys_w;

endmodule

`default_nettype wire
