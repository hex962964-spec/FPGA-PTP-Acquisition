`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module: net_acq_csr
// Description:
//   Unified CSR register block for:
//     1) acquisition_core front-end control / status
//     2) wrapper-level debug observation registers
//     3) Ethernet/UDP backend shadow configuration + status/counters
//     4) PTP reserve register space (configuration reserve + latched txstamp readback)
//
// Notes:
//   - 0x00 ~ 0x44 keeps the original acquisition/debug register layout.
//   - 0x80 ~ 0xBC is the new Ethernet/UDP backend register group.
//   - 0xC0 ~ 0xFC holds compact PTP control/status.
//   - 0x100 ~ 0x124 holds the minimal PTP RX mailbox extension.
//   - 0x140+ is reserved for debug/scheduler-extension observability.
//   - 0x1EC ~ 0x1F8 adds the optional PTP peer-address split shadow bank.
//   - Ethernet network parameters are SHADOW registers here. They become ACTIVE
//     in net_acq_wrapper only after cfg_eth_commit_pulse is accepted.
//////////////////////////////////////////////////////////////////////////////////

module net_acq_csr(
    input  wire         clk_in,
    input  wire         resetn,

    // ------------------------------------------------------------------------
    // Simplified local CSR interface
    // ------------------------------------------------------------------------
    input  wire         csr_en,
    input  wire         csr_we,
    input  wire [11:0]  csr_addr,
    input  wire [31:0]  csr_wdata,
    output reg  [31:0]  csr_rdata,
    output wire         csr_ready,

    // ------------------------------------------------------------------------
    // acquisition_core control outputs
    // ------------------------------------------------------------------------
    output reg          cfg_enable,
    output reg          cfg_soft_reset_pulse,
    output reg          cfg_clear_status_pulse,
    output reg  [31:0]  cfg_sample_period,
    output reg  [15:0]  cfg_samples_per_packet,
    output reg  [3:0]   cfg_mode,
    output reg  [15:0]  cfg_channel_id,
    output reg          cfg_src_sel,
    output reg  [1:0]   cfg_time_quality,

    // ------------------------------------------------------------------------
    // Ethernet/UDP backend SHADOW configuration outputs
    // ------------------------------------------------------------------------
    output reg          cfg_eth_backend_enable,
    output reg          cfg_eth_commit_pulse,
    output reg          cfg_eth_tx_mirror_enable,
    output reg          cfg_eth_append_fcs,
    output reg          cfg_eth_udp_checksum_enable,
    output reg          cfg_eth_stall_drop_enable,
    output reg  [47:0]  cfg_eth_local_mac,
    output reg  [47:0]  cfg_eth_dst_mac,
    output reg  [31:0]  cfg_eth_local_ip,
    output reg  [31:0]  cfg_eth_dst_ip,
    output reg  [15:0]  cfg_eth_src_port,
    output reg  [15:0]  cfg_eth_dst_port,
    output reg  [7:0]   cfg_eth_ttl,
    output reg  [7:0]   cfg_eth_dscp_ecn,
    output reg  [15:0]  cfg_eth_ip_ident_seed,

    // ------------------------------------------------------------------------
    // Optional PTP peer-address split shadow bank
    //   local_mac/local_ip keeps describing the DUT itself.
    //   cfg_eth_dst_* remains the normal UDP data-plane destination.
    //   When cfg_ptp_addr_split_enable=1, active PTP TX/RX matching uses
    //   cfg_ptp_peer_mac/cfg_ptp_peer_ip as the observe/peer address.
    // ------------------------------------------------------------------------
    output reg          cfg_ptp_addr_split_enable,
    output reg  [47:0]  cfg_ptp_peer_mac,
    output reg  [31:0]  cfg_ptp_peer_ip,

    // ------------------------------------------------------------------------
    // TSN-lite v1 scheduler shadow configuration
    //   These fields are committed together with the normal ETH shadow bank.
    // ------------------------------------------------------------------------
    output reg          cfg_txs_enable,
    output reg          cfg_txs_block_when_time_invalid,
    output reg          cfg_txs_strict_window,
    output reg  [63:0]  cfg_txs_base_time_ns,
    output reg  [31:0]  cfg_txs_period_ns,
    output reg  [31:0]  cfg_txs_slot_offset_ns,
    output reg  [31:0]  cfg_txs_slot_width_ns,
    output reg  [31:0]  cfg_txs_advance_ns,
    output reg  [31:0]  cfg_txs_late_tol_ns,

    // ------------------------------------------------------------------------
    // PTP reserve SHADOW configuration outputs
    // ------------------------------------------------------------------------
    output reg          cfg_ptp_enable,
    output reg          cfg_ptp_one_step,
    output reg          cfg_ptp_two_step,
    output reg          cfg_ptp_event_priority_enable,
    output reg          cfg_ptp_hold,
    output reg          cfg_ptp_strict_priority,
    output reg  [7:0]   cfg_ptp_domain,
    output reg  [63:0]  cfg_ptp_clock_id,
    output reg  [15:0]  cfg_ptp_port_num,
    output reg          cfg_ptp_update_pulse,
    output reg          cfg_ptp_set_time_pulse,
    output reg          cfg_ptp_step_pulse,
    output reg          cfg_ptp_tx_abort_pulse,
    output reg          cfg_ptp_tx_trigger_pulse,
    output reg  [63:0]  cfg_ptp_set_time_ns,
    output reg  [31:0]  cfg_ptp_step_ns,
    output reg  [31:0]  cfg_ptp_freq_adj_ppb,
    output reg  [3:0]   cfg_ptp_tx_message_type,
    output reg  [3:0]   cfg_ptp_tx_transport_specific,
    output reg  [15:0]  cfg_ptp_tx_sequence_id,
    output reg  [15:0]  cfg_ptp_tx_flags,
    output reg  [7:0]   cfg_ptp_tx_log_message_interval,
    output reg          cfg_ptp_tx_need_timestamp,

    // ------------------------------------------------------------------------
    // acquisition/debug status inputs
    // ------------------------------------------------------------------------
    input  wire         stat_busy,
    input  wire         stat_error,
    input  wire [31:0]  stat_fifo_level,
    input  wire [31:0]  stat_packet_count,
    input  wire [63:0]  stat_last_timestamp,

    input  wire         dbg_core_error,
    input  wire         dbg_error_sticky,
    input  wire [31:0]  dbg_good_packet_count,
    input  wire [31:0]  dbg_bad_packet_count,
    input  wire [31:0]  dbg_last_seq_id,
    input  wire [63:0]  dbg_last_timestamp,
    input  wire [15:0]  dbg_last_sample_count,
    input  wire [15:0]  dbg_last_payload_count,
    input  wire [15:0]  dbg_last_error_code,
    input  wire [15:0]  dbg_last_error_word_index,
    input  wire [7:0]   dbg_last_protocol_version,
    input  wire [1:0]   dbg_last_quality,

    // ------------------------------------------------------------------------
    // Ethernet backend status inputs
    // ------------------------------------------------------------------------
    input  wire         eth_stat_tx_busy,
    input  wire         eth_stat_tx_error_sticky,
    input  wire         eth_stat_pkt_buf_overflow,
    input  wire         eth_stat_mac_stall_timeout,
    input  wire         eth_stat_cfg_active,
    input  wire         eth_stat_backend_selected,
    input  wire         eth_stat_link_up,
    input  wire [31:0]  eth_stat_tx_packet_count,
    input  wire [31:0]  eth_stat_tx_byte_count,
    input  wire [31:0]  eth_stat_drop_count,
    input  wire [15:0]  eth_stat_last_ip_total_length,
    input  wire [15:0]  eth_stat_last_udp_length,
    input  wire [15:0]  eth_stat_last_error_code,

    // ------------------------------------------------------------------------
    // PTP latched status inputs / live time readback
    // ------------------------------------------------------------------------
    input  wire [63:0]  ptp_txstamp_ns_latched,
    input  wire         ptp_txstamp_valid_latched,
    input  wire [63:0]  ptp_cur_time_ns,
    input  wire         ptp_cur_time_valid,
    input  wire         ptp_txstamp_overflow_sticky,
    input  wire         ptp_tx_busy,
    input  wire         ptp_tx_cmd_pending,

    // ------------------------------------------------------------------------
    // PTP RX mailbox (minimal closed-loop RX path)
    // ------------------------------------------------------------------------
    output reg          ptp_rx_mbox_pop_pulse,
    input  wire         ptp_rx_mbox_valid,
    input  wire [3:0]   ptp_rx_mbox_msg_type,
    input  wire [15:0]  ptp_rx_mbox_sequence_id,
    input  wire [63:0]  ptp_rx_mbox_rx_timestamp_ns,
    input  wire [79:0]  ptp_rx_mbox_msg_timestamp_80,
    input  wire [79:0]  ptp_rx_mbox_requesting_port_identity,
    input  wire         ptp_rx_mbox_overflow_sticky,

    // ------------------------------------------------------------------------
    // Debug / scheduler-extension observability (sys domain, RO)
    // ------------------------------------------------------------------------
    input  wire [31:0]  dbgext_arb_data_frame_count,
    input  wire [31:0]  dbgext_arb_ptp_frame_count,
    input  wire [31:0]  dbgext_arb_stall_cycles,
    input  wire [15:0]  dbgext_arb_last_error,
    input  wire [31:0]  dbgext_backend_backpressure_cycles,
    input  wire [31:0]  dbgext_ptp_tx_trigger_req_count,
    input  wire [31:0]  dbgext_ptp_tx_trigger_accept_count,
    input  wire [31:0]  dbgext_ptp_tx_trigger_while_busy_count,
    input  wire [15:0]  dbgext_ptp_tx_last_error,
    input  wire [31:0]  dbgext_ptp_rx_accept_count,
    input  wire [31:0]  dbgext_ptp_rx_drop_count,
    input  wire [15:0]  dbgext_ptp_rx_last_error,
    input  wire [31:0]  dbgext_ptp_rx_status,

    // ------------------------------------------------------------------------
    // TSN-lite v1 scheduler observability (sys domain, RO)
    // ------------------------------------------------------------------------
    input  wire [31:0]  txs_stat_status,
    input  wire [31:0]  txs_stat_held_count,
    input  wire [31:0]  txs_stat_pass_count,
    input  wire [31:0]  txs_stat_late_count,
    input  wire [31:0]  txs_stat_slot_miss_count,
    input  wire [63:0]  txs_stat_last_plan_ns,
    input  wire [63:0]  txs_stat_last_actual_ns,
    input  wire [31:0]  txs_stat_last_delta_ns,

    // ------------------------------------------------------------------------
    // TSN-lite v1 ACTIVE scheduler bank (sys domain, RO)
    //   These signals reflect the committed ACTIVE configuration after the ETH
    //   shadow->active safe-commit FSM has completed. They are intentionally
    //   separate from the TXS shadow registers so software can distinguish:
    //     - what was requested (shadow)
    //     - what is actually in force (active)
    // ------------------------------------------------------------------------
    input  wire         txs_active_enable,
    input  wire         txs_active_block_when_time_invalid,
    input  wire         txs_active_strict_window,
    input  wire [63:0]  txs_active_base_time_ns,
    input  wire [31:0]  txs_active_period_ns,
    input  wire [31:0]  txs_active_slot_offset_ns,
    input  wire [31:0]  txs_active_slot_width_ns,
    input  wire [31:0]  txs_active_advance_ns,
    input  wire [31:0]  txs_active_late_tol_ns,

    // ------------------------------------------------------------------------
    // PHY management control / status
    // ------------------------------------------------------------------------
    output reg          cfg_phy_soft_reset_pulse,
    output reg          cfg_phy_restart_an_pulse,
    input  wire [31:0]  phy_status_word,
    input  wire [127:0] phy_reg_snapshot
);

    // ------------------------------------------------------------------------
    // Register address map (front-end legacy)
    // ------------------------------------------------------------------------
    localparam [11:0] ADDR_CTRL               = 8'h00;
    localparam [11:0] ADDR_MODE               = 8'h04;
    localparam [11:0] ADDR_SAMPLE_PERIOD      = 8'h08;
    localparam [11:0] ADDR_SAMPLES_PER_PACKET = 8'h0C;
    localparam [11:0] ADDR_CHANNEL_ID         = 8'h10;
    localparam [11:0] ADDR_STATUS             = 8'h14;
    localparam [11:0] ADDR_PACKET_COUNT       = 8'h18;
    localparam [11:0] ADDR_LAST_TIMESTAMP_L   = 8'h1C;
    localparam [11:0] ADDR_LAST_TIMESTAMP_H   = 8'h20;
    localparam [11:0] ADDR_FIFO_LEVEL         = 8'h24;
    localparam [11:0] ADDR_DBG_STATUS         = 8'h28;
    localparam [11:0] ADDR_DBG_GOOD_PKT_CNT   = 8'h2C;
    localparam [11:0] ADDR_DBG_BAD_PKT_CNT    = 8'h30;
    localparam [11:0] ADDR_DBG_LAST_SEQ_ID    = 8'h34;
    localparam [11:0] ADDR_DBG_LAST_TS_L      = 8'h38;
    localparam [11:0] ADDR_DBG_LAST_TS_H      = 8'h3C;
    localparam [11:0] ADDR_DBG_LAST_COUNTS    = 8'h40;
    localparam [11:0] ADDR_DBG_LAST_ERROR     = 8'h44;
    localparam [11:0] ADDR_DBG_LAST_QUALITY   = 8'h48;

    // ------------------------------------------------------------------------
    // PTP RX mailbox register group
    //   说明：
    //     - 原先临时放在 0x48~0x70 的 RX mailbox 现整体迁移到 PTP 扩展区。
    //     - 保留 0xC0~0xFC 的 compact PTP 控制/状态布局不变，避免影响现有
    //       TX-only / PTP TX bring-up 软件。
    //     - 新的 RX mailbox 从 0x100 开始，逻辑上属于“0xD8 之后的 PTP 运行态扩展”。
    // ------------------------------------------------------------------------
    localparam [11:0] ADDR_PTP_RX_MBOX_STATUS = 12'h100;
    localparam [11:0] ADDR_PTP_RX_RXTS_L      = 12'h104;
    localparam [11:0] ADDR_PTP_RX_RXTS_H      = 12'h108;
    localparam [11:0] ADDR_PTP_RX_MSGTS_L     = 12'h10C;
    localparam [11:0] ADDR_PTP_RX_MSGTS_M     = 12'h110;
    localparam [11:0] ADDR_PTP_RX_MSGTS_H     = 12'h114;
    localparam [11:0] ADDR_PTP_RX_REQPORT_L   = 12'h118;
    localparam [11:0] ADDR_PTP_RX_REQPORT_M   = 12'h11C;
    localparam [11:0] ADDR_PTP_RX_REQPORT_H   = 12'h120;
    localparam [11:0] ADDR_PTP_RX_MBOX_CTRL   = 12'h124;
    localparam [11:0] ADDR_PHY_CTRL           = 12'h128;
    localparam [11:0] ADDR_PHY_STATUS         = 12'h12C;
    localparam [11:0] ADDR_PHY_ID             = 12'h130;
    localparam [11:0] ADDR_PHY_BASIC          = 12'h134;
    localparam [11:0] ADDR_PHY_AN             = 12'h138;
    localparam [11:0] ADDR_PHY_GIG            = 12'h13C;

    // ------------------------------------------------------------------------
    // Debug / scheduler-extension register group (read-only in this phase)
    //   These registers provide the observability layer needed before the
    //   future data_tx_scheduler is inserted. They intentionally live above
    //   0x13C so the stable legacy map is untouched.
    // ------------------------------------------------------------------------
    localparam [11:0] ADDR_DBGEXT_FEATURES                = 12'h140;
    localparam [11:0] ADDR_DBGEXT_ARB_DATA_FRAME_COUNT    = 12'h144;
    localparam [11:0] ADDR_DBGEXT_ARB_PTP_FRAME_COUNT     = 12'h148;
    localparam [11:0] ADDR_DBGEXT_ARB_STALL_CYCLES        = 12'h14C;
    localparam [11:0] ADDR_DBGEXT_ARB_LAST_ERROR          = 12'h150;
    localparam [11:0] ADDR_DBGEXT_BACKPRESSURE_CYCLES     = 12'h154;
    localparam [11:0] ADDR_DBGEXT_PTP_TX_TRIGGER_REQ_CNT  = 12'h158;
    localparam [11:0] ADDR_DBGEXT_PTP_TX_TRIGGER_ACC_CNT  = 12'h15C;
    localparam [11:0] ADDR_DBGEXT_PTP_TX_TRIGGER_BUSY_CNT = 12'h160;
    localparam [11:0] ADDR_DBGEXT_PTP_TX_LAST_ERROR       = 12'h164;
    localparam [11:0] ADDR_DBGEXT_PTP_RX_ACCEPT_COUNT     = 12'h168;
    localparam [11:0] ADDR_DBGEXT_PTP_RX_DROP_COUNT       = 12'h16C;
    localparam [11:0] ADDR_DBGEXT_PTP_RX_LAST_ERROR       = 12'h170;
    localparam [11:0] ADDR_DBGEXT_PTP_RX_STATUS           = 12'h174;
    localparam [11:0] ADDR_TIME_QUALITY                    = 12'h178;

    // ------------------------------------------------------------------------
    // TSN-lite v1 scheduler shadow / status group
    // ------------------------------------------------------------------------
    localparam [11:0] ADDR_TXS_CTRL               = 12'h180;
    localparam [11:0] ADDR_TXS_BASE_TIME_L        = 12'h184;
    localparam [11:0] ADDR_TXS_BASE_TIME_H        = 12'h188;
    localparam [11:0] ADDR_TXS_PERIOD_NS          = 12'h18C;
    localparam [11:0] ADDR_TXS_SLOT_OFFSET_NS     = 12'h190;
    localparam [11:0] ADDR_TXS_SLOT_WIDTH_NS      = 12'h194;
    localparam [11:0] ADDR_TXS_ADVANCE_NS         = 12'h198;
    localparam [11:0] ADDR_TXS_LATE_TOL_NS        = 12'h19C;
    localparam [11:0] ADDR_TXS_STATUS             = 12'h1A0;
    localparam [11:0] ADDR_TXS_HELD_COUNT         = 12'h1A4;
    localparam [11:0] ADDR_TXS_PASS_COUNT         = 12'h1A8;
    localparam [11:0] ADDR_TXS_LATE_COUNT         = 12'h1AC;
    localparam [11:0] ADDR_TXS_SLOT_MISS_COUNT    = 12'h1B0;
    localparam [11:0] ADDR_TXS_LAST_PLAN_L        = 12'h1B4;
    localparam [11:0] ADDR_TXS_LAST_PLAN_H        = 12'h1B8;
    localparam [11:0] ADDR_TXS_LAST_ACTUAL_L      = 12'h1BC;
    localparam [11:0] ADDR_TXS_LAST_ACTUAL_H      = 12'h1C0;
    localparam [11:0] ADDR_TXS_LAST_DELTA_NS      = 12'h1C4;
    localparam [11:0] ADDR_TXS_FEATURES           = 12'h1C8;
    localparam [11:0] ADDR_TXS_ACTIVE_CTRL        = 12'h1CC;
    localparam [11:0] ADDR_TXS_ACTIVE_BASE_TIME_L = 12'h1D0;
    localparam [11:0] ADDR_TXS_ACTIVE_BASE_TIME_H = 12'h1D4;
    localparam [11:0] ADDR_TXS_ACTIVE_PERIOD_NS   = 12'h1D8;
    localparam [11:0] ADDR_TXS_ACTIVE_SLOT_OFFSET_NS = 12'h1DC;
    localparam [11:0] ADDR_TXS_ACTIVE_SLOT_WIDTH_NS  = 12'h1E0;
    localparam [11:0] ADDR_TXS_ACTIVE_ADVANCE_NS  = 12'h1E4;
    localparam [11:0] ADDR_TXS_ACTIVE_LATE_TOL_NS = 12'h1E8;
    localparam [11:0] ADDR_PTP_PEER_MAC_L        = 12'h1EC;
    localparam [11:0] ADDR_PTP_PEER_MAC_H        = 12'h1F0;
    localparam [11:0] ADDR_PTP_PEER_IP           = 12'h1F4;
    localparam [11:0] ADDR_PTP_ADDR_CFG          = 12'h1F8;

    // 0x1FC+ reserved for future TSN-lite v2 metadata / launch-time extensions.

    // ------------------------------------------------------------------------
    // SAMPLES_PER_PACKET safety policy
    // ------------------------------------------------------------------------
    // The front-end sample FIFO stores up to 256 payload samples. The packet
    // ingress buffer is sized for a 10-word acquisition header plus up to 256
    // payload words. Clamp the software-visible CSR value at write time so
    // downstream modules, CSR readback, and packet length calculation all see
    // the same safe effective value.
    localparam [15:0] C_SAMPLES_PER_PACKET_MIN     = 16'd1;
    localparam [15:0] C_SAMPLES_PER_PACKET_MAX     = 16'd256;
    localparam [15:0] C_SAMPLES_PER_PACKET_DEFAULT = 16'd4;

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
    // Ethernet/UDP backend register group
    // ------------------------------------------------------------------------
    localparam [11:0] ADDR_ETH_CTRL           = 8'h80;
    localparam [11:0] ADDR_ETH_STATUS         = 8'h84;
    localparam [11:0] ADDR_LOCAL_MAC_L        = 8'h88;
    localparam [11:0] ADDR_LOCAL_MAC_H        = 8'h8C;
    localparam [11:0] ADDR_DST_MAC_L          = 8'h90;
    localparam [11:0] ADDR_DST_MAC_H          = 8'h94;
    localparam [11:0] ADDR_LOCAL_IP           = 8'h98;
    localparam [11:0] ADDR_DST_IP             = 8'h9C;
    localparam [11:0] ADDR_UDP_PORTS          = 8'hA0;
    localparam [11:0] ADDR_IPV4_CFG           = 8'hA4;
    localparam [11:0] ADDR_ETH_TX_PKT_COUNT   = 8'hA8;
    localparam [11:0] ADDR_ETH_TX_BYTE_COUNT  = 8'hAC;
    localparam [11:0] ADDR_ETH_DROP_COUNT     = 8'hB0;
    localparam [11:0] ADDR_ETH_LAST_LENGTHS   = 8'hB4;
    localparam [11:0] ADDR_ETH_LAST_ERROR     = 8'hB8;
    localparam [11:0] ADDR_ETH_FEATURES       = 8'hBC;

    // ------------------------------------------------------------------------
    // PTP reserve register group
    // ------------------------------------------------------------------------
    localparam [11:0] ADDR_PTP_CFG            = 8'hC0;
    localparam [11:0] ADDR_PTP_CLOCK_ID_L     = 8'hC4;
    localparam [11:0] ADDR_PTP_CLOCK_ID_H     = 8'hC8;
    localparam [11:0] ADDR_PTP_PORT_NUM       = 8'hCC;
    localparam [11:0] ADDR_PTP_TXSTAMP_L      = 8'hD0;
    localparam [11:0] ADDR_PTP_TXSTAMP_H      = 8'hD4;
    localparam [11:0] ADDR_PTP_STATUS         = 8'hD8;
    // Compact PTP control extension (aligned to current C test program)
    localparam [11:0] ADDR_PTP_CTRL           = 8'hDC;
    localparam [11:0] ADDR_PTP_TOD_SET_L      = 8'hE0;
    localparam [11:0] ADDR_PTP_TOD_SET_H      = 8'hE4;
    localparam [11:0] ADDR_PTP_STEP_NS        = 8'hE8;
    localparam [11:0] ADDR_PTP_FREQ_ADJ       = 8'hEC;
    localparam [11:0] ADDR_PTP_CUR_TIME_L     = 8'hF0;
    localparam [11:0] ADDR_PTP_CUR_TIME_H     = 8'hF4;
    localparam [11:0] ADDR_PTP_TX_CFG         = 8'hF8;
    localparam [11:0] ADDR_PTP_FEATURES       = 8'hFC;

    localparam [31:0] PTP_FEATURES_VALUE = {
        16'h0002, // compact PTP contract version (v2 adds peer-address split shadow bank)
        9'd0,
        1'b1,     // bit6 : independent PTP peer-address split registers present
        6'h3F     // bit5:0 legacy compact-PTP feature mask (unchanged)
    };
    localparam [31:0] DBGEXT_FEATURES_VALUE = {
        16'h0002, // extension version
        9'd0,
        1'b1,     // bit6 : scheduler launch-time observability present
        1'b1,     // bit5 : scheduler control registers present
        1'b1,     // bit4 : PTP RX observability present
        1'b1,     // bit3 : PTP TX observability present
        1'b1,     // bit2 : backend backpressure observability present
        1'b1,     // bit1 : tx_arbiter observability present
        1'b1      // bit0 : debug/scheduler extension present
    };
    localparam [31:0] TXS_FEATURES_VALUE = {
        16'h0001, // TSN-lite v1 contract version
        8'd0,
        1'b0,     // bit7 : MAC-edge actual launch timestamp present (v2)
        1'b0,     // bit6 : per-packet launch-time mode present (v2)
        1'b1,     // bit5 : active scheduler bank readback present
        1'b1,     // bit4 : last_plan/last_actual/last_delta present
        1'b1,     // bit3 : held/pass/late/miss counters present
        1'b1,     // bit2 : strict_window / block_when_time_invalid controls present
        1'b1,     // bit1 : shadow + active split present
        1'b1      // bit0 : TSN-lite scheduler block present
    };

    // ------------------------------------------------------------------------
    // Request pipeline: sample request in current cycle, respond in next cycle
    // ------------------------------------------------------------------------
    reg         req_valid_r;
    reg         req_we_r;
    reg  [11:0] req_addr_r;
    reg  [31:0] req_wdata_r;

    // PTP_CTRL 里有一部分位只是软件接口语义（如 tod_enable / TX 命令字段）。
    // 当前比赛版主动 TX 已收口为 Delay_Req，因此 need_timestamp / msg_type
    // 读回值保持为固定兼容语义，避免软件把本节点当成通用 PTP TX 端。
    localparam [3:0] C_PTP_TX_MSG_DELAY_REQ = 4'h1;
    reg         cfg_ptp_tod_enable_r;

    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            req_valid_r <= 1'b0;
            req_we_r    <= 1'b0;
            req_addr_r  <= 12'd0;
            req_wdata_r <= 32'd0;
        end else begin
            req_valid_r <= csr_en;
            req_we_r    <= csr_we;
            req_addr_r  <= csr_addr;
            req_wdata_r <= csr_wdata;
        end
    end

    assign csr_ready = req_valid_r;

    // ------------------------------------------------------------------------
    // Write path
    // ------------------------------------------------------------------------
    always @(posedge clk_in or negedge resetn) begin
        if (!resetn) begin
            cfg_enable                     <= 1'b0;
            cfg_soft_reset_pulse           <= 1'b0;
            cfg_clear_status_pulse         <= 1'b0;
            cfg_sample_period              <= 32'd0;
            cfg_samples_per_packet         <= C_SAMPLES_PER_PACKET_DEFAULT;
            cfg_mode                       <= 4'd0;
            cfg_channel_id                 <= 16'd0;
            cfg_src_sel                    <= 1'b0;
            cfg_time_quality               <= 2'd0;

            cfg_eth_backend_enable         <= 1'b0;
            cfg_eth_commit_pulse           <= 1'b0;
            cfg_eth_tx_mirror_enable       <= 1'b1;
            cfg_eth_append_fcs             <= 1'b0;
            cfg_eth_udp_checksum_enable    <= 1'b0;
            cfg_eth_stall_drop_enable      <= 1'b0;
            cfg_eth_local_mac              <= 48'd0;
            cfg_eth_dst_mac                <= 48'd0;
            cfg_eth_local_ip               <= 32'd0;
            cfg_eth_dst_ip                 <= 32'd0;
            cfg_eth_src_port               <= 16'd0;
            cfg_eth_dst_port               <= 16'd0;
            cfg_eth_ttl                    <= 8'd64;
            cfg_eth_dscp_ecn               <= 8'd0;
            cfg_eth_ip_ident_seed          <= 16'd0;
            cfg_ptp_addr_split_enable       <= 1'b0;
            cfg_ptp_peer_mac                <= 48'd0;
            cfg_ptp_peer_ip                 <= 32'd0;
            cfg_txs_enable                 <= 1'b0;
            cfg_txs_block_when_time_invalid<= 1'b1;
            cfg_txs_strict_window          <= 1'b1;
            cfg_txs_base_time_ns           <= 64'd0;
            cfg_txs_period_ns              <= 32'd0;
            cfg_txs_slot_offset_ns         <= 32'd0;
            cfg_txs_slot_width_ns          <= 32'd0;
            cfg_txs_advance_ns             <= 32'd0;
            cfg_txs_late_tol_ns            <= 32'd0;

            cfg_ptp_enable                 <= 1'b0;
            cfg_ptp_one_step               <= 1'b0;
            cfg_ptp_two_step               <= 1'b0;
            cfg_ptp_event_priority_enable  <= 1'b0;
            cfg_ptp_hold                   <= 1'b0;
            cfg_ptp_strict_priority        <= 1'b0;
            cfg_ptp_domain                 <= 8'd0;
            cfg_ptp_clock_id               <= 64'd0;
            cfg_ptp_port_num               <= 16'd0;
            cfg_ptp_update_pulse           <= 1'b0;
            cfg_ptp_set_time_pulse         <= 1'b0;
            cfg_ptp_step_pulse             <= 1'b0;
            cfg_ptp_tx_abort_pulse         <= 1'b0;
            cfg_ptp_tx_trigger_pulse       <= 1'b0;
            cfg_ptp_set_time_ns            <= 64'd0;
            cfg_ptp_step_ns                <= 32'd0;
            cfg_ptp_freq_adj_ppb           <= 32'd0;
            cfg_ptp_tx_message_type        <= C_PTP_TX_MSG_DELAY_REQ;
            cfg_ptp_tx_transport_specific  <= 4'd0;
            cfg_ptp_tx_sequence_id         <= 16'd0;
            cfg_ptp_tx_flags               <= 16'd0;
            cfg_ptp_tx_log_message_interval<= 8'd0;
            cfg_ptp_tod_enable_r           <= 1'b0;
            cfg_ptp_tx_need_timestamp      <= 1'b1;
            ptp_rx_mbox_pop_pulse          <= 1'b0;
            cfg_phy_soft_reset_pulse       <= 1'b0;
            cfg_phy_restart_an_pulse       <= 1'b0;
        end else begin
            cfg_soft_reset_pulse   <= 1'b0;
            cfg_clear_status_pulse <= 1'b0;
            cfg_eth_commit_pulse   <= 1'b0;
            cfg_ptp_update_pulse     <= 1'b0;
            cfg_ptp_set_time_pulse   <= 1'b0;
            cfg_ptp_step_pulse       <= 1'b0;
            cfg_ptp_tx_abort_pulse   <= 1'b0;
            cfg_ptp_tx_trigger_pulse <= 1'b0;
            ptp_rx_mbox_pop_pulse  <= 1'b0;
            cfg_phy_soft_reset_pulse <= 1'b0;
            cfg_phy_restart_an_pulse <= 1'b0;

            if (req_valid_r && req_we_r) begin
                case (req_addr_r)
                    ADDR_CTRL: begin
                        cfg_enable <= req_wdata_r[0];
                        if (req_wdata_r[1])
                            cfg_soft_reset_pulse <= 1'b1;
                        if (req_wdata_r[2])
                            cfg_clear_status_pulse <= 1'b1;
                        cfg_src_sel <= req_wdata_r[3];
                    end

                    ADDR_MODE: begin
                        cfg_mode <= req_wdata_r[3:0];
                    end

                    ADDR_SAMPLE_PERIOD: begin
                        cfg_sample_period <= req_wdata_r;
                    end

                    ADDR_SAMPLES_PER_PACKET: begin
                        cfg_samples_per_packet <= f_sanitize_samples_per_packet(req_wdata_r[15:0]);
                    end

                    ADDR_CHANNEL_ID: begin
                        cfg_channel_id <= req_wdata_r[15:0];
                    end

                    ADDR_TIME_QUALITY: begin
                        cfg_time_quality <= req_wdata_r[1:0];
                    end

                    ADDR_PTP_RX_MBOX_CTRL: begin
                        // bit0: pop 当前 mailbox 中的一条消息
                        if (req_wdata_r[0])
                            ptp_rx_mbox_pop_pulse <= 1'b1;
                    end

                    ADDR_PHY_CTRL: begin
                        // bit0: soft reset pulse
                        // bit1: restart autoneg pulse
                        if (req_wdata_r[0])
                            cfg_phy_soft_reset_pulse <= 1'b1;
                        if (req_wdata_r[1])
                            cfg_phy_restart_an_pulse <= 1'b1;
                    end

                    ADDR_ETH_CTRL: begin
                        cfg_eth_backend_enable      <= req_wdata_r[0];
                        if (req_wdata_r[1])
                            cfg_eth_commit_pulse    <= 1'b1;
                        cfg_eth_tx_mirror_enable    <= req_wdata_r[2];
                        cfg_eth_append_fcs          <= req_wdata_r[3];
                        cfg_eth_udp_checksum_enable <= req_wdata_r[4];
                        cfg_eth_stall_drop_enable   <= req_wdata_r[5];
                    end

                    ADDR_LOCAL_MAC_L: begin
                        cfg_eth_local_mac[31:0] <= req_wdata_r;
                    end

                    ADDR_LOCAL_MAC_H: begin
                        cfg_eth_local_mac[47:32] <= req_wdata_r[15:0];
                    end

                    ADDR_DST_MAC_L: begin
                        cfg_eth_dst_mac[31:0] <= req_wdata_r;
                    end

                    ADDR_DST_MAC_H: begin
                        cfg_eth_dst_mac[47:32] <= req_wdata_r[15:0];
                    end

                    ADDR_LOCAL_IP: begin
                        cfg_eth_local_ip <= req_wdata_r;
                    end

                    ADDR_DST_IP: begin
                        cfg_eth_dst_ip <= req_wdata_r;
                    end

                    ADDR_UDP_PORTS: begin
                        cfg_eth_src_port <= req_wdata_r[31:16];
                        cfg_eth_dst_port <= req_wdata_r[15:0];
                    end

                    ADDR_IPV4_CFG: begin
                        cfg_eth_ttl           <= req_wdata_r[7:0];
                        cfg_eth_dscp_ecn      <= req_wdata_r[15:8];
                        cfg_eth_ip_ident_seed <= req_wdata_r[31:16];
                    end

                    ADDR_PTP_PEER_MAC_L: begin
                        cfg_ptp_peer_mac[31:0] <= req_wdata_r;
                    end

                    ADDR_PTP_PEER_MAC_H: begin
                        cfg_ptp_peer_mac[47:32] <= req_wdata_r[15:0];
                    end

                    ADDR_PTP_PEER_IP: begin
                        cfg_ptp_peer_ip <= req_wdata_r;
                    end

                    ADDR_PTP_ADDR_CFG: begin
                        cfg_ptp_addr_split_enable <= req_wdata_r[0];
                    end

                    ADDR_TXS_CTRL: begin
                        cfg_txs_enable                  <= req_wdata_r[0];
                        cfg_txs_block_when_time_invalid <= req_wdata_r[1];
                        cfg_txs_strict_window           <= req_wdata_r[2];
                    end

                    ADDR_TXS_BASE_TIME_L: begin
                        cfg_txs_base_time_ns[31:0] <= req_wdata_r;
                    end

                    ADDR_TXS_BASE_TIME_H: begin
                        cfg_txs_base_time_ns[63:32] <= req_wdata_r;
                    end

                    ADDR_TXS_PERIOD_NS: begin
                        cfg_txs_period_ns <= req_wdata_r;
                    end

                    ADDR_TXS_SLOT_OFFSET_NS: begin
                        cfg_txs_slot_offset_ns <= req_wdata_r;
                    end

                    ADDR_TXS_SLOT_WIDTH_NS: begin
                        cfg_txs_slot_width_ns <= req_wdata_r;
                    end

                    ADDR_TXS_ADVANCE_NS: begin
                        cfg_txs_advance_ns <= req_wdata_r;
                    end

                    ADDR_TXS_LATE_TOL_NS: begin
                        cfg_txs_late_tol_ns <= req_wdata_r;
                    end

                    ADDR_PTP_CFG: begin
                        cfg_ptp_enable                <= req_wdata_r[0];
                        cfg_ptp_one_step              <= req_wdata_r[1];
                        cfg_ptp_two_step              <= req_wdata_r[2];
                        cfg_ptp_event_priority_enable <= req_wdata_r[3];
                        cfg_ptp_hold                  <= req_wdata_r[4];
                        cfg_ptp_strict_priority       <= req_wdata_r[5];
                        cfg_ptp_domain                <= req_wdata_r[15:8];
                    end

                    ADDR_PTP_CLOCK_ID_L: begin
                        cfg_ptp_clock_id[31:0] <= req_wdata_r;
                    end

                    ADDR_PTP_CLOCK_ID_H: begin
                        cfg_ptp_clock_id[63:32] <= req_wdata_r;
                    end

                    ADDR_PTP_PORT_NUM: begin
                        cfg_ptp_port_num <= req_wdata_r[15:0];
                    end

                    ADDR_PTP_CTRL: begin
                        // 与 C 侧 compact PTP_CTRL 对齐：
                        // bit0/1 : tod_enable / tod_hold 影子控制位（可读回）
                        // bit2   : tx_abort / clear_outstanding 单拍脉冲（不读回）
                        // bit3   : set_time 单拍脉冲
                        // bit4   : step_time 单拍脉冲
                        // bit5   : TOD_FREQ_PULSE（当前通过 update_pulse 本身生效）
                        // bit6   : tx_trigger 单拍脉冲（不读回）
                        // bit7   : 兼容位；比赛版主动 TX 固定要求时间戳
                        // bit11:8: 兼容位；比赛版主动 TX 固定为 Delay_Req
                        cfg_ptp_tod_enable_r          <= req_wdata_r[0];
                        cfg_ptp_hold                  <= req_wdata_r[1];
                        cfg_ptp_tx_abort_pulse        <= req_wdata_r[2];
                        cfg_ptp_set_time_pulse        <= req_wdata_r[3];
                        cfg_ptp_step_pulse            <= req_wdata_r[4];
                        cfg_ptp_tx_trigger_pulse      <= req_wdata_r[6];
                        cfg_ptp_tx_need_timestamp     <= 1'b1;
                        cfg_ptp_tx_message_type       <= C_PTP_TX_MSG_DELAY_REQ;
                        cfg_ptp_tx_transport_specific <= req_wdata_r[15:12];
                        cfg_ptp_tx_sequence_id        <= req_wdata_r[31:16];
                        cfg_ptp_update_pulse          <= 1'b1;
                    end

                    ADDR_PTP_TOD_SET_L: begin
                        cfg_ptp_set_time_ns[31:0] <= req_wdata_r;
                    end

                    ADDR_PTP_TOD_SET_H: begin
                        cfg_ptp_set_time_ns[63:32] <= req_wdata_r;
                    end

                    ADDR_PTP_STEP_NS: begin
                        cfg_ptp_step_ns <= req_wdata_r;
                    end

                    ADDR_PTP_FREQ_ADJ: begin
                        cfg_ptp_freq_adj_ppb   <= req_wdata_r;
                    end

                    ADDR_PTP_TX_CFG: begin
                        cfg_ptp_tx_flags                <= req_wdata_r[15:0];
                        cfg_ptp_tx_log_message_interval <= req_wdata_r[23:16];
                    end

                    default: begin
                        // no-op
                    end
                endcase
            end
        end
    end

    // ------------------------------------------------------------------------
    // Read path
    // ------------------------------------------------------------------------
    always @(*) begin
        csr_rdata = 32'd0;

        if (req_valid_r && !req_we_r) begin
            case (req_addr_r)
                ADDR_CTRL: begin
                    csr_rdata = {28'd0, cfg_src_sel, 1'b0, 1'b0, cfg_enable};
                end

                ADDR_MODE: begin
                    csr_rdata = {28'd0, cfg_mode};
                end

                ADDR_SAMPLE_PERIOD: begin
                    csr_rdata = cfg_sample_period;
                end

                ADDR_SAMPLES_PER_PACKET: begin
                    csr_rdata = {16'd0, cfg_samples_per_packet};
                end

                ADDR_CHANNEL_ID: begin
                    csr_rdata = {16'd0, cfg_channel_id};
                end

                ADDR_STATUS: begin
                    csr_rdata = {30'd0, stat_error, stat_busy};
                end

                ADDR_PACKET_COUNT: begin
                    csr_rdata = stat_packet_count;
                end

                ADDR_LAST_TIMESTAMP_L: begin
                    csr_rdata = stat_last_timestamp[31:0];
                end

                ADDR_LAST_TIMESTAMP_H: begin
                    csr_rdata = stat_last_timestamp[63:32];
                end

                ADDR_FIFO_LEVEL: begin
                    csr_rdata = stat_fifo_level;
                end

                ADDR_DBG_STATUS: begin
                    csr_rdata = {28'd0, dbg_error_sticky, dbg_core_error, stat_error, stat_busy};
                end

                ADDR_DBG_GOOD_PKT_CNT: begin
                    csr_rdata = dbg_good_packet_count;
                end

                ADDR_DBG_BAD_PKT_CNT: begin
                    csr_rdata = dbg_bad_packet_count;
                end

                ADDR_DBG_LAST_SEQ_ID: begin
                    csr_rdata = dbg_last_seq_id;
                end

                ADDR_DBG_LAST_TS_L: begin
                    csr_rdata = dbg_last_timestamp[31:0];
                end

                ADDR_DBG_LAST_TS_H: begin
                    csr_rdata = dbg_last_timestamp[63:32];
                end

                ADDR_DBG_LAST_COUNTS: begin
                    csr_rdata = {dbg_last_payload_count, dbg_last_sample_count};
                end

                ADDR_DBG_LAST_ERROR: begin
                    csr_rdata = {dbg_last_error_word_index, dbg_last_error_code};
                end

                ADDR_DBG_LAST_QUALITY: begin
                    csr_rdata = {16'd0, dbg_last_protocol_version, 6'd0, dbg_last_quality};
                end

                ADDR_PTP_RX_MBOX_STATUS: begin
                    // bit0   : valid
                    // bit1   : overflow_sticky
                    // bit7:4 : msg_type
                    // bit31:16 = sequence_id
                    csr_rdata = {
                        ptp_rx_mbox_sequence_id,
                        8'd0,
                        ptp_rx_mbox_msg_type,
                        2'b00,
                        ptp_rx_mbox_overflow_sticky,
                        ptp_rx_mbox_valid
                    };
                end

                ADDR_PTP_RX_RXTS_L: begin
                    csr_rdata = ptp_rx_mbox_rx_timestamp_ns[31:0];
                end

                ADDR_PTP_RX_RXTS_H: begin
                    csr_rdata = ptp_rx_mbox_rx_timestamp_ns[63:32];
                end

                ADDR_PTP_RX_MSGTS_L: begin
                    csr_rdata = ptp_rx_mbox_msg_timestamp_80[31:0];
                end

                ADDR_PTP_RX_MSGTS_M: begin
                    csr_rdata = ptp_rx_mbox_msg_timestamp_80[63:32];
                end

                ADDR_PTP_RX_MSGTS_H: begin
                    csr_rdata = {16'd0, ptp_rx_mbox_msg_timestamp_80[79:64]};
                end

                ADDR_PTP_RX_REQPORT_L: begin
                    csr_rdata = ptp_rx_mbox_requesting_port_identity[31:0];
                end

                ADDR_PTP_RX_REQPORT_M: begin
                    csr_rdata = ptp_rx_mbox_requesting_port_identity[63:32];
                end

                ADDR_PTP_RX_REQPORT_H: begin
                    csr_rdata = {16'd0, ptp_rx_mbox_requesting_port_identity[79:64]};
                end

                ADDR_PTP_RX_MBOX_CTRL: begin
                    csr_rdata = 32'd0;
                end

                ADDR_PHY_CTRL: begin
                    csr_rdata = 32'd0;
                end

                ADDR_PHY_STATUS: begin
                    csr_rdata = phy_status_word;
                end

                ADDR_PHY_ID: begin
                    csr_rdata = {phy_reg_snapshot[127:112], phy_reg_snapshot[111:96]};
                end

                ADDR_PHY_BASIC: begin
                    csr_rdata = {phy_reg_snapshot[95:80], phy_reg_snapshot[79:64]};
                end

                ADDR_PHY_AN: begin
                    csr_rdata = {phy_reg_snapshot[63:48], phy_reg_snapshot[47:32]};
                end

                ADDR_PHY_GIG: begin
                    csr_rdata = {phy_reg_snapshot[31:16], phy_reg_snapshot[15:0]};
                end

                ADDR_ETH_CTRL: begin
                    csr_rdata = {
                        26'd0,
                        cfg_eth_stall_drop_enable,
                        cfg_eth_udp_checksum_enable,
                        cfg_eth_append_fcs,
                        cfg_eth_tx_mirror_enable,
                        1'b0,
                        cfg_eth_backend_enable
                    };
                end

                ADDR_ETH_STATUS: begin
                    csr_rdata = {
                        eth_stat_last_error_code,
                        9'd0,
                        eth_stat_link_up,
                        eth_stat_backend_selected,
                        eth_stat_cfg_active,
                        eth_stat_mac_stall_timeout,
                        eth_stat_pkt_buf_overflow,
                        eth_stat_tx_error_sticky,
                        eth_stat_tx_busy
                    };
                end

                ADDR_LOCAL_MAC_L: begin
                    csr_rdata = cfg_eth_local_mac[31:0];
                end

                ADDR_LOCAL_MAC_H: begin
                    csr_rdata = {16'd0, cfg_eth_local_mac[47:32]};
                end

                ADDR_DST_MAC_L: begin
                    csr_rdata = cfg_eth_dst_mac[31:0];
                end

                ADDR_DST_MAC_H: begin
                    csr_rdata = {16'd0, cfg_eth_dst_mac[47:32]};
                end

                ADDR_LOCAL_IP: begin
                    csr_rdata = cfg_eth_local_ip;
                end

                ADDR_DST_IP: begin
                    csr_rdata = cfg_eth_dst_ip;
                end

                ADDR_UDP_PORTS: begin
                    csr_rdata = {cfg_eth_src_port, cfg_eth_dst_port};
                end

                ADDR_IPV4_CFG: begin
                    csr_rdata = {cfg_eth_ip_ident_seed, cfg_eth_dscp_ecn, cfg_eth_ttl};
                end

                ADDR_PTP_PEER_MAC_L: begin
                    csr_rdata = cfg_ptp_peer_mac[31:0];
                end

                ADDR_PTP_PEER_MAC_H: begin
                    csr_rdata = {16'd0, cfg_ptp_peer_mac[47:32]};
                end

                ADDR_PTP_PEER_IP: begin
                    csr_rdata = cfg_ptp_peer_ip;
                end

                ADDR_PTP_ADDR_CFG: begin
                    // bit0: 0 -> PTP peer follows cfg_eth_dst_* (legacy mode)
                    //       1 -> PTP peer uses dedicated cfg_ptp_peer_* bank
                    csr_rdata = {31'd0, cfg_ptp_addr_split_enable};
                end

                ADDR_ETH_TX_PKT_COUNT: begin
                    csr_rdata = eth_stat_tx_packet_count;
                end

                ADDR_ETH_TX_BYTE_COUNT: begin
                    csr_rdata = eth_stat_tx_byte_count;
                end

                ADDR_ETH_DROP_COUNT: begin
                    csr_rdata = eth_stat_drop_count;
                end

                ADDR_ETH_LAST_LENGTHS: begin
                    csr_rdata = {eth_stat_last_ip_total_length, eth_stat_last_udp_length};
                end

                ADDR_ETH_LAST_ERROR: begin
                    csr_rdata = {16'd0, eth_stat_last_error_code};
                end

                ADDR_ETH_FEATURES: begin
                    // [31:16] version
                    // bit0  : backend present
                    // bit1  : IPv4/UDP TX path present
                    // bit2  : UDP checksum HW implemented (0: reserved)
                    // bit3  : append FCS in backend implemented (0: reserved)
                    // bit4  : compact PTP sideband path present (legacy software keeps this as a hint bit)
                    // bit5  : commit/shadow config supported
                    // bit6  : legacy pkt mirror supported
                    // bit7  : dedicated PTP peer addr split shadow bank present
                    csr_rdata = {
                        16'h0001,
                        8'd0,
                        1'b1,
                        1'b1,
                        1'b1,
                        1'b1,
                        1'b0,
                        1'b0,
                        1'b1,
                        1'b1
                    };
                end

                ADDR_PTP_CFG: begin
                    // 与当前软件地址图保持一致：
                    //   bit0   enable
                    //   bit1   one_step
                    //   bit2   two_step
                    //   bit3   event_priority_enable
                    //   bit15:8 domain
                    // 其余位读 0。
                    csr_rdata = {
                        16'd0,
                        cfg_ptp_domain,
                        4'd0,
                        cfg_ptp_event_priority_enable,
                        cfg_ptp_two_step,
                        cfg_ptp_one_step,
                        cfg_ptp_enable
                    };
                end

                ADDR_PTP_CLOCK_ID_L: begin
                    csr_rdata = cfg_ptp_clock_id[31:0];
                end

                ADDR_PTP_CLOCK_ID_H: begin
                    csr_rdata = cfg_ptp_clock_id[63:32];
                end

                ADDR_PTP_PORT_NUM: begin
                    csr_rdata = {16'd0, cfg_ptp_port_num};
                end

                ADDR_PTP_TXSTAMP_L: begin
                    csr_rdata = ptp_txstamp_ns_latched[31:0];
                end

                ADDR_PTP_TXSTAMP_H: begin
                    csr_rdata = ptp_txstamp_ns_latched[63:32];
                end

                ADDR_PTP_STATUS: begin
                    // bit0 txstamp_valid
                    // bit1 txstamp_overflow
                    // bit2 tx_busy        （含 sys wait_ack/outstanding）
                    // bit3 tx_cmd_pending （含 sys wait_ack/outstanding）
                    // bit4 cur_time_valid
                    // bit5 tod_enable
                    // bit6 tod_hold
                    // bit7 ptp_enable
                    csr_rdata = {24'd0,
                                 cfg_ptp_enable,
                                 cfg_ptp_hold,
                                 cfg_ptp_tod_enable_r,
                                 ptp_cur_time_valid,
                                 ptp_tx_cmd_pending,
                                 ptp_tx_busy,
                                 ptp_txstamp_overflow_sticky,
                                 ptp_txstamp_valid_latched};
                end

                ADDR_PTP_CTRL: begin
                    // bit7 / bit11:8 读回当前固定主动 TX 策略：need_timestamp=1, msg_type=Delay_Req
                    csr_rdata = {cfg_ptp_tx_sequence_id,
                                 cfg_ptp_tx_transport_specific,
                                 cfg_ptp_tx_message_type,
                                 cfg_ptp_tx_need_timestamp,
                                 1'b0,
                                 1'b0,
                                 1'b0,
                                 1'b0,
                                 1'b0,
                                 cfg_ptp_hold,
                                 cfg_ptp_tod_enable_r};
                end

                ADDR_PTP_TOD_SET_L: begin
                    csr_rdata = cfg_ptp_set_time_ns[31:0];
                end

                ADDR_PTP_TOD_SET_H: begin
                    csr_rdata = cfg_ptp_set_time_ns[63:32];
                end

                ADDR_PTP_STEP_NS: begin
                    csr_rdata = cfg_ptp_step_ns;
                end

                ADDR_PTP_FREQ_ADJ: begin
                    csr_rdata = cfg_ptp_freq_adj_ppb;
                end

                ADDR_PTP_CUR_TIME_L: begin
                    csr_rdata = ptp_cur_time_ns[31:0];
                end

                ADDR_PTP_CUR_TIME_H: begin
                    csr_rdata = ptp_cur_time_ns[63:32];
                end

                ADDR_PTP_TX_CFG: begin
                    csr_rdata = {8'd0, cfg_ptp_tx_log_message_interval, cfg_ptp_tx_flags};
                end

                ADDR_PTP_FEATURES: begin
                    // [31:16] version
                    // bit0  : sys-domain ToD present
                    // bit1  : eth-domain ToD present
                    // bit2  : compact TX engine present
                    // bit3  : tx_arbiter integration present
                    // bit4  : txstamp CDC bridge present
                    // bit5  : compact TX register interface present
                    // bit6  : optional PTP peer/data-dst address split present
                    csr_rdata = PTP_FEATURES_VALUE;
                end

                ADDR_DBGEXT_FEATURES: begin
                    csr_rdata = DBGEXT_FEATURES_VALUE;
                end

                ADDR_DBGEXT_ARB_DATA_FRAME_COUNT: begin
                    csr_rdata = dbgext_arb_data_frame_count;
                end

                ADDR_DBGEXT_ARB_PTP_FRAME_COUNT: begin
                    csr_rdata = dbgext_arb_ptp_frame_count;
                end

                ADDR_DBGEXT_ARB_STALL_CYCLES: begin
                    csr_rdata = dbgext_arb_stall_cycles;
                end

                ADDR_DBGEXT_ARB_LAST_ERROR: begin
                    csr_rdata = {16'd0, dbgext_arb_last_error};
                end

                ADDR_DBGEXT_BACKPRESSURE_CYCLES: begin
                    csr_rdata = dbgext_backend_backpressure_cycles;
                end

                ADDR_DBGEXT_PTP_TX_TRIGGER_REQ_CNT: begin
                    csr_rdata = dbgext_ptp_tx_trigger_req_count;
                end

                ADDR_DBGEXT_PTP_TX_TRIGGER_ACC_CNT: begin
                    csr_rdata = dbgext_ptp_tx_trigger_accept_count;
                end

                ADDR_DBGEXT_PTP_TX_TRIGGER_BUSY_CNT: begin
                    csr_rdata = dbgext_ptp_tx_trigger_while_busy_count;
                end

                ADDR_DBGEXT_PTP_TX_LAST_ERROR: begin
                    csr_rdata = {16'd0, dbgext_ptp_tx_last_error};
                end

                ADDR_DBGEXT_PTP_RX_ACCEPT_COUNT: begin
                    csr_rdata = dbgext_ptp_rx_accept_count;
                end

                ADDR_DBGEXT_PTP_RX_DROP_COUNT: begin
                    csr_rdata = dbgext_ptp_rx_drop_count;
                end

                ADDR_DBGEXT_PTP_RX_LAST_ERROR: begin
                    csr_rdata = {16'd0, dbgext_ptp_rx_last_error};
                end

                ADDR_DBGEXT_PTP_RX_STATUS: begin
                    csr_rdata = dbgext_ptp_rx_status;
                end

                ADDR_TIME_QUALITY: begin
                    csr_rdata = {30'd0, cfg_time_quality};
                end

                ADDR_TXS_CTRL: begin
                    csr_rdata = {29'd0, cfg_txs_strict_window, cfg_txs_block_when_time_invalid, cfg_txs_enable};
                end

                ADDR_TXS_BASE_TIME_L: begin
                    csr_rdata = cfg_txs_base_time_ns[31:0];
                end

                ADDR_TXS_BASE_TIME_H: begin
                    csr_rdata = cfg_txs_base_time_ns[63:32];
                end

                ADDR_TXS_PERIOD_NS: begin
                    csr_rdata = cfg_txs_period_ns;
                end

                ADDR_TXS_SLOT_OFFSET_NS: begin
                    csr_rdata = cfg_txs_slot_offset_ns;
                end

                ADDR_TXS_SLOT_WIDTH_NS: begin
                    csr_rdata = cfg_txs_slot_width_ns;
                end

                ADDR_TXS_ADVANCE_NS: begin
                    csr_rdata = cfg_txs_advance_ns;
                end

                ADDR_TXS_LATE_TOL_NS: begin
                    csr_rdata = cfg_txs_late_tol_ns;
                end

                ADDR_TXS_STATUS: begin
                    // Runtime status bit layout is produced by net_acq_data_tx_scheduler:
                    //   bit0 enable, bit1 tod_valid, bit2 pending_frame,
                    //   bit3 release_armed, bit4 frame_active, bit5 cfg_valid,
                    //   bit6 block_on_bad_time, bit7 strict_window,
                    //   bit8 last_late, bit9 last_slot_miss,
                    //   bit10 window_open, bit11 bypass.
                    csr_rdata = txs_stat_status;
                end

                ADDR_TXS_HELD_COUNT: begin
                    csr_rdata = txs_stat_held_count;
                end

                ADDR_TXS_PASS_COUNT: begin
                    csr_rdata = txs_stat_pass_count;
                end

                ADDR_TXS_LATE_COUNT: begin
                    csr_rdata = txs_stat_late_count;
                end

                ADDR_TXS_SLOT_MISS_COUNT: begin
                    csr_rdata = txs_stat_slot_miss_count;
                end

                ADDR_TXS_LAST_PLAN_L: begin
                    csr_rdata = txs_stat_last_plan_ns[31:0];
                end

                ADDR_TXS_LAST_PLAN_H: begin
                    csr_rdata = txs_stat_last_plan_ns[63:32];
                end

                ADDR_TXS_LAST_ACTUAL_L: begin
                    csr_rdata = txs_stat_last_actual_ns[31:0];
                end

                ADDR_TXS_LAST_ACTUAL_H: begin
                    csr_rdata = txs_stat_last_actual_ns[63:32];
                end

                ADDR_TXS_LAST_DELTA_NS: begin
                    csr_rdata = txs_stat_last_delta_ns;
                end

                ADDR_TXS_FEATURES: begin
                    csr_rdata = TXS_FEATURES_VALUE;
                end

                ADDR_TXS_ACTIVE_CTRL: begin
                    csr_rdata = {29'd0,
                                 txs_active_strict_window,
                                 txs_active_block_when_time_invalid,
                                 txs_active_enable};
                end

                ADDR_TXS_ACTIVE_BASE_TIME_L: begin
                    csr_rdata = txs_active_base_time_ns[31:0];
                end

                ADDR_TXS_ACTIVE_BASE_TIME_H: begin
                    csr_rdata = txs_active_base_time_ns[63:32];
                end

                ADDR_TXS_ACTIVE_PERIOD_NS: begin
                    csr_rdata = txs_active_period_ns;
                end

                ADDR_TXS_ACTIVE_SLOT_OFFSET_NS: begin
                    csr_rdata = txs_active_slot_offset_ns;
                end

                ADDR_TXS_ACTIVE_SLOT_WIDTH_NS: begin
                    csr_rdata = txs_active_slot_width_ns;
                end

                ADDR_TXS_ACTIVE_ADVANCE_NS: begin
                    csr_rdata = txs_active_advance_ns;
                end

                ADDR_TXS_ACTIVE_LATE_TOL_NS: begin
                    csr_rdata = txs_active_late_tol_ns;
                end

                default: begin
                    csr_rdata = 32'd0;
                end
            endcase
        end
    end

endmodule
